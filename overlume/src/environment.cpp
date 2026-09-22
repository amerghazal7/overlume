// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "environment.hpp"
#include "environment_test_hooks.hpp"
#include "gltf_normals.hpp"
#include "renderer_internal.hpp"
#include "overlume/api.h"

#include <yaml-cpp/yaml.h>

#include <filament/RenderableManager.h>

#include <cmath>
#include <fstream>
#include <utility>

namespace overlume {

namespace {

double distance(const Vec3& a, const Vec3& b) {
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

BakedEnvironmentSource::BakedEnvironmentSource(std::string dir,
                                               std::vector<EnvironmentChunk> chunks,
                                               GeoAnchor anchor)
    : dir_(std::move(dir)), chunks_(std::move(chunks)), anchor_(anchor) {}

void BakedEnvironmentSource::update(VisualRenderer& r, Vec3 ego_map_pos) {
    for (const EnvironmentChunk& chunk : chunks_) {
        if (loaded_.find(chunk.id) != loaded_.end()) continue;
        if (failed_.count(chunk.id) != 0) continue;
        if (distance(chunk.center, ego_map_pos) > kLoadRadiusM) continue;

        if (!ensure_gltf_loader(r)) continue;
        std::ifstream file(dir_ + "/" + chunk.path, std::ios::binary | std::ios::ate);
        if (!file) {
            failed_.insert(chunk.id);
            continue;
        }
        const std::streamsize size = file.tellg();
        if (size <= 0) {
            failed_.insert(chunk.id);
            continue;
        }
        std::vector<uint8_t> bytes(static_cast<size_t>(size));
        file.seekg(0);
        if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) {
            failed_.insert(chunk.id);
            continue;
        }

        bytes = ensure_flat_normals(std::move(bytes));

        filament::gltfio::FilamentAsset* asset =
            r.sharedAssetLoader->createAsset(bytes.data(), static_cast<uint32_t>(bytes.size()));
        if (asset == nullptr) {
            failed_.insert(chunk.id);
            continue;
        }
        if (!r.sharedResourceLoader->loadResources(asset)) {
            r.sharedAssetLoader->destroyAsset(asset);
            failed_.insert(chunk.id);
            continue;
        }
        asset->releaseSourceData();

        filament::RenderableManager& rm = r.engine->getRenderableManager();
        const utils::Entity* renderables = asset->getRenderableEntities();
        const size_t renderableCount = asset->getRenderableEntityCount();
        for (size_t i = 0; i < renderableCount; ++i) {
            const auto inst = rm.getInstance(renderables[i]);
            if (!inst.isValid()) continue;
            const size_t primCount = rm.getPrimitiveCount(inst);
            for (size_t p = 0; p < primCount; ++p) {
                rm.setMaterialInstanceAt(inst, p, r.buildingMaterial);
            }
            rm.setCastShadows(inst, true);
            rm.setReceiveShadows(inst, true);
        }

        if (visible_) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        }
        loaded_.emplace(chunk.id, LoadedChunk{asset, chunk.center});
    }

    for (auto it = loaded_.begin(); it != loaded_.end();) {
        if (distance(it->second.center, ego_map_pos) > kUnloadRadiusM) {
            filament::gltfio::FilamentAsset* asset = it->second.asset;
            if (visible_) {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
            r.sharedAssetLoader->destroyAsset(asset);
            it = loaded_.erase(it);
        } else {
            ++it;
        }
    }
    for (auto it = failed_.begin(); it != failed_.end();) {
        const EnvironmentChunk* chunk = find_chunk(*it);
        if (chunk == nullptr || distance(chunk->center, ego_map_pos) > kUnloadRadiusM) {
            it = failed_.erase(it);
        } else {
            ++it;
        }
    }
}

void BakedEnvironmentSource::teardown(VisualRenderer& r) {
    for (auto& [id, chunk] : loaded_) {
        (void)id;
        if (visible_) {
            r.scene->removeEntities(chunk.asset->getEntities(), chunk.asset->getEntityCount());
        }
        r.sharedAssetLoader->destroyAsset(chunk.asset);
    }
    loaded_.clear();
}

size_t BakedEnvironmentSource::scene_membership_count(VisualRenderer& r) const {
    size_t count = 0;
    for (const auto& [id, chunk] : loaded_) {
        (void)id;
        if (chunk.asset->getEntityCount() > 0 &&
            r.scene->hasEntity(chunk.asset->getEntities()[0])) {
            ++count;
        }
    }
    return count;
}

void BakedEnvironmentSource::set_visible(VisualRenderer& r, bool visible) {
    if (visible == visible_) return;
    visible_ = visible;
    for (auto& [id, chunk] : loaded_) {
        (void)id;
        if (visible_) {
            r.scene->addEntities(chunk.asset->getEntities(), chunk.asset->getEntityCount());
        } else {
            r.scene->removeEntities(chunk.asset->getEntities(), chunk.asset->getEntityCount());
        }
    }
}

std::unique_ptr<BakedEnvironmentSource> open_baked_environment_source(const std::string& dir,
                                                                      GeoAnchor anchor) {
    std::vector<EnvironmentChunk> chunks;
    try {
        const YAML::Node root = YAML::LoadFile(dir + "/index.yaml");
        const YAML::Node chunkList = root["chunks"];
        if (!chunkList || !chunkList.IsSequence()) return nullptr;
        for (const YAML::Node& c : chunkList) {
            EnvironmentChunk chunk;
            chunk.id = c["id"].as<std::string>();
            chunk.path = c["path"].as<std::string>();
            const YAML::Node center = c["center"];
            if (!center || !center.IsSequence() || center.size() != 3) return nullptr;
            chunk.center =
                Vec3{center[0].as<double>(), center[1].as<double>(), center[2].as<double>()};
            chunk.radius_m = c["radius_m"].as<double>();
            chunks.push_back(std::move(chunk));
        }
    } catch (const std::exception&) {
        return nullptr;
    }
    return std::make_unique<BakedEnvironmentSource>(dir, std::move(chunks), anchor);
}

}  // namespace overlume

namespace overlume {

namespace {
constexpr char kIonPrefix[] = "ion://";
constexpr size_t kIonPrefixLen = sizeof(kIonPrefix) - 1;
}  // namespace

bool set_environment_source(VisualRenderer* r, const char* source_uri, GeoAnchor anchor) {
    if (r == nullptr || source_uri == nullptr || source_uri[0] == '\0') return false;

    std::unique_ptr<EnvironmentSource> source;
    const std::string uri(source_uri);
    if (uri.compare(0, kIonPrefixLen, kIonPrefix) == 0) {
#ifdef OVERLUME_ENABLE_CESIUM
        source = open_streaming_environment_source(uri.substr(kIonPrefixLen), anchor);
#else
        return false;
#endif
    } else {
        source = open_baked_environment_source(uri, anchor);
    }
    if (!source) return false;
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) {
        r->environmentSource->teardown(*r);
    }
    r->environmentSource = std::move(source);
    return true;
}

bool set_environment_visible(VisualRenderer* r, bool visible) {
    if (r == nullptr) return false;
    r->environmentVisible = visible;
    if (r->environmentSource) {
        r->environmentSource->set_visible(*r, visible);
    }
    return true;
}

bool environment_visible(VisualRenderer* r) {
    if (r == nullptr) return false;
    return r->environmentVisible;
}

EnvironmentSourceState environment_source_state(VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return EnvironmentSourceState::NONE;
    return r->environmentSource->state();
}

}  // namespace overlume

namespace overlume::testing {

uint64_t environment_loaded_chunk_count(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return 0;
    return static_cast<uint64_t>(r->environmentSource->loaded_count());
}

uint64_t environment_scene_membership_count(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return 0;
    return static_cast<uint64_t>(r->environmentSource->scene_membership_count(*r));
}

bool environment_stream_provides_ground(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    return r->environmentSource->provides_ground();
}

bool renderer_ground_plane_in_scene(overlume::VisualRenderer* r) {
    if (r == nullptr || r->scene == nullptr) return false;
    return r->scene->hasEntity(r->ground.entity);
}

}  // namespace overlume::testing
