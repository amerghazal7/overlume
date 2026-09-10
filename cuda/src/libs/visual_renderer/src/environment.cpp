// environment.cpp — see environment.hpp. Runtime chunk load/unload behind
// EnvironmentSource, distance-culled against SceneGraph::EgoState::position
// (Decision 1: no per-tick SceneGraph field needed -- ego position is
// already there). Buildings are OPAQUE clay (r.buildingMaterial, Decision
// 4) the entire time they're loaded -- chunks load/unload by distance, they
// never stale-fade, so there is no fade-blended building material to
// introduce (the standing library convention since the 2026-09-10 flicker
// root-cause fix: a fade-blended always-on material on a category with no
// real staleness concept is exactly the mistake that bit ribbon/carpet).
#include "environment.hpp"
#include "environment_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/api.h"

#include <yaml-cpp/yaml.h>

#include <filament/RenderableManager.h>

#include <cmath>
#include <fstream>
#include <utility>

namespace mpviz {

namespace {

double distance(const Vec3& a, const Vec3& b) {
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

BakedEnvironmentSource::BakedEnvironmentSource(std::string dir, std::vector<EnvironmentChunk> chunks,
                                                GeoAnchor anchor)
    : dir_(std::move(dir)), chunks_(std::move(chunks)), anchor_(anchor) {}

void BakedEnvironmentSource::update(VisualRenderer& r, Vec3 ego_map_pos) {
    // Load: any indexed chunk within kLoadRadiusM not already loaded.
    for (const EnvironmentChunk& chunk : chunks_) {
        if (loaded_.find(chunk.id) != loaded_.end()) continue;
        if (distance(chunk.center, ego_map_pos) > kLoadRadiusM) continue;

        if (!ensure_gltf_loader(r)) continue;  // non-fatal: this chunk stays unloaded this tick
        std::ifstream file(dir_ + "/" + chunk.path, std::ios::binary | std::ios::ate);
        if (!file) continue;
        const std::streamsize size = file.tellg();
        if (size <= 0) continue;
        std::vector<uint8_t> bytes(static_cast<size_t>(size));
        file.seekg(0);
        if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) continue;

        filament::gltfio::FilamentAsset* asset =
            r.sharedAssetLoader->createAsset(bytes.data(), static_cast<uint32_t>(bytes.size()));
        if (asset == nullptr) continue;
        if (!r.sharedResourceLoader->loadResources(asset)) {
            r.sharedAssetLoader->destroyAsset(asset);
            continue;
        }
        asset->releaseSourceData();

        // Material remap: every primitive reads as building clay
        // (r.buildingMaterial, Decision 4) -- same ifstream -> createAsset
        // -> loadResources -> releaseSourceData + remap sequence
        // set_ego_model() (ego.cpp) uses, per Task 3's own Files list.
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

        r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        loaded_.emplace(chunk.id, LoadedChunk{asset, chunk.center});
    }

    // Unload: any loaded chunk now beyond kUnloadRadiusM -- a wider radius
    // than kLoadRadiusM above (named hysteresis band, environment.hpp), so
    // a chunk right at one boundary doesn't reload/unload every tick.
    for (auto it = loaded_.begin(); it != loaded_.end();) {
        if (distance(it->second.center, ego_map_pos) > kUnloadRadiusM) {
            filament::gltfio::FilamentAsset* asset = it->second.asset;
            r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            r.sharedAssetLoader->destroyAsset(asset);
            it = loaded_.erase(it);
        } else {
            ++it;
        }
    }
}

void BakedEnvironmentSource::teardown(VisualRenderer& r) {
    for (auto& [id, chunk] : loaded_) {
        (void)id;
        r.scene->removeEntities(chunk.asset->getEntities(), chunk.asset->getEntityCount());
        r.sharedAssetLoader->destroyAsset(chunk.asset);
    }
    loaded_.clear();
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
        // Missing dir/file, unreadable, malformed YAML, or a missing/
        // mistyped key -- all non-fatal (same load_theme() convention,
        // theme.cpp): caller (set_environment_source) reports false, its
        // own caller (the node) WARNs once.
        return nullptr;
    }
    return std::make_unique<BakedEnvironmentSource>(dir, std::move(chunks), anchor);
}

}  // namespace mpviz

namespace mpviz {

// Constructs the (currently only) baked backend unconditionally -- Epic 6's
// VM-063 adds baked|streamed selection logic, not this epic's job (scene.h's
// own comment on this function).
bool set_environment_source(VisualRenderer* r, const char* source_uri, GeoAnchor anchor) {
    if (r == nullptr || source_uri == nullptr || source_uri[0] == '\0') return false;
    std::unique_ptr<BakedEnvironmentSource> source = open_baked_environment_source(source_uri, anchor);
    if (!source) return false;
    r->environmentSource = std::move(source);
    return true;
}

}  // namespace mpviz

namespace mpviz::testing {

uint64_t environment_loaded_chunk_count(mpviz::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return 0;
    // Downcast is safe: the only EnvironmentSource concrete type this epic
    // ever constructs is BakedEnvironmentSource (a future streamed backend,
    // Epic 6, is a different concrete type this hook doesn't need to see
    // yet).
    return static_cast<uint64_t>(
        static_cast<mpviz::BakedEnvironmentSource*>(r->environmentSource.get())
            ->loaded_chunk_count());
}

}  // namespace mpviz::testing
