// environment.cpp — see environment.hpp. Runtime chunk load/unload behind
// EnvironmentSource, distance-culled against SceneGraph::EgoState::position
// (no per-tick SceneGraph field needed -- ego position is already there).
// Buildings are OPAQUE clay (r.buildingMaterial) the entire time they're
// loaded -- chunks load/unload by distance, they never stale-fade, so there
// is no fade-blended building material to introduce (the standing library
// convention since the flicker root-cause fix: a fade-blended always-on
// material on a category with no real staleness concept is exactly the
// mistake that bit ribbon/carpet).
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
        // A chunk that failed to load is memoed, not retried per tick -- a
        // truncated .glb would otherwise cost a full file read + createAsset
        // attempt at frame rate. The memo clears when the ego leaves the
        // unload radius (see the unload loop), so a fixed file is retried on
        // the next approach.
        if (failed_.count(chunk.id) != 0) continue;
        if (distance(chunk.center, ego_map_pos) > kLoadRadiusM) continue;

        if (!ensure_gltf_loader(r)) continue;  // global, not per-chunk: keep retrying
        std::ifstream file(dir_ + "/" + chunk.path, std::ios::binary | std::ios::ate);
        if (!file) { failed_.insert(chunk.id); continue; }
        const std::streamsize size = file.tellg();
        if (size <= 0) { failed_.insert(chunk.id); continue; }
        std::vector<uint8_t> bytes(static_cast<size_t>(size));
        file.seekg(0);
        if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) {
            failed_.insert(chunk.id);
            continue;
        }

        filament::gltfio::FilamentAsset* asset =
            r.sharedAssetLoader->createAsset(bytes.data(), static_cast<uint32_t>(bytes.size()));
        if (asset == nullptr) { failed_.insert(chunk.id); continue; }
        if (!r.sharedResourceLoader->loadResources(asset)) {
            r.sharedAssetLoader->destroyAsset(asset);
            failed_.insert(chunk.id);
            continue;
        }
        asset->releaseSourceData();

        // Material remap: every primitive reads as building clay
        // (r.buildingMaterial) -- same ifstream -> createAsset ->
        // loadResources -> releaseSourceData + remap sequence
        // set_ego_model() (ego.cpp) uses.
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

        // visible_ invariant (environment.hpp): only add to the scene if
        // currently visible -- a chunk that loads while hidden must not
        // pop into view. It stays in loaded_ either way (loaded_count()
        // doesn't care), so a later set_visible(true) picks it up without
        // a reload.
        if (visible_) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        }
        loaded_.emplace(chunk.id, LoadedChunk{asset, chunk.center});
    }

    // Unload: any loaded chunk now beyond kUnloadRadiusM -- a wider radius
    // than kLoadRadiusM above (named hysteresis band, environment.hpp), so
    // a chunk right at one boundary doesn't reload/unload every tick.
    for (auto it = loaded_.begin(); it != loaded_.end();) {
        if (distance(it->second.center, ego_map_pos) > kUnloadRadiusM) {
            filament::gltfio::FilamentAsset* asset = it->second.asset;
            // visible_ invariant: only remove from the scene if it was
            // ever added there (skipped entirely while hidden, see the
            // load loop above) -- removing an entity never added would
            // still be harmless in Filament, but this keeps the intent
            // explicit rather than relying on that.
            if (visible_) {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
            r.sharedAssetLoader->destroyAsset(asset);
            it = loaded_.erase(it);
        } else {
            ++it;
        }
    }
    // Retry-on-re-approach: a failed chunk's memo clears once the ego is
    // beyond the unload radius, mirroring the loaded-chunk lifecycle.
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

void BakedEnvironmentSource::set_visible(VisualRenderer& r, bool visible) {
    if (visible == visible_) return;  // no-op: matches the current state already
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

namespace {
// Epic 6 (VM-062) Decision 5: dispatch on a scheme prefix inside this
// existing, otherwise-unchanged entry point. No URL library -- a literal
// prefix compare, exactly as wide as the one distinction this function
// needs to make.
constexpr char kIonPrefix[] = "ion://";
constexpr size_t kIonPrefixLen = sizeof(kIonPrefix) - 1;
}  // namespace

// Epic 6 (VM-062): `source_uri` with no "ion://" prefix opens the baked
// backend, byte-for-byte today's behavior (Decision 5) -- every existing
// caller/config/test unaffected. An "ion://" prefix opens the streaming
// backend instead (`open_streaming_environment_source`, environment_stream.cpp,
// the one C++20 TU); with MPVIZ_ENABLE_CESIUM off (the ordinary build/ tree,
// see CMakeLists.txt), that TU isn't compiled in at all, so this branch
// returns false rather than referencing an undefined symbol -- an "ion://"
// source_uri configured against a cesium-less build degrades the same way
// a missing bake dir does (non-fatal, caller WARNs).
bool set_environment_source(VisualRenderer* r, const char* source_uri, GeoAnchor anchor) {
    if (r == nullptr || source_uri == nullptr || source_uri[0] == '\0') return false;

    std::unique_ptr<EnvironmentSource> source;
    const std::string uri(source_uri);
    if (uri.compare(0, kIonPrefixLen, kIonPrefix) == 0) {
#ifdef MPVIZ_ENABLE_CESIUM
        source = open_streaming_environment_source(uri.substr(kIonPrefixLen), anchor);
#else
        return false;  // cesium not compiled into this build (MPVIZ_ENABLE_CESIUM off)
#endif
    } else {
        source = open_baked_environment_source(uri, anchor);
    }
    if (!source) return false;
    // This task: a freshly constructed source always defaults visible_ =
    // true -- sync it to whatever set_environment_visible() last recorded
    // on `r` BEFORE installing it, so a preset switch (baked/osm/clipped/
    // google) made while the GUI's toggle is off doesn't pop the new
    // source into view. Harmless no-op the very first time this ever runs
    // (r->environmentVisible defaults true too, matching every pre-this-task
    // call site's behavior byte-for-byte).
    source->set_visible(*r, r->environmentVisible);
    // on_activate() runs again after on_deactivate() on the SAME renderer
    // (on_deactivate does not destroy it), so this entry point is
    // re-entrant -- the old source's chunks must be released here, its
    // destructor cannot.
    if (r->environmentSource) {
        r->environmentSource->teardown(*r);
    }
    r->environmentSource = std::move(source);
    return true;
}

// This task (vcam GUI Environment Tiles toggle): see scene.h's own comment
// for the full contract. r->environmentVisible is the persisted flag
// set_environment_source() above re-applies to every newly installed
// source; here it is also pushed live onto whatever source is installed
// right now (a no-op inside EnvironmentSource::set_visible() if it already
// matches).
bool set_environment_visible(VisualRenderer* r, bool visible) {
    if (r == nullptr) return false;
    r->environmentVisible = visible;
    if (r->environmentSource) {
        r->environmentSource->set_visible(*r, visible);
    }
    return true;
}

// Epic 6 (VM-063) Decision 11: the node's only window into a streaming
// source's live health, since the library can't WARN itself (POD-boundary
// convention, same as set_environment_source above) and network loss
// happens mid-run, long after set_environment_source() returned true.
// NONE on null r or when no source is configured -- everything else goes
// through the EnvironmentSource virtual (Decision 12's precedent: a
// downcast here would be unsafe against a second concrete type).
EnvironmentSourceState environment_source_state(VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return EnvironmentSourceState::NONE;
    return r->environmentSource->state();
}

}  // namespace mpviz

namespace mpviz::testing {

uint64_t environment_loaded_chunk_count(mpviz::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return 0;
    // Epic 6 (VM-062) Decision 12: goes through the virtual now that a
    // second concrete EnvironmentSource type (StreamingEnvironmentSource)
    // exists -- a static_cast to BakedEnvironmentSource* here would be
    // undefined behavior against a streaming source. Same numbers as
    // before through the virtual (BakedEnvironmentSource::loaded_count()
    // forwards to loaded_chunk_count()), so every existing test stays
    // green unchanged.
    return static_cast<uint64_t>(r->environmentSource->loaded_count());
}

uint64_t environment_scene_membership_count(mpviz::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return 0;
    return static_cast<uint64_t>(r->environmentSource->scene_membership_count());
}

}  // namespace mpviz::testing
