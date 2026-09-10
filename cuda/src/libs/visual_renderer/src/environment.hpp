// environment.hpp — library-internal (`-I src`), not installed, not POD.
// Same "never included by tests" rule as renderer_internal.hpp (pulls in
// <gltfio/...>/Filament types); tests/test_environment.cpp uses
// environment_test_hooks.hpp instead. Never #include'd by node code either
// (Decision 2) -- EnvironmentSource/BakedEnvironmentSource are ordinary
// library-internal C++, not subject to the POD rule scene.h/api.h enforce.
//
// EnvironmentSource abstract seam (Decision 2): render_frame()'s call site
// (renderer.cpp) is the ONLY thing that depends on this interface shape, so
// Epic 6's streaming backend (VM-062) can implement it identically with no
// change to set_environment_source()'s own public signature.
//
// BakedEnvironmentSource (Decision 5): chunks are placed in the map frame
// at BAKE time (Decision 3) -- update() below does distance math only,
// never WgsToMap/GeoAnchor conversion; `anchor_` is stored, never read for
// placement (see scene.h's set_environment_source comment for why that's
// deliberate, not a half-used parameter).
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gltfio/FilamentAsset.h>

#include "visual_renderer/scene.h"

namespace mpviz {

class VisualRenderer;

// Same (VisualRenderer&, Vec3 ego_map_pos) shape render_frame()'s call site
// uses -- one update() per tick, gated by the CALLER on ego.valid (Task 3
// Step 2); this interface itself does not re-check validity.
class EnvironmentSource {
public:
    virtual ~EnvironmentSource() = default;
    virtual void update(VisualRenderer& r, Vec3 ego_map_pos) = 0;
    // Tears down every Filament resource this source ever loaded (scene
    // removal + destroyAsset). destroy_renderer() calls this explicitly,
    // BEFORE r.sharedAssetLoader/sharedResourceLoader are torn down -- a
    // bare C++ destructor has no VisualRenderer& to do this teardown with,
    // so relying on ~EnvironmentSource() alone would leak/use-after-free.
    virtual void teardown(VisualRenderer& r) = 0;
};

// kLoadRadiusM/kUnloadRadiusM: named hysteresis band (Files list, Task 3
// Step 1) -- kUnloadRadiusM > kLoadRadiusM so a chunk sitting near one
// boundary doesn't reload/unload every tick as the ego jitters across it
// (same shape VM-040's quality-governor hysteresis is expected to use,
// Epic 5). kLoadRadiusM comfortably covers one bake chunk's own
// bounding-sphere radius_m (~181 m for the shipped chunk_size_m=256
// convention, Task 2's index.yaml) plus a margin, so a chunk loads before
// its footprint is already on screen. Dev-box proxy values (Decision 11:
// the on-robot rerun is Epic 5's), not a tuned budget.
inline constexpr double kLoadRadiusM = 300.0;
inline constexpr double kUnloadRadiusM = 400.0;

struct EnvironmentChunk {
    std::string id;
    std::string path;       // relative to the source dir, per index.yaml
    Vec3 center{};           // map frame (Decision 3 -- already geo-projected)
    double radius_m = 0.0;   // bounding-sphere radius; parsed for schema
                             // completeness/a future precise-AABB cull
                             // upgrade -- v1's cull test is center-distance
                             // only (Files list Step 1), not center+radius.
};

// Opens `<dir>/index.yaml` (Decision 5's yaml-cpp parse), lazily loading
// each indexed chunk's `.glb` as the ego comes within kLoadRadiusM and
// unloading it past kUnloadRadiusM. Loaded primitives are remapped onto
// VisualRenderer::buildingMaterial (Decision 4) -- an OPAQUE clay.mat
// instance; buildings never stale-fade (chunks load/unload by distance,
// not by a publish going stale), so there is no fade-blended twin here,
// unlike ribbon/trajectory-carpet's opaque<->translucent swap (the
// 2026-09-10 flicker-fix convention this deliberately does NOT need).
class BakedEnvironmentSource : public EnvironmentSource {
public:
    BakedEnvironmentSource(std::string dir, std::vector<EnvironmentChunk> chunks, GeoAnchor anchor);

    void update(VisualRenderer& r, Vec3 ego_map_pos) override;
    void teardown(VisualRenderer& r) override;

    // testing-only: environment_test_hooks.hpp's environment_loaded_chunk_count().
    size_t loaded_chunk_count() const { return loaded_.size(); }

private:
    std::string dir_;
    std::vector<EnvironmentChunk> chunks_;
    GeoAnchor anchor_;  // stored, never read for placement -- see file header.

    struct LoadedChunk {
        filament::gltfio::FilamentAsset* asset = nullptr;
        Vec3 center{};  // mirrors the index entry -- avoids a chunks_ re-lookup per unload check
    };
    std::unordered_map<std::string, LoadedChunk> loaded_;  // keyed by chunk id
};

// Parses `<dir>/index.yaml` (Decision 5) and returns a ready
// BakedEnvironmentSource, or nullptr on any open/parse failure (missing
// dir, missing/malformed index.yaml, a chunk entry missing a required
// key) -- mirrors set_ego_model's non-fatal-on-missing-file shape (Task 3
// Step 0); logs nothing itself (POD-adjacent convention), caller WARNs.
std::unique_ptr<BakedEnvironmentSource> open_baked_environment_source(const std::string& dir,
                                                                       GeoAnchor anchor);

}  // namespace mpviz
