// bowl.hpp — internal-only (`-I src`), not installed, not POD. VM-091
// (unified-engine migration Task 2): the camera bowl's Filament mesh +
// material wiring, built from bowl_mesh.hpp's CPU-only bake +
// bowl.mat's per-fragment sampling (see that .mat's own header for the
// full design + the empirically-discovered parameter-budget constraints
// that shaped it).
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp -- test_bowl.cpp's Step 3 tests call
// bowl_mesh.hpp's Filament-free bake function directly instead; Step 4's
// render_frame test only needs the public api.h/scene.h surface, same as
// every other GPU test in this suite.
#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

struct BowlState {
    filament::Material* material = nullptr;
    filament::MaterialInstance* instance = nullptr;
    Mesh mesh;
    // Scene::addEntity/remove has no "is this entity in the scene" query,
    // so update_bowl() mirrors what it last did here (same "mirror what
    // was pushed" convention as every other per-slot state in this
    // library).
    bool inScene = false;
};

// (Re)builds r.bowl's mesh + material instance from `cfg` -- tears down
// whatever bowl currently exists first (a first call has nothing to tear
// down). Called from camera_textures.cpp's set_bowl_config(), AFTER that
// function has (re)allocated the per-camera textures (Task 1) -- this is
// the "and, once Task 2 lands, the bowl mesh + per-vertex weight/index bake
// and per-camera K/plumb_bob uniforms" half of that function's own
// documented contract (scene.h). Bakes the mesh via
// bowl_mesh::BakeBowlMesh (Decision 3's construction rule), builds the
// bowl.mat MaterialInstance, and uploads every per-camera uniform that
// does NOT change per-tick (K, plumb_bob dist/k3, BASE right/fwd/t,
// skyColor) -- the ego-motion-delta-composed EFFECTIVE right/fwd/t are
// pushed every render_frame() call instead, by update_bowl() below, from
// whatever set_camera_motion_delta() last wrote into
// r.cameraSlots[i].motionDelta (camera_textures.hpp). Returns false (and
// leaves r.bowl null) only if the bake produces no geometry -- callers
// (set_bowl_config) have already validated camera_count themselves.
bool build_bowl(VisualRenderer& r, const BowlConfig& cfg);

// Per-render_frame() bowl maintenance, called from render_frame() right
// alongside update_ego_transform() (Decision 3's frame convention: same
// ego-pose source, so bowl-vs-ego depth compositing, Task 3, stays
// aligned). No-op if r.bowl is null (no set_bowl_config() call has ever
// succeeded). Three responsibilities, all cheap (no re-bake, no texture
// touch, no allocation):
//   1. Composes each configured camera's stored ego-motion delta
//      (CameraTextureSlot::motionDelta) with its base extrinsics
//      (CameraTextureSlot::extrinsics) into the EFFECTIVE right/fwd/t
//      bowl.mat's fragment shader samples -- see bowl.mat's header for why
//      this composition happens here, CPU-side, instead of uploading a
//      per-camera mat4 (a real matc parameter-budget ceiling, discovered
//      empirically).
//   2. Syncs the bowl entity's Scene membership with r.bowlVisible
//      (set_bowl_visible) -- added when true and not already in the
//      scene, removed when false and still in it.
//   3. Sets the bowl entity's TransformManager transform to `ego` (map
//      frame; ego.valid == 0 snaps to the map origin, same convention as
//      update_ground_grid_transform) -- the bowl mesh itself is baked in
//      RIG frame (robot at the origin), so this is what anchors it into
//      the map-frame scene each tick, exactly like the ego mesh.
void update_bowl(VisualRenderer& r, const EgoState& ego);

}  // namespace overlume
