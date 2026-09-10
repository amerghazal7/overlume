// camera_textures.hpp — internal-only (`-I src`), not installed, not POD.
// Declares CameraTextureSlot, the per-camera state VisualRenderer holds
// directly (renderer_internal.hpp) for the up-to-kMaxBowlCameras persistent
// filament::Texture* the bowl samples from (VM-090/ADR-0005). Task 2's
// bowl.cpp is the first real mesh/material consumer of these slots; this
// task (camera_textures.cpp) owns only texture allocation/upload/dirty-
// tracking plus the motion-delta/visibility storage.
//
// Never included by tests/*.cpp -- see camera_textures_test_hooks.hpp
// (Filament-free) for what tests use instead.
#pragma once

#include "visual_renderer/scene.h"

#include <filament/Texture.h>

#include <cstdint>

namespace mpviz {

// One persistent camera texture + its per-tick dirty/config state. No
// getter on Filament's Texture/MaterialInstance for any of this, so every
// field mirrors what was last pushed -- same "mirror what was pushed"
// convention as every other per-slot state in this library
// (renderer_internal.hpp's GroundGridSlot, RibbonSlot, ...).
struct CameraTextureSlot {
    filament::Texture* texture = nullptr;
    uint32_t width = 0, height = 0;
    // false = never uploaded, so the FIRST set_camera_frame() call for this
    // camera always uploads regardless of frame_id (Task 1 Step 2).
    bool hasUploaded = false;
    uint64_t lastFrameId = 0;
    // Bumped only on an actual setImage() call -- camera_frame_upload_count()
    // test hook reads this to prove the dirty-tracking gate suppresses a
    // repeated frame_id.
    uint32_t uploadCount = 0;
    CameraExtrinsics extrinsics{};
    CameraIntrinsics intrinsics{};
    // Ego-motion-delta uniform (Decision 3's addendum) -- identity default,
    // written per-tick by set_camera_motion_delta(). This task only stores
    // it; Task 2's bowl.cpp is what actually pushes it into a material
    // uniform every render_frame() call.
    double motionDelta[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

}  // namespace mpviz
