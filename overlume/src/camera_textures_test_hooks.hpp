// camera_textures_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_camera_textures.cpp links only against `overlume` and
// has no access to its PRIVATE Filament include dir, so it can't include
// renderer_internal.hpp (or camera_textures.hpp) directly. Declared only
// against api.h's opaque VisualRenderer -- defined in camera_textures.cpp,
// where the state it reads actually lives. Same shape as
// ground_grid_test_hooks.hpp.
#pragma once

#include <cstdint>

#include "overlume/api.h"

namespace overlume::testing {

// Counts how many times camera `cam_idx`'s texture has actually had
// setImage() run against it -- not bumped when set_camera_frame()'s
// dirty-tracking gate skips a redundant re-upload of an unchanged frame_id
// (Task 1 Step 2). 0 if `r` is null, `cam_idx` is out of range, or
// set_bowl_config() has never succeeded.
uint64_t camera_frame_upload_count(overlume::VisualRenderer* r, uint32_t cam_idx);

// Reads back camera `cam_idx`'s currently-stored ego-motion-delta uniform
// (row-major 4x4, scene.h's set_camera_motion_delta contract) into `out` --
// lets a test verify the EXACT delta a caller pushed, not just that the
// call returned true. No-op (leaves `out` untouched) if `r` is null or
// `cam_idx` is out of range.
void camera_motion_delta(overlume::VisualRenderer* r, uint32_t cam_idx, double out[16]);

}  // namespace overlume::testing
