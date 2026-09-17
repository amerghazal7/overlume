// trajectory_carpet.hpp — internal-only (`-I src`), not installed, not POD.
// Declares the per-frame update for the ONE trajectory_carpet.mat instance
// the whole TrajectoryCarpetLayer shares (VM-077), mirroring point_cloud.hpp
// exactly.
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see trajectory_carpet_test_hooks.hpp
// (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

// Diffs `scene.trajectory_carpets`/`_count` against `r.trajectoryCarpetSlots`
// (keyed by slot index, same shape as pointCloudSlots): a slot whose
// ribbon_signature()-shaped content signature changed (position/half-width/
// ego-clip -- deliberately NO color term, see trajectory_carpet.cpp's file
// header for why) gets its (possibly chunked, past polyline.hpp's
// kMaxPointsPerMesh) meshes rebuilt from scratch as an extruded ribbon strip
// (ribbon.cpp's extrude_polyline()/extrude_polyline_indices() machinery,
// reused verbatim on the carpet's own centerline stations) at a half-width
// derived from the theme's ribbon.margin_velocity_m token. Slots >=
// trajectory_carpet_count are torn down. The single shared
// trajectory_carpet.mat instance's "alpha" parameter is set once per call
// from the freshest live carpet's staleness_alpha() (same "freshest of
// several" stated choice as point_cloud.cpp -- today's shipped profile
// carries exactly one row, so this never fires in practice); OPAQUE while
// fresh (2026-09-10 redirect), no fixed base-alpha multiplier. Called from
// render_frame() on the thread that owns the Engine.
void update_trajectory_carpets(VisualRenderer& r, const SceneGraph& scene);

}  // namespace overlume
