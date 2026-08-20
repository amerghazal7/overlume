// ground_grid_test_hooks.hpp — internal-only, `-I src` visibility, not
// installed, not POD (Epic 2 Task 6 / VM-025). Same reasoning as Task 2's
// map_elements_test_hooks.hpp / Task 5's ribbon_test_hooks.hpp:
// tests/test_ground_grid.cpp links only against `visual_renderer` and is
// never granted that target's PRIVATE Filament include dir, so it can't
// include renderer_internal.hpp (or ground_grid.hpp) directly. These hooks
// are declared against nothing but api.h's opaque mpviz::VisualRenderer —
// defined in ground_grid.cpp, where the state they read actually lives.
#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"

namespace mpviz::testing {

// The freeColor/occupiedColor values last actually pushed to EVERY
// ground_grid.mat instance's ramp-endpoint parameters (Task 6 Step 4) --
// NOT a read-back of live GPU state (MaterialInstance has no getter),
// mirrors push_theme_to_scene()'s own last push. Both kind instances always
// carry the SAME values (theme.hpp has no dedicated per-kind OGM token, see
// renderer.cpp's own comment), so one pair covers both. {0,0,0} if `r` is
// null.
mpviz::detail::Float3 ground_grid_free_color(mpviz::VisualRenderer* r);
mpviz::detail::Float3 ground_grid_occupied_color(mpviz::VisualRenderer* r);

// Opaque identity of the GPU texture bound to ground-grid slot `slot` (an
// index into the last-published SceneGraph::grids, same slot numbering as
// ribbon_test_hooks.hpp's ribbon_mesh_count) — proves
// TextureIsUpdatedInPlaceNotRecreated (identical value across two frames
// whose cell data changed but whose dims didn't) and
// DimensionChangeRecreatesTheTexture (a different, non-null value after a
// dims change). nullptr if `r` is null, `slot` is out of range, or that
// slot has never had a texture built.
const void* ground_grid_texture_handle(mpviz::VisualRenderer* r, size_t slot);

// Counts how many times slot `slot`'s texture has been destroyed+rebuilt
// (never bumped by an in-place setImage() upload) -- the robust proof
// DimensionChangeRecreatesTheTexture uses instead of pointer comparison
// (see renderer_internal.hpp's GroundGridSlot::textureGeneration comment
// for why raw addresses aren't reliable here). 0 if `r` is null or `slot`
// is out of range.
uint32_t ground_grid_texture_generation(mpviz::VisualRenderer* r, size_t slot);

// Counts how many times slot `slot`'s texture has actually had
// upload_occupancy_texture() run against it -- bumped on every real
// setImage() call (both the destroy+rebuild path and the same-dims
// in-place path), NOT bumped when update_ground_grids()'s upload gate
// (GroundGridSlot::last_upload_sec vs. the grid's last_update_sec) skips a
// redundant re-upload of byte-identical content. 0 if `r` is null or
// `slot` is out of range.
uint32_t ground_grid_texture_upload_count(mpviz::VisualRenderer* r, size_t slot);

// The alpha value last actually passed to `kind`'s ground_grid.mat
// instance's "alpha" parameter (the staleness knob, Task 6 Step 4) — NOT a
// read-back of live GPU state (MaterialInstance has no getter), mirrors
// whatever update_ground_grids() last pushed for that kind. 1.0f if `r` is
// null or `kind` is out of range (>= 2).
float ground_grid_material_alpha(mpviz::VisualRenderer* r, uint8_t kind);

}  // namespace mpviz::testing
