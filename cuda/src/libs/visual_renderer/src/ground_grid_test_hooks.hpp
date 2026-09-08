// ground_grid_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_ground_grid.cpp links only against `visual_renderer` and has
// no access to its PRIVATE Filament include dir, so it can't include
// renderer_internal.hpp (or ground_grid.hpp) directly. These hooks are
// declared only against api.h's opaque VisualRenderer — defined in
// ground_grid.cpp, where the state they read actually lives.
#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"

namespace mpviz::testing {

// The freeColor/occupiedColor values last pushed to every ground_grid.mat
// instance's ramp-endpoint parameters (mirrored CPU copy — MaterialInstance
// has no getter). Both kind instances always carry the same values
// (theme.hpp has no dedicated per-kind OGM token), so one pair covers both.
// {0,0,0} if `r` is null.
mpviz::detail::Float3 ground_grid_free_color(mpviz::VisualRenderer* r);
mpviz::detail::Float3 ground_grid_occupied_color(mpviz::VisualRenderer* r);

// Opaque identity of the GPU texture bound to ground-grid slot `slot` (an
// index into the last-published SceneGraph::grids) — proves
// TextureIsUpdatedInPlaceNotRecreated (identical value across frames whose
// cell data changed but dims didn't) and DimensionChangeRecreatesTheTexture
// (a different, non-null value after a dims change). nullptr if `r` is
// null, `slot` is out of range, or that slot has never had a texture built.
const void* ground_grid_texture_handle(mpviz::VisualRenderer* r, size_t slot);

// Counts how many times slot `slot`'s texture has been destroyed+rebuilt
// (never bumped by an in-place setImage() upload) -- see
// renderer_internal.hpp's GroundGridSlot::textureGeneration for why raw
// pointer comparison isn't reliable here. 0 if `r` is null or `slot` is
// out of range.
uint32_t ground_grid_texture_generation(mpviz::VisualRenderer* r, size_t slot);

// Counts how many times slot `slot`'s texture has actually had
// upload_occupancy_texture() run against it (both the destroy+rebuild and
// the same-dims in-place path) -- not bumped when update_ground_grids()'s
// upload gate skips a redundant re-upload of byte-identical content. 0 if
// `r` is null or `slot` is out of range.
uint32_t ground_grid_texture_upload_count(mpviz::VisualRenderer* r, size_t slot);

// The alpha value last passed to `kind`'s ground_grid.mat instance's
// "alpha" parameter (the staleness knob) — mirrored CPU copy, since
// MaterialInstance has no getter. 1.0f if `r` is null or `kind` is out of
// range (>= 2).
float ground_grid_material_alpha(mpviz::VisualRenderer* r, uint8_t kind);

}  // namespace mpviz::testing
