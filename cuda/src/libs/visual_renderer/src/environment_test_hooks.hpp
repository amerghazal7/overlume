// environment_test_hooks.hpp — internal-only, not installed, not POD. Same
// reasoning as map_elements_test_hooks.hpp: tests/test_environment.cpp
// links only against `visual_renderer` and has no access to its PRIVATE
// Filament include dir, so it can't include environment.hpp directly.
#pragma once

#include <cstdint>

#include "visual_renderer/api.h"

namespace mpviz::testing {

// Live chunk count BakedEnvironmentSource currently holds loaded (added to
// r->scene) -- proves distance culling gates LOADING, not merely drawing
// (Task 3 Step 1's own AC: "distance-enabled chunks"). 0 if `r` is null or
// no source is configured (set_environment_source() never called, or it
// failed) -- same null-`r` contract as map_element_rebuild_count().
uint64_t environment_loaded_chunk_count(mpviz::VisualRenderer* r);

}  // namespace mpviz::testing
