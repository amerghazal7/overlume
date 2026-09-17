// renderer_quality_test_hooks.hpp — internal-only, not installed, not POD.
// Epic 3 Task 5 (VM-032) Step 3: `create_renderer()`'s quality dispatch
// (renderer.cpp) picks shadow-map resolution / shadow enable / low-preset
// render scale straight off `RenderConfig::quality` -- these are all
// Filament View/LightManager state with real getters, so the hooks below
// just read that state back through the opaque VisualRenderer*, same
// Filament-free-header pattern as ground_grid_test_hooks.hpp (tests link
// only against api.h's public POD surface, never renderer_internal.hpp's
// <filament/...> includes).
#pragma once

#include <cstdint>

#include "overlume/api.h"

namespace overlume::testing {

// false if `r` is null.
bool quality_shadows_enabled(overlume::VisualRenderer* r);

// The sun light's shadow map texel size (LightManager::ShadowOptions::
// mapSize). 0 if `r` is null.
uint32_t quality_shadow_map_size(overlume::VisualRenderer* r);

struct QualityRenderSize {
    uint32_t width = 0;
    uint32_t height = 0;
};

// The internal render target size implied by create_renderer()'s dynamic-
// resolution setup: at low preset, the fixed 960x540 (or whatever the
// pinned minScale/maxScale computes to for this RenderConfig::width/
// height); at medium/high (dynamic resolution left off), the requested
// output size itself. {0, 0} if `r` is null.
QualityRenderSize quality_internal_render_size(overlume::VisualRenderer* r);

struct QualitySsao {
    bool enabled = false;
    float resolution = 0.0f;
};

// View::getAmbientOcclusionOptions() read back -- {false, 0} if `r` is null.
QualitySsao quality_ssao(overlume::VisualRenderer* r);

// View::getTemporalAntiAliasingOptions().enabled -- false if `r` is null.
bool quality_taa_enabled(overlume::VisualRenderer* r);

// Mirrors filament::AntiAliasing without leaking a <filament/...> include
// into this Filament-free test header (same convention as the rest of this
// file). NONE if `r` is null.
enum class QualityAntiAliasing : uint8_t { NONE = 0, FXAA = 1 };
QualityAntiAliasing quality_antialiasing(overlume::VisualRenderer* r);

}  // namespace overlume::testing
