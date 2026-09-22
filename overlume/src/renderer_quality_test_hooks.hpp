// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>

#include "overlume/api.h"

namespace overlume::testing {

bool quality_shadows_enabled(overlume::VisualRenderer* r);

uint32_t quality_shadow_map_size(overlume::VisualRenderer* r);

struct QualityRenderSize {
    uint32_t width = 0;
    uint32_t height = 0;
};

QualityRenderSize quality_internal_render_size(overlume::VisualRenderer* r);

struct QualitySsao {
    bool enabled = false;
    float resolution = 0.0f;
};

QualitySsao quality_ssao(overlume::VisualRenderer* r);

bool quality_taa_enabled(overlume::VisualRenderer* r);

enum class QualityAntiAliasing : uint8_t { NONE = 0, FXAA = 1 };
QualityAntiAliasing quality_antialiasing(overlume::VisualRenderer* r);

}  // namespace overlume::testing
