// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "theme.hpp"

namespace overlume::detail {

struct Oklab {
    float L = 0.0f, a = 0.0f, b = 0.0f;
};

Oklab linear_srgb_to_oklab(const Float3& c);
Float3 oklab_to_linear_srgb(const Oklab& c);

float smoothstep01(float t);

Float3 blend_color(const Float3& a, const Float3& b, float t);

Theme blend(const Theme& a, const Theme& b, float t);

struct ThemeTransition {
    Theme from;
    Theme to;
    double start_sec = 0.0;
    double duration_sec = 0.8;
};

}
