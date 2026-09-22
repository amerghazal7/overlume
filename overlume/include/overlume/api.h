// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <cstddef>

namespace overlume {

struct CameraPose {
    double eye[3];
    double target[3];
    double vfov_deg;
};

struct RenderConfig {
    uint32_t width;
    uint32_t height;
    uint8_t quality;
    const char* theme_assets_dir;
    const char* initial_theme;
};

struct FrameView {
    uint8_t* rgb;
    uint32_t width;
    uint32_t height;
};

class VisualRenderer;

VisualRenderer* create_renderer(const RenderConfig&);
void destroy_renderer(VisualRenderer*);
bool render_frame(VisualRenderer*, const CameraPose&, FrameView out);

}
