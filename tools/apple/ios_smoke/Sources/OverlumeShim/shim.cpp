// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "shim.h"

#include <Overlume/api.h>
#include <Overlume/scene.h>

#include <cstdint>
#include <vector>

int overlume_shim_zero_size_is_null(void) {
    overlume::RenderConfig c{};
    return overlume::create_renderer(c) == nullptr ? 1 : 0;
}

int overlume_shim_render(void) {
    overlume::RenderConfig c{};
    c.width = 320;
    c.height = 240;
    c.quality = 1;
    c.theme_assets_dir = nullptr;  // framework default: <Overlume.framework>/Resources/themes
    c.initial_theme = "dark_adas";
    overlume::VisualRenderer* r = overlume::create_renderer(c);
    if (r == nullptr) return 0;
    const bool themes = overlume::theme_assets_loaded(r);
    std::vector<uint8_t> rgb(320u * 240u * 3u, 0);
    overlume::FrameView view{rgb.data(), 320, 240};
    overlume::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;
    const bool ok = overlume::render_frame(r, pose, view);
    overlume::destroy_renderer(r);
    if (!ok) return -1;
    return themes ? 2 : 1;
}
