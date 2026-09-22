// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <vector>

int main(int argc, char** argv) {
    const overlume_examples::ExampleArgs args =
        overlume_examples::parse_args(argc, argv, "01_hello_frame.png");

    overlume::RenderConfig config{};
    config.width = 640;
    config.height = 480;
    config.quality = 1;
    config.theme_assets_dir = args.theme_dir.c_str();
    config.initial_theme = "dark_adas";

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        std::fprintf(stderr, "01_hello_frame: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }

    if (!overlume::theme_assets_loaded(renderer)) {
        std::fprintf(stderr,
                     "01_hello_frame: theme dir '%s' did not load — using the "
                     "compiled-in fallback theme instead\n",
                     args.theme_dir.c_str());
    }

    overlume::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;

    std::vector<uint8_t> rgb(static_cast<size_t>(config.width) * config.height * 3);
    overlume::FrameView view{rgb.data(), config.width, config.height};

    if (!overlume::render_frame(renderer, pose, view)) {
        std::fprintf(stderr, "01_hello_frame: render_frame() failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }

    const bool wrote =
        overlume_examples::write_png(args.output_path, config.width, config.height, rgb.data());
    overlume::destroy_renderer(renderer);
    return wrote ? 0 : 1;
}
