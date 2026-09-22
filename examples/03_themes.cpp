// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <vector>

namespace {

bool render_at(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
               const overlume::RenderConfig& config, double sim_time_sec,
               std::vector<uint8_t>& rgb) {
    overlume::SceneGraph scene{};
    scene.sim_time_sec = sim_time_sec;
    scene.ego.valid = 1;
    overlume::set_scene(r, scene);
    overlume::FrameView view{rgb.data(), config.width, config.height};
    return overlume::render_frame(r, pose, view);
}

}  // namespace

int main(int argc, char** argv) {
    const overlume_examples::ExampleArgs args =
        overlume_examples::parse_args(argc, argv, "03_themes.png");

    for (const char* name : {"dark_adas", "light_clay"}) {
        const bool parses = overlume::theme_parses(args.theme_dir.c_str(), name);
        std::printf("theme_parses(%s, \"%s\") = %s\n", args.theme_dir.c_str(), name,
                    parses ? "true" : "false");
    }

    overlume::RenderConfig config{};
    config.width = 640;
    config.height = 480;
    config.quality = 1;
    config.theme_assets_dir = args.theme_dir.c_str();
    config.initial_theme = "dark_adas";

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        std::fprintf(stderr, "03_themes: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }

    const overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> rgb(static_cast<size_t>(config.width) * config.height * 3);

    if (!render_at(renderer, pose, config, 0.0, rgb)) {
        std::fprintf(stderr, "03_themes: render_frame() at t=0 failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }

    if (!overlume::set_theme(renderer, "light_clay", 0.0, 0.8)) {
        std::fprintf(stderr, "03_themes: set_theme() returned false unexpectedly\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }

    if (!render_at(renderer, pose, config, 0.4, rgb)) {
        std::fprintf(stderr, "03_themes: render_frame() mid-transition failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }
    const bool wroteMid =
        overlume_examples::write_png(args.output_path, config.width, config.height, rgb.data());

    if (!render_at(renderer, pose, config, 0.8, rgb)) {
        std::fprintf(stderr, "03_themes: render_frame() at t=0.8 failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }
    std::printf(
        "rendered t=0.0 (dark_adas), t=0.4 (mid-blend, written above), "
        "t=0.8 (fully light_clay)\n");

    overlume::destroy_renderer(renderer);
    return wroteMid ? 0 : 1;
}
