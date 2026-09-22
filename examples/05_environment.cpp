// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <cstdlib>
#include <vector>

#ifndef OVERLUME_EXAMPLES_ENV_FIXTURE_DIR
#error "OVERLUME_EXAMPLES_ENV_FIXTURE_DIR must be defined by examples/CMakeLists.txt"
#endif

int main(int argc, char** argv) {
    const overlume_examples::ExampleArgs args =
        overlume_examples::parse_args(argc, argv, "05_environment.png");

    overlume::RenderConfig config{};
    config.width = 640;
    config.height = 480;
    config.quality = 1;
    config.theme_assets_dir = args.theme_dir.c_str();
    config.initial_theme = "dark_adas";

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        std::fprintf(stderr, "05_environment: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }

    const overlume::GeoAnchor anchor{25.0803, 55.3910, 0.0};

    const bool opened =
        overlume::set_environment_source(renderer, OVERLUME_EXAMPLES_ENV_FIXTURE_DIR, anchor);
    if (!opened) {
        std::fprintf(stderr,
                     "05_environment: set_environment_source() couldn't open the fixture dir "
                     "'%s' (non-fatal per the API -- rendering continues with no buildings)\n",
                     OVERLUME_EXAMPLES_ENV_FIXTURE_DIR);
    }

    overlume::SceneGraph scene{};
    scene.ego.valid = 1;
    scene.ego.position = {-128.0, -128.0, 0.0};
    overlume::set_scene(renderer, scene);

    const overlume::Vec3 centroid{-109.2, -17.1, 3.0};
    const overlume::CameraPose pose{
        {centroid.x + 50, centroid.y - 70, 40}, {centroid.x, centroid.y, centroid.z}, 60.0};

    std::vector<uint8_t> rgb(static_cast<size_t>(config.width) * config.height * 3);
    overlume::FrameView view{rgb.data(), config.width, config.height};
    if (!overlume::render_frame(renderer, pose, view)) {
        std::fprintf(stderr, "05_environment: render_frame() failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }
    std::printf(
        "environment_source_state() = %u (0=NONE,1=BAKED,2=STREAMING,3=STREAMING_FALLBACK)\n",
        static_cast<unsigned>(overlume::environment_source_state(renderer)));

    const bool wrote =
        overlume_examples::write_png(args.output_path, config.width, config.height, rgb.data());

    if (std::getenv("CESIUM_ION_TOKEN") == nullptr) {
        std::printf("05_environment: CESIUM_ION_TOKEN not set -- streaming demo skipped\n");
    } else {
        const bool streaming_opened =
            overlume::set_environment_source(renderer, "ion://96188", anchor);
        if (!streaming_opened) {
            std::printf(
                "05_environment: streaming set_environment_source(\"ion://96188\", ...) "
                "returned false (network/build config?) -- continuing\n");
        } else {
            for (int i = 0; i < 5; ++i) {
                overlume::render_frame(renderer, pose, view);
            }
            std::printf("05_environment: streamed briefly, environment_source_state() = %u\n",
                        static_cast<unsigned>(overlume::environment_source_state(renderer)));
        }
    }

    overlume::destroy_renderer(renderer);
    return wrote ? 0 : 1;
}
