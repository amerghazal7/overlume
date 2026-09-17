// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// 05_environment.cpp — loads a baked environment chunk index from the
// committed fixture dir the library's own environment tests use, then,
// ONLY if CESIUM_ION_TOKEN is set in the environment, ALSO tries the
// streaming (ion://) backend briefly. Public headers only: <overlume/api.h>,
// <overlume/scene.h>.
//
// set_environment_source()'s ONE signature covers both backends (scene.h):
// no "ion://" prefix -> the baked backend (a local chunk-index directory);
// an "ion://<assetId>..." source_uri -> cesium-native streaming, built only
// when this library was compiled with OVERLUME_ENABLE_CESIUM=ON. Either way
// a failure to open degrades to "no buildings," never a crash — this
// example relies on exactly that non-fatal contract for the no-token path.
#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <cstdlib>
#include <vector>

// OVERLUME_EXAMPLES_ENV_FIXTURE_DIR: baked in by examples/CMakeLists.txt,
// points at overlume/tests/fixtures/environment_test_town_0 — the same
// committed fixture overlume/tests/test_environment.cpp loads (2 real baked
// chunks, produced by bake_environment.py; see that test file's own
// provenance comment).
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

    // Same WGS84 anchor the fixture was baked against (test_environment.cpp);
    // the baked backend uses this only for bookkeeping (its chunks are
    // already placed in map-frame offline), the streaming backend below
    // uses it for real on-the-fly ECEF->map placement.
    const overlume::GeoAnchor anchor{25.0803, 55.3910, 0.0};

    const bool opened =
        overlume::set_environment_source(renderer, OVERLUME_EXAMPLES_ENV_FIXTURE_DIR, anchor);
    if (!opened) {
        std::fprintf(stderr,
                     "05_environment: set_environment_source() couldn't open the fixture dir "
                     "'%s' (non-fatal per the API -- rendering continues with no buildings)\n",
                     OVERLUME_EXAMPLES_ENV_FIXTURE_DIR);
    }

    // The fixture's chunk_-1_-1 cell center (index.yaml) -- placing the ego
    // here is what makes the baked backend load that chunk at all (it's
    // distance-culled from the ego position every set_scene() tick).
    overlume::SceneGraph scene{};
    scene.ego.valid = 1;
    scene.ego.position = {-128.0, -128.0, 0.0};
    overlume::set_scene(renderer, scene);

    // Framed on the fixture's real footprint centroid (measured directly
    // from the .glb, same value test_environment.cpp uses), not the chunk
    // cell's nominal center.
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

    // Streaming is opt-in and network-dependent -- checked by NAME only
    // (never printed), per this repo's token-handling rule. A CI run has no
    // token, so this branch is a clean, honest skip there; a developer box
    // with CESIUM_ION_TOKEN exported gets a brief live pump against the OSM
    // Buildings preset (asset 96188 -- same id the library's own streaming
    // tests use).
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
            // Pump a handful of frames so tiles have a few ticks to arrive
            // asynchronously; this is a brief demo, not a throughput test.
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
