// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// 01_hello_frame.cpp — the smallest possible overlume program: create a
// headless renderer, load a theme, render one frame, write it out.
//
// Public API used: overlume::RenderConfig, overlume::create_renderer(),
// overlume::CameraPose, overlume::FrameView, overlume::render_frame(),
// overlume::destroy_renderer() (<overlume/api.h>), plus
// overlume::theme_assets_loaded() (<overlume/scene.h> — an additive,
// scene-graph-adjacent entry point, not a SceneGraph field itself). This
// program never calls set_scene() at all: rendering with no published
// scene is a legal, fully-defined state (just ground + sky + whatever
// ego/theme defaults apply).
#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <vector>

int main(int argc, char** argv) {
    const overlume_examples::ExampleArgs args =
        overlume_examples::parse_args(argc, argv, "01_hello_frame.png");

    // RenderConfig is POD: width/height of the render target, a quality
    // preset (0=low, 1=med, 2=high — api.h's own comment), and two
    // borrowed-only-for-this-call C strings naming the theme dir + initial
    // theme. create_renderer() copies both into its own storage before
    // returning, so the RenderConfig itself doesn't need to outlive the
    // call (see api.h's RenderConfig comment for the exact lifetime rule).
    overlume::RenderConfig config{};
    config.width = 640;
    config.height = 480;
    config.quality = 1;  // medium
    config.theme_assets_dir = args.theme_dir.c_str();
    config.initial_theme = "dark_adas";  // one of the two shipped themes

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        // create_renderer() returns nullptr on failure (no GPU/EGL device,
        // most commonly) — never a thrown exception or an abort. This is
        // the one legitimate "nothing to render" exit for this example.
        std::fprintf(stderr, "01_hello_frame: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }

    // create_renderer() silently falls back to a compiled-in theme if the
    // requested one failed to load from disk (non-fatal by design) —
    // theme_assets_loaded() tells the caller which happened, purely so a
    // real integration can WARN once; it changes nothing about rendering.
    if (!overlume::theme_assets_loaded(renderer)) {
        std::fprintf(stderr,
                     "01_hello_frame: theme dir '%s' did not load — using the "
                     "compiled-in fallback theme instead\n",
                     args.theme_dir.c_str());
    }

    // CameraPose is eye/target/vfov_deg — a plain look-at camera, nothing
    // more. This is the same pose the library's own hello-frame test uses
    // (see overlume/tests/test_hello_frame.cpp): pitched down enough that
    // both sky and ground land in frame.
    overlume::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;

    // FrameView wraps a CALLER-OWNED rgb buffer — overlume writes into it,
    // it never allocates or frees it. w*h*3 bytes, RGB8, row-major,
    // top-to-bottom (see api.h's FrameView + scene.h's project_to_screen()
    // comment, which states the row order explicitly).
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
