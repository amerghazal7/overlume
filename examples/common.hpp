// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// examples/common.hpp — tiny shared helpers for the example programs below.
// NOT part of overlume's public API: this is example-only infrastructure,
// the same role tests/golden.hpp plays for the test suite. Every overlume::
// call the examples themselves make still goes through <overlume/api.h> /
// <overlume/scene.h> only — nothing here reaches into overlume/src.
//
// STB_IMAGE_WRITE_IMPLEMENTATION lives in this header only because every
// example below is a single translation unit that includes it once; a
// consumer with more than one .cpp including common.hpp must move this
// #define + #include into its own dedicated .cpp to avoid duplicate symbols.
#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cstdint>
#include <cstdio>
#include <string>

// OVERLUME_EXAMPLES_THEME_DIR: baked in by examples/CMakeLists.txt, points at
// the shipped overlume/assets/themes dir — same OVERLUME_..._DIR pattern the
// test suite uses for OVERLUME_DEFAULT_THEME_DIR (see
// overlume/tests/test_paths.hpp). Every example below falls back to this
// when argv[2] (a theme dir override) isn't given.
#ifndef OVERLUME_EXAMPLES_THEME_DIR
#error "OVERLUME_EXAMPLES_THEME_DIR must be defined by examples/CMakeLists.txt"
#endif

namespace overlume_examples {

// argv[1] (optional): output image path, defaulting to `default_output`.
// argv[2] (optional): theme assets dir, defaulting to the compiled-in
// shipped theme dir above.
struct ExampleArgs {
    std::string output_path;
    std::string theme_dir = OVERLUME_EXAMPLES_THEME_DIR;
};

inline ExampleArgs parse_args(int argc, char** argv, const char* default_output) {
    ExampleArgs args;
    args.output_path = default_output;
    if (argc > 1) args.output_path = argv[1];
    if (argc > 2) args.theme_dir = argv[2];
    return args;
}

// Writes an RGB8 buffer (row-major, top-to-bottom, w*h*3 bytes — exactly the
// layout overlume::FrameView already requires) out as a PNG. stb_image_write.h
// is a single-header, public-domain, dependency-free encoder already vendored
// by overlume/CMakeLists.txt for the test suite's own golden captures (every
// tests/*.cpp AND examples/*.cpp binary already gets its include dir) — using
// it here is not a new third-party dependency.
inline bool write_png(const std::string& path, uint32_t width, uint32_t height,
                      const uint8_t* rgb) {
    const int ok = stbi_write_png(path.c_str(), static_cast<int>(width), static_cast<int>(height),
                                  3, rgb, static_cast<int>(width) * 3);
    if (ok) {
        std::printf("wrote %s (%ux%u)\n", path.c_str(), width, height);
    } else {
        std::fprintf(stderr, "failed to write %s\n", path.c_str());
    }
    return ok != 0;
}

}  // namespace overlume_examples
