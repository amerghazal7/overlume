// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <overlume/scene.h>

#include <cstdint>
#include <cstdio>
#include <string>

#ifndef OVERLUME_EXAMPLES_THEME_DIR
#error "OVERLUME_EXAMPLES_THEME_DIR must be defined by examples/CMakeLists.txt"
#endif

#ifndef OVERLUME_EXAMPLES_MODEL_DIR
#error "OVERLUME_EXAMPLES_MODEL_DIR must be defined by examples/CMakeLists.txt"
#endif

namespace overlume_examples {

struct ExampleArgs {
    std::string output_path;
    std::string theme_dir = OVERLUME_EXAMPLES_THEME_DIR;
    std::string model_dir = OVERLUME_EXAMPLES_MODEL_DIR;
};

inline ExampleArgs parse_args(int argc, char** argv, const char* default_output) {
    ExampleArgs args;
    args.output_path = default_output;
    if (argc > 1) args.output_path = argv[1];
    if (argc > 2) args.theme_dir = argv[2];
    if (argc > 3) args.model_dir = argv[3];
    return args;
}

inline void apply_model_dir(overlume::VisualRenderer* renderer, const ExampleArgs& args) {
    const uint32_t loaded = overlume::set_object_model_dir(renderer, args.model_dir.c_str());
    std::printf("set_object_model_dir(\"%s\"): %u class model(s) loaded\n", args.model_dir.c_str(),
                loaded);
}

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
