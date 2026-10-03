// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
#pragma once

#include <filament/Engine.h>

namespace overlume::detail {

struct HeadlessPlatform {
    filament::backend::Platform* platform = nullptr;  // owned; nullptr = Filament default
    filament::Engine::Backend backend = filament::Engine::Backend::OPENGL;
    // false = no usable device (Metal without an MTLDevice): create_renderer returns nullptr
    // without ever calling Engine::Builder::build(), which would abort.
    bool usable = true;
};

// Compile-time-selected headless back end. Never aborts; on failure
// Engine::Builder::build() returns nullptr exactly as today.
HeadlessPlatform make_headless_platform();
// Call after Engine::destroy.
void destroy_headless_platform(HeadlessPlatform&);

}  // namespace overlume::detail
