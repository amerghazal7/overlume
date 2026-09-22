// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <filament/Engine.h>

int main() {
    filament::Engine* engine =
        filament::Engine::Builder().backend(filament::Engine::Backend::NOOP).build();
    if (engine == nullptr) {
        return 1;
    }
    filament::Engine::destroy(&engine);
    return engine == nullptr ? 0 : 1;
}
