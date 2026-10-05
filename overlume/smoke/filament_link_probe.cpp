// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <filament/Engine.h>

int main() {
#if defined(__APPLE__)
    // Apple builds carry Metal-only materials, which the Noop backend rejects at engine creation
    // (CI 37331041427, macOS x86_64); referencing an Engine symbol is enough to prove the link.
    return filament::Engine::getSteadyClockTimeNano() > 0 ? 0 : 1;
#else
    filament::Engine* engine =
        filament::Engine::Builder().backend(filament::Engine::Backend::NOOP).build();
    if (engine == nullptr) {
        return 1;
    }
    filament::Engine::destroy(&engine);
    return engine == nullptr ? 0 : 1;
#endif
}
