// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
// filament_link_probe.cpp — Epic 0 Task 1 build-integration proof.
//
// Reviewer finding (Task 1 fix-up): GetFilament.cmake links Filament's ~30
// mutually-cross-referencing static archives (filament <-> backend <-> utils
// <-> filabridge <-> filaflat <-> smol-v, ...) as a flat, alphabetically
// ordered list of file paths. CMake only auto-repeats *targets* for cyclic
// target dependencies, not raw archive paths, so a flat list is one
// unresolved-forward-reference away from an undefined-symbol link failure —
// and nothing in the Task 1 build (a static library with no consumer) ever
// forces the linker to actually resolve those cross-archive symbols.
//
// This probe is that consumer: it links `overlume` (which propagates
// Filament::filament) into a real executable and calls a Filament entry
// point (Engine::Builder().build()) whose implementation is known to reach
// across the filament/backend/utils/filabridge/filaflat archive boundary.
// If the archive list ever regresses to a naive flat link, this fails to
// *link* (undefined symbols) long before it would fail to run. Wired as a
// ctest so `ctest --test-dir <build>` proves the whole build recipe, not
// just header compliance (scripts/check_pod_header.sh).
//
// Real scene/rendering logic is Task 2's job (see api.h); this file only
// proves the link graph and that engine creation doesn't crash headless.
#include <filament/Engine.h>

int main() {
    // NOOP backend: no GPU/EGL/driver dependency, just proves the
    // filament/backend/utils/filabridge/filaflat/smol-v archive graph
    // resolves and that Engine construction/teardown doesn't crash. Task 2's
    // hello-frame example is the real OPENGL/EGL headless-rendering proof.
    filament::Engine* engine =
        filament::Engine::Builder().backend(filament::Engine::Backend::NOOP).build();
    if (engine == nullptr) {
        return 1;
    }
    filament::Engine::destroy(&engine);
    return engine == nullptr ? 0 : 1;
}
