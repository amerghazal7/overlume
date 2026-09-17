// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// ribbon.hpp — internal-only (`-I src`), not installed, not POD. Declares
// the per-frame counterpart to the three ribbon-role MaterialInstances
// renderer.cpp creates/themes; render_frame() calls this every tick.
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see ribbon_test_hooks.hpp (Filament-free) for
// what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

// Diffs `scene.paths`/`path_count` against `r.ribbonSlots`, keyed by slot
// index, not role — see renderer_internal.hpp's RibbonSlot comment for why.
// Each slot's own content signature (role + point data) decides whether it
// rebuilds this frame; a role change re-homes it to the right material.
// Slots >= path_count are released. Called from render_frame() on the
// thread that owns the Engine.
void update_ribbons(VisualRenderer& r, const SceneGraph& scene);

}  // namespace overlume
