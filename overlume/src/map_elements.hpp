// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// map_elements.hpp — internal-only (`-I src`), not installed, not POD.
// Declares the per-frame update for the lane MaterialInstances
// renderer.cpp creates/themes; render_frame() calls this every tick.
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see map_elements_test_hooks.hpp (Filament-free)
// for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

// Diffs `scene.map_elements`/`map_element_count` against
// `r.mapElementMeshes` (keyed by content signature, not array position —
// see renderer_internal.hpp) and rebuilds only what changed: new
// signatures get a Mesh built (polyline -> extruded ribbon; polygon ->
// triangulated fan + hatch), matching signatures are kept untouched, and
// signatures no longer present are destroyed. Called from render_frame()
// on the thread that owns the Engine.
void update_map_elements(VisualRenderer& r, const SceneGraph& scene);

}  // namespace overlume
