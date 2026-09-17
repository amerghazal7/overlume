// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// generic_markers.hpp — internal-only (`-I src`), not installed, not POD.
// Declares the per-frame update for the eager theme-neutral
// MaterialInstance renderer.cpp creates/themes; render_frame() calls this
// every tick.
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see generic_markers_test_hooks.hpp
// (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

// Diffs `scene.markers`/`marker_count` against `r.genericMarkerSlots`,
// keyed by slot index (GenericMarker has no id, same reasoning as
// RibbonSlot/AlertSlot). Each slot's own primitive + content signature
// decides whether its geometry rebuilds this frame; a primitive change
// tears down and re-acquires the slot fresh. Slots >= marker_count are
// released. Called from render_frame() on the thread that owns the
// Engine, and must run after update_alert_polygons() -- generic markers
// are the last thing drawn, a debug/parity layer not meant to hide under
// anything.
void update_generic_markers(VisualRenderer& r, const SceneGraph& scene);

}  // namespace overlume
