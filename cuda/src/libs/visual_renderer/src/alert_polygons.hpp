// alert_polygons.hpp — internal-only (`-I src`), not installed, not POD.
// Declares the per-frame update for the three per-severity MaterialInstances
// renderer.cpp creates/themes; render_frame() calls this every tick.
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see alert_polygons_test_hooks.hpp (Filament-free)
// for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.alerts`/`alert_count` against `r.alertSlots`, keyed by slot
// index (AlertPolygon has no id; same reasoning as ribbon.cpp's RibbonSlot
// keying). Each slot's own signature (severity + point data) decides
// whether it rebuilds this frame; slots >= alert_count are released.
// Called from render_frame() on the thread that owns the Engine.
void update_alert_polygons(VisualRenderer& r, const SceneGraph& scene);

}  // namespace mpviz
