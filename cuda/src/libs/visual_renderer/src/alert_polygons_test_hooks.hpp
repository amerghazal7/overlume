// alert_polygons_test_hooks.hpp — internal-only, `-I src` visibility, not
// installed, not POD (Epic 2 Task 7 / VM-026). Same reasoning as
// ribbon_test_hooks.hpp/map_elements_test_hooks.hpp: tests/
// test_alert_polygons.cpp links only against `visual_renderer` and is
// never granted that target's PRIVATE Filament include dir, so it can't
// include alert_polygons.hpp/renderer_internal.hpp directly. These hooks
// are declared against nothing but api.h's opaque mpviz::VisualRenderer,
// scene.h's POD types, and theme.hpp's Filament-free mpviz::detail::Float3
// — defined in alert_polygons.cpp, where the state they read actually
// lives.
#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// The color value last actually passed to severity `severity`'s (0 info/1
// warning/2 critical) MaterialInstance's "baseColor" setParameter call
// (rgb only — NOT a read-back of live GPU state, Filament's
// MaterialInstance exposes no getter for a set parameter), mirroring
// ribbon_role_base_color()'s/object_class_tint()'s own pattern. {0,0,0} if
// `r` is null or `severity` is out of range.
mpviz::detail::Float3 alert_severity_base_color(mpviz::VisualRenderer* r, uint8_t severity);

// Number of live alert slots (0-indexed into active().alerts, the plan's
// "diff by polygon index" decision) — 0 if `r` is null.
size_t alert_slot_count(mpviz::VisualRenderer* r);

// Which material slot `slot`'s renderable is CURRENTLY bound to, read via
// RenderableManager::getMaterialInstanceAt() (a real GPU-state getter,
// unlike MaterialInstance's own parameters, which are write-only) — same
// shape as ribbon_test_hooks.hpp's RibbonMaterialInfo, but UNLIKE ribbons/
// objects there is no opaque state to distinguish: every alert polygon
// lives on clay_translucent.mat from the instant it exists, so
// `bound_to_severity_template` is true while FRESH (bound directly to the
// shared alertMaterial[severity] template) and false while STALE (bound to
// a per-slot fadeInstance instead). `alpha` mirrors the CPU-stored
// baseColor.a last actually applied to this slot (MaterialInstance has no
// parameter getter) — the severity's constant alpha while fresh, that
// constant MULTIPLIED by staleness_alpha while stale (Alerts.
// StaleAlertFadesViaSharedStalenessAlpha: a fading critical must not
// brighten past its constant).
struct AlertMaterialInfo {
    bool bound_to_severity_template = false;
    float alpha = 0.0f;
};
AlertMaterialInfo alert_slot_material_info(mpviz::VisualRenderer* r, size_t slot);

}  // namespace mpviz::testing
