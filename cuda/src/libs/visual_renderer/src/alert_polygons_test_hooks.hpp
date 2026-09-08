// alert_polygons_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_alert_polygons.cpp links only against `visual_renderer` and
// has no access to its PRIVATE Filament include dir, so it can't include
// alert_polygons.hpp/renderer_internal.hpp directly. These hooks are
// declared only against api.h's opaque VisualRenderer, scene.h's POD
// types, and theme.hpp's Filament-free Float3 — defined in
// alert_polygons.cpp, where the state they read actually lives.
#pragma once

#include <cstddef>
#include <cstdint>

#include "theme.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// The color last passed to severity `severity`'s (0 info/1 warning/2
// critical) MaterialInstance "baseColor" param (rgb only — Filament has no
// getter for a set parameter, so this isn't a GPU read-back). {0,0,0} if
// `r` is null or `severity` is out of range.
mpviz::detail::Float3 alert_severity_base_color(mpviz::VisualRenderer* r, uint8_t severity);

// Number of live alert slots (0-indexed into active().alerts) — 0 if `r`
// is null.
size_t alert_slot_count(mpviz::VisualRenderer* r);

// Which material `slot`'s renderable is currently bound to, read via
// RenderableManager::getMaterialInstanceAt() (a real GPU-state getter,
// unlike MaterialInstance's write-only params). `bound_to_severity_template`
// is true while fresh (bound to the shared alertMaterial[severity]) and
// false while stale (bound to a per-slot fadeInstance instead). `alpha`
// mirrors the CPU-stored baseColor.a last applied to this slot: the
// severity's constant alpha while fresh, that constant multiplied by
// staleness_alpha while stale — a fading critical must never brighten past
// its constant.
struct AlertMaterialInfo {
    bool bound_to_severity_template = false;
    float alpha = 0.0f;
};
AlertMaterialInfo alert_slot_material_info(mpviz::VisualRenderer* r, size_t slot);

}  // namespace mpviz::testing
