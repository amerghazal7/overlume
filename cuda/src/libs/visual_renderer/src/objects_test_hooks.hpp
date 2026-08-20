// objects_test_hooks.hpp — internal-only, `-I src` visibility, not
// installed, not POD (Epic 2 Task 4 / VM-022). Same reasoning as
// map_elements_test_hooks.hpp / ego_test_hooks.hpp: tests/test_objects.cpp
// links only against `visual_renderer` and is never granted that target's
// PRIVATE Filament include dir, so it can't include objects.hpp/
// renderer_internal.hpp directly. These hooks are declared against nothing
// but api.h's opaque mpviz::VisualRenderer and scene.h's POD Vec3 —
// defined in objects.cpp, where the state they read actually lives.
#pragma once

#include "theme.hpp"
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

namespace mpviz::testing {

// The color value last actually passed to objectClassMaterial[cls]'s
// "baseColor" setParameter call (Filament's MaterialInstance has no
// getter) — same mirrored-CPU-copy pattern as map_elements_test_hooks.hpp's
// lane_material_base_color() / ego_test_hooks.hpp's ego_material_base_color().
// {0,0,0} if `r` is null.
mpviz::detail::Float3 object_class_tint(mpviz::VisualRenderer* r, mpviz::ObjectClass cls);

// True iff object `id` currently has a live renderable entity (gltfio
// instance root, or procedural clay box) that is a member of the live
// filament::Scene. False if `id` was never acquired, or has already been
// released/recycled (its track vanished from a later set_scene()).
bool object_in_scene(mpviz::VisualRenderer* r, uint32_t id);

// Opaque numeric identity of the FilamentInstance/entity currently backing
// object `id` (0 if none) — used to prove the free-list REUSE mechanism (the
// SAME identity reappearing for a DIFFERENT id after the original object's
// track vanished and a new one was acquired), never to assert destruction:
// gltfio has no destroyInstance(), so recycling — not destruction — is the
// only shape available (see Objects.ObjectDisappearingIsRemovedFromTheScene
// AndRecycled, deliberately not named "...DestroysItsEntity").
uint64_t object_entity_identity(mpviz::VisualRenderer* r, uint32_t id);

// The APPLIED TransformManager scale for object `id`: dims / the class
// model's own normalized unit footprint (or dims / (1,1,1) for the
// procedural-box fallback, which is always built as a unit cube — see
// renderer_internal.hpp's ObjectEntity comment). NEVER
// RenderableManager::getAxisAlignedBoundingBox(), which add_mesh() hard-
// codes to the SAME 40x40x2 box for every renderable regardless of its
// actual geometry (Epic 1's documented trap; renderer_internal.hpp's
// egoFallbackDims comment is the precedent this hook follows instead of
// re-discovering it). {0,0,0} if `id` has no live entity.
mpviz::Vec3 object_transform_scale(mpviz::VisualRenderer* r, uint32_t id);

// Which material object `id`'s renderable is CURRENTLY bound to, read via
// RenderableManager::getMaterialInstanceAt() — a real GPU-state getter
// (unlike MaterialInstance's own parameters, which are write-only) — so a
// bug that skips the setMaterialInstanceAt() swap shows up here as "still
// on the opaque template", not as a CPU bookkeeping flag agreeing with
// itself. `alpha` mirrors the CPU-stored fade alpha (MaterialInstance has
// no parameter getter) and is only meaningful when bound_to_translucent.
struct ObjectMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
ObjectMaterialInfo object_material_info(mpviz::VisualRenderer* r, uint32_t id);

}  // namespace mpviz::testing
