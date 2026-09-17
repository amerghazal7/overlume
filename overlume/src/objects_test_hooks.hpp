// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// objects_test_hooks.hpp — internal-only, not installed, not POD.
// tests/test_objects.cpp links only against `overlume` and has no
// access to its PRIVATE Filament include dir, so it can't include
// objects.hpp/renderer_internal.hpp directly. These hooks are declared
// only against api.h's opaque VisualRenderer and scene.h's POD Vec3 —
// defined in objects.cpp, where the state they read actually lives.
#pragma once

#include "theme.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// The color last passed to objectClassMaterial[cls]'s "baseColor" param
// (mirrored CPU copy — Filament's MaterialInstance has no getter). {0,0,0}
// if `r` is null.
overlume::detail::Float3 object_class_tint(overlume::VisualRenderer* r, overlume::ObjectClass cls);

// True iff object `id` currently has a live renderable entity (gltfio
// instance root, or procedural clay box) that is a member of the live
// filament::Scene. False if `id` was never acquired, or has already been
// released/recycled (its track vanished from a later set_scene()).
bool object_in_scene(overlume::VisualRenderer* r, uint32_t id);

// Opaque numeric identity of the FilamentInstance/entity currently backing
// object `id` (0 if none) — used to prove the free-list reuse mechanism
// (the same identity reappearing for a different id after the original
// object's track vanished), never to assert destruction: gltfio has no
// destroyInstance(), so recycling is the only shape available.
uint64_t object_entity_identity(overlume::VisualRenderer* r, uint32_t id);

// The applied TransformManager scale for object `id`: dims / the class
// model's own normalized unit footprint (or dims / (1,1,1) for the
// procedural-box fallback, always a unit cube — see renderer_internal.hpp's
// ObjectEntity comment). Never RenderableManager::getAxisAlignedBoundingBox(),
// which add_mesh() hard-codes to the same box for every renderable
// regardless of actual geometry (same reasoning as egoFallbackDims).
// {0,0,0} if `id` has no live entity.
overlume::Vec3 object_transform_scale(overlume::VisualRenderer* r, uint32_t id);

// Which material object `id`'s renderable is currently bound to, read via
// RenderableManager::getMaterialInstanceAt() — a real GPU-state getter
// (unlike MaterialInstance's write-only params) — so a bug that skips the
// setMaterialInstanceAt() swap shows up here as "still on the opaque
// template", not as a CPU flag agreeing with itself. `alpha` mirrors the
// CPU-stored fade alpha and is only meaningful when bound_to_translucent.
struct ObjectMaterialInfo {
    bool bound_to_translucent = false;
    float alpha = 1.0f;
};
ObjectMaterialInfo object_material_info(overlume::VisualRenderer* r, uint32_t id);

}  // namespace overlume::testing
