// objects.hpp — internal-only (`-I src`), not installed, not POD. Declares
// the per-frame counterpart to set_object_model_dir() (scene.h/objects.cpp):
// set_object_model_dir() loads models once, update_objects() acquires/
// updates/releases entities every render_frame() tick.
//
// Pulls in renderer_internal.hpp (and <filament/...>), so this is never
// included by tests/*.cpp — see objects_test_hooks.hpp (Filament-free) for
// what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.objects`/`object_count` against `r.objectEntities` (keyed by
// TrackedObject::id) and acquires/updates/releases only what changed.
// Called from render_frame() on the thread that owns the Engine.
void update_objects(VisualRenderer& r, const SceneGraph& scene);

// Tears down one ObjectEntity: recycles a gltfio instance to its class's
// free list (removed from the scene, not destroyed — there is no
// destroyInstance() in gltfio) or destroys a procedural box outright,
// destroys the arrow entity / path-ribbon mesh / any live staleness-fade
// MaterialInstance. Shared by update_objects()'s per-frame "track vanished"
// path and destroy_renderer()'s final teardown loop.
void release_object_entity(VisualRenderer& r, ObjectEntity& entity);

}  // namespace mpviz
