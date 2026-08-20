// objects.hpp — internal-only, `-I src` visibility, not installed, not POD
// (Epic 2 Task 4 / VM-022). Declares the per-frame counterpart to
// set_object_model_dir() (mpviz::set_object_model_dir, declared publicly in
// scene.h, defined in objects.cpp): render_frame() calls update_objects()
// every tick with the last-published active() scene, the same
// create-once-in-set_object_model_dir() / update-every-frame-in-
// update_objects() split Task 2 (map_elements) and Task 4's own
// set_ego_model()/update_ego_transform() already established.
//
// Pulls in renderer_internal.hpp (and therefore <filament/...>), so — like
// that header — this is never included by tests/*.cpp; see
// objects_test_hooks.hpp (Filament-free) for what tests use instead.
#pragma once

#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

namespace mpviz {

// Diffs `scene.objects`/`object_count` against `r.objectEntities` (keyed by
// TrackedObject::id) and acquires/updates/releases only what changed —
// mirroring update_map_elements()'s existing diff-cache shape. Called from
// render_frame() on the one thread that owns the Engine.
void update_objects(VisualRenderer& r, const SceneGraph& scene);

// Tears down one ObjectEntity: recycles a gltfio instance to its class's
// free list (removed from the scene, NOT destroyed — there is no
// destroyInstance() in gltfio) or destroys a procedural box outright,
// destroys the arrow entity / path-ribbon mesh / any live staleness-fade
// MaterialInstance. Shared by update_objects()'s per-frame "track vanished"
// path and destroy_renderer()'s final teardown loop.
void release_object_entity(VisualRenderer& r, ObjectEntity& entity);

}  // namespace mpviz
