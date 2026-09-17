// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// scene_buffer.hpp — internal, `-I src` visibility, not installed, not POD.
// Owns the double-buffered deep-copy staging for overlume::SceneGraph. std::
// usage is fine here — it's a `.hpp` under `src/`, never shipped across the
// POD boundary.
#pragma once
#include <mutex>
#include <string>
#include <vector>
#include "overlume/scene.h"

namespace overlume::detail {

// Owns std::vector storage for every array SceneGraph points into, so a
// overlume::SceneGraph handed out by active() has valid pointers for as long as
// this object isn't republished. NOT itself passed across the POD boundary —
// internal only.
//
// Every category struct in scene.h has, in addition to its own flat array,
// pointers into further caller-owned buffers (TrackedObject::predicted_path/
// label, PathRibbon::points, MapElement::points, GroundGridLayer::cells,
// AlertPolygon::points, GenericMarker::points/text/mesh_path,
// AlertChip::text). A flat std::vector<TrackedObject> copy alone still
// leaves those inner pointers aimed at the CALLER's memory — exactly the use-
// after-free the frozen set_scene contract ("scene's arrays may be freed/
// reused the instant this call returns") promises can't happen. So each
// category gets one parallel "nested storage" vector alongside its flat
// vector, and assign() repoints every entry's pointer fields at its own copy
// after copying.
struct OwnedScene {
    overlume::SceneGraph view{};  // pointers below point into this object's own vectors
    std::vector<TrackedObject> objects;
    std::vector<std::vector<Vec3>> object_paths;  // objects[i].predicted_path storage
    std::vector<std::string> object_labels;       // objects[i].label storage
    std::vector<PathRibbon> paths;
    std::vector<std::vector<Vec3>> path_points;  // paths[i].points storage
    std::vector<MapElement> map_elements;
    std::vector<std::vector<Vec3>> map_element_points;
    std::vector<GroundGridLayer> grids;
    std::vector<std::vector<uint8_t>> grid_cells;  // grids[i].cells storage
    std::vector<AlertPolygon> alerts;
    std::vector<std::vector<Vec3>> alert_points;
    std::vector<GenericMarker> markers;
    std::vector<std::vector<Vec3>> marker_points;
    std::vector<std::string> marker_texts;       // "" stored for a nullptr text
    std::vector<std::string> marker_mesh_paths;  // "" stored for a nullptr mesh_path
    std::vector<AlertChip> chips;
    std::vector<std::string> chip_texts;
    std::vector<PointCloud> point_clouds;
    std::vector<std::vector<PointCloudPoint>> point_cloud_points;  // point_clouds[i].points storage
    std::vector<TrajectoryCarpet> trajectory_carpets;
    // trajectory_carpets[i].points storage
    std::vector<std::vector<PointCloudPoint>> trajectory_carpet_points;
    // Deep-copies `src` — including every nested Vec3[]/uint8_t[]/char*
    // payload reached by the arrays above — into this object's vectors, and
    // repoints view's pointers (both the top-level array pointers AND each
    // entry's own nested pointer fields) at the copies.
    void assign(const overlume::SceneGraph& src);
};

class SceneBuffer {
public:
    void publish(const overlume::SceneGraph& scene);  // deep-copy + atomic swap
    const overlume::SceneGraph& active() const;       // last-published scene
    // Fade-out multiplier in [0,1] for an entity last touched `last_update_sec`
    // ago relative to `now_sec`: 1.0 while younger than fade_start_sec, ramps
    // to 0.0 by timeout_sec, 0.0 beyond. One function, every stale-able
    // category (TrackedObject, PathRibbon, GroundGridLayer, AlertPolygon,
    // GenericMarker) calls it the same way — no per-category branches.
    static float staleness_alpha(double now_sec, double last_update_sec, double fade_start_sec,
                                 double timeout_sec);

private:
    mutable std::mutex mutex_;  // ponytail: cheap at this call rate (<=30 Hz);
                                // serializes active_idx_ only. Doesn't by
                                // itself make multi-threaded ingest safe —
                                // active() still hands back a bare
                                // reference aliased into slot storage, so
                                // a second publisher could overwrite a
                                // slot a reader still holds. Real
                                // multi-threaded ingest needs active() to
                                // return an owned/refcounted snapshot; see
                                // set_scene()'s threading contract in
                                // scene.h.
    OwnedScene slots_[2];
    int active_idx_{0};
};

}  // namespace overlume::detail
