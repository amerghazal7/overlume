// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// 02_scene_population.cpp — populates the core driving-scene categories the
// scene graph carries (ego, one tracked object per ObjectClass, one map
// element per MapKind including the road surface + a crosswalk, and one
// behavior path ribbon), then renders it. See scene.h for the full field
// docs; ground grids, generic markers, HUD, and trajectory carpets are not
// built here (point clouds/alerts: 06_overlays_and_pointcloud.cpp). Public
// headers only: <overlume/api.h>, <overlume/scene.h>.
//
// THE FREEZE-FRAME / DOUBLE-BUFFER CONTRACT, demonstrated + enforced below:
// set_scene() deep-copies `scene` into the renderer's own staging buffer and
// atomically swaps which slot is "active" — it does NOT render, and every
// array below may be freed the instant it returns. render_frame() always
// re-derives the picture from whatever was last PUBLISHED, so a
// render_frame() with no intervening set_scene() simply re-renders the same
// scene — one set_scene(), two render_frame()s, failing exit if they differ.
#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <iterator>
#include <cstdlib>
#include <vector>

namespace {

// Counts bytes that differ by more than `tolerance` between two equal-sized
// RGB8 buffers. GPU rendering has a little dithering/AO-sampling noise frame
// to frame even with a fully static scene/camera (measured here: 1.74% of
// bytes, 16002/921600) -- an exact byte-for-byte compare would be the wrong
// tool, so this reuses the tolerance-then-threshold shape of
// overlume/tests/test_environment.cpp's count_differing_bytes().
size_t count_differing_bytes(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b,
                             int tolerance = 1) {
    size_t n = 0;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
        if (std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i])) > tolerance) ++n;
    }
    return n;
}

}  // namespace

int main(int argc, char** argv) {
    const overlume_examples::ExampleArgs args =
        overlume_examples::parse_args(argc, argv, "02_scene_population.png");

    overlume::RenderConfig config{};
    config.width = 640;
    config.height = 480;
    config.quality = 1;
    config.theme_assets_dir = args.theme_dir.c_str();
    config.initial_theme = "dark_adas";

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        std::fprintf(stderr, "02_scene_population: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }
    overlume_examples::apply_model_dir(renderer, args);

    // ── Ego ──────────────────────────────────────────────────────────────
    overlume::EgoState ego{};
    ego.position = {0.0, 0.0, 0.0};
    ego.heading_rad = 0.0;
    ego.speed_mps = 8.0;
    ego.valid = 1;  // 0 would hide the ego entirely (no TF yet)

    // ── One TrackedObject per ObjectClass, fanned out along +X ──────────
    std::vector<overlume::TrackedObject> objects;
    const overlume::ObjectClass classes[] = {
        overlume::ObjectClass::CAR,     overlume::ObjectClass::TRUCK_VAN,
        overlume::ObjectClass::BUS,     overlume::ObjectClass::PEDESTRIAN,
        overlume::ObjectClass::CYCLIST, overlume::ObjectClass::UNKNOWN,
    };
    for (uint32_t i = 0; i < std::size(classes); ++i) {
        overlume::TrackedObject obj{};
        obj.id = i + 1;
        obj.cls = classes[i];
        obj.position = {10.0 + static_cast<double>(i) * 6.0, 6.0, 0.0};
        obj.heading_rad = 0.0;
        obj.dimensions = {4.5, 1.9,
                          1.6};  // a plausible car-sized box; per-class fidelity is cosmetic
        obj.velocity = {5.0, 0.0, 0.0};
        obj.predicted_path = nullptr;
        obj.predicted_path_count = 0;
        obj.label = nullptr;
        obj.last_update_sec = 0.0;
        objects.push_back(obj);
    }

    // ── One MapElement per MapKind (scene.h enumerates 9); CROSSWALK and
    //    ROAD_SURFACE are polygons (is_polygon=1), the rest polylines.
    //    Illustrative points, not a real HD-map extraction (see
    //    ros/src/overlume_ros for a real adapter) ─────────────────────────
    std::vector<overlume::Vec3> centerline_pts = {{0, -2, 0}, {40, -2, 0}};
    std::vector<overlume::Vec3> left_boundary_pts = {{0, -3.5, 0}, {40, -3.5, 0}};
    std::vector<overlume::Vec3> right_boundary_pts = {{0, -0.5, 0}, {40, -0.5, 0}};
    std::vector<overlume::Vec3> crosswalk_pts = {
        {18, -3.5, 0}, {22, -3.5, 0}, {22, -0.5, 0}, {18, -0.5, 0}};
    std::vector<overlume::Vec3> stopline_pts = {{17, -3.5, 0}, {17, -0.5, 0}};
    std::vector<overlume::Vec3> junction_pts = {{30, -6, 0}, {36, -6, 0}};
    std::vector<overlume::Vec3> road_edge_pts = {{0, -4.0, 0}, {40, -4.0, 0}};
    std::vector<overlume::Vec3> road_surface_pts = {
        {0, -3.5, 0}, {40, -3.5, 0}, {40, -0.5, 0}, {0, -0.5, 0}};
    std::vector<overlume::Vec3> other_pts = {{0, 4, 0}, {10, 4, 0}};

    struct MapKindSpec {
        overlume::MapKind kind;
        std::vector<overlume::Vec3>* pts;
        uint8_t is_polygon;
    };
    const MapKindSpec kinds[] = {
        {overlume::MapKind::OTHER, &other_pts, 0},
        {overlume::MapKind::CENTERLINE, &centerline_pts, 0},
        {overlume::MapKind::LEFT_BOUNDARY, &left_boundary_pts, 0},
        {overlume::MapKind::RIGHT_BOUNDARY, &right_boundary_pts, 0},
        {overlume::MapKind::CROSSWALK, &crosswalk_pts, 1},
        {overlume::MapKind::STOPLINE, &stopline_pts, 0},
        {overlume::MapKind::JUNCTION, &junction_pts, 0},
        {overlume::MapKind::ROAD_EDGE, &road_edge_pts, 0},
        {overlume::MapKind::ROAD_SURFACE, &road_surface_pts, 1},
    };
    std::vector<overlume::MapElement> map_elements;
    for (const auto& spec : kinds) {
        overlume::MapElement el{};
        el.points = spec.pts->data();
        el.point_count = static_cast<uint32_t>(spec.pts->size());
        el.is_polygon = spec.is_polygon;
        el.kind = spec.kind;
        el.lane_id = 0;
        el.last_update_sec = 0.0;
        map_elements.push_back(el);
    }

    // ── One behavior-role path ribbon (ego's planned/predicted path) ────
    std::vector<overlume::Vec3> ribbon_pts = {{0, -2, 0}, {15, -2, 0}, {25, -3, 0}, {40, -3, 0}};
    overlume::PathRibbon ribbon{};
    ribbon.role = overlume::PathRole::BEHAVIOR;
    ribbon.points = ribbon_pts.data();
    ribbon.point_count = static_cast<uint32_t>(ribbon_pts.size());
    ribbon.last_update_sec = 0.0;

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    scene.ego = ego;
    scene.objects = objects.data();
    scene.object_count = static_cast<uint32_t>(objects.size());
    scene.paths = &ribbon;
    scene.path_count = 1;
    scene.map_elements = map_elements.data();
    scene.map_element_count = static_cast<uint32_t>(map_elements.size());

    // One publish. Every array above may now be left alone or go out of
    // scope after this call — overlume already copied what it needs.
    overlume::set_scene(renderer, scene);

    overlume::CameraPose pose{{-6.0, -14.0, 14.0}, {18.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> rgb(static_cast<size_t>(config.width) * config.height * 3);
    overlume::FrameView view{rgb.data(), config.width, config.height};

    // First render of the published scene.
    if (!overlume::render_frame(renderer, pose, view)) {
        std::fprintf(stderr, "02_scene_population: render_frame() failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }
    std::vector<uint8_t> first = rgb;

    // Second render, no set_scene() in between: the freeze-frame contract
    // says this must reproduce the same picture, not go blank or drift.
    if (!overlume::render_frame(renderer, pose, view)) {
        std::fprintf(stderr, "02_scene_population: render_frame() (freeze-frame check) failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }
    // Same content, not necessarily the same bytes: a little GPU dithering/
    // AO/TAA-jitter noise between two static renders is normal -- measured
    // 1.74% here, so nBytes/4 leaves a wide margin above that (single-GPU-box
    // calibration; a noisier GPU/driver may need it raised) while still
    // catching a real regression (a blank/dropped frame differs by most of
    // the buffer). Enforced, not just printed: a violation fails this example.
    const size_t diff = count_differing_bytes(first, rgb);
    const size_t nBytes = first.size();
    if (diff < nBytes / 4) {
        std::printf(
            "freeze-frame check: second render_frame() with no new set_scene() "
            "reproduced the same picture (%zu/%zu bytes of GPU noise), as documented.\n",
            diff, nBytes);
    } else {
        std::fprintf(stderr,
                     "freeze-frame check FAILED: the two frames differ by %zu/%zu bytes -- the "
                     "freeze-frame/double-buffer contract was violated\n",
                     diff, nBytes);
        overlume::destroy_renderer(renderer);
        return 1;
    }

    const bool wrote =
        overlume_examples::write_png(args.output_path, config.width, config.height, rgb.data());
    overlume::destroy_renderer(renderer);
    return wrote ? 0 : 1;
}
