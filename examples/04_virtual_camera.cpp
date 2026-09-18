// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// 04_virtual_camera.cpp — a small set of named camera "presets" (just plain
// overlume::CameraPose literals — the POD public API has no preset registry
// of its own, see the NOTE below), a linear tween between two of them, and
// overlume::project_to_screen() to project a world point onto each pose's
// screen. Public headers only: <overlume/api.h>, <overlume/scene.h>.
//
// NOTE on scope (see this task's skipped_or_deviated notes): named camera
// presets and pose tweening are a ROS-node/GUI concept (vcam_ws_bridge,
// overlume_ros) — the public library only exposes the raw CameraPose POD
// struct render_frame() takes each call. There is no `set_camera_preset()`
// or `tween_camera()` entry point to call here, so this example builds its
// own tiny pose set and does its own linear interpolation entirely in
// application code, using nothing overlume doesn't already expose.
#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <vector>

namespace {

// A linear tween between two poses at t in [0, 1] — eye/target lerp
// componentwise, vfov_deg lerps too. Plain application-level math; nothing
// here is an overlume API.
overlume::CameraPose tween(const overlume::CameraPose& a, const overlume::CameraPose& b, double t) {
    overlume::CameraPose out{};
    for (int i = 0; i < 3; ++i) {
        out.eye[i] = a.eye[i] + (b.eye[i] - a.eye[i]) * t;
        out.target[i] = a.target[i] + (b.target[i] - a.target[i]) * t;
    }
    out.vfov_deg = a.vfov_deg + (b.vfov_deg - a.vfov_deg) * t;
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    const overlume_examples::ExampleArgs args =
        overlume_examples::parse_args(argc, argv, "04_virtual_camera.png");

    overlume::RenderConfig config{};
    config.width = 640;
    config.height = 480;
    config.quality = 1;
    config.theme_assets_dir = args.theme_dir.c_str();
    config.initial_theme = "dark_adas";

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        std::fprintf(stderr, "04_virtual_camera: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }
    overlume_examples::apply_model_dir(renderer, args);

    // A world point every preset below frames toward — a spot on the road
    // roughly 10m ahead of the ego.
    const overlume::Vec3 world_point{10.0, 0.0, 0.0};

    // A small but visible scene, so each preset's frame shows something to
    // judge the framing by: a road surface with its centerline and
    // boundaries running +X, the ego at the origin, and three tracked
    // objects ahead. (02_scene_population.cpp covers every kind in depth;
    // this is the minimum that makes camera choices legible.)
    overlume::EgoState ego{};
    ego.valid = 1;
    ego.position = {0.0, 0.0, 0.0};
    ego.heading_rad = 0.0;

    std::vector<overlume::Vec3> road_surface_pts = {
        {-10, -3.5, 0}, {40, -3.5, 0}, {40, 3.5, 0}, {-10, 3.5, 0}};
    std::vector<overlume::Vec3> centerline_pts = {{-10, 0, 0}, {40, 0, 0}};
    std::vector<overlume::Vec3> left_pts = {{-10, 3.5, 0}, {40, 3.5, 0}};
    std::vector<overlume::Vec3> right_pts = {{-10, -3.5, 0}, {40, -3.5, 0}};
    const struct {
        overlume::MapKind kind;
        std::vector<overlume::Vec3>* pts;
        uint8_t is_polygon;
    } kinds[] = {{overlume::MapKind::ROAD_SURFACE, &road_surface_pts, 1},
                 {overlume::MapKind::CENTERLINE, &centerline_pts, 0},
                 {overlume::MapKind::LEFT_BOUNDARY, &left_pts, 0},
                 {overlume::MapKind::RIGHT_BOUNDARY, &right_pts, 0}};
    std::vector<overlume::MapElement> map_elements;
    for (const auto& spec : kinds) {
        overlume::MapElement el{};
        el.points = spec.pts->data();
        el.point_count = static_cast<uint32_t>(spec.pts->size());
        el.is_polygon = spec.is_polygon;
        el.kind = spec.kind;
        map_elements.push_back(el);
    }

    std::vector<overlume::TrackedObject> objects;
    const overlume::ObjectClass classes[] = {overlume::ObjectClass::CAR,
                                             overlume::ObjectClass::TRUCK_VAN,
                                             overlume::ObjectClass::PEDESTRIAN};
    for (uint32_t i = 0; i < 3; ++i) {
        overlume::TrackedObject obj{};
        obj.id = i + 1;
        obj.cls = classes[i];
        obj.position = {10.0 + 8.0 * i, (i % 2 == 0) ? 1.8 : -1.8, 0.0};
        obj.dimensions = {4.5, 1.9, 1.6};
        objects.push_back(obj);
    }

    overlume::SceneGraph scene{};
    scene.ego = ego;
    scene.objects = objects.data();
    scene.object_count = static_cast<uint32_t>(objects.size());
    scene.map_elements = map_elements.data();
    scene.map_element_count = static_cast<uint32_t>(map_elements.size());
    overlume::set_scene(renderer, scene);  // one publish; the arrays above are copied

    // Three named presets — a chase view, an overhead view, and a low
    // first-person-ish view. Plain data, no special API for "preset".
    const overlume::CameraPose chase{{-8.0, 0.0, 4.0}, {10.0, 0.0, 0.0}, 60.0};
    const overlume::CameraPose overhead{{5.0, 0.0, 40.0}, {5.0, 0.0, 0.0}, 50.0};
    const overlume::CameraPose first_person{{0.0, 0.0, 1.5}, {10.0, 0.0, 1.2}, 90.0};
    const struct {
        const char* name;
        overlume::CameraPose pose;
    } presets[] = {{"chase", chase}, {"overhead", overhead}, {"first_person", first_person}};

    std::vector<uint8_t> rgb(static_cast<size_t>(config.width) * config.height * 3);
    overlume::FrameView view{rgb.data(), config.width, config.height};

    for (const auto& preset : presets) {
        if (!overlume::render_frame(renderer, preset.pose, view)) {
            std::fprintf(stderr, "04_virtual_camera: render_frame() failed for preset '%s'\n",
                         preset.name);
            overlume::destroy_renderer(renderer);
            return 1;
        }
        // project_to_screen() uses the camera state the MOST RECENT
        // render_frame() call set — not a separately passed pose — so this
        // call must follow the render_frame() above for this same preset.
        float x = -1.0f, y = -1.0f;
        const bool on_screen = overlume::project_to_screen(renderer, world_point, &x, &y);
        if (on_screen) {
            std::printf(
                "preset '%-12s' -> world point (%.1f, %.1f, %.1f) projects to "
                "screen-fraction (%.3f, %.3f)\n",
                preset.name, world_point.x, world_point.y, world_point.z, x, y);
        } else {
            std::printf(
                "preset '%-12s' -> world point (%.1f, %.1f, %.1f) is off-screen "
                "(behind the camera or outside the frustum)\n",
                preset.name, world_point.x, world_point.y, world_point.z);
        }
    }

    // Tween from chase to overhead and render at t=0.5; that mid-flight frame
    // is the one written to disk. It shows the road markings and the three
    // clay-box objects from an elevated pose neither preset uses on its own
    // (the ego at the origin sits below the frame edge from here).
    const overlume::CameraPose midway = tween(chase, overhead, 0.5);
    if (!overlume::render_frame(renderer, midway, view)) {
        std::fprintf(stderr, "04_virtual_camera: render_frame() failed for the tween\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }
    float mx = -1.0f, my = -1.0f;
    if (overlume::project_to_screen(renderer, world_point, &mx, &my)) {
        std::printf("tween(chase, overhead, 0.5) -> world point projects to (%.3f, %.3f)\n", mx,
                    my);
    }

    const bool wrote =
        overlume_examples::write_png(args.output_path, config.width, config.height, rgb.data());
    overlume::destroy_renderer(renderer);
    return wrote ? 0 : 1;
}
