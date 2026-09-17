// 06_overlays_and_pointcloud.cpp — a point cloud, a severity-ramped alert
// polygon, a live HUD-color query, and a live quality-preset switch, all in
// one scene. Public headers only: <overlume/api.h>, <overlume/scene.h>.
#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <vector>

namespace {

// Packs r/g/b/a into the uint32 layout scene.h's PointCloudPoint comment
// documents: byte 0 (LSB)=r, byte1=g, byte2=b, byte3(MSB)=a.
// a!=0 means "trust r/g/b verbatim" (vs. the flat-color fallback sentinel).
uint32_t pack_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    return static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(a) << 24);
}

}  // namespace

int main(int argc, char** argv) {
    const overlume_examples::ExampleArgs args =
        overlume_examples::parse_args(argc, argv, "06_overlays_and_pointcloud.png");

    overlume::RenderConfig config{};
    config.width = 640;
    config.height = 480;
    config.quality = 1;  // start at medium -- switched live below
    config.theme_assets_dir = args.theme_dir.c_str();
    config.initial_theme = "dark_adas";

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        std::fprintf(stderr, "06_overlays_and_pointcloud: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }

    // ── A small synthetic point cloud, a simple ramp along +X colored from
    //    red to blue ──────────────────────────────────────────────────────
    constexpr uint32_t kPointCount = 200;
    std::vector<overlume::PointCloudPoint> points(kPointCount);
    for (uint32_t i = 0; i < kPointCount; ++i) {
        const double t = static_cast<double>(i) / (kPointCount - 1);
        points[i].position = {5.0 + t * 15.0, -6.0 + t * 4.0, 0.5};
        const uint8_t r = static_cast<uint8_t>(255.0 * (1.0 - t));
        const uint8_t b = static_cast<uint8_t>(255.0 * t);
        points[i].rgba = pack_rgba(r, 0, b, 255);  // a=255: use this color verbatim
    }
    overlume::PointCloud cloud{};
    cloud.points = points.data();
    cloud.point_count = kPointCount;
    cloud.last_update_sec = 0.0;

    // ── One warning-severity alert polygon -- severity picks the theme's
    //    alert ramp color, the caller never hand-picks a color itself ─────
    const std::vector<overlume::Vec3> alert_pts = {
        {12, 2, 0}, {16, 2, 0}, {16, 5, 0}, {12, 5, 0}};
    overlume::AlertPolygon alert{};
    alert.points = alert_pts.data();
    alert.point_count = static_cast<uint32_t>(alert_pts.size());
    alert.severity = 1;  // 0 info / 1 warning / 2 critical
    alert.last_update_sec = 0.0;

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    scene.ego.valid = 1;
    scene.point_clouds = &cloud;
    scene.point_cloud_count = 1;
    scene.alerts = &alert;
    scene.alert_count = 1;
    overlume::set_scene(renderer, scene);

    const overlume::CameraPose pose{{0.0, -18.0, 16.0}, {10.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> rgb(static_cast<size_t>(config.width) * config.height * 3);
    overlume::FrameView view{rgb.data(), config.width, config.height};

    if (!overlume::render_frame(renderer, pose, view)) {
        std::fprintf(stderr, "06_overlays_and_pointcloud: render_frame() failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }

    // ── HUD color query -- reads the theme's live HUD colors, including
    //    mid-transition blend values (get_hud_colors() re-reads them every
    //    render_frame() call, no separate refresh needed) ─────────────────
    const overlume::HudColors hud = overlume::get_hud_colors(renderer);
    std::printf("HUD colors: text=(%.2f,%.2f,%.2f) accent=(%.2f,%.2f,%.2f) scale=%.2f\n",
                hud.text_color[0], hud.text_color[1], hud.text_color[2], hud.accent_color[0],
                hud.accent_color[1], hud.accent_color[2], hud.scale);

    // ── Live quality-preset switch -- re-applies the SSAO/AA/shadow/
    //    render-scale mapping against this SAME renderer, no re-create ────
    std::printf("quality preset before: %u\n", overlume::get_quality(renderer));
    overlume::set_quality(renderer, 2);  // high
    std::printf("quality preset after set_quality(2): %u\n", overlume::get_quality(renderer));
    if (!overlume::render_frame(renderer, pose, view)) {
        std::fprintf(stderr,
                     "06_overlays_and_pointcloud: render_frame() after set_quality() failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }

    const bool wrote = overlume_examples::write_png(args.output_path, config.width,
                                                      config.height, rgb.data());
    overlume::destroy_renderer(renderer);
    return wrote ? 0 : 1;
}
