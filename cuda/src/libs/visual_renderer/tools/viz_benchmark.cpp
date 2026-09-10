// viz_benchmark.cpp — VM-041 (Epic 5). Renders one representative scene
// (ribbons + trajectory carpet + map elements + point cloud, the same shapes
// tests/test_ribbon.cpp / test_trajectory_carpet.cpp / test_map_elements.cpp /
// test_point_cloud.cpp build) for N frames per quality preset and prints
// render_ms p50/p99 per preset. These are the numbers VM-040's governor will
// later be tuned against — this tool does not tune anything itself.
//
// GPU-less box: create_renderer() returns nullptr for every preset; each
// preset line then reads SKIP (no GPU/EGL) rather than fabricating numbers.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr uint32_t kWidth = 1280;
constexpr uint32_t kHeight = 720;
constexpr int kFramesPerPreset = 120;
constexpr int kWarmupFrames = 5;  // first-frame shader/texture upload cost excluded

// Three path roles, ~40 points each — same role shape as
// tests/test_ribbon.cpp's ThreeRoles_DarkAdas golden.
std::vector<mpviz::Vec3> make_path(double y_offset, int n) {
    std::vector<mpviz::Vec3> pts(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        pts[static_cast<size_t>(i)] = {static_cast<double>(i) * 0.5, y_offset, 0.0};
    }
    return pts;
}

// Centerline station colors, same PointCloudPoint reuse as
// tests/test_trajectory_carpet.cpp's make_stations().
std::vector<mpviz::PointCloudPoint> make_carpet_stations(int n) {
    std::vector<mpviz::PointCloudPoint> pts(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        pts[static_cast<size_t>(i)].position = {static_cast<double>(i) * 0.5, -1.0, 0.0};
        pts[static_cast<size_t>(i)].rgba = 0x00FF00FFu;
    }
    return pts;
}

// A lane centerline + two boundaries + a crosswalk polygon — same MapKind
// mix tests/test_map_elements.cpp exercises.
std::vector<mpviz::Vec3> make_lane_line(double x_offset, int n) {
    std::vector<mpviz::Vec3> pts(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        pts[static_cast<size_t>(i)] = {static_cast<double>(i) * 2.0, x_offset, 0.0};
    }
    return pts;
}

std::vector<mpviz::PointCloudPoint> make_point_cloud(uint32_t n) {
    std::vector<mpviz::PointCloudPoint> pts(n);
    for (uint32_t i = 0; i < n; ++i) {
        double t = static_cast<double>(i);
        pts[i].position = {std::fmod(t, 60.0) - 30.0, std::fmod(t * 0.37, 30.0) - 15.0,
                            std::fmod(t * 0.11, 3.0)};
        pts[i].rgba = 0xFF8040FFu;
    }
    return pts;
}

struct Frame {
    std::chrono::duration<double, std::milli> render_ms;
};

double percentile(std::vector<double>& sorted_ms, double p) {
    if (sorted_ms.empty()) return 0.0;
    size_t idx = static_cast<size_t>(p * static_cast<double>(sorted_ms.size() - 1));
    return sorted_ms[idx];
}

}  // namespace

int main() {
    // Built once, reused across every quality preset — same scene, only the
    // renderer's quality changes.
    std::vector<mpviz::Vec3> behavior_pts = make_path(0.0, 40);
    std::vector<mpviz::Vec3> global_pts = make_path(3.0, 40);
    std::vector<mpviz::Vec3> local_pts = make_path(-3.0, 40);
    mpviz::PathRibbon ribbons[3]{};
    ribbons[0] = {mpviz::PathRole::BEHAVIOR, behavior_pts.data(),
                  static_cast<uint32_t>(behavior_pts.size()), 0.0};
    ribbons[1] = {mpviz::PathRole::GLOBAL, global_pts.data(),
                  static_cast<uint32_t>(global_pts.size()), 0.0};
    ribbons[2] = {mpviz::PathRole::LOCAL, local_pts.data(),
                  static_cast<uint32_t>(local_pts.size()), 0.0};

    std::vector<mpviz::PointCloudPoint> carpet_pts = make_carpet_stations(60);
    mpviz::TrajectoryCarpet carpet{carpet_pts.data(), static_cast<uint32_t>(carpet_pts.size()), 0.0};

    std::vector<mpviz::Vec3> centerline = make_lane_line(0.0, 20);
    std::vector<mpviz::Vec3> left_boundary = make_lane_line(1.8, 20);
    std::vector<mpviz::Vec3> right_boundary = make_lane_line(-1.8, 20);
    const mpviz::Vec3 crosswalk_pts[4] = {
        {10.0, -1.8, 0.0}, {10.0, 1.8, 0.0}, {13.0, 1.8, 0.0}, {13.0, -1.8, 0.0}};
    mpviz::MapElement map_elements[4]{};
    map_elements[0] = {centerline.data(), static_cast<uint32_t>(centerline.size()), 0,
                       mpviz::MapKind::CENTERLINE, 1, 0.0};
    map_elements[1] = {left_boundary.data(), static_cast<uint32_t>(left_boundary.size()), 0,
                        mpviz::MapKind::LEFT_BOUNDARY, 1, 0.0};
    map_elements[2] = {right_boundary.data(), static_cast<uint32_t>(right_boundary.size()), 0,
                        mpviz::MapKind::RIGHT_BOUNDARY, 1, 0.0};
    map_elements[3] = {crosswalk_pts, 4, 1, mpviz::MapKind::CROSSWALK, 0, 0.0};

    std::vector<mpviz::PointCloudPoint> cloud_pts = make_point_cloud(20000);
    mpviz::PointCloud cloud{cloud_pts.data(), static_cast<uint32_t>(cloud_pts.size()), 0.0};

    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    scene.ego = {{0.0, 0.0, 0.0}, 0.0, 5.0, /*valid=*/1};
    scene.paths = ribbons;
    scene.path_count = 3;
    scene.trajectory_carpets = &carpet;
    scene.trajectory_carpet_count = 1;
    scene.map_elements = map_elements;
    scene.map_element_count = 4;
    scene.point_clouds = &cloud;
    scene.point_cloud_count = 1;

    mpviz::CameraPose pose{{-8.0, -12.0, 8.0}, {5.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3);
    mpviz::FrameView view{rgb.data(), kWidth, kHeight};

    const struct { uint8_t quality; const char* name; } kPresets[] = {
        {0, "low"}, {1, "medium"}, {2, "high"}};

    std::printf("viz_benchmark: %ux%u, N=%d frames/preset (+%d warmup)\n", kWidth, kHeight,
                kFramesPerPreset, kWarmupFrames);

    for (const auto& preset : kPresets) {
        mpviz::RenderConfig cfg{};
        cfg.width = kWidth;
        cfg.height = kHeight;
        cfg.quality = preset.quality;
        auto* r = mpviz::create_renderer(cfg);
        if (r == nullptr) {
            std::printf("quality=%u (%-6s)  SKIP (no GPU/EGL)\n", preset.quality, preset.name);
            continue;
        }
        mpviz::set_scene(r, scene);

        for (int i = 0; i < kWarmupFrames; ++i) {
            mpviz::render_frame(r, pose, view);
        }

        std::vector<double> ms;
        ms.reserve(kFramesPerPreset);
        for (int i = 0; i < kFramesPerPreset; ++i) {
            auto t0 = std::chrono::steady_clock::now();
            bool ok = mpviz::render_frame(r, pose, view);
            auto t1 = std::chrono::steady_clock::now();
            if (!ok) {
                std::printf("quality=%u (%-6s)  render_frame() failed at frame %d\n",
                            preset.quality, preset.name, i);
                mpviz::destroy_renderer(r);
                return 1;
            }
            ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        mpviz::destroy_renderer(r);

        std::sort(ms.begin(), ms.end());
        std::printf("quality=%u (%-6s)  render_ms p50=%.3f p99=%.3f\n", preset.quality,
                    preset.name, percentile(ms, 0.50), percentile(ms, 0.99));
    }
    return 0;
}
