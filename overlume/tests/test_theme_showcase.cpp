// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "test_paths.hpp"

#include "stb_image_write.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

using overlume::Vec3;

const std::string kTestTownDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";

constexpr overlume::GeoAnchor kAnchor{25.0803, 55.3910, 0.0};

constexpr Vec3 kBuildingsCentroid{-109.2, -17.1, 3.0};

constexpr Vec3 kSceneOrigin{kBuildingsCentroid.x - 20.0, kBuildingsCentroid.y - 6.0, 0.0};

Vec3 translated(const Vec3& p, double dx, double dy) { return Vec3{p.x + dx, p.y + dy, p.z}; }

constexpr uint32_t kWidth = 1280;
constexpr uint32_t kHeight = 960;

struct ContentStats {
    double mean_luminance = 0.0;
    double non_background_fraction = 0.0;
};

double luminance(uint8_t r, uint8_t g, uint8_t b) { return 0.2126 * r + 0.7152 * g + 0.0722 * b; }

ContentStats analyze_capture(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t height) {
    const size_t n = static_cast<size_t>(width) * height;
    double sum = 0.0;
    for (size_t i = 0; i < n; ++i) sum += luminance(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
    const double mean = sum / static_cast<double>(n);

    const double bgL = luminance(rgb[0], rgb[1], rgb[2]);
    constexpr double kTolerance = 10.0;
    size_t differing = 0;
    for (size_t i = 0; i < n; ++i) {
        const double l = luminance(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
        if (std::abs(l - bgL) > kTolerance) ++differing;
    }
    return ContentStats{mean, static_cast<double>(differing) / static_cast<double>(n)};
}

std::string env_or(const char* name, const std::string& fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : fallback;
}

}  // namespace

TEST(ThemeShowcase, Capture) {
    if (std::getenv("OVERLUME_SHOWCASE") == nullptr) {
        GTEST_SKIP() << "opt-in palette-iteration capture -- set OVERLUME_SHOWCASE=1 to run "
                        "(never required by ctest); see this file's header comment for the "
                        "full env contract (OVERLUME_SHOWCASE_THEME/_THEME_DIR/_OUT)";
    }

    const std::string themeName = env_or("OVERLUME_SHOWCASE_THEME", "dark_adas");
    const std::string themeDir = env_or("OVERLUME_SHOWCASE_THEME_DIR", kThemeDir);
    const std::string outPath =
        env_or("OVERLUME_SHOWCASE_OUT", "/tmp/theme_showcase_" + themeName + ".png");

    overlume::RenderConfig cfg{kWidth, kHeight, 2, themeDir.c_str(), themeName.c_str()};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::theme_assets_loaded(r))
        << "[ThemeShowcase] '" << themeName << "' failed to load from '" << themeDir
        << "' -- rendering the compiled-in fallback theme instead, NOT "
        << "the requested candidate. Check the theme name/dir.";

    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), kAnchor))
        << "baked fixture town failed to open -- see tests/fixtures/environment_test_town_0";

    constexpr Vec3 kEgoDims{4.5, 1.9, 1.6};
    overlume::set_ego_model(r, "/nonexistent/theme_showcase_ego.glb", kEgoDims);

    const double now = 42.0;

    constexpr double kHalfWidth = 3.5;
    constexpr double kEdgeOffset = kHalfWidth + 0.6;
    constexpr double kRoadX0 = -15.0;
    constexpr double kRoadX1 = 45.0;
    constexpr double kCrosswalkX = 12.0;

    auto world = [](double lx, double ly) {
        return Vec3{kSceneOrigin.x + lx, kSceneOrigin.y + ly, 0.0};
    };

    const std::vector<Vec3> roadSurfacePts = {
        world(kRoadX0, -kHalfWidth), world(kRoadX1, -kHalfWidth), world(kRoadX1, kHalfWidth),
        world(kRoadX0, kHalfWidth)};
    const std::vector<Vec3> leftBoundaryPts = {world(kRoadX0, -kHalfWidth),
                                               world(kRoadX1, -kHalfWidth)};
    const std::vector<Vec3> rightBoundaryPts = {world(kRoadX0, kHalfWidth),
                                                world(kRoadX1, kHalfWidth)};
    const std::vector<Vec3> centerlinePts = {world(kRoadX0, 0.0), world(kRoadX1, 0.0)};
    const std::vector<Vec3> roadEdgeLeftPts = {world(kRoadX0, -kEdgeOffset),
                                               world(kRoadX1, -kEdgeOffset)};
    const std::vector<Vec3> roadEdgeRightPts = {world(kRoadX0, kEdgeOffset),
                                                world(kRoadX1, kEdgeOffset)};
    const std::vector<Vec3> crosswalkPts = {
        world(kCrosswalkX - 1.5, -kEdgeOffset - 0.3), world(kCrosswalkX + 1.5, -kEdgeOffset - 0.3),
        world(kCrosswalkX + 1.5, kEdgeOffset + 0.3), world(kCrosswalkX - 1.5, kEdgeOffset + 0.3)};

    auto make_elem = [&](const std::vector<Vec3>& pts, uint8_t isPolygon, overlume::MapKind kind) {
        overlume::MapElement e{};
        e.points = pts.data();
        e.point_count = static_cast<uint32_t>(pts.size());
        e.is_polygon = isPolygon;
        e.kind = kind;
        e.last_update_sec = now;
        return e;
    };
    const std::vector<overlume::MapElement> mapElements = {
        make_elem(roadSurfacePts, 1, overlume::MapKind::ROAD_SURFACE),
        make_elem(leftBoundaryPts, 0, overlume::MapKind::LEFT_BOUNDARY),
        make_elem(rightBoundaryPts, 0, overlume::MapKind::RIGHT_BOUNDARY),
        make_elem(centerlinePts, 0, overlume::MapKind::CENTERLINE),
        make_elem(roadEdgeLeftPts, 0, overlume::MapKind::ROAD_EDGE),
        make_elem(roadEdgeRightPts, 0, overlume::MapKind::ROAD_EDGE),
        make_elem(crosswalkPts, 1, overlume::MapKind::CROSSWALK),
    };

    overlume::testing::ObjectScene objects = overlume::testing::make_mixed_class_objects(now);
    constexpr Vec3 kObjectFanOffsets[] = {
        {0.0, -9.0, 0.0},  {0.0, 12.0, 0.0},  {6.0, -55.0, 0.0},
        {-8.0, 11.0, 0.0}, {-2.0, -6.0, 0.0}, {2.0, 20.0, 0.0},
    };
    constexpr size_t kTruckVanIdx = static_cast<size_t>(overlume::ObjectClass::TRUCK_VAN);
    for (auto& p : objects.path_points) {
        p = translated(p, kSceneOrigin.x + kObjectFanOffsets[kTruckVanIdx].x,
                       kSceneOrigin.y + kObjectFanOffsets[kTruckVanIdx].y);
    }
    for (size_t i = 0; i < objects.objects.size(); ++i) {
        objects.objects[i].position =
            translated(objects.objects[i].position, kSceneOrigin.x + kObjectFanOffsets[i].x,
                       kSceneOrigin.y + kObjectFanOffsets[i].y);
    }

    const Vec3 carPos = objects.objects[0].position;
    const std::vector<Vec3> criticalAlertPts = {{carPos.x - 2.5, carPos.y - 2.5, 0.0},
                                                {carPos.x + 2.5, carPos.y - 2.5, 0.0},
                                                {carPos.x + 2.5, carPos.y + 2.5, 0.0},
                                                {carPos.x - 2.5, carPos.y + 2.5, 0.0}};
    overlume::AlertPolygon criticalAlert{};
    criticalAlert.points = criticalAlertPts.data();
    criticalAlert.point_count = static_cast<uint32_t>(criticalAlertPts.size());
    criticalAlert.severity = 2;
    criticalAlert.last_update_sec = now;

    struct RibbonLaneSpec {
        overlume::PathRole role;
        double laneY;
    };
    constexpr std::array<RibbonLaneSpec, 3> kRibbonLanes{{
        {overlume::PathRole::BEHAVIOR, 0.0},
        {overlume::PathRole::GLOBAL, -3.0},
        {overlume::PathRole::LOCAL, 3.0},
    }};
    constexpr double kRibbonX0 = -2.0;
    constexpr double kRibbonX1 = 18.0;

    std::vector<Vec3> ribbonPoints;
    ribbonPoints.reserve(kRibbonLanes.size() * 2);
    for (const auto& lane : kRibbonLanes) {
        ribbonPoints.push_back(world(kRibbonX0, lane.laneY));
        ribbonPoints.push_back(world(kRibbonX1, lane.laneY));
    }
    std::vector<overlume::PathRibbon> ribbonList;
    ribbonList.reserve(kRibbonLanes.size());
    for (size_t i = 0; i < kRibbonLanes.size(); ++i) {
        overlume::PathRibbon r{};
        r.role = kRibbonLanes[i].role;
        r.points = ribbonPoints.data() + i * 2;
        r.point_count = 2;
        r.last_update_sec = now;
        ribbonList.push_back(r);
    }

    overlume::testing::GridScene grids = overlume::testing::make_two_layer_grids(now);
    for (auto& g : grids.grids) g.origin = translated(g.origin, kSceneOrigin.x, kSceneOrigin.y);

    overlume::SceneGraph s{};
    s.sim_time_sec = now;
    s.ego = overlume::EgoState{kSceneOrigin, 0.0, 8.0, 1};
    s.objects = objects.objects.data();
    s.object_count = static_cast<uint32_t>(objects.objects.size());
    s.paths = ribbonList.data();
    s.path_count = static_cast<uint32_t>(ribbonList.size());
    s.map_elements = mapElements.data();
    s.map_element_count = static_cast<uint32_t>(mapElements.size());
    s.grids = grids.grids.data();
    s.grid_count = static_cast<uint32_t>(grids.grids.size());
    s.alerts = &criticalAlert;
    s.alert_count = 1;
    overlume::set_scene(r, s);

    const overlume::CameraPose pose{{kSceneOrigin.x - 20.0, kSceneOrigin.y - 20.0, 15.0},
                                    {kSceneOrigin.x + 20.0, kSceneOrigin.y + 6.0, 1.5},
                                    60.0};

    std::vector<uint8_t> warm(static_cast<size_t>(kWidth) * kHeight * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {warm.data(), kWidth, kHeight}));

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {rgb.data(), kWidth, kHeight}));
    stbi_write_png(outPath.c_str(), static_cast<int>(kWidth), static_cast<int>(kHeight), 3,
                   rgb.data(), static_cast<int>(kWidth) * 3);

    const ContentStats stats = analyze_capture(rgb, kWidth, kHeight);
    std::cerr << "[ThemeShowcase] theme='" << themeName << "' theme_dir='" << themeDir << "' -> "
              << outPath << " mean_luminance=" << stats.mean_luminance
              << " non_background_fraction=" << stats.non_background_fraction << "\n";

    overlume::destroy_renderer(r);
}
