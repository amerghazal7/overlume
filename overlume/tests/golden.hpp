// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// Whole-frame SSIM floor for golden comparisons. The goldens come from desktop GL (llvmpipe);
// the GLES back end (ANGLE/SwiftShader on the Android emulator) lands at 0.974-0.979 on the
// same scenes, so Android builds set OVERLUME_GOLDEN_SSIM_MIN lower. Goldens are never rewritten.
#ifdef OVERLUME_GOLDEN_SSIM_MIN
inline constexpr double kSsimMin = OVERLUME_GOLDEN_SSIM_MIN;
#else
inline constexpr double kSsimMin = 0.98;
#endif

struct MapGeom {
    std::vector<overlume::Vec3> points;
    std::vector<overlume::MapElement> elements;

    MapGeom() = default;
    MapGeom(const MapGeom&) = delete;
    MapGeom& operator=(const MapGeom&) = delete;
    MapGeom(MapGeom&&) = default;
    MapGeom& operator=(MapGeom&&) = default;
};

MapGeom load_map_geom(const char* path);

struct ObjectScene {
    std::vector<overlume::Vec3> path_points;
    std::vector<overlume::TrackedObject> objects;

    ObjectScene() = default;
    ObjectScene(const ObjectScene&) = delete;
    ObjectScene& operator=(const ObjectScene&) = delete;
    ObjectScene(ObjectScene&&) = default;
    ObjectScene& operator=(ObjectScene&&) = default;
};

ObjectScene make_mixed_class_objects(double now);

struct RibbonScene {
    std::vector<overlume::Vec3> point_storage;
    std::vector<overlume::PathRibbon> ribbons;
    overlume::EgoState ego{};

    RibbonScene() = default;
    RibbonScene(const RibbonScene&) = delete;
    RibbonScene& operator=(const RibbonScene&) = delete;
    RibbonScene(RibbonScene&&) = default;
    RibbonScene& operator=(RibbonScene&&) = default;
};

RibbonScene make_three_role_ribbons(double now);

struct AlertScene {
    std::vector<overlume::Vec3> point_storage;
    std::vector<overlume::AlertPolygon> alerts;

    AlertScene() = default;
    AlertScene(const AlertScene&) = delete;
    AlertScene& operator=(const AlertScene&) = delete;
    AlertScene(AlertScene&&) = default;
    AlertScene& operator=(AlertScene&&) = default;
};

AlertScene make_sweep_and_predicted_alerts(double now);

struct GenericMarkerScene {
    std::vector<overlume::Vec3> point_storage;
    std::vector<overlume::GenericMarker> markers;

    GenericMarkerScene() = default;
    GenericMarkerScene(const GenericMarkerScene&) = delete;
    GenericMarkerScene& operator=(const GenericMarkerScene&) = delete;
    GenericMarkerScene(GenericMarkerScene&&) = default;
    GenericMarkerScene& operator=(GenericMarkerScene&&) = default;
};

GenericMarkerScene make_all_primitive_markers(double now, const char* mesh_glb_path);

overlume::Vec3 centroid(const std::vector<overlume::MapElement>& elems);

struct GridScene {
    std::vector<std::vector<uint8_t>> cell_storage;
    std::vector<overlume::GroundGridLayer> grids;

    GridScene() = default;
    GridScene(const GridScene&) = delete;
    GridScene& operator=(const GridScene&) = delete;
    GridScene(GridScene&&) = default;
    GridScene& operator=(GridScene&&) = default;
};

GridScene make_two_layer_grids(double now);

struct HeightGridScene {
    std::vector<float> heights;
    std::vector<uint8_t> cost_cells;
    std::vector<overlume::HeightGridLayer> height_grids;
    std::vector<overlume::GroundGridLayer> grids;

    HeightGridScene() = default;
    HeightGridScene(const HeightGridScene&) = delete;
    HeightGridScene& operator=(const HeightGridScene&) = delete;
    HeightGridScene(HeightGridScene&&) = default;
    HeightGridScene& operator=(HeightGridScene&&) = default;
};

// 120x120 grid at 0.2 m centred ahead of an ego at the origin: a +1.2 m berm, a -0.8 m ditch,
// a 0 -> +1.5 m ramp half, a NaN (unknown) patch, and a cost grid with a few high-cost cells.
// yaw_rad rotates both grids about their shared origin corner.
HeightGridScene make_height_grid_terrain_scene(double now, double yaw_rad = 0.0);

double render_and_compare(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
                          const char* golden_png_path, const char* out_png_path);

struct FrameStats {
    double mean = 0.0;
    int distinct_levels = 0;
    double top_third_mean = 0.0;
    double bottom_third_mean = 0.0;
    double sky_row_mean = 0.0;
    double horizon_row_mean = 0.0;
};
FrameStats analyze_png(const char* png_path);

}
