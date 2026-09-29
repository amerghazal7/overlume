// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once
#include <cstdint>
#include <cstddef>
#include "overlume/api.h"

namespace overlume {

constexpr uint32_t kSceneVersion = 8;

struct Vec3 {
    double x, y, z;
};

enum class ObjectClass : uint8_t {
    CAR = 0,
    TRUCK_VAN = 1,
    BUS = 2,
    PEDESTRIAN = 3,
    CYCLIST = 4,
    UNKNOWN = 5
};
enum class PathRole : uint8_t { BEHAVIOR = 0, GLOBAL = 1, LOCAL = 2 };
enum class MarkerPrimitive : uint8_t {
    CUBE = 0,
    SPHERE = 1,
    CYLINDER = 2,
    ARROW = 3,
    LINE_STRIP = 4,
    LINE_LIST = 5,
    POINTS = 6,
    TEXT = 7,
    TRIANGLE_LIST = 8,
    MESH = 9
};

struct EgoState {
    Vec3 position;
    double heading_rad;
    double speed_mps;
    uint8_t valid;
};

struct TrackedObject {
    uint32_t id;
    ObjectClass cls;
    Vec3 position;
    double heading_rad;
    Vec3 dimensions;
    Vec3 velocity;
    const Vec3* predicted_path;
    uint32_t predicted_path_count;
    const char* label;
    double last_update_sec;
};

struct PathRibbon {
    PathRole role;
    const Vec3* points;
    uint32_t point_count;
    double last_update_sec;
};

enum class MapKind : uint8_t {
    OTHER = 0,
    CENTERLINE = 1,
    LEFT_BOUNDARY = 2,
    RIGHT_BOUNDARY = 3,
    CROSSWALK = 4,
    STOPLINE = 5,
    JUNCTION = 6,
    ROAD_EDGE = 7,
    ROAD_SURFACE = 8
};

struct MapElement {
    const Vec3* points;
    uint32_t point_count;
    uint8_t is_polygon;
    MapKind kind;
    uint32_t lane_id;
    double last_update_sec;
};

struct GroundGridLayer {
    uint8_t kind;
    Vec3 origin;
    double resolution_m;
    uint32_t width_cells, height_cells;
    const uint8_t* cells;
    double last_update_sec;
    double yaw_rad;
};

struct AlertPolygon {
    const Vec3* points;
    uint32_t point_count;
    uint8_t severity;
    double last_update_sec;
};

struct GenericMarker {
    MarkerPrimitive primitive;
    Vec3 position;
    double heading_rad;
    Vec3 scale;
    const Vec3* points;
    uint32_t point_count;
    const char* text;
    const char* mesh_path;
    float color[4];
    double last_update_sec;
};

struct PointCloudPoint {
    Vec3 position;
    uint32_t rgba;
};

struct PointCloud {
    const PointCloudPoint* points;
    uint32_t point_count;
    double last_update_sec;
};

struct TrajectoryCarpet {
    const PointCloudPoint* points;
    uint32_t point_count;
    double last_update_sec;
};

struct AlertChip {
    const char* text;
    Vec3 anchor;
};

struct Hud {
    double speed_mps;
    uint8_t active_mode;
    const AlertChip* chips;
    uint32_t chip_count;
};

struct SceneGraph {
    double sim_time_sec;

    EgoState ego;
    const TrackedObject* objects;
    uint32_t object_count;
    const PathRibbon* paths;
    uint32_t path_count;
    const MapElement* map_elements;
    uint32_t map_element_count;
    const GroundGridLayer* grids;
    uint32_t grid_count;
    const AlertPolygon* alerts;
    uint32_t alert_count;
    const GenericMarker* markers;
    uint32_t marker_count;
    Hud hud;
    const PointCloud* point_clouds;
    uint32_t point_cloud_count;
    const TrajectoryCarpet* trajectory_carpets;
    uint32_t trajectory_carpet_count;
};

void set_scene(VisualRenderer*, const SceneGraph& scene);

bool set_theme(VisualRenderer*, const char* theme_name, double at_sec, double transition_sec);

bool set_ego_model(VisualRenderer*, const char* gltf_path, Vec3 fallback_dims);

uint32_t set_object_model_dir(VisualRenderer*, const char* dir);

bool theme_assets_loaded(VisualRenderer*);

struct HudColors {
    float text_color[3];
    float accent_color[3];
    float scale;
};

HudColors get_hud_colors(VisualRenderer*);

bool project_to_screen(VisualRenderer*, Vec3 world_point, float* out_x, float* out_y);

bool theme_parses(const char* dir, const char* theme_name);

struct GeoAnchor {
    double origin_lat_deg;
    double origin_lon_deg;
    double heading_rad;
    double origin_height_m;
};

constexpr uint32_t kMaxBowlCameras = 6;

struct CameraExtrinsics {
    double R[9];
    double t[3];
};

struct CameraIntrinsics {
    double fx, fy, cx, cy;
    double dist[5];
};

struct BowlConfig {
    uint32_t camera_count;
    const CameraExtrinsics* extrinsics;
    const CameraIntrinsics* intrinsics;
    const uint32_t* cam_width;
    const uint32_t* cam_height;
    double bowl_R0, bowl_k, bowl_Rmax;
    double feather_margin;
    uint8_t fill_blind_zone;
    uint8_t exposure_match;
    float sky_color[3];
    float exposure_compensation = 1.56f;
};

bool set_bowl_config(VisualRenderer*, const BowlConfig&);

bool set_bowl_visible(VisualRenderer*, bool visible);

bool set_self_view_masks(VisualRenderer*, bool enabled);

bool set_camera_motion_delta(VisualRenderer*, uint32_t cam_idx,
                             const double delta_4x4_row_major[16]);

bool set_camera_frame(VisualRenderer*, uint32_t cam_idx, const uint8_t* rgb, uint32_t width,
                      uint32_t height, uint64_t frame_id,
                      void (*release)(void*, size_t, void*) = nullptr, void* user = nullptr);

bool set_environment_source(VisualRenderer*, const char* source_uri, GeoAnchor anchor);

enum class EnvironmentSourceState : uint8_t {
    NONE = 0,
    BAKED = 1,
    STREAMING = 2,
    STREAMING_FALLBACK = 3,
};

EnvironmentSourceState environment_source_state(VisualRenderer*);

bool set_environment_visible(VisualRenderer* r, bool visible);

bool environment_visible(VisualRenderer* r);

void set_quality(VisualRenderer*, uint32_t preset);

uint32_t get_quality(VisualRenderer*);

}
