// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/scene.h"

#include <cstddef>

#include <gtest/gtest.h>

static_assert(overlume::kSceneVersion == 8, "node/library scene.h version drifted");
static_assert(sizeof(overlume::GroundGridLayer) == 72, "node/library scene.h version drifted");
static_assert(offsetof(overlume::GroundGridLayer, yaw_rad) == 64,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::MapElement) == 32, "node/library scene.h version drifted");
static_assert(offsetof(overlume::MapElement, points) == 0, "node/library scene.h version drifted");
static_assert(offsetof(overlume::MapElement, point_count) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::MapElement, is_polygon) == 12,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::MapElement, kind) == 13, "node/library scene.h version drifted");
static_assert(offsetof(overlume::MapElement, lane_id) == 16,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::MapElement, last_update_sec) == 24,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::PointCloudPoint) == 32, "node/library scene.h version drifted");
static_assert(offsetof(overlume::PointCloudPoint, position) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::PointCloudPoint, rgba) == 24,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::PointCloud) == 24, "node/library scene.h version drifted");
static_assert(offsetof(overlume::PointCloud, points) == 0, "node/library scene.h version drifted");
static_assert(offsetof(overlume::PointCloud, point_count) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::PointCloud, last_update_sec) == 16,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::SceneGraph) == 216, "node/library scene.h version drifted");
static_assert(offsetof(overlume::SceneGraph, point_clouds) == 184,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::SceneGraph, point_cloud_count) == 192,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::TrajectoryCarpet) == 24, "node/library scene.h version drifted");
static_assert(offsetof(overlume::TrajectoryCarpet, points) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::TrajectoryCarpet, point_count) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::TrajectoryCarpet, last_update_sec) == 16,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::SceneGraph, trajectory_carpets) == 200,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::SceneGraph, trajectory_carpet_count) == 208,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::GeoAnchor) == 32, "node/library scene.h version drifted");
static_assert(offsetof(overlume::GeoAnchor, origin_lat_deg) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::GeoAnchor, origin_lon_deg) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::GeoAnchor, heading_rad) == 16,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::GeoAnchor, origin_height_m) == 24,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::CameraExtrinsics) == 96, "node/library scene.h version drifted");
static_assert(offsetof(overlume::CameraExtrinsics, R) == 0, "node/library scene.h version drifted");
static_assert(offsetof(overlume::CameraExtrinsics, t) == 72,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::CameraIntrinsics) == 72, "node/library scene.h version drifted");
static_assert(offsetof(overlume::CameraIntrinsics, fx) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::CameraIntrinsics, fy) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::CameraIntrinsics, cx) == 16,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::CameraIntrinsics, cy) == 24,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::CameraIntrinsics, dist) == 32,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::BowlConfig) == 96, "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, camera_count) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, extrinsics) == 8,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, intrinsics) == 16,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, cam_width) == 24,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, cam_height) == 32,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, bowl_R0) == 40,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, bowl_k) == 48, "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, bowl_Rmax) == 56,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, feather_margin) == 64,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, fill_blind_zone) == 72,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, exposure_match) == 73,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, sky_color) == 76,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::BowlConfig, exposure_compensation) == 88,
              "node/library scene.h version drifted");

static_assert(sizeof(overlume::EnvironmentSourceState) == 1,
              "node/library scene.h version drifted");

TEST(SceneLayout, Placeholder) {}
