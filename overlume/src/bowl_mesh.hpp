// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "overlume/scene.h"

#include <cstdint>
#include <vector>

namespace overlume::bowl {

struct BowlVertex {
    overlume::Vec3 position;
    float coverage_a = 0.0f;
    float coverage_b = 0.0f;
    float coverage_c = 0.0f;
    uint32_t index_a = 0;
    uint32_t index_b = 0;
    uint32_t index_c = 0;
};

struct BowlMesh {
    std::vector<BowlVertex> vertices;
    std::vector<uint32_t> indices;
};

struct BowlMeshParams {
    uint32_t theta_segments = 64;
    uint32_t radial_rings = 24;
};

struct EgoBox {
    overlume::Vec3 center{0.0, 0.0, 0.0};
    overlume::Vec3 half_extents{0.0, 0.0, 0.0};
};

BowlMesh BakeBowlMesh(const BowlMeshParams& mesh_params, double bowl_R0, double bowl_k,
                      double bowl_Rmax, uint32_t camera_count,
                      const overlume::CameraExtrinsics* extrinsics,
                      const overlume::CameraIntrinsics* intrinsics, const uint32_t* cam_width,
                      const uint32_t* cam_height, const EgoBox& ego_box = EgoBox{});

}  // namespace overlume::bowl
