// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "overlume/scene.h"

#include <cstdint>

namespace overlume::bowl {

bool ProjectToCameraUv(const overlume::CameraExtrinsics& ext, const overlume::CameraIntrinsics& in,
                       uint32_t width, uint32_t height, overlume::Vec3 rig_point, float* out_u,
                       float* out_v);

overlume::Vec3 BowlSurfacePoint(double bowl_R0, double bowl_k, double bowl_Rmax, double theta,
                                double r);

float BorderFeather(float xp, float yp, uint32_t width, uint32_t height, double margin);

float CameraAlignment(const overlume::CameraExtrinsics& ext, overlume::Vec3 rig_point);

}  // namespace overlume::bowl
