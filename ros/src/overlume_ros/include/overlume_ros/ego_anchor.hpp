// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cmath>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::ros {

constexpr double kEgoForwardYaw = 0.0;

inline overlume::CameraPose compose_ego_anchored_pose(const overlume::CameraPose& offset_pose,
                                                      const overlume::EgoState& ego) {
    const double rot = ego.heading_rad - kEgoForwardYaw;
    const double c = std::cos(rot);
    const double s = std::sin(rot);
    auto rotate_and_translate = [&](const double in[3], double out[3]) {
        out[0] = in[0] * c - in[1] * s + ego.position.x;
        out[1] = in[0] * s + in[1] * c + ego.position.y;
        out[2] = in[2] + ego.position.z;
    };
    overlume::CameraPose out = offset_pose;
    rotate_and_translate(offset_pose.eye, out.eye);
    rotate_and_translate(offset_pose.target, out.target);
    return out;
}

}
