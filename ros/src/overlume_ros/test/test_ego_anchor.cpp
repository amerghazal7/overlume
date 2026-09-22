// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <cmath>

#include <gtest/gtest.h>

#include "overlume_ros/ego_anchor.hpp"

namespace overlume::ros {
namespace {

constexpr double kEps = 1e-9;

overlume::CameraPose DefaultOffset() {
    overlume::CameraPose p{};
    p.eye[0] = -4.0;
    p.eye[1] = 0.0;
    p.eye[2] = 3.5;
    p.target[0] = 2.0;
    p.target[1] = 0.0;
    p.target[2] = -0.5;
    p.vfov_deg = 80.0;
    return p;
}

TEST(ComposeEgoAnchoredPose, HeadingZeroKeepsOffsetBehindEgo) {
    overlume::EgoState ego{};
    ego.position = overlume::Vec3{10.0, 20.0, 1.0};
    ego.heading_rad = 0.0;
    ego.valid = 1;

    const overlume::CameraPose out = compose_ego_anchored_pose(DefaultOffset(), ego);

    EXPECT_NEAR(out.eye[0], -4.0 + ego.position.x, kEps);
    EXPECT_NEAR(out.eye[1], 0.0 + ego.position.y, kEps);
    EXPECT_NEAR(out.eye[2], 3.5 + ego.position.z, kEps);
    EXPECT_NEAR(out.target[0], 2.0 + ego.position.x, kEps);
    EXPECT_NEAR(out.target[1], 0.0 + ego.position.y, kEps);
    EXPECT_NEAR(out.target[2], -0.5 + ego.position.z, kEps);
}

TEST(ComposeEgoAnchoredPose, HeadingNegHalfPiRotatesEyeBehindEgoFacingMinusY) {
    overlume::EgoState ego{};
    ego.position = overlume::Vec3{5.0, -3.0, 2.0};
    ego.heading_rad = -M_PI_2;
    ego.valid = 1;

    const overlume::CameraPose out = compose_ego_anchored_pose(DefaultOffset(), ego);

    EXPECT_NEAR(out.eye[0], 0.0 + ego.position.x, kEps);
    EXPECT_NEAR(out.eye[1], 4.0 + ego.position.y, kEps);
    EXPECT_NEAR(out.eye[2], 3.5 + ego.position.z, kEps);
    EXPECT_NEAR(out.target[0], 0.0 + ego.position.x, kEps);
    EXPECT_NEAR(out.target[1], -2.0 + ego.position.y, kEps);
    EXPECT_NEAR(out.target[2], -0.5 + ego.position.z, kEps);
}

}  // namespace
}  // namespace overlume::ros
