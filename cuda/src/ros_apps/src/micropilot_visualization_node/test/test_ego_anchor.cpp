/** @file test_ego_anchor.cpp
 *  @brief Unit coverage for compose_ego_anchored_pose/kEgoForwardYaw -- the
 *  yaw convention has regressed once already (a pi/2 broadside bug caught
 *  only by a human on a live frame). Exercises the header directly; no
 *  ROS/renderer runtime needed.
 */

#include <cmath>

#include <gtest/gtest.h>

#include "micropilot_visualization_node/ego_anchor.hpp"

namespace micropilot::visualization_app
{
namespace
{

constexpr double kEps = 1e-9;

// This node's real default virtual_pose param (visualization_node.cpp
// on_configure): eye (-4, 0, 3.5), target (2, 0, -0.5).
mpviz::CameraPose DefaultOffset()
{
    mpviz::CameraPose p{};
    p.eye[0] = -4.0; p.eye[1] = 0.0; p.eye[2] = 3.5;
    p.target[0] = 2.0; p.target[1] = 0.0; p.target[2] = -0.5;
    p.vfov_deg = 80.0;
    return p;
}

TEST(ComposeEgoAnchoredPose, HeadingZeroKeepsOffsetBehindEgo)
{
    mpviz::EgoState ego{};
    ego.position = mpviz::Vec3{10.0, 20.0, 1.0};
    ego.heading_rad = 0.0;
    ego.valid = 1;

    const mpviz::CameraPose out = compose_ego_anchored_pose(DefaultOffset(), ego);

    // heading == kEgoForwardYaw (0) -> no rotation, pure translation.
    EXPECT_NEAR(out.eye[0], -4.0 + ego.position.x, kEps);
    EXPECT_NEAR(out.eye[1], 0.0 + ego.position.y, kEps);
    EXPECT_NEAR(out.eye[2], 3.5 + ego.position.z, kEps);
    EXPECT_NEAR(out.target[0], 2.0 + ego.position.x, kEps);
    EXPECT_NEAR(out.target[1], 0.0 + ego.position.y, kEps);
    EXPECT_NEAR(out.target[2], -0.5 + ego.position.z, kEps);
}

TEST(ComposeEgoAnchoredPose, HeadingNegHalfPiRotatesEyeBehindEgoFacingMinusY)
{
    mpviz::EgoState ego{};
    ego.position = mpviz::Vec3{5.0, -3.0, 2.0};
    ego.heading_rad = -M_PI_2;
    ego.valid = 1;

    const mpviz::CameraPose out = compose_ego_anchored_pose(DefaultOffset(), ego);

    // rot = -pi/2: (x,y) -> (y, -x). eye (-4,0) -> (0, 4); target (2,0) -> (0,-2).
    EXPECT_NEAR(out.eye[0], 0.0 + ego.position.x, kEps);
    EXPECT_NEAR(out.eye[1], 4.0 + ego.position.y, kEps);
    EXPECT_NEAR(out.eye[2], 3.5 + ego.position.z, kEps);
    EXPECT_NEAR(out.target[0], 0.0 + ego.position.x, kEps);
    EXPECT_NEAR(out.target[1], -2.0 + ego.position.y, kEps);
    EXPECT_NEAR(out.target[2], -0.5 + ego.position.z, kEps);
}

}  // namespace
}  // namespace micropilot::visualization_app
