#include <gtest/gtest.h>

#include <vector>

#include "rendering_reprojector/reprojector.hpp"

using micropilot::rendering::BowlParams;
using micropilot::rendering::CameraParams;
using micropilot::rendering::Reprojector;

TEST(Reprojector, ConstructsAndRendersBufferOfCorrectSize)
{
    Reprojector r(8, 6);
    std::vector<float> out(8 * 6 * 4, -1.0f);
    BowlParams bowl{6.0f, 0.08f, 20.0f};
    CameraParams v{};
    v.width = 8;
    v.height = 6;
    r.render_bowl(v, bowl, out.data());
    // skeleton zeroes the buffer; just assert it wrote all elements
    for (float val : out) EXPECT_EQ(val, 0.0f);
}
