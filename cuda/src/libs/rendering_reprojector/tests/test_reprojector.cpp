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
    // Task 2 scaffolding: fill_kernel writes constant RGBA (0.1, 0.2, 0.3, 1.0) per pixel.
    // Task 3 will update this test when the real reprojection kernel replaces fill.
    for (int i = 0; i < 8 * 6; ++i)
    {
        EXPECT_NEAR(out[i * 4 + 0], 0.1f, 1e-4f);
        EXPECT_NEAR(out[i * 4 + 1], 0.2f, 1e-4f);
        EXPECT_NEAR(out[i * 4 + 2], 0.3f, 1e-4f);
        EXPECT_NEAR(out[i * 4 + 3], 1.0f, 1e-4f);
    }
}
