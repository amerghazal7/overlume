/** @file test_hud_overlay.cpp
 *  @brief Epic 3 Task 3 (VM-030) tests: Step 0's scene.hud population,
 *  Step 3's CompositeHud() presence/non-fatal-failure behavior, and the
 *  sanctioned-red node-side 720p HUD golden (see this file's own comment on
 *  HudOverlayGolden below).
 */
#include "micropilot_visualization_node/hud_overlay.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

// ── Step 0 ───────────────────────────────────────────────────────────────────
TEST(HudOverlay, PopulateHudCopiesEgoSpeedAndActiveMode)
{
    mpviz::SceneGraph scene{};
    scene.ego.speed_mps = 12.3;
    mpviz_node::PopulateHud(scene, /*active_mode=*/3);
    EXPECT_DOUBLE_EQ(scene.hud.speed_mps, 12.3);
    EXPECT_EQ(scene.hud.active_mode, 3);
    // chips/chip_count untouched -- Task 4 (VM-031) scope, not this one's.
    EXPECT_EQ(scene.hud.chips, nullptr);
    EXPECT_EQ(scene.hud.chip_count, 0u);
}

// ── Step 3 ───────────────────────────────────────────────────────────────────
namespace
{
constexpr uint32_t kWidth = 1280;
constexpr uint32_t kHeight = 720;
constexpr uint8_t kBackground = 40;  // known mid-gray frame
}  // namespace

TEST(HudOverlay, CompositesLegibleTextAtLowPreset)
{
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    mpviz_node::HudSnapshot hud{/*speed_mps=*/12.3, /*active_mode=*/3};

    ASSERT_TRUE(mpviz_node::CompositeHud(rgb.data(), kWidth, kHeight, hud,
                                          /*text_rgb=*/{0.9f, 0.95f, 1.0f},
                                          /*accent_rgb=*/{0.1f, 1.0f, 0.4f},
                                          /*scale=*/1.0f, MPVIZ_NODE_FONT_PATH));

    // Presence check, not a pixel-exact glyph golden (font rasterization is
    // deterministic per stb_truetype version, but this test shouldn't need
    // to pin exact glyph bitmaps) -- some pixel in the region the two HUD
    // lines are drawn into (hud_overlay.cpp's own kMarginX/kMarginY/
    // kLineGap) must differ from the uniform background by more than a
    // legibility threshold.
    constexpr int kRegionX0 = 20, kRegionX1 = 400;
    constexpr int kRegionY0 = 0, kRegionY1 = 100;
    constexpr int kLegibilityThreshold = 20;  // out of 255
    bool found_text_pixel = false;
    for (int y = kRegionY0; y < kRegionY1 && !found_text_pixel; ++y)
    {
        for (int x = kRegionX0; x < kRegionX1; ++x)
        {
            const size_t idx = (static_cast<size_t>(y) * kWidth + x) * 3;
            const int diff = std::abs(static_cast<int>(rgb[idx]) - static_cast<int>(kBackground)) +
                              std::abs(static_cast<int>(rgb[idx + 1]) - static_cast<int>(kBackground)) +
                              std::abs(static_cast<int>(rgb[idx + 2]) - static_cast<int>(kBackground));
            if (diff > kLegibilityThreshold)
            {
                found_text_pixel = true;
                break;
            }
        }
    }
    EXPECT_TRUE(found_text_pixel) << "no pixel in the expected HUD region differs from the "
                                      "uniform background -- text was not drawn";
}

TEST(HudOverlay, MissingOrUnloadableFontIsNonFatalAndLeavesFrameUnchanged)
{
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    mpviz_node::HudSnapshot hud{/*speed_mps=*/12.3, /*active_mode=*/3};

    // spec §9 "asset load failure -> non-fatal, WARN once" -- same
    // philosophy as set_ego_model's clay-box fallback, applied here as "no
    // HUD drawn, not a crash, not garbage pixels."
    const bool ok = mpviz_node::CompositeHud(rgb.data(), kWidth, kHeight, hud,
                                              /*text_rgb=*/{0.9f, 0.95f, 1.0f},
                                              /*accent_rgb=*/{0.1f, 1.0f, 0.4f},
                                              /*scale=*/1.0f, "/nonexistent/does_not_exist.ttf");
    EXPECT_FALSE(ok);
    EXPECT_EQ(rgb, before);
}

TEST(HudOverlay, EmptyFontPathIsNonFatal)
{
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    mpviz_node::HudSnapshot hud{12.3, 3};
    EXPECT_FALSE(mpviz_node::CompositeHud(rgb.data(), kWidth, kHeight, hud, {0, 0, 0}, {0, 0, 0},
                                           1.0f, ""));
    EXPECT_EQ(rgb, before);
}

// ── Task 4 (VM-031): primitives exposed for callouts.cpp ────────────────────
TEST(HudOverlay, DrawTextAndDrawLineComposeACallout)
{
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);

    mpviz_node::DrawLine(rgb.data(), kWidth, kHeight, /*x0=*/100, /*y0=*/100, /*x1=*/140,
                          /*y1=*/60, mpviz_node::HudRgb{1.0f, 0.2f, 0.2f});
    ASSERT_TRUE(mpviz_node::DrawText(rgb.data(), kWidth, kHeight, "3.2 m", /*x=*/140, /*y=*/60,
                                      mpviz_node::HudRgb{1.0f, 0.2f, 0.2f}, /*scale=*/1.0f,
                                      MPVIZ_NODE_FONT_PATH));

    // One presence check over the whole region the line+text pair was drawn
    // into -- same legibility-threshold shape as CompositesLegibleTextAtLowPreset
    // above, not a pixel-exact golden.
    constexpr int kLegibilityThreshold = 20;
    bool found_drawn_pixel = false;
    for (int y = 40; y < 100 && !found_drawn_pixel; ++y)
    {
        for (int x = 90; x < 300; ++x)
        {
            const size_t idx = (static_cast<size_t>(y) * kWidth + x) * 3;
            const int diff = std::abs(static_cast<int>(rgb[idx]) - static_cast<int>(kBackground)) +
                              std::abs(static_cast<int>(rgb[idx + 1]) - static_cast<int>(kBackground)) +
                              std::abs(static_cast<int>(rgb[idx + 2]) - static_cast<int>(kBackground));
            if (diff > kLegibilityThreshold)
            {
                found_drawn_pixel = true;
                break;
            }
        }
    }
    EXPECT_TRUE(found_drawn_pixel) << "neither the leader line nor the chip text drew any pixel";
}

TEST(HudOverlay, DrawTextMissingFontIsNonFatal)
{
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    EXPECT_FALSE(mpviz_node::DrawText(rgb.data(), kWidth, kHeight, "3.2 m", 10, 10,
                                       mpviz_node::HudRgb{1, 1, 1}, 1.0f,
                                       "/nonexistent/does_not_exist.ttf"));
    EXPECT_EQ(rgb, before);
}

// ── Sanctioned-red node-side 720p HUD golden ────────────────────────────────
// Renders a real (headless-EGL, low-preset) frame from a small synthetic
// scene, composites the HUD on top, writes it to
// /tmp/hud_overlay_720p_actual.png, and SSIMs it against a golden this task
// deliberately does NOT commit -- same "first run of a new golden always
// fails loudly, never silently passes" convention as the library's own
// mpviz::testing::render_and_compare (visual_renderer/tests/golden.cpp).
// UNPROMOTED, sanctioned red: the user promotes the actual PNG into
// test/fixtures/hud_overlay_720p_golden.png once satisfied with it.
//
// The stb_image/stb_image_write use below is this test's own vendored copy
// (Step 1's CMakeLists.txt comment) -- NOT visual_renderer's, a different
// build tree entirely -- pinned to the same upstream commit so the two are
// byte-identical.
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace
{

double luminance(uint8_t r, uint8_t g, uint8_t b)
{
    return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

// ponytail: block-wise (8x8, non-overlapping, luminance-only) mean/
// variance/covariance SSIM, same deliberately-simplified approximation as
// visual_renderer/tests/golden.cpp's own block_ssim (not shared code --
// this build tree can't reach that one, see this file's stb comment above
// -- reimplemented here at the same small size rather than promoted to a
// shared header neither toolchain can safely include from the other).
double block_ssim(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, uint32_t width,
                   uint32_t height)
{
    constexpr int kBlock = 8;
    constexpr double kC1 = (0.01 * 255) * (0.01 * 255);
    constexpr double kC2 = (0.03 * 255) * (0.03 * 255);
    double total = 0.0;
    int blockCount = 0;
    for (uint32_t by = 0; by + kBlock <= height; by += kBlock)
    {
        for (uint32_t bx = 0; bx + kBlock <= width; bx += kBlock)
        {
            double sumA = 0, sumB = 0, sumAA = 0, sumBB = 0, sumAB = 0;
            const int n = kBlock * kBlock;
            for (int y = 0; y < kBlock; ++y)
            {
                for (int x = 0; x < kBlock; ++x)
                {
                    const uint32_t px = bx + x, py = by + y;
                    const size_t idx = (static_cast<size_t>(py) * width + px) * 3;
                    const double la = luminance(a[idx], a[idx + 1], a[idx + 2]);
                    const double lb = luminance(b[idx], b[idx + 1], b[idx + 2]);
                    sumA += la;
                    sumB += lb;
                    sumAA += la * la;
                    sumBB += lb * lb;
                    sumAB += la * lb;
                }
            }
            const double meanA = sumA / n, meanB = sumB / n;
            const double varA = sumAA / n - meanA * meanA;
            const double varB = sumBB / n - meanB * meanB;
            const double covAB = sumAB / n - meanA * meanB;
            const double ssim = ((2 * meanA * meanB + kC1) * (2 * covAB + kC2)) /
                                 ((meanA * meanA + meanB * meanB + kC1) * (varA + varB + kC2));
            total += ssim;
            ++blockCount;
        }
    }
    return blockCount > 0 ? total / blockCount : 0.0;
}

}  // namespace

TEST(HudOverlayGolden, SyntheticSceneWithHud720pLowPreset)
{
    mpviz::RenderConfig config{};
    config.width = kWidth;
    config.height = kHeight;
    config.quality = 0;  // low preset -- spec AC "text legible at 720p low preset"
    config.theme_assets_dir = nullptr;   // compiled-in fallback theme -- no asset dir needed
    config.initial_theme = nullptr;
    mpviz::VisualRenderer* r = mpviz::create_renderer(config);
    ASSERT_NE(r, nullptr);

    mpviz::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;

    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 1.0;
    scene.ego.position = {0.0, 0.0, 0.0};
    scene.ego.heading_rad = 0.0;
    scene.ego.speed_mps = 12.3;
    scene.ego.valid = 1;
    mpviz_node::PopulateHud(scene, /*active_mode=*/3);
    mpviz::set_scene(r, scene);

    std::vector<uint8_t> frame(static_cast<size_t>(kWidth) * kHeight * 3, 0);
    mpviz::FrameView view{frame.data(), kWidth, kHeight};
    ASSERT_TRUE(mpviz::render_frame(r, pose, view));

    const mpviz::HudColors colors = mpviz::get_hud_colors(r);
    const mpviz_node::HudSnapshot hud{scene.hud.speed_mps, scene.hud.active_mode};
    ASSERT_TRUE(mpviz_node::CompositeHud(
        frame.data(), kWidth, kHeight, hud,
        mpviz_node::HudRgb{colors.text_color[0], colors.text_color[1], colors.text_color[2]},
        mpviz_node::HudRgb{colors.accent_color[0], colors.accent_color[1], colors.accent_color[2]},
        colors.scale, MPVIZ_NODE_FONT_PATH));

    const char* actual_path = "/tmp/hud_overlay_720p_actual.png";
    stbi_write_png(actual_path, static_cast<int>(kWidth), static_cast<int>(kHeight), 3,
                   frame.data(), static_cast<int>(kWidth) * 3);

    const std::string golden_path = std::string(MPVIZ_NODE_FIXTURES_DIR) + "/hud_overlay_720p_golden.png";
    int golden_w = 0, golden_h = 0, golden_c = 0;
    uint8_t* golden = stbi_load(golden_path.c_str(), &golden_w, &golden_h, &golden_c, 3);
    double ssim = 0.0;  // no committed golden yet -- same "missing golden -> 0.0" convention
                         // as visual_renderer/tests/golden.cpp's render_and_compare
    if (golden != nullptr && static_cast<uint32_t>(golden_w) == kWidth &&
        static_cast<uint32_t>(golden_h) == kHeight)
    {
        const std::vector<uint8_t> golden_pixels(
            golden, golden + static_cast<size_t>(golden_w) * golden_h * 3);
        ssim = block_ssim(frame, golden_pixels, kWidth, kHeight);
    }
    if (golden != nullptr) stbi_image_free(golden);

    // SANCTIONED RED (plan's own instruction): unpromoted until a human
    // looks at actual_path and copies it to golden_path.
    EXPECT_GT(ssim, 0.98) << "actual frame written to " << actual_path
                          << " -- promote to " << golden_path << " once reviewed";

    mpviz::destroy_renderer(r);
}
