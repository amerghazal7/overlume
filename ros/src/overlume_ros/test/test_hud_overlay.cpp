// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/hud_overlay.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

TEST(HudOverlay, PopulateHudCopiesEgoSpeedAndActiveMode) {
    overlume::SceneGraph scene{};
    scene.ego.speed_mps = 12.3;
    overlume::ros::PopulateHud(scene, 3);
    EXPECT_DOUBLE_EQ(scene.hud.speed_mps, 12.3);
    EXPECT_EQ(scene.hud.active_mode, 3);
    EXPECT_EQ(scene.hud.chips, nullptr);
    EXPECT_EQ(scene.hud.chip_count, 0u);
}

namespace {
constexpr uint32_t kWidth = 1280;
constexpr uint32_t kHeight = 720;
constexpr uint8_t kBackground = 40;
}

TEST(HudOverlay, CompositesLegibleTextAtLowPreset) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    overlume::ros::HudSnapshot hud{12.3, 3};

    ASSERT_TRUE(overlume::ros::CompositeHud(rgb.data(), kWidth, kHeight, hud, {0.9f, 0.95f, 1.0f},
                                            {0.1f, 1.0f, 0.4f}, 1.0f, OVERLUME_NODE_FONT_PATH));

    constexpr int kRegionX0 = 20, kRegionX1 = 400;
    constexpr int kRegionY0 = 0, kRegionY1 = 100;
    constexpr int kLegibilityThreshold = 20;
    bool found_text_pixel = false;
    for (int y = kRegionY0; y < kRegionY1 && !found_text_pixel; ++y) {
        for (int x = kRegionX0; x < kRegionX1; ++x) {
            const size_t idx = (static_cast<size_t>(y) * kWidth + x) * 3;
            const int diff =
                std::abs(static_cast<int>(rgb[idx]) - static_cast<int>(kBackground)) +
                std::abs(static_cast<int>(rgb[idx + 1]) - static_cast<int>(kBackground)) +
                std::abs(static_cast<int>(rgb[idx + 2]) - static_cast<int>(kBackground));
            if (diff > kLegibilityThreshold) {
                found_text_pixel = true;
                break;
            }
        }
    }
    EXPECT_TRUE(found_text_pixel) << "no pixel in the expected HUD region differs from the "
                                     "uniform background -- text was not drawn";
}

TEST(HudOverlay, MissingOrUnloadableFontIsNonFatalAndLeavesFrameUnchanged) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    overlume::ros::HudSnapshot hud{12.3, 3};

    const bool ok =
        overlume::ros::CompositeHud(rgb.data(), kWidth, kHeight, hud, {0.9f, 0.95f, 1.0f},
                                    {0.1f, 1.0f, 0.4f}, 1.0f, "/nonexistent/does_not_exist.ttf");
    EXPECT_FALSE(ok);
    EXPECT_EQ(rgb, before);
}

TEST(HudOverlay, EmptyFontPathIsNonFatal) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    overlume::ros::HudSnapshot hud{12.3, 3};
    EXPECT_FALSE(overlume::ros::CompositeHud(rgb.data(), kWidth, kHeight, hud, {0, 0, 0}, {0, 0, 0},
                                             1.0f, ""));
    EXPECT_EQ(rgb, before);
}

TEST(HudOverlay, DrawTextAndDrawLineComposeACallout) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);

    overlume::ros::DrawLine(rgb.data(), kWidth, kHeight, 100, 100, 140, 60,
                            overlume::ros::HudRgb{1.0f, 0.2f, 0.2f});
    ASSERT_TRUE(overlume::ros::DrawText(rgb.data(), kWidth, kHeight, "3.2 m", 140, 60,
                                        overlume::ros::HudRgb{1.0f, 0.2f, 0.2f}, 1.0f,
                                        OVERLUME_NODE_FONT_PATH));

    constexpr int kLegibilityThreshold = 20;
    bool found_drawn_pixel = false;
    for (int y = 40; y < 100 && !found_drawn_pixel; ++y) {
        for (int x = 90; x < 300; ++x) {
            const size_t idx = (static_cast<size_t>(y) * kWidth + x) * 3;
            const int diff =
                std::abs(static_cast<int>(rgb[idx]) - static_cast<int>(kBackground)) +
                std::abs(static_cast<int>(rgb[idx + 1]) - static_cast<int>(kBackground)) +
                std::abs(static_cast<int>(rgb[idx + 2]) - static_cast<int>(kBackground));
            if (diff > kLegibilityThreshold) {
                found_drawn_pixel = true;
                break;
            }
        }
    }
    EXPECT_TRUE(found_drawn_pixel) << "neither the leader line nor the chip text drew any pixel";
}

TEST(HudOverlay, DrawTextMissingFontIsNonFatal) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    EXPECT_FALSE(overlume::ros::DrawText(rgb.data(), kWidth, kHeight, "3.2 m", 10, 10,
                                         overlume::ros::HudRgb{1, 1, 1}, 1.0f,
                                         "/nonexistent/does_not_exist.ttf"));
    EXPECT_EQ(rgb, before);
}

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "stb_truetype.h"

namespace {

double luminance(uint8_t r, uint8_t g, uint8_t b) { return 0.2126 * r + 0.7152 * g + 0.0722 * b; }

double block_ssim(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, uint32_t width,
                  uint32_t height) {
    constexpr int kBlock = 8;
    constexpr double kC1 = (0.01 * 255) * (0.01 * 255);
    constexpr double kC2 = (0.03 * 255) * (0.03 * 255);
    double total = 0.0;
    int blockCount = 0;
    for (uint32_t by = 0; by + kBlock <= height; by += kBlock) {
        for (uint32_t bx = 0; bx + kBlock <= width; bx += kBlock) {
            double sumA = 0, sumB = 0, sumAA = 0, sumBB = 0, sumAB = 0;
            const int n = kBlock * kBlock;
            for (int y = 0; y < kBlock; ++y) {
                for (int x = 0; x < kBlock; ++x) {
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

}

TEST(HudOverlayGolden, SyntheticSceneWithHud720pLowPreset) {
    overlume::RenderConfig config{};
    config.width = kWidth;
    config.height = kHeight;
    config.quality = 0;
    config.theme_assets_dir = nullptr;
    config.initial_theme = nullptr;
    overlume::VisualRenderer* r = overlume::create_renderer(config);
    ASSERT_NE(r, nullptr);

    overlume::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;

    overlume::SceneGraph scene{};
    scene.sim_time_sec = 1.0;
    scene.ego.position = {0.0, 0.0, 0.0};
    scene.ego.heading_rad = 0.0;
    scene.ego.speed_mps = 12.3;
    scene.ego.valid = 1;
    overlume::ros::PopulateHud(scene, 3);
    overlume::set_scene(r, scene);

    std::vector<uint8_t> frame(static_cast<size_t>(kWidth) * kHeight * 3, 0);
    overlume::FrameView view{frame.data(), kWidth, kHeight};
    ASSERT_TRUE(overlume::render_frame(r, pose, view));

    const overlume::HudColors colors = overlume::get_hud_colors(r);
    const overlume::ros::HudSnapshot hud{scene.hud.speed_mps, scene.hud.active_mode};
    ASSERT_TRUE(overlume::ros::CompositeHud(
        frame.data(), kWidth, kHeight, hud,
        overlume::ros::HudRgb{colors.text_color[0], colors.text_color[1], colors.text_color[2]},
        overlume::ros::HudRgb{colors.accent_color[0], colors.accent_color[1],
                              colors.accent_color[2]},
        colors.scale, OVERLUME_NODE_FONT_PATH));

    const char* actual_path = "/tmp/hud_overlay_720p_actual.png";
    stbi_write_png(actual_path, static_cast<int>(kWidth), static_cast<int>(kHeight), 3,
                   frame.data(), static_cast<int>(kWidth) * 3);

    const std::string golden_path =
        std::string(OVERLUME_NODE_FIXTURES_DIR) + "/hud_overlay_720p_golden.png";
    int golden_w = 0, golden_h = 0, golden_c = 0;
    uint8_t* golden = stbi_load(golden_path.c_str(), &golden_w, &golden_h, &golden_c, 3);
    double ssim = 0.0;
    if (golden != nullptr && static_cast<uint32_t>(golden_w) == kWidth &&
        static_cast<uint32_t>(golden_h) == kHeight) {
        const std::vector<uint8_t> golden_pixels(
            golden, golden + static_cast<size_t>(golden_w) * golden_h * 3);
        ssim = block_ssim(frame, golden_pixels, kWidth, kHeight);
    }
    if (golden != nullptr) stbi_image_free(golden);

    EXPECT_GT(ssim, 0.98) << "actual frame written to " << actual_path << " -- promote to "
                          << golden_path << " once reviewed";

    overlume::destroy_renderer(r);
}

namespace {

// The pre-Latin-1 renderer, verbatim: an ASCII-only 512 x 512 bake and the old draw loop. DrawText
// must reproduce it byte for byte on ASCII text; that is what keeps the HUD and callout goldens
// where they are.
void draw_ascii_reference(std::vector<uint8_t>& rgb, const std::string& text, float x, float y,
                          float scale, overlume::ros::HudRgb c) {
    constexpr int kDim = 512, kFirst = 32, kCount = 95;
    std::ifstream in(OVERLUME_NODE_FONT_PATH, std::ios::binary);
    const std::vector<unsigned char> ttf((std::istreambuf_iterator<char>(in)),
                                         std::istreambuf_iterator<char>());
    const float pixel_height = 32.0f * (std::round(scale * 10.0f) / 10.0f);
    std::vector<unsigned char> bitmap(static_cast<size_t>(kDim) * kDim, 0);
    std::vector<stbtt_bakedchar> chars(kCount);
    ASSERT_GT(stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, bitmap.data(), kDim, kDim, kFirst,
                                   kCount, chars.data()),
              0);
    for (const unsigned char ch : text) {
        if (ch < kFirst || ch >= kFirst + kCount) {
            x += pixel_height * 0.5f;
            continue;
        }
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(chars.data(), kDim, kDim, ch - kFirst, &x, &y, &q, 1);
        const int gx0 = static_cast<int>(std::floor(q.x0));
        const int gx1 = static_cast<int>(std::ceil(q.x1));
        const int gy0 = static_cast<int>(std::floor(q.y0));
        const int gy1 = static_cast<int>(std::ceil(q.y1));
        if (gx1 <= gx0 || gy1 <= gy0) continue;
        const float u_span = q.s1 - q.s0, v_span = q.t1 - q.t0;
        const float gw = static_cast<float>(gx1 - gx0), gh = static_cast<float>(gy1 - gy0);
        for (int py = gy0; py < gy1; ++py) {
            const float v = q.t0 + ((py - gy0) + 0.5f) / gh * v_span;
            const int ay = static_cast<int>(v * kDim);
            if (ay < 0 || ay >= kDim) continue;
            for (int px = gx0; px < gx1; ++px) {
                const float u = q.s0 + ((px - gx0) + 0.5f) / gw * u_span;
                const int ax = static_cast<int>(u * kDim);
                if (ax < 0 || ax >= kDim) continue;
                const float coverage = bitmap[static_cast<size_t>(ay) * kDim + ax] / 255.0f;
                if (px < 0 || py < 0 || px >= static_cast<int>(kWidth) ||
                    py >= static_cast<int>(kHeight) || coverage <= 0.0f) {
                    continue;
                }
                uint8_t* p = rgb.data() + (static_cast<size_t>(py) * kWidth + px) * 3;
                p[0] = static_cast<uint8_t>(p[0] * (1.0f - coverage) + c.r * 255.0f * coverage);
                p[1] = static_cast<uint8_t>(p[1] * (1.0f - coverage) + c.g * 255.0f * coverage);
                p[2] = static_cast<uint8_t>(p[2] * (1.0f - coverage) + c.b * 255.0f * coverage);
            }
        }
    }
}

}

TEST(HudOverlay, AsciiTextRasterisesExactlyAsTheAsciiOnlyAtlasDid) {
    std::string printable;
    for (char ch = 32; ch < 127; ++ch) printable += ch;
    const std::string lines[] = {printable.substr(0, 24), printable.substr(24, 24),
                                 printable.substr(48, 24), printable.substr(72),
                                 "12.3 m/s\tVISUAL\n3.2 m"};
    const overlume::ros::HudRgb color{0.90f, 0.92f, 0.95f};
    for (const float scale : {1.0f, 1.5f, 2.5f}) {
        std::vector<uint8_t> actual(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
        std::vector<uint8_t> expected = actual;
        for (size_t i = 0; i < 5; ++i) {
            const float y = 90.0f + 130.0f * static_cast<float>(i);
            ASSERT_TRUE(overlume::ros::DrawText(actual.data(), kWidth, kHeight, lines[i].c_str(),
                                                24.0f, y, color, scale, OVERLUME_NODE_FONT_PATH));
            draw_ascii_reference(expected, lines[i], 24.0f, y, scale, color);
        }
        EXPECT_TRUE(actual == expected)
            << "ASCII text rasterised differently at HUD scale " << scale;
    }
}

TEST(HudOverlay, Latin1CopyrightSignDrawsPixels) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    ASSERT_TRUE(overlume::ros::DrawText(rgb.data(), kWidth, kHeight, "\xC2\xA9", 100.0f, 100.0f,
                                        overlume::ros::HudRgb{1.0f, 1.0f, 1.0f}, 1.0f,
                                        OVERLUME_NODE_FONT_PATH));
    EXPECT_TRUE(rgb != before) << "U+00A9 drew nothing";
}

TEST(HudOverlay, MeasureTextDecodesUtf8AndSkipsInvalidBytes) {
    const auto width = [](const std::string& text) {
        return overlume::ros::MeasureText(text.c_str(), 1.0f, OVERLUME_NODE_FONT_PATH);
    };
    const std::string lone_continuation = std::string("A\x80") + "B";
    const std::string truncated_pair = "A\xC2";
    const std::string three_byte_euro = std::string("A\xE2\x82\xAC") + "B";
    const std::string c1_control = std::string("A\xC2\x85") + "B";  // U+0085 has no glyph
    EXPECT_GT(width("\xC2\xA9"), 0.0f);
    EXPECT_FLOAT_EQ(width(lone_continuation), width("AB"));
    EXPECT_FLOAT_EQ(width(truncated_pair), width("A"));
    EXPECT_FLOAT_EQ(width(three_byte_euro), width("AB"));
    EXPECT_NEAR(width(c1_control), width("AB") + 16.0f, 1e-3f);
}

TEST(HudOverlay, MeasureTextMatchesTheDrawnExtent) {
    const char* kText = "Improve this map";
    const float short_width = overlume::ros::MeasureText("Improve", 1.0f, OVERLUME_NODE_FONT_PATH);
    const float width = overlume::ros::MeasureText(kText, 1.0f, OVERLUME_NODE_FONT_PATH);
    EXPECT_GT(short_width, 0.0f);
    EXPECT_GT(width, short_width);
    EXPECT_EQ(overlume::ros::MeasureText(kText, 1.0f, "/nonexistent/does_not_exist.ttf"), 0.0f);

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    constexpr float kX = 100.0f;
    ASSERT_TRUE(overlume::ros::DrawText(rgb.data(), kWidth, kHeight, kText, kX, 100.0f,
                                        overlume::ros::HudRgb{1.0f, 1.0f, 1.0f}, 1.0f,
                                        OVERLUME_NODE_FONT_PATH));
    int left = static_cast<int>(kWidth), right = -1;
    for (uint32_t y = 0; y < kHeight; ++y) {
        for (uint32_t x = 0; x < kWidth; ++x) {
            if (rgb[(static_cast<size_t>(y) * kWidth + x) * 3] == kBackground) continue;
            left = std::min(left, static_cast<int>(x));
            right = std::max(right, static_cast<int>(x));
        }
    }
    EXPECT_NEAR(left, kX, 2.0);
    EXPECT_NEAR(right + 1, kX + width, 2.0);
}

TEST(HudOverlay, DrawImageRgbaBlendsAndIgnoresANullImage) {
    constexpr uint32_t kW = 8, kH = 6;
    std::vector<uint8_t> background(static_cast<size_t>(kW) * kH * 3, 0);
    for (size_t i = 2; i < background.size(); i += 3) background[i] = 255;
    // 3 x 2: an opaque red top row; below it red at alpha 128, 128 and 0.
    const uint8_t image[] = {255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255,
                             255, 0, 0, 128, 255, 0, 0, 128, 255, 0, 0, 0};
    const auto pixel = [](const std::vector<uint8_t>& f, uint32_t x, uint32_t y) {
        const uint8_t* p = &f[(static_cast<size_t>(y) * kW + x) * 3];
        return std::array<int, 3>{p[0], p[1], p[2]};
    };
    const std::array<int, 3> red{255, 0, 0}, blue{0, 0, 255}, half{128, 0, 127};

    std::vector<uint8_t> frame = background;
    overlume::ros::DrawImageRgba(frame.data(), kW, kH, nullptr, 3, 2, 1, 1);
    EXPECT_TRUE(frame == background);

    overlume::ros::DrawImageRgba(frame.data(), kW, kH, image, 3, 2, 1, 1);
    EXPECT_EQ(pixel(frame, 1, 1), red);
    EXPECT_EQ(pixel(frame, 3, 1), red);
    EXPECT_EQ(pixel(frame, 1, 2), half);
    EXPECT_EQ(pixel(frame, 3, 2), blue);
    EXPECT_EQ(pixel(frame, 0, 0), blue);
    EXPECT_EQ(pixel(frame, 4, 1), blue);
}

TEST(HudOverlay, DrawImageRgbaClipsWithoutWrappingOrOverrun) {
    constexpr uint32_t kW = 8, kH = 6;
    // The frame sits between two guard rows of 7s: a missing vertical clip writes into a guard row,
    // and a missing horizontal clip wraps onto the neighbouring row, inside the frame.
    std::vector<uint8_t> buf(static_cast<size_t>(kW) * (kH + 2) * 3, 7);
    uint8_t* frame = buf.data() + kW * 3;
    std::fill(frame, frame + kW * kH * 3, 0);
    const uint8_t red[] = {255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255,
                           255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255};
    overlume::ros::DrawImageRgba(frame, kW, kH, red, 3, 2, kW - 2, 1);  // past the right edge
    overlume::ros::DrawImageRgba(frame, kW, kH, red, 3, 2, -1, 3);      // past the left edge
    overlume::ros::DrawImageRgba(frame, kW, kH, red, 3, 2, 2, -1);      // past the top
    overlume::ros::DrawImageRgba(frame, kW, kH, red, 3, 2, 2, kH - 1);  // past the bottom
    std::vector<uint8_t> expected(buf.size(), 7);
    std::fill(expected.begin() + kW * 3, expected.begin() + kW * (kH + 1) * 3, 0);
    const auto paint = [&](uint32_t x, uint32_t y) { expected[((y + 1) * kW + x) * 3] = 255; };
    for (const uint32_t y : {1u, 2u}) {
        paint(6, y);
        paint(7, y);
    }
    for (const uint32_t y : {3u, 4u}) {
        paint(0, y);
        paint(1, y);
    }
    for (const uint32_t x : {2u, 3u, 4u}) {
        paint(x, 0);
        paint(x, kH - 1);
    }
    EXPECT_TRUE(buf == expected);
}
