// golden.cpp — Epic 1 Task 2 Step 9. See golden.hpp. Not a gtest file
// (excluded from CMakeLists.txt's auto-glob-as-gtest-binary loop by name;
// compiled as a plain extra source into every other test binary instead).
#include "golden.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cstdint>
#include <vector>

namespace mpviz::testing {
namespace {

// This epic's goldens are all committed at a fixed 320x240 (see golden.hpp
// / spec: "purely for CI speed/determinism, not a statement about the
// shipped default").
constexpr uint32_t kWidth = 320;
constexpr uint32_t kHeight = 240;

double luminance(uint8_t r, uint8_t g, uint8_t b) {
    return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

// ponytail: block-wise (8x8, non-overlapping, luminance-only) mean/
// variance/covariance SSIM averaged over blocks -- a deliberately simpler
// approximation of the full windowed-Gaussian SSIM (unnecessary precision
// for a pass/fail regression gate at low-preset resolution).
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

}  // namespace

double render_and_compare(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose,
                           const char* golden_png_path, const char* out_png_path) {
    if (r == nullptr) return -1.0;

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3);
    mpviz::FrameView view{rgb.data(), kWidth, kHeight};
    if (!mpviz::render_frame(r, pose, view)) return 0.0;

    stbi_write_png(out_png_path, static_cast<int>(kWidth), static_cast<int>(kHeight), 3,
                    rgb.data(), static_cast<int>(kWidth) * 3);

    int goldenWidth = 0, goldenHeight = 0, goldenChannels = 0;
    uint8_t* golden = stbi_load(golden_png_path, &goldenWidth, &goldenHeight, &goldenChannels, 3);
    if (golden == nullptr) {
        // Golden missing -- first run of a new golden always fails loudly,
        // never silently "passes" with nothing to compare against.
        return 0.0;
    }
    if (static_cast<uint32_t>(goldenWidth) != kWidth || static_cast<uint32_t>(goldenHeight) != kHeight) {
        stbi_image_free(golden);
        return 0.0;
    }
    const std::vector<uint8_t> goldenPixels(golden, golden + static_cast<size_t>(goldenWidth) *
                                                                 goldenHeight * 3);
    stbi_image_free(golden);

    return block_ssim(rgb, goldenPixels, kWidth, kHeight);
}

FrameStats analyze_png(const char* png_path) {
    int width = 0, height = 0, channels = 0;
    uint8_t* img = stbi_load(png_path, &width, &height, &channels, 3);
    if (img == nullptr) return {};

    FrameStats stats{};
    const size_t n = static_cast<size_t>(width) * height;
    bool seenLevel[256] = {};
    double topSum = 0.0, bottomSum = 0.0;
    size_t topN = 0, bottomN = 0;
    const int topEnd = height / 3;
    const int bottomStart = (2 * height) / 3;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t idx = (static_cast<size_t>(y) * width + x) * 3;
            const double l = luminance(img[idx], img[idx + 1], img[idx + 2]);
            stats.mean += l;
            const int level = static_cast<int>(l + 0.5);
            if (level >= 0 && level < 256) seenLevel[level] = true;
            if (y < topEnd) {
                topSum += l;
                ++topN;
            } else if (y >= bottomStart) {
                bottomSum += l;
                ++bottomN;
            }
        }
    }
    stbi_image_free(img);

    stats.mean /= static_cast<double>(n);
    for (bool seen : seenLevel) stats.distinct_levels += seen ? 1 : 0;
    stats.top_third_mean = topN > 0 ? topSum / static_cast<double>(topN) : 0.0;
    stats.bottom_third_mean = bottomN > 0 ? bottomSum / static_cast<double>(bottomN) : 0.0;
    return stats;
}

}  // namespace mpviz::testing
