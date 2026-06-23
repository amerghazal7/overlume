/**
 * @file test_golden.cpp
 * @brief C++ golden test for micropilot::rendering::Reprojector.
 *
 * Reads the pre-generated fixture from cuda/tests/golden/ (path injected via
 * GOLDEN_DIR compile definition), builds a Reprojector, calls render_bowl with
 * identical inputs, and asserts PSNR > 40 dB vs the NumpyRenderer reference.
 *
 * The fixture is produced by:
 *   python3 cuda/tools/export_golden.py
 *
 * This test proves that the library is usable from pure C++ with no Python,
 * no GL, and no display — all it needs is librendering_reprojector.so + CUDA.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rendering_reprojector/reprojector.hpp"
#include "rendering_reprojector/types.hpp"

// GOLDEN_DIR is injected via CMake target_compile_definitions.
#ifndef GOLDEN_DIR
#error "GOLDEN_DIR not set — define it in CMakeLists.txt"
#endif

using micropilot::rendering::BowlParams;
using micropilot::rendering::CameraParams;
using micropilot::rendering::Reprojector;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static std::string golden_path(const char* name)
{
    return std::string(GOLDEN_DIR) + "/" + name;
}

struct Manifest
{
    int ncam  = 0;
    int cam_w = 0;
    int cam_h = 0;
    int out_w = 0;
    int out_h = 0;
    float R0   = 0.f;
    float k    = 0.f;
    float Rmax = 0.f;
};

static Manifest read_manifest()
{
    std::ifstream f(golden_path("manifest.txt"));
    if (!f.is_open())
        throw std::runtime_error("Cannot open manifest.txt — did you run export_golden.py?");
    Manifest m;
    std::string key;
    while (f >> key)
    {
        if      (key == "ncam" ) f >> m.ncam;
        else if (key == "cam_w") f >> m.cam_w;
        else if (key == "cam_h") f >> m.cam_h;
        else if (key == "out_w") f >> m.out_w;
        else if (key == "out_h") f >> m.out_h;
        else if (key == "R0"   ) f >> m.R0;
        else if (key == "k"    ) f >> m.k;
        else if (key == "Rmax" ) f >> m.Rmax;
    }
    return m;
}

/** Parse one camera line: 9 K + 9 R + 3 t + w + h (22 fields). */
static CameraParams parse_cam_line(const std::string& line)
{
    std::istringstream ss(line);
    CameraParams cam{};
    for (int i = 0; i < 9; ++i) ss >> cam.K[i];
    for (int i = 0; i < 9; ++i) ss >> cam.R[i];
    for (int i = 0; i < 3; ++i) ss >> cam.t[i];
    ss >> cam.width >> cam.height;
    return cam;
}

static std::vector<CameraParams> read_cameras(const char* filename)
{
    std::ifstream f(golden_path(filename));
    if (!f.is_open())
        throw std::runtime_error(std::string("Cannot open ") + filename);
    std::vector<CameraParams> cams;
    std::string line;
    while (std::getline(f, line))
    {
        if (!line.empty())
            cams.push_back(parse_cam_line(line));
    }
    return cams;
}

/** Read a raw float32 binary file into a host vector. */
static std::vector<float> read_bin(const char* filename)
{
    std::ifstream f(golden_path(filename), std::ios::binary);
    if (!f.is_open())
        throw std::runtime_error(std::string("Cannot open ") + filename);
    f.seekg(0, std::ios::end);
    size_t n_bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<float> buf(n_bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(n_bytes));
    return buf;
}

/** PSNR assuming pixel values in [0,1]. Returns 99.0 if MSE < 1e-12. */
static double psnr(const float* a, const float* b, size_t n)
{
    double mse = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        double diff = static_cast<double>(a[i]) - static_cast<double>(b[i]);
        mse += diff * diff;
    }
    mse /= static_cast<double>(n);
    if (mse < 1e-12) return 99.0;
    return 10.0 * std::log10(1.0 / mse);
}

// ---------------------------------------------------------------------------
// Golden test
// ---------------------------------------------------------------------------
TEST(GoldenBowl, PSNRAbove40dB)
{
    // ---- check fixture exists ----
    {
        std::ifstream probe(golden_path("manifest.txt"));
        if (!probe.is_open())
        {
            GTEST_SKIP() << "Golden fixture not found at " << GOLDEN_DIR
                         << " — run python3 cuda/tools/export_golden.py first.";
        }
    }

    Manifest m = read_manifest();
    ASSERT_GT(m.ncam,  0) << "manifest parse failed";
    ASSERT_GT(m.cam_w, 0);
    ASSERT_GT(m.cam_h, 0);
    ASSERT_GT(m.out_w, 0);
    ASSERT_GT(m.out_h, 0);

    // ---- load cameras ----
    std::vector<CameraParams> cams = read_cameras("cameras.txt");
    ASSERT_EQ(static_cast<int>(cams.size()), m.ncam);

    std::vector<CameraParams> vcams = read_cameras("vcam.txt");
    ASSERT_EQ(vcams.size(), size_t(1));
    CameraParams vcam = vcams[0];

    // ---- load images ----
    // images.bin: (N, cam_h, cam_w, 3) float32 row-major
    std::vector<float> images = read_bin("images.bin");
    size_t expected_img_elems = static_cast<size_t>(m.ncam) * m.cam_h * m.cam_w * 3;
    ASSERT_EQ(images.size(), expected_img_elems)
        << "images.bin size mismatch: got " << images.size()
        << " expected " << expected_img_elems;

    // ---- load golden reference ----
    // golden_bowl.bin: (out_h, out_w, 3) float32 row-major
    std::vector<float> golden = read_bin("golden_bowl.bin");
    size_t expected_out_elems = static_cast<size_t>(m.out_h) * m.out_w * 3;
    ASSERT_EQ(golden.size(), expected_out_elems)
        << "golden_bowl.bin size mismatch";

    // ---- build + render ----
    Reprojector rep(m.out_w, m.out_h);
    rep.set_cameras(cams);
    rep.upload_images(images.data(), m.ncam, m.cam_h, m.cam_w);

    BowlParams bowl{m.R0, m.k, m.Rmax};
    // Output is (out_h, out_w, 4) RGBA
    std::vector<float> out(static_cast<size_t>(m.out_h) * m.out_w * 4, 0.f);
    rep.render_bowl(vcam, bowl, out.data());

    // ---- extract RGB and valid mask ----
    const size_t n_pix = static_cast<size_t>(m.out_h) * m.out_w;
    std::vector<float> cuda_rgb(n_pix * 3);
    std::vector<bool>  cuda_valid(n_pix);
    for (size_t i = 0; i < n_pix; ++i)
    {
        cuda_rgb[i * 3 + 0] = out[i * 4 + 0];
        cuda_rgb[i * 3 + 1] = out[i * 4 + 1];
        cuda_rgb[i * 3 + 2] = out[i * 4 + 2];
        cuda_valid[i] = (out[i * 4 + 3] > 0.5f);
    }

    // ---- determine jointly-valid pixels ----
    // NumpyRenderer fills invalid pixels with 0; build a mask from the golden.
    // We replicate the Python parity check: mask = alpha > 0.5 for CUDA side.
    // For the golden side we treat pixels as valid when any channel is non-zero
    // OR when the alpha from the CUDA output is set (both must agree).
    // To mirror test_cuda_parity exactly: compute PSNR over the CUDA-valid mask.
    std::vector<float> cuda_rgb_valid, golden_rgb_valid;
    cuda_rgb_valid.reserve(n_pix * 3);
    golden_rgb_valid.reserve(n_pix * 3);
    for (size_t i = 0; i < n_pix; ++i)
    {
        if (cuda_valid[i])
        {
            cuda_rgb_valid.push_back(cuda_rgb[i * 3 + 0]);
            cuda_rgb_valid.push_back(cuda_rgb[i * 3 + 1]);
            cuda_rgb_valid.push_back(cuda_rgb[i * 3 + 2]);
            golden_rgb_valid.push_back(golden[i * 3 + 0]);
            golden_rgb_valid.push_back(golden[i * 3 + 1]);
            golden_rgb_valid.push_back(golden[i * 3 + 2]);
        }
    }

    ASSERT_GT(cuda_rgb_valid.size(), size_t(0)) << "No valid pixels — rendering failed";

    double p = psnr(cuda_rgb_valid.data(), golden_rgb_valid.data(),
                    cuda_rgb_valid.size());

    // Diagnostic info even on pass
    std::printf("[GoldenBowl] valid pixels: %zu / %zu  PSNR: %.2f dB\n",
                cuda_rgb_valid.size() / 3, n_pix, p);

    EXPECT_GT(p, 40.0) << "PSNR " << p << " dB < 40 dB threshold. "
        "The C++ reprojection output does not match the NumpyRenderer reference.";
}
