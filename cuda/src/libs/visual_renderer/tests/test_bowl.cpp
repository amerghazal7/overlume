// test_bowl.cpp — VM-091 (unified-engine migration Task 2).
//
// Step 3: bowl_mesh.hpp's CPU-only mesh generation + per-vertex
// weight/camera-slot-index bake, in isolation -- no Filament/GPU involved.
// Step 4 (Filament sentinel-pixel render test) lives in this same file per
// the plan's Files list, guarded by HasGpuEglDevice()/GTEST_SKIP() same as
// every other GPU test in this suite.
#include "bowl_mesh.hpp"
#include "bowl_projection.hpp"

#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "test_paths.hpp"

#include <cmath>
#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>

namespace {

using mpviz::CameraExtrinsics;
using mpviz::CameraIntrinsics;
namespace bowl = mpviz::bowl;

// Overhead camera, directly above the bowl looking straight down (fwd =
// -Z), TIGHT field of view (fx/fy large) so only vertices close to the
// bowl's center (small radial extent) land within the image bounds --
// everything else is "in front of the camera" but outside its FOV, giving
// a clean covered/uncovered split driven by radius alone (not camera
// facing, so it needs no elaborate rig geometry to reason about).
CameraExtrinsics OverheadCamera() {
    // col0=right=(1,0,0), col1=down=(0,-1,0), col2=fwd=(0,0,-1); R is
    // row-major, column j = (R[j],R[3+j],R[6+j]) (bowl_projection.cpp's own
    // convention, mirroring reproject.cu's CamDev).
    return CameraExtrinsics{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
}

CameraIntrinsics TightFovIntrinsics() {
    return CameraIntrinsics{5000, 5000, 320, 240, {0, 0, 0, 0, 0}};
}

}  // namespace

// ---- Step 3: CPU-only bake -------------------------------------------

TEST(BowlMeshBake, InnerRingCoveredOuterRingUncoveredBySingleCamera) {
    const CameraExtrinsics ext = OverheadCamera();
    const CameraIntrinsics in = TightFovIntrinsics();
    const uint32_t w = 640, h = 480;

    bowl::BowlMeshParams params;
    params.theta_segments = 16;
    params.radial_rings = 8;
    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, /*R0=*/0.5, /*k=*/0.3, /*Rmax=*/8.0,
                           /*camera_count=*/1, &ext, &in, &w, &h, /*feather_margin=*/5.0);

    ASSERT_FALSE(mesh.vertices.empty());

    bool found_covered = false, found_uncovered = false;
    double max_r = 0.0;
    for (const auto& v : mesh.vertices) {
        const double r = std::sqrt(v.position.x * v.position.x + v.position.y * v.position.y);
        max_r = std::max(max_r, r);
    }
    for (const auto& v : mesh.vertices) {
        const double r = std::sqrt(v.position.x * v.position.x + v.position.y * v.position.y);
        // Only one camera is configured -- every vertex's index_a must be
        // camera 0, and weight_b must be forced to 0 (never double-counts
        // the single camera as both slots).
        EXPECT_EQ(v.index_a, 0u);
        EXPECT_FLOAT_EQ(v.weight_b, 0.0f);
        if (r < 1.0 && v.weight_a > 0.0f) found_covered = true;
        if (r > max_r * 0.9 && v.weight_a == 0.0f) found_uncovered = true;
    }
    EXPECT_TRUE(found_covered) << "a vertex near the bowl center should be inside the tight FOV";
    EXPECT_TRUE(found_uncovered)
        << "a vertex near Rmax should fall outside the tight FOV (zero weight)";
}

TEST(BowlMeshBake, MidEdgeWeightInterpolationIsBoundedByTessellation) {
    // Fine tessellation -> the rasterizer's linear interpolation of two
    // baked vertices' weight floats should closely reproduce an
    // independent weight evaluation at their true geometric midpoint --
    // the property bit-packing (banned by Decision 3) could never have.
    const CameraExtrinsics ext = OverheadCamera();
    const CameraIntrinsics in = TightFovIntrinsics();
    const uint32_t w = 640, h = 480;

    bowl::BowlMeshParams params;
    params.theta_segments = 128;
    params.radial_rings = 64;
    const bowl::BowlMesh mesh = bowl::BakeBowlMesh(params, 0.5, 0.3, 8.0, 1, &ext, &in, &w, &h,
                                                    /*feather_margin=*/5.0);

    // Find a triangle whose first two vertices are both covered by camera
    // 0 with a meaningfully nonzero weight (well inside the FOV, away from
    // the feather falloff edge, so the interpolation isn't dominated by
    // the smoothstep's own curvature at this tessellation density).
    bool checked = false;
    for (size_t t = 0; t + 2 < mesh.indices.size() && !checked; t += 3) {
        const auto& v0 = mesh.vertices[mesh.indices[t]];
        const auto& v1 = mesh.vertices[mesh.indices[t + 1]];
        if (v0.weight_a < 0.3f || v1.weight_a < 0.3f) continue;

        const mpviz::Vec3 mid{(v0.position.x + v1.position.x) * 0.5,
                               (v0.position.y + v1.position.y) * 0.5,
                               (v0.position.z + v1.position.z) * 0.5};
        float u, v;
        ASSERT_TRUE(bowl::ProjectToCameraUv(ext, in, w, h, mid, &u, &v));
        const float xp = u * w, yp = v * h;
        const float expected =
            bowl::BorderFeather(xp, yp, w, h, 5.0) * bowl::CameraAlignment(ext, mid) *
            bowl::CameraAlignment(ext, mid);
        const float interpolated = (v0.weight_a + v1.weight_a) * 0.5f;
        EXPECT_NEAR(interpolated, expected, 0.05f);
        checked = true;
    }
    ASSERT_TRUE(checked) << "no well-covered adjacent-vertex pair found to check";
}

TEST(BowlMeshBake, EveryTriangleCarriesIdenticalCameraIndexPairAcrossAllThreeVertices) {
    // Decision 3's construction rule, checked as a whole-mesh invariant: no
    // triangle may have vertices that disagree on (index_a, index_b) --
    // that's exactly the "index interpolated into fractional garbage"
    // corruption class the rule exists to prevent. Two cameras with
    // different placements (splitting bowl coverage) exercise this at a
    // real seam, not just trivially with one camera.
    const CameraExtrinsics extA{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    const CameraExtrinsics extB{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, -10.0}};  // faces the opposite way
    const CameraExtrinsics exts[2] = {extA, extB};
    const CameraIntrinsics in{800, 800, 320, 240, {0, 0, 0, 0, 0}};
    const CameraIntrinsics ins[2] = {in, in};
    const uint32_t widths[2] = {640, 640};
    const uint32_t heights[2] = {480, 480};

    bowl::BowlMeshParams params;
    params.theta_segments = 32;
    params.radial_rings = 12;
    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, 0.5, 0.3, 8.0, 2, exts, ins, widths, heights, 5.0);

    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const auto& v0 = mesh.vertices[mesh.indices[t]];
        const auto& v1 = mesh.vertices[mesh.indices[t + 1]];
        const auto& v2 = mesh.vertices[mesh.indices[t + 2]];
        EXPECT_EQ(v0.index_a, v1.index_a);
        EXPECT_EQ(v0.index_a, v2.index_a);
        EXPECT_EQ(v0.index_b, v1.index_b);
        EXPECT_EQ(v0.index_b, v2.index_b);
    }
}

// ---- Step 4: set_bowl_config() + set_camera_frame() + render_frame() -----

TEST(Bowl, RenderFrameWithBowlConfiguredProducesSentinelPixels) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    // Angled view of the bowl (NOT straight-down -- an eye directly above
    // the target with a world-up vector is a degenerate lookAt, forward and
    // up parallel; every existing render_once() test in this suite uses an
    // angled eye for the same reason).
    mpviz::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // One overhead camera, wide enough FOV to see the whole (small) bowl.
    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    mpviz::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    mpviz::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 5.0;
    bc.sky_color[0] = 0.1f;
    bc.sky_color[1] = 0.1f;
    bc.sky_color[2] = 0.1f;
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));
    // set_bowl_config() defaults to hidden (bowlVisible=false, matching
    // set_bowl_visible's own documented default and Task 4's later
    // per-mode dispatch) -- this test explicitly opts in, same as Task 4's
    // mode dispatch will do for BOWL/HYBRID modes.
    ASSERT_TRUE(mpviz::set_bowl_visible(r, true));

    // Saturated magenta sentinel: no theme emits it on all three channels
    // at once, so "the bowl sampled the camera texture" is distinguishable
    // from "the scene happens to contain a similar color".
    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < cam_pixels.size(); i += 3) {
        cam_pixels[i] = 255;
        cam_pixels[i + 1] = 0;
        cam_pixels[i + 2] = 255;
    }
    ASSERT_TRUE(mpviz::set_camera_frame(r, 0, cam_pixels.data(), w, h, /*frame_id=*/1));

    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {buf.data(), 320, 240}));

    // Magenta-signature check, not an exact (255,0,255) proximity match:
    // this renderer's fixed camera exposure (renderer.cpp's setExposure)
    // tonemaps UNLIT baseColor nonlinearly (measured empirically -- a raw
    // unlit (1,1,1) renders at only ~15% output brightness with no lighting
    // to compensate the way clay.mat's albedo relies on the sun), so a
    // sampled-and-EXPOSURE_COMPENSATION-boosted magenta pixel does not land
    // tightly at (255,0,255); what it DOES do reliably is push R and B well
    // above anything the "dark_adas" theme's own background/sky/ground ever
    // produces (measured background max component ~90) while G stays
    // comparatively low. R>150 AND B>150 is therefore a robust "the bowl
    // sampled its camera texture" fingerprint no scene content could
    // accidentally trigger, without depending on this renderer's exact
    // tonemap curve.
    size_t magenta_pixels = 0;
    for (size_t i = 0; i < buf.size(); i += 3) {
        if (buf[i] > 150 && buf[i + 2] > 150) ++magenta_pixels;
    }
    // Measured coverage for this exact camera/bowl config is ~1600 pixels
    // (~2% of the 320x240 frame) -- the bowl's real on-screen footprint is
    // modest at this FOV/Rmax, not the whole frame. 200 is a wide margin
    // below that measured value while still being far too many to explain
    // by noise/antialiasing at a mesh edge.
    EXPECT_GT(magenta_pixels, 200u)
        << "expected a meaningful magenta-tinted (bowl-sampled) pixel count";
    mpviz::destroy_renderer(r);
}

TEST(Bowl, SetBowlVisibleFalseHidesTheBowlEntirely) {
    // Inverted sanity check for the same setup: with bowlVisible left
    // false (the default), no sentinel pixels should appear at all.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    mpviz::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    mpviz::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    mpviz::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 5.0;
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < cam_pixels.size(); i += 3) {
        cam_pixels[i] = 255;
        cam_pixels[i + 1] = 0;
        cam_pixels[i + 2] = 255;
    }
    ASSERT_TRUE(mpviz::set_camera_frame(r, 0, cam_pixels.data(), w, h, 1));

    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {buf.data(), 320, 240}));
    // Same magenta-signature criterion as the companion test above -- with
    // the bowl hidden, none of it should fire at all.
    size_t magenta_pixels = 0;
    for (size_t i = 0; i < buf.size(); i += 3) {
        if (buf[i] > 150 && buf[i + 2] > 150) ++magenta_pixels;
    }
    EXPECT_EQ(magenta_pixels, 0u) << "bowl rendered while bowlVisible defaulted false";
    mpviz::destroy_renderer(r);
}
