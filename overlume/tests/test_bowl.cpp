// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "bowl_mesh.hpp"
#include "bowl_projection.hpp"

#include "overlume/api.h"
#include "overlume/scene.h"

#include "test_paths.hpp"

#include <cmath>
#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>

namespace {

using overlume::CameraExtrinsics;
using overlume::CameraIntrinsics;
namespace bowl = overlume::bowl;

CameraExtrinsics OverheadCamera() {
    return CameraExtrinsics{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
}

CameraIntrinsics TightFovIntrinsics() {
    return CameraIntrinsics{5000, 5000, 320, 240, {0, 0, 0, 0, 0}};
}

CameraExtrinsics LookAtCamera(overlume::Vec3 from, overlume::Vec3 to) {
    auto sub = [](overlume::Vec3 a, overlume::Vec3 b) {
        return overlume::Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
    };
    auto norm = [](overlume::Vec3 v) {
        const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        return overlume::Vec3{v.x / len, v.y / len, v.z / len};
    };
    auto cross = [](overlume::Vec3 a, overlume::Vec3 b) {
        return overlume::Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    };
    const overlume::Vec3 fwd = norm(sub(to, from));
    overlume::Vec3 up{0, 0, 1};
    if (std::abs(fwd.z) > 0.999) up = overlume::Vec3{0, 1, 0};
    const overlume::Vec3 right = norm(cross(fwd, up));
    const overlume::Vec3 down = cross(fwd, right);
    CameraExtrinsics ext{};
    ext.R[0] = right.x;
    ext.R[1] = down.x;
    ext.R[2] = fwd.x;
    ext.R[3] = right.y;
    ext.R[4] = down.y;
    ext.R[5] = fwd.y;
    ext.R[6] = right.z;
    ext.R[7] = down.z;
    ext.R[8] = fwd.z;
    ext.t[0] = from.x;
    ext.t[1] = from.y;
    ext.t[2] = from.z;
    return ext;
}

}

TEST(BowlMeshBake, InnerRingCoveredOuterRingUncoveredBySingleCamera) {
    const CameraExtrinsics ext = OverheadCamera();
    const CameraIntrinsics in = TightFovIntrinsics();
    const uint32_t w = 640, h = 480;

    bowl::BowlMeshParams params;
    params.theta_segments = 16;
    params.radial_rings = 8;
    const bowl::BowlMesh mesh = bowl::BakeBowlMesh(params, 0.5, 0.3, 8.0, 1, &ext, &in, &w, &h);

    ASSERT_FALSE(mesh.vertices.empty());

    bool found_covered = false, found_uncovered = false;
    double max_r = 0.0;
    for (const auto& v : mesh.vertices) {
        const double r = std::sqrt(v.position.x * v.position.x + v.position.y * v.position.y);
        max_r = std::max(max_r, r);
    }
    for (const auto& v : mesh.vertices) {
        const double r = std::sqrt(v.position.x * v.position.x + v.position.y * v.position.y);
        EXPECT_EQ(v.index_a, 0u);
        EXPECT_FLOAT_EQ(v.coverage_b, 0.0f);
        EXPECT_FLOAT_EQ(v.coverage_c, 0.0f);
        if (r < 1.0 && v.coverage_a > 0.0f) found_covered = true;
        if (r > max_r * 0.9 && v.coverage_a == 0.0f) found_uncovered = true;
    }
    EXPECT_TRUE(found_covered) << "a vertex near the bowl center should be inside the tight FOV";
    EXPECT_TRUE(found_uncovered)
        << "a vertex near Rmax should fall outside the tight FOV (zero weight)";
}

TEST(BowlMeshBake, MidEdgeWeightInterpolationIsBoundedByTessellation) {
    const CameraExtrinsics ext = OverheadCamera();
    const CameraIntrinsics in = TightFovIntrinsics();
    const uint32_t w = 640, h = 480;

    bowl::BowlMeshParams params;
    params.theta_segments = 128;
    params.radial_rings = 64;
    const bowl::BowlMesh mesh = bowl::BakeBowlMesh(params, 0.5, 0.3, 8.0, 1, &ext, &in, &w, &h);

    bool checked = false;
    for (size_t t = 0; t + 2 < mesh.indices.size() && !checked; t += 3) {
        const auto& v0 = mesh.vertices[mesh.indices[t]];
        const auto& v1 = mesh.vertices[mesh.indices[t + 1]];
        if (v0.coverage_a < 0.3f || v1.coverage_a < 0.3f) continue;

        const overlume::Vec3 mid{(v0.position.x + v1.position.x) * 0.5,
                                 (v0.position.y + v1.position.y) * 0.5,
                                 (v0.position.z + v1.position.z) * 0.5};
        float u, v;
        ASSERT_TRUE(bowl::ProjectToCameraUv(ext, in, w, h, mid, &u, &v));
        const float expected = bowl::CameraAlignment(ext, mid) * bowl::CameraAlignment(ext, mid);
        const float interpolated = (v0.coverage_a + v1.coverage_a) * 0.5f;
        EXPECT_NEAR(interpolated, expected, 0.05f);
        checked = true;
    }
    ASSERT_TRUE(checked) << "no well-covered adjacent-vertex pair found to check";
}

TEST(BowlMeshBake, EveryTriangleCarriesIdenticalCameraIndexPairAcrossAllThreeVertices) {
    const CameraExtrinsics extA{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    const CameraExtrinsics extB{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, -10.0}};
    const CameraExtrinsics exts[2] = {extA, extB};
    const CameraIntrinsics in{800, 800, 320, 240, {0, 0, 0, 0, 0}};
    const CameraIntrinsics ins[2] = {in, in};
    const uint32_t widths[2] = {640, 640};
    const uint32_t heights[2] = {480, 480};

    bowl::BowlMeshParams params;
    params.theta_segments = 32;
    params.radial_rings = 12;
    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, 0.5, 0.3, 8.0, 2, exts, ins, widths, heights);

    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const auto& v0 = mesh.vertices[mesh.indices[t]];
        const auto& v1 = mesh.vertices[mesh.indices[t + 1]];
        const auto& v2 = mesh.vertices[mesh.indices[t + 2]];
        EXPECT_EQ(v0.index_a, v1.index_a);
        EXPECT_EQ(v0.index_a, v2.index_a);
        EXPECT_EQ(v0.index_b, v1.index_b);
        EXPECT_EQ(v0.index_b, v2.index_b);
        EXPECT_EQ(v0.index_c, v1.index_c);
        EXPECT_EQ(v0.index_c, v2.index_c);
    }
}

TEST(BowlMeshBake, FourCameraOverlapStillPicksAConsistentTripletPerTriangle) {
    const CameraExtrinsics ext = OverheadCamera();
    const CameraIntrinsics in{800, 800, 320, 240, {0, 0, 0, 0, 0}};
    const CameraExtrinsics exts[4] = {ext, ext, ext, ext};
    const CameraIntrinsics ins[4] = {in, in, in, in};
    const uint32_t widths[4] = {640, 640, 640, 640};
    const uint32_t heights[4] = {480, 480, 480, 480};

    bowl::BowlMeshParams params;
    params.theta_segments = 16;
    params.radial_rings = 8;
    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, 0.5, 0.3, 8.0, 4, exts, ins, widths, heights);

    bool found_quad_overlap_triangle = false;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const auto& v0 = mesh.vertices[mesh.indices[t]];
        const auto& v1 = mesh.vertices[mesh.indices[t + 1]];
        const auto& v2 = mesh.vertices[mesh.indices[t + 2]];
        EXPECT_EQ(v0.index_a, v1.index_a);
        EXPECT_EQ(v0.index_a, v2.index_a);
        EXPECT_EQ(v0.index_b, v1.index_b);
        EXPECT_EQ(v0.index_b, v2.index_b);
        EXPECT_EQ(v0.index_c, v1.index_c);
        EXPECT_EQ(v0.index_c, v2.index_c);

        if (v0.coverage_a > 0.0f && v0.index_a != v0.index_b && v0.index_a != v0.index_c) {
            found_quad_overlap_triangle = true;
            EXPECT_EQ(v0.index_a, 0u);
            EXPECT_EQ(v0.index_b, 1u);
            EXPECT_EQ(v0.index_c, 2u);
        }
    }
    ASSERT_TRUE(found_quad_overlap_triangle)
        << "no triangle found where all 4 identical cameras genuinely overlap";
}

TEST(BowlMeshBake, EgoOcclusionZeroesCoverageForTheOccludedCameraOnly) {
    constexpr double kR0 = 0.1, kK = 0.3, kRmax = 1.0;
    const overlume::Vec3 vertex{0.0, 1.0, kK * (kRmax - kR0) * (kRmax - kR0) + 0.01};

    const CameraExtrinsics exts[2] = {
        LookAtCamera({0.0, -3.0, 0.5}, vertex),
        LookAtCamera({10.0, -3.0, 0.5}, vertex),
    };
    const CameraIntrinsics in{800, 800, 320, 240, {0, 0, 0, 0, 0}};
    const CameraIntrinsics ins[2] = {in, in};
    const uint32_t widths[2] = {640, 640};
    const uint32_t heights[2] = {480, 480};

    bowl::BowlMeshParams params;
    params.theta_segments = 4;
    params.radial_rings = 1;
    const bowl::EgoBox ego_box{{0.0, 0.0, 0.5}, {0.5, 0.5, 0.5}};

    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, kR0, kK, kRmax, 2, exts, ins, widths, heights, ego_box);

    bool checked = false;
    for (const auto& v : mesh.vertices) {
        if (std::abs(v.position.x - vertex.x) > 1e-6 || std::abs(v.position.y - vertex.y) > 1e-6 ||
            std::abs(v.position.z - vertex.z) > 1e-6) {
            continue;
        }
        checked = true;
        const uint32_t idx[2] = {v.index_a, v.index_b};
        const float cov[2] = {v.coverage_a, v.coverage_b};
        for (int slot = 0; slot < 2; ++slot) {
            if (idx[slot] == 0) {
                EXPECT_FLOAT_EQ(cov[slot], 0.0f)
                    << "camera 0's line of sight to this vertex crosses the ego box -- its "
                       "coverage must be zeroed";
            } else {
                EXPECT_NEAR(cov[slot], 1.0f, 1e-4f)
                    << "camera 1's line of sight clears the box entirely -- its weight must be "
                       "untouched";
            }
        }
    }
    ASSERT_TRUE(checked) << "expected vertex not found in the baked mesh";

    const bowl::BowlMesh noop_mesh =
        bowl::BakeBowlMesh(params, kR0, kK, kRmax, 2, exts, ins, widths, heights, bowl::EgoBox{});
    bool noop_checked = false;
    for (const auto& v : noop_mesh.vertices) {
        if (std::abs(v.position.x - vertex.x) > 1e-6 || std::abs(v.position.y - vertex.y) > 1e-6 ||
            std::abs(v.position.z - vertex.z) > 1e-6) {
            continue;
        }
        noop_checked = true;
        EXPECT_NEAR(v.coverage_a, 1.0f, 1e-4f);
        EXPECT_NEAR(v.coverage_b, 1.0f, 1e-4f);
    }
    ASSERT_TRUE(noop_checked);
}

TEST(BowlMeshBake, OccludedCameraNeverBurnsATopThreeSlotAVisibleCameraCouldHaveTaken) {
    constexpr double kR0 = 0.1, kK = 0.3, kRmax = 1.0;
    const overlume::Vec3 vertex{0.0, 1.0, kK * (kRmax - kR0) * (kRmax - kR0) + 0.01};

    const CameraExtrinsics exts[4] = {
        LookAtCamera({0.0, -11.0, 0.5}, vertex),
        LookAtCamera({14.0, -3.0, 0.5}, vertex),
        LookAtCamera({10.0, 3.0, 0.5}, vertex),
        LookAtCamera({-6.0, -3.0, 0.5}, vertex),
    };
    const CameraIntrinsics in{800, 800, 320, 240, {0, 0, 0, 0, 0}};
    const CameraIntrinsics ins[4] = {in, in, in, in};
    const uint32_t widths[4] = {640, 640, 640, 640};
    const uint32_t heights[4] = {480, 480, 480, 480};

    bowl::BowlMeshParams params;
    params.theta_segments = 4;
    params.radial_rings = 1;
    const bowl::EgoBox ego_box{{0.0, 0.0, 0.5}, {0.5, 0.5, 0.5}};

    auto idx_has = [](const bowl::BowlVertex& v, uint32_t cam) {
        return v.index_a == cam || v.index_b == cam || v.index_c == cam;
    };

    const bowl::BowlMesh baseline =
        bowl::BakeBowlMesh(params, kR0, kK, kRmax, 4, exts, ins, widths, heights, bowl::EgoBox{});
    int baseline_rows = 0;
    for (const auto& v : baseline.vertices) {
        if (std::abs(v.position.x - vertex.x) > 1e-6 || std::abs(v.position.y - vertex.y) > 1e-6 ||
            std::abs(v.position.z - vertex.z) > 1e-6) {
            continue;
        }
        ++baseline_rows;
        ASSERT_TRUE(idx_has(v, 0))
            << "premise broken: camera 0 must be a genuine top-3 pick BEFORE occlusion is even "
               "considered, or the occluded case below proves nothing about occlusion";
        ASSERT_FALSE(idx_has(v, 3))
            << "premise broken: camera 3 must NOT be a pre-occlusion top-3 pick, or its "
               "post-occlusion promotion below proves nothing";
    }
    ASSERT_GT(baseline_rows, 0) << "expected vertex not found in the baseline mesh";

    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, kR0, kK, kRmax, 4, exts, ins, widths, heights, ego_box);
    int occluded_rows = 0;
    for (const auto& v : mesh.vertices) {
        if (std::abs(v.position.x - vertex.x) > 1e-6 || std::abs(v.position.y - vertex.y) > 1e-6 ||
            std::abs(v.position.z - vertex.z) > 1e-6) {
            continue;
        }
        ++occluded_rows;
        const uint32_t idx[3] = {v.index_a, v.index_b, v.index_c};
        const float cov[3] = {v.coverage_a, v.coverage_b, v.coverage_c};
        for (int slot = 0; slot < 3; ++slot) {
            EXPECT_NE(idx[slot], 0u)
                << "camera 0 is occluded for this vertex -- it must never win a slot";
            EXPECT_GT(cov[slot], 0.9f)
                << "every occupied slot should be a genuinely visible, high-weight camera";
        }
        EXPECT_TRUE(idx_has(v, 3))
            << "camera 3 was demoted out of the top 3 pre-occlusion -- once occlusion removes "
               "camera 0, camera 3 must be promoted into the freed slot instead of staying "
               "excluded";
    }
    ASSERT_GT(occluded_rows, 0) << "expected vertex not found in the occluded mesh";
}

TEST(BowlMeshBake, DeployedRigWithEveryCameraInsideItsOwnEgoBoxIsNotWholesaleZeroed) {
    const CameraExtrinsics exts[6] = {
        {{0.9961946980917455, -0.06269459458646724, 0.06054346623323177, -0.08715574274765814,
          -0.7166024952237391, 0.6920149856363047, 0.0, -0.6946583704589973, -0.7193398003386511},
         {0.1594, 0.90401, 1.314}},
        {{0.0, -0.7193398003386511, 0.6946583704589973, -1.0, 0.0, 0.0, 0.0, -0.6946583704589973,
          -0.7193398003386511},
         {1.05472, 0.0, 0.854}},
        {{-0.9961946980917455, -0.06269459458646724, 0.06054346623323177, -0.08715574274765814,
          0.7166024952237391, -0.6920149856363047, 0.0, -0.6946583704589973, -0.7193398003386511},
         {0.16735, -0.94909, 1.314}},
        {{0.9961946980917455, 0.06269459458646731, -0.06054346623323184, 0.08715574274765824,
          -0.7166024952237391, 0.6920149856363047, 0.0, -0.6946583704589973, -0.7193398003386511},
         {-0.15922, 0.90298, 1.314}},
        {{1.2246467991473532e-16, 0.7193398003386511, -0.6946583704589973, 1.0,
          -8.809371839840251e-17, 8.507111498835273e-17, 0.0, -0.6946583704589973,
          -0.7193398003386511},
         {-1.05472, 0.0, 0.854}},
        {{-0.9961946980917455, 0.06269459458646731, -0.06054346623323184, 0.08715574274765824,
          0.7166024952237391, -0.6920149856363047, 0.0, -0.6946583704589973, -0.7193398003386511},
         {-0.16731, -0.94884, 1.314}},
    };
    const CameraIntrinsics in{800, 800, 640, 480, {0, 0, 0, 0, 0}};
    const CameraIntrinsics ins[6] = {in, in, in, in, in, in};
    const uint32_t widths[6] = {1280, 1280, 1280, 1280, 1280, 1280};
    const uint32_t heights[6] = {960, 960, 960, 960, 960, 960};

    bowl::BowlMeshParams params;
    params.theta_segments = 64;
    params.radial_rings = 24;
    const bowl::EgoBox ego_box{{0.0, 0.0, 0.9}, {2.25, 1.0, 0.9}};

    auto count_covered = [](const bowl::BowlMesh& m) {
        size_t covered = 0;
        for (const auto& v : m.vertices) {
            if (v.coverage_a > 0.0f || v.coverage_b > 0.0f || v.coverage_c > 0.0f) ++covered;
        }
        return covered;
    };
    const bowl::BowlMesh baseline =
        bowl::BakeBowlMesh(params, 17.0, 0.06, 40.0, 6, exts, ins, widths, heights, bowl::EgoBox{});
    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, 17.0, 0.06, 40.0, 6, exts, ins, widths, heights, ego_box);
    ASSERT_FALSE(mesh.vertices.empty());
    ASSERT_GT(count_covered(baseline), 0u) << "sanity: these 6 cameras should cover something";

    EXPECT_EQ(count_covered(mesh), count_covered(baseline))
        << "every deployed camera sits inside its own ego AABB -- a camera positioned inside the "
           "box can never be occluded by it, so enabling self-view masks on this rig must not "
           "change coverage at all, let alone zero the bake wholesale";
}

TEST(Bowl, EgoMeshOccludesBowlSurfaceBehindIt) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    const overlume::Vec3 ego_pos{3.0, 2.0, 0.0};
    overlume::CameraPose pose{{ego_pos.x, ego_pos.y - 6.0, 6.0}, {ego_pos.x, ego_pos.y, 0.0}, 70.0};
    overlume::SceneGraph scene{};
    scene.ego = {ego_pos, 0.0, 0.0, 1};

    overlume::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    overlume::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 5.0;
    bc.sky_color[0] = 0.05f;
    bc.sky_color[1] = 0.05f;
    bc.sky_color[2] = 0.05f;

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < cam_pixels.size(); i += 3) {
        cam_pixels[i] = 255;
        cam_pixels[i + 1] = 0;
        cam_pixels[i + 2] = 255;
    }

    auto* base_r = overlume::create_renderer(cfg);
    if (!base_r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::set_bowl_config(base_r, bc));
    ASSERT_TRUE(overlume::set_bowl_visible(base_r, true));
    ASSERT_TRUE(overlume::set_camera_frame(base_r, 0, cam_pixels.data(), w, h, 1));
    overlume::set_scene(base_r, scene);
    std::vector<uint8_t> baseline(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(base_r, pose, {baseline.data(), 320, 240}));
    auto is_magenta = [](uint8_t r8, uint8_t g8, uint8_t b8) {
        return r8 > 150 && b8 > 150 && g8 < 100;
    };
    size_t baseline_magenta = 0;
    for (size_t i = 0; i < baseline.size(); i += 3) {
        if (is_magenta(baseline[i], baseline[i + 1], baseline[i + 2])) ++baseline_magenta;
    }
    overlume::destroy_renderer(base_r);
    ASSERT_GT(baseline_magenta, 0u) << "bowl-only baseline should show its magenta sentinel";

    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));
    ASSERT_TRUE(overlume::set_bowl_visible(r, true));
    ASSERT_TRUE(overlume::set_camera_frame(r, 0, cam_pixels.data(), w, h, 1));
    overlume::set_ego_model(r, "/nonexistent/path.glb", {1.5, 1.5, 1.2});
    overlume::set_scene(r, scene);
    std::vector<uint8_t> with_ego(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {with_ego.data(), 320, 240}));

    size_t with_ego_magenta = 0;
    size_t differing_from_baseline = 0;
    for (size_t i = 0; i < with_ego.size(); i += 3) {
        if (is_magenta(with_ego[i], with_ego[i + 1], with_ego[i + 2])) ++with_ego_magenta;
        if (with_ego[i] != baseline[i] || with_ego[i + 1] != baseline[i + 1] ||
            with_ego[i + 2] != baseline[i + 2]) {
            ++differing_from_baseline;
        }
    }
    EXPECT_LT(with_ego_magenta, baseline_magenta)
        << "adding the ego should occlude some of the bowl's own magenta sentinel -- proving "
           "robot-over-bowl depth compositing, not the reverse (or no compositing at all)";
    EXPECT_GT(differing_from_baseline, 0u)
        << "the ego mesh produced no visible difference at all -- it may not share the bowl's "
           "scene, or may not be rendering";
    overlume::destroy_renderer(r);
}

TEST(Bowl, SelfViewMasksOffByDefaultThenEnabledSuppressesOccludedCameraViaExistingSkyColorPath) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};

    overlume::CameraExtrinsics ext{{0, 0, 1, -1, 0, 0, 0, -1, 0}, {-5.0, 0, 0.3}};
    overlume::CameraIntrinsics in{250, 250, 320, 240, {0, 0, 0, 0, 0}};
    uint32_t w = 640, h = 480;
    overlume::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 0.0;
    bc.sky_color[0] = 0.05f;
    bc.sky_color[1] = 0.05f;
    bc.sky_color[2] = 0.05f;

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < cam_pixels.size(); i += 3) {
        cam_pixels[i] = 255;
        cam_pixels[i + 1] = 0;
        cam_pixels[i + 2] = 255;
    }

    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::set_ego_model(r, "/nonexistent/path.glb", {1.6, 1.6, 0.7});
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));
    ASSERT_TRUE(overlume::set_bowl_visible(r, true));
    ASSERT_TRUE(overlume::set_camera_frame(r, 0, cam_pixels.data(), w, h, 1));

    std::vector<uint8_t> masks_off(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {masks_off.data(), 320, 240}));
    size_t magenta_off = 0;
    for (size_t i = 0; i < masks_off.size(); i += 3) {
        if (masks_off[i] > 150 && masks_off[i + 2] > 150) ++magenta_off;
    }
    ASSERT_GT(magenta_off, masks_off.size() / 3 / 100)
        << "sanity: the bowl should show its magenta sentinel with masks off";

    ASSERT_TRUE(overlume::set_self_view_masks(r, true));
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));
    ASSERT_TRUE(overlume::set_bowl_visible(r, true));
    ASSERT_TRUE(overlume::set_camera_frame(r, 0, cam_pixels.data(), w, h, 2));

    std::vector<uint8_t> masks_on(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {masks_on.data(), 320, 240}));
    size_t magenta_on = 0;
    for (size_t i = 0; i < masks_on.size(); i += 3) {
        if (masks_on[i] > 150 && masks_on[i + 2] > 150) ++magenta_on;
    }
    EXPECT_LT(magenta_on, magenta_off)
        << "camera 0's own body between it and part of the bowl should have its contribution "
           "suppressed once self_view_masks is enabled (declared off by default, Decision 4)";

    size_t unmasked_pixels = 0;
    size_t unmasked_and_shaded = 0;
    for (size_t i = 0; i < masks_off.size(); i += 3) {
        const bool was_magenta = masks_off[i] > 150 && masks_off[i + 2] > 150;
        const bool now_magenta = masks_on[i] > 150 && masks_on[i + 2] > 150;
        if (!was_magenta || now_magenta) continue;
        ++unmasked_pixels;
        uint8_t maxc = masks_on[i];
        if (masks_on[i + 1] > maxc) maxc = masks_on[i + 1];
        if (masks_on[i + 2] > maxc) maxc = masks_on[i + 2];
        if (maxc > 5) ++unmasked_and_shaded;
    }
    ASSERT_GT(unmasked_pixels, 0u)
        << "sanity: expected some pixel to lose its magenta camera-0 contribution once "
           "self_view_masks suppressed it";
    EXPECT_EQ(unmasked_and_shaded, unmasked_pixels)
        << "every pixel that lost its occluded-camera contribution must fall back to "
           "sky_color/theme shading (bowl.mat's `if (wsum > 0.0) ... else skyColor`), not be "
           "left black/unshaded";
    overlume::destroy_renderer(r);
}

TEST(Bowl, RenderFrameWithBowlConfiguredProducesSentinelPixels) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    overlume::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc{};
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
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));
    ASSERT_TRUE(overlume::set_bowl_visible(r, true));

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < cam_pixels.size(); i += 3) {
        cam_pixels[i] = 255;
        cam_pixels[i + 1] = 0;
        cam_pixels[i + 2] = 255;
    }
    ASSERT_TRUE(overlume::set_camera_frame(r, 0, cam_pixels.data(), w, h, 1));

    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));

    size_t magenta_pixels = 0;
    for (size_t i = 0; i < buf.size(); i += 3) {
        if (buf[i] > 150 && buf[i + 2] > 150) ++magenta_pixels;
    }
    EXPECT_GT(magenta_pixels, buf.size() / 3 / 10)
        << "expected the bowl's sampled surface to cover a meaningful fraction of the frame";
    overlume::destroy_renderer(r);
}

TEST(Bowl, PerFragmentSamplingReadsTheCorrectPixelNotAMirroredOne) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    overlume::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 0.0;
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));
    ASSERT_TRUE(overlume::set_bowl_visible(r, true));

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const bool left = x < w / 2;
            const bool top = y < h / 2;
            uint8_t* px = &cam_pixels[(static_cast<size_t>(y) * w + x) * 3];
            if (top && left) {
                px[0] = 255;
                px[1] = 0;
                px[2] = 0;
            } else if (top && !left) {
                px[0] = 0;
                px[1] = 255;
                px[2] = 0;
            } else if (!top && left) {
                px[0] = 0;
                px[1] = 0;
                px[2] = 255;
            } else {
                px[0] = 255;
                px[1] = 255;
                px[2] = 0;
            }
        }
    }
    ASSERT_TRUE(overlume::set_camera_frame(r, 0, cam_pixels.data(), w, h, 1));

    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));

    auto classify = [](uint8_t r8, uint8_t g8, uint8_t b8) -> char {
        const bool R = r8 > 120, G = g8 > 120, B = b8 > 120;
        if (R && G && !B) return 'Y';
        if (R && !G && !B) return 'R';
        if (!R && G && !B) return 'G';
        if (!R && !G && B) return 'B';
        return '.';
    };
    int left_red = 0, left_blue = 0, right_green = 0, right_yellow = 0;
    int wrong_left_is_greenish = 0, wrong_right_is_reddish = 0;
    for (int y = 0; y < 240; ++y) {
        for (int x = 0; x < 320; ++x) {
            const size_t i = (static_cast<size_t>(y) * 320 + x) * 3;
            const char c = classify(buf[i], buf[i + 1], buf[i + 2]);
            if (c == '.') continue;
            const bool screenLeft = x < 160;
            if (screenLeft) {
                if (c == 'R') ++left_red;
                if (c == 'B') ++left_blue;
                if (c == 'G' || c == 'Y') ++wrong_left_is_greenish;
            } else {
                if (c == 'G') ++right_green;
                if (c == 'Y') ++right_yellow;
                if (c == 'R' || c == 'B') ++wrong_right_is_reddish;
            }
        }
    }
    EXPECT_GT(left_red + left_blue, 50) << "expected red/blue (image-left) quadrants on screen";
    EXPECT_GT(right_green + right_yellow, 50)
        << "expected green/yellow (image-right) quadrants on screen";
    EXPECT_LT(wrong_left_is_greenish, (left_red + left_blue) / 2)
        << "too much green/yellow leaking onto the red/blue screen side -- position/orientation "
           "corruption suspected";
    EXPECT_LT(wrong_right_is_reddish, (right_green + right_yellow) / 2)
        << "too much red/blue leaking onto the green/yellow screen side -- position/orientation "
           "corruption suspected";
    overlume::destroy_renderer(r);
}

TEST(Bowl, SetBowlVisibleFalseHidesTheBowlEntirely) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::CameraExtrinsics ext{{1, 0, 0, 0, -1, 0, 0, 0, -1}, {0, 0, 10.0}};
    overlume::CameraIntrinsics in{200, 200, 160, 120, {0, 0, 0, 0, 0}};
    uint32_t w = 320, h = 240;
    overlume::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 5.0;
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < cam_pixels.size(); i += 3) {
        cam_pixels[i] = 255;
        cam_pixels[i + 1] = 0;
        cam_pixels[i + 2] = 255;
    }
    ASSERT_TRUE(overlume::set_camera_frame(r, 0, cam_pixels.data(), w, h, 1));

    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    size_t magenta_pixels = 0;
    for (size_t i = 0; i < buf.size(); i += 3) {
        if (buf[i] > 150 && buf[i + 2] > 150) ++magenta_pixels;
    }
    EXPECT_EQ(magenta_pixels, 0u) << "bowl rendered while bowlVisible defaulted false";
    overlume::destroy_renderer(r);
}

TEST(Bowl, TwoCamerasWithNonzeroSlotIndexBothAppearInFrame) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::CameraExtrinsics exts[2] = {
        {{1, 0, 0, 0, -1, 0, 0, 0, -1}, {-2.0, 0, 10.0}},
        {{1, 0, 0, 0, -1, 0, 0, 0, -1}, {2.0, 0, 10.0}},
    };
    overlume::CameraIntrinsics ins[2] = {
        {300, 300, 160, 120, {0, 0, 0, 0, 0}},
        {300, 300, 160, 120, {0, 0, 0, 0, 0}},
    };
    uint32_t widths[2] = {320, 320};
    uint32_t heights[2] = {240, 240};
    overlume::BowlConfig bc{};
    bc.camera_count = 2;
    bc.extrinsics = exts;
    bc.intrinsics = ins;
    bc.cam_width = widths;
    bc.cam_height = heights;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 5.0;
    bc.sky_color[0] = 0.05f;
    bc.sky_color[1] = 0.05f;
    bc.sky_color[2] = 0.05f;
    ASSERT_TRUE(overlume::set_bowl_config(r, bc));
    ASSERT_TRUE(overlume::set_bowl_visible(r, true));

    std::vector<uint8_t> red(static_cast<size_t>(widths[0]) * heights[0] * 3);
    std::vector<uint8_t> green(static_cast<size_t>(widths[1]) * heights[1] * 3);
    for (size_t i = 0; i < red.size(); i += 3) {
        red[i] = 255;
        red[i + 1] = 0;
        red[i + 2] = 0;
    }
    for (size_t i = 0; i < green.size(); i += 3) {
        green[i] = 0;
        green[i + 1] = 255;
        green[i + 2] = 0;
    }
    ASSERT_TRUE(overlume::set_camera_frame(r, 0, red.data(), widths[0], heights[0], 1));
    ASSERT_TRUE(overlume::set_camera_frame(r, 1, green.data(), widths[1], heights[1], 1));

    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));

    size_t red_pixels = 0, green_pixels = 0;
    for (size_t i = 0; i < buf.size(); i += 3) {
        const bool R = buf[i] > 150, G = buf[i + 1] > 150, B = buf[i + 2] > 150;
        if (R && !G && !B) ++red_pixels;
        if (!R && G && !B) ++green_pixels;
    }
    EXPECT_GT(red_pixels, 20u) << "camera 0 (slot index 0) never sampled";
    EXPECT_GT(green_pixels, 20u)
        << "camera 1 (slot index 1, the non-zero-slot case this test targets) never sampled -- "
           "a non-zero idxA/idxB shader block or its uniforms/sampler are not wired correctly";
    overlume::destroy_renderer(r);
}
