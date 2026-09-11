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

// Builds a CameraExtrinsics whose R aims exactly at `to` from `from`
// (fwd == normalize(to - from)) -- CameraAlignment is then exactly 1.0 for
// that one point by construction, and ProjectToCameraUv lands it exactly
// at the intrinsics' principal point, regardless of camera position. Used
// to get an exact, non-approximated per-camera weight without hand-deriving
// R by hand for several cameras at once.
CameraExtrinsics LookAtCamera(mpviz::Vec3 from, mpviz::Vec3 to) {
    auto sub = [](mpviz::Vec3 a, mpviz::Vec3 b) {
        return mpviz::Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
    };
    auto norm = [](mpviz::Vec3 v) {
        const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        return mpviz::Vec3{v.x / len, v.y / len, v.z / len};
    };
    auto cross = [](mpviz::Vec3 a, mpviz::Vec3 b) {
        return mpviz::Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    };
    const mpviz::Vec3 fwd = norm(sub(to, from));
    mpviz::Vec3 up{0, 0, 1};
    if (std::abs(fwd.z) > 0.999) up = mpviz::Vec3{0, 1, 0};  // fwd near-vertical -- pick another up
    const mpviz::Vec3 right = norm(cross(fwd, up));
    const mpviz::Vec3 down = cross(fwd, right);
    CameraExtrinsics ext{};
    ext.R[0] = right.x; ext.R[1] = down.x; ext.R[2] = fwd.x;
    ext.R[3] = right.y; ext.R[4] = down.y; ext.R[5] = fwd.y;
    ext.R[6] = right.z; ext.R[7] = down.z; ext.R[8] = fwd.z;
    ext.t[0] = from.x; ext.t[1] = from.y; ext.t[2] = from.z;
    return ext;
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
                           /*camera_count=*/1, &ext, &in, &w, &h);

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
        // camera 0, and coverage_b/coverage_c must be forced to 0 (never
        // double-counts the single camera across slots).
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
    // Fine tessellation -> the rasterizer's linear interpolation of two
    // baked vertices' coverage floats should closely reproduce an
    // independent alignment^2 evaluation at their true geometric midpoint --
    // the property bit-packing (banned by Decision 3) could never have.
    // Coverage is alignment^2 ONLY -- border feather is computed
    // per-fragment by bowl.mat, not baked, so the reference value below
    // doesn't include a BorderFeather factor either.
    const CameraExtrinsics ext = OverheadCamera();
    const CameraIntrinsics in = TightFovIntrinsics();
    const uint32_t w = 640, h = 480;

    bowl::BowlMeshParams params;
    params.theta_segments = 128;
    params.radial_rings = 64;
    const bowl::BowlMesh mesh = bowl::BakeBowlMesh(params, 0.5, 0.3, 8.0, 1, &ext, &in, &w, &h);

    // Find a triangle whose first two vertices are both covered by camera
    // 0 with a meaningfully nonzero weight (well inside the FOV).
    bool checked = false;
    for (size_t t = 0; t + 2 < mesh.indices.size() && !checked; t += 3) {
        const auto& v0 = mesh.vertices[mesh.indices[t]];
        const auto& v1 = mesh.vertices[mesh.indices[t + 1]];
        if (v0.coverage_a < 0.3f || v1.coverage_a < 0.3f) continue;

        const mpviz::Vec3 mid{(v0.position.x + v1.position.x) * 0.5,
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
    // Decision 3's construction rule, checked as a whole-mesh invariant: no
    // triangle may have vertices that disagree on (index_a, index_b,
    // index_c) -- that's exactly the "index interpolated into fractional
    // garbage" corruption class the rule exists to prevent. Two cameras with
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
    const bowl::BowlMesh mesh = bowl::BakeBowlMesh(params, 0.5, 0.3, 8.0, 2, exts, ins, widths, heights);

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
    // Named parity exception (migration plan doc's Task 4 Step 1
    // checklist, VM-091 gate close-out finding 7): this material contributes
    // at most 3 cameras per fragment (CUSTOM0/CUSTOM2/CUSTOM3 are all spoken
    // for), so a genuine FOUR-way overlap region -- four cameras all
    // covering the same vertex -- can only ever surface 3 of the 4. Four
    // IDENTICAL overhead cameras (same extrinsics/intrinsics, different slot
    // indices) is the simplest real 4-way overlap: every vertex they cover,
    // all four cover with EQUAL weight, so this isn't a near-miss, it's exact.
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
        // Existing per-triangle-uniform-index invariant still holds with 4
        // configured cameras, not just 2/3.
        EXPECT_EQ(v0.index_a, v1.index_a);
        EXPECT_EQ(v0.index_a, v2.index_a);
        EXPECT_EQ(v0.index_b, v1.index_b);
        EXPECT_EQ(v0.index_b, v2.index_b);
        EXPECT_EQ(v0.index_c, v1.index_c);
        EXPECT_EQ(v0.index_c, v2.index_c);

        if (v0.coverage_a > 0.0f && v0.index_a != v0.index_b && v0.index_a != v0.index_c) {
            found_quad_overlap_triangle = true;
            // Identical cameras -> ties resolve to the lowest slot indices
            // examined first (bowl_mesh.cpp's strict `>` comparisons never
            // displace a first-seen max on a tie) -- camera 3 is excluded
            // even though it covers this triangle exactly as strongly as
            // cameras 0/1/2. This is the named parity exception, pinned so
            // a future 4-camera-per-fragment fix changes this test
            // deliberately instead of silently.
            EXPECT_EQ(v0.index_a, 0u);
            EXPECT_EQ(v0.index_b, 1u);
            EXPECT_EQ(v0.index_c, 2u);
        }
    }
    ASSERT_TRUE(found_quad_overlap_triangle)
        << "no triangle found where all 4 identical cameras genuinely overlap";
}

// ---- VM-092 (Task 3) Step 1: analytic ego-occlusion, now inside the bake -

TEST(BowlMeshBake, EgoOcclusionZeroesCoverageForTheOccludedCameraOnly) {
    // Retargeted (review round 1 finding 2) from the pre-refactor
    // ApplyEgoOcclusion() unit test: occlusion now runs INSIDE
    // BakeBowlMesh's own per-vertex weight loop rather than a separate
    // post-pass, so it's exercised here through the public bake API. A
    // minimal (2-ring, 4-seg) mesh puts a known vertex exactly at
    // (r=Rmax=1.0, theta=90deg) -- BowlSurfacePoint's own formula gives its
    // z, plus the bake's 1cm cosmetic lift.
    //   - camera 0 sits on the FAR side of the ego box from that vertex
    //     (straight line from camera to vertex crosses the box);
    //   - camera 1 sits well clear of the box's shadow for the SAME vertex
    //     (the line misses the box entirely).
    // Both cameras are aimed exactly at the vertex (LookAtCamera) so their
    // pre-occlusion weights are both == 1.0 -- the only difference the
    // result can be attributed to is the occlusion test itself.
    constexpr double kR0 = 0.1, kK = 0.3, kRmax = 1.0;
    const mpviz::Vec3 vertex{0.0, 1.0, kK * (kRmax - kR0) * (kRmax - kR0) + 0.01};

    const CameraExtrinsics exts[2] = {
        LookAtCamera({0.0, -3.0, 0.5}, vertex),   // 0 -- occluded
        LookAtCamera({10.0, -3.0, 0.5}, vertex),  // 1 -- clear
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

    // A zero-extent box (bowl.cpp's "no ego configured" convention) is a
    // documented no-op -- pin it so a future change can't silently start
    // occluding everything when no ego was ever set.
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
    // Review round 1 finding 2: the old code ran ApplyEgoOcclusion() AFTER
    // BakeBowlMesh's per-triangle top-3 camera selection, so a self-occluded
    // camera still WON a slot (ranked by its pre-occlusion weight) and then
    // had its coverage zeroed there -- burning a slot instead of ever
    // letting a genuinely visible, lower-ranked camera take it. Folding the
    // occlusion test into the same per-vertex loop that FEEDS the selection
    // fixes this by construction: a zeroed-out camera can't win a slot in
    // the first place.
    //
    // Four cameras all aimed exactly at the same known vertex (LookAtCamera
    // -> weight == 1.0 each, a genuine 4-way tie). Camera 3 loses the tie
    // (FourCameraOverlapStillPicksAConsistentTripletPerTriangle's own
    // documented strict-`>` tie-break) UNLESS camera 0 -- occluded here --
    // is zeroed first, in which case camera 3 must be promoted into the
    // freed slot.
    constexpr double kR0 = 0.1, kK = 0.3, kRmax = 1.0;
    const mpviz::Vec3 vertex{0.0, 1.0, kK * (kRmax - kR0) * (kRmax - kR0) + 0.01};

    const CameraExtrinsics exts[4] = {
        LookAtCamera({0.0, -3.0, 0.5}, vertex),    // 0 -- occluded, would win the tie-break first
        LookAtCamera({10.0, -3.0, 0.5}, vertex),   // 1 -- clear
        LookAtCamera({10.0, 3.0, 0.5}, vertex),    // 2 -- clear
        LookAtCamera({-10.0, -3.0, 0.5}, vertex),  // 3 -- clear, loses the tie-break pre-occlusion
    };
    const CameraIntrinsics in{800, 800, 320, 240, {0, 0, 0, 0, 0}};
    const CameraIntrinsics ins[4] = {in, in, in, in};
    const uint32_t widths[4] = {640, 640, 640, 640};
    const uint32_t heights[4] = {480, 480, 480, 480};

    bowl::BowlMeshParams params;
    params.theta_segments = 4;
    params.radial_rings = 1;
    const bowl::EgoBox ego_box{{0.0, 0.0, 0.5}, {0.5, 0.5, 0.5}};

    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, kR0, kK, kRmax, 4, exts, ins, widths, heights, ego_box);

    bool checked = false;
    for (const auto& v : mesh.vertices) {
        if (std::abs(v.position.x - vertex.x) > 1e-6 || std::abs(v.position.y - vertex.y) > 1e-6 ||
            std::abs(v.position.z - vertex.z) > 1e-6) {
            continue;
        }
        checked = true;
        const uint32_t idx[3] = {v.index_a, v.index_b, v.index_c};
        const float cov[3] = {v.coverage_a, v.coverage_b, v.coverage_c};
        for (int slot = 0; slot < 3; ++slot) {
            EXPECT_NE(idx[slot], 0u)
                << "camera 0 is occluded for this vertex -- it must never win a slot";
            EXPECT_GT(cov[slot], 0.9f)
                << "every occupied slot should be a genuinely visible, high-weight camera";
        }
        EXPECT_TRUE(idx[0] == 3 || idx[1] == 3 || idx[2] == 3)
            << "camera 3 loses the tie-break pre-occlusion -- once occlusion demotes camera 0, "
               "camera 3 must be promoted into the freed slot instead of staying excluded";
    }
    ASSERT_TRUE(checked) << "expected vertex not found in the baked mesh";
}

TEST(BowlMeshBake, DeployedRigWithEveryCameraInsideItsOwnEgoBoxIsNotWholesaleZeroed) {
    // Review round 1 finding 1: every camera in the deployed 6-camera rig
    // (default_params.yaml's camera_extrinsics) sits INSIDE the
    // ego_fallback_dims box -- before the fix, SegmentIntersectsAabb's tmin
    // stayed clamped at 0 whenever a segment started inside the box, so
    // EVERY (camera, vertex) pair read as occluded and the whole bake
    // zeroed to sky_color. A segment starting inside a convex box can never
    // be occluded BY that box, so the fixed test returns false immediately
    // for a camera in this configuration, and the bake below must retain
    // real coverage.
    const CameraExtrinsics exts[6] = {
        {{0.9961946980917455, -0.06269459458646724, 0.06054346623323177, -0.08715574274765814,
          -0.7166024952237391, 0.6920149856363047, 0.0, -0.6946583704589973,
          -0.7193398003386511},
         {0.1594, 0.90401, 1.314}},
        {{0.0, -0.7193398003386511, 0.6946583704589973, -1.0, 0.0, 0.0, 0.0,
          -0.6946583704589973, -0.7193398003386511},
         {1.05472, 0.0, 0.854}},
        {{-0.9961946980917455, -0.06269459458646724, 0.06054346623323177, -0.08715574274765814,
          0.7166024952237391, -0.6920149856363047, 0.0, -0.6946583704589973,
          -0.7193398003386511},
         {0.16735, -0.94909, 1.314}},
        {{0.9961946980917455, 0.06269459458646731, -0.06054346623323184, 0.08715574274765824,
          -0.7166024952237391, 0.6920149856363047, 0.0, -0.6946583704589973,
          -0.7193398003386511},
         {-0.15922, 0.90298, 1.314}},
        {{1.2246467991473532e-16, 0.7193398003386511, -0.6946583704589973, 1.0,
          -8.809371839840251e-17, 8.507111498835273e-17, 0.0, -0.6946583704589973,
          -0.7193398003386511},
         {-1.05472, 0.0, 0.854}},
        {{-0.9961946980917455, 0.06269459458646731, -0.06054346623323184, 0.08715574274765824,
          0.7166024952237391, -0.6920149856363047, 0.0, -0.6946583704589973,
          -0.7193398003386511},
         {-0.16731, -0.94884, 1.314}},
    };
    const CameraIntrinsics in{800, 800, 640, 480, {0, 0, 0, 0, 0}};
    const CameraIntrinsics ins[6] = {in, in, in, in, in, in};
    const uint32_t widths[6] = {1280, 1280, 1280, 1280, 1280, 1280};
    const uint32_t heights[6] = {960, 960, 960, 960, 960, 960};

    bowl::BowlMeshParams params;
    params.theta_segments = 64;
    params.radial_rings = 24;
    // ego_fallback_dims: [4.5, 2.0, 1.8] (default_params.yaml) -- centered on
    // X/Y, resting on the ground plane, the same box ego_rig_frame_box()
    // builds for the clay-box fallback (bowl.cpp).
    const bowl::EgoBox ego_box{{0.0, 0.0, 0.9}, {2.25, 1.0, 0.9}};

    // bowl_R0/bowl_k/bowl_Rmax: default_params.yaml's own deployed values.
    auto count_covered = [](const bowl::BowlMesh& m) {
        size_t covered = 0;
        for (const auto& v : m.vertices) {
            if (v.coverage_a > 0.0f || v.coverage_b > 0.0f || v.coverage_c > 0.0f) ++covered;
        }
        return covered;
    };
    // Baseline: no ego configured at all -- how much of the bowl these 6
    // cameras cover before self-view masking enters the picture (this rig's
    // narrow-ish assumed intrinsics don't cover the whole 40m-radius bowl,
    // which is expected and irrelevant to what this test checks).
    const bowl::BowlMesh baseline =
        bowl::BakeBowlMesh(params, /*bowl_R0=*/17.0, /*bowl_k=*/0.06, /*bowl_Rmax=*/40.0, 6, exts,
                           ins, widths, heights, bowl::EgoBox{});
    const bowl::BowlMesh mesh =
        bowl::BakeBowlMesh(params, /*bowl_R0=*/17.0, /*bowl_k=*/0.06, /*bowl_Rmax=*/40.0, 6, exts,
                           ins, widths, heights, ego_box);
    ASSERT_FALSE(mesh.vertices.empty());
    ASSERT_GT(count_covered(baseline), 0u) << "sanity: these 6 cameras should cover something";

    // Every camera sits inside the box, so self-view masking must be a
    // complete no-op here -- before the fix, it zeroed the bake to nothing
    // (4320/4320 sampled camera/vertex pairs occluded, review round 1
    // finding 1's own measurement); after the fix it must match the
    // no-ego-configured baseline exactly.
    EXPECT_EQ(count_covered(mesh), count_covered(baseline))
        << "every deployed camera sits inside its own ego AABB -- a camera positioned inside the "
           "box can never be occluded by it, so enabling self-view masks on this rig must not "
           "change coverage at all, let alone zero the bake wholesale";
}

// ---- VM-092 (Task 3) Step 0: shared Filament Scene composites -----------
// ---- robot-over-bowl with zero new code ---------------------------------

TEST(Bowl, EgoMeshOccludesBowlSurfaceBehindIt) {
    // Decision 4: robot-proxy compositing needs no bowl-specific rasterizer
    // at all -- the ego entity (set_ego_model) and the bowl entity both
    // live in the SAME r->scene (renderer.cpp), so Filament's own depth
    // test composites robot-over-bowl for free. Proven with a NONZERO
    // map-frame ego pose (not the origin): both entities are anchored from
    // the SAME scene.ego every render_frame() call (update_ego_transform /
    // update_bowl, Decision 3's frame convention), so this also catches a
    // bug where one of them stayed at the map origin while the other moved.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    const mpviz::Vec3 ego_pos{3.0, 2.0, 0.0};
    mpviz::CameraPose pose{{ego_pos.x, ego_pos.y - 6.0, 6.0}, {ego_pos.x, ego_pos.y, 0.0}, 70.0};
    mpviz::SceneGraph scene{};
    scene.ego = {ego_pos, /*heading_rad=*/0.0, /*speed_mps=*/0.0, /*valid=*/1};

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
    bc.sky_color[0] = 0.05f;
    bc.sky_color[1] = 0.05f;
    bc.sky_color[2] = 0.05f;

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < cam_pixels.size(); i += 3) {
        cam_pixels[i] = 255;
        cam_pixels[i + 1] = 0;
        cam_pixels[i + 2] = 255;
    }

    // Baseline: bowl only, no ego -- how much of the frame the bowl's own
    // camera-texture sentinel covers.
    auto* base_r = mpviz::create_renderer(cfg);
    if (!base_r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(mpviz::set_bowl_config(base_r, bc));
    ASSERT_TRUE(mpviz::set_bowl_visible(base_r, true));
    ASSERT_TRUE(mpviz::set_camera_frame(base_r, 0, cam_pixels.data(), w, h, /*frame_id=*/1));
    mpviz::set_scene(base_r, scene);
    std::vector<uint8_t> baseline(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(base_r, pose, {baseline.data(), 320, 240}));
    // R>150 && B>150 && G<100: the dark_adas theme's own ego color
    // ([0.82, 0.80, 0.76], near-white clay) would ALSO satisfy a bare
    // R>150&&B>150 check once the ego renders into this same frame --
    // requiring G to stay low is what keeps this a genuine "the bowl
    // sampled its camera texture" signature rather than "any bright pixel".
    auto is_magenta = [](uint8_t r8, uint8_t g8, uint8_t b8) {
        return r8 > 150 && b8 > 150 && g8 < 100;
    };
    size_t baseline_magenta = 0;
    for (size_t i = 0; i < baseline.size(); i += 3) {
        if (is_magenta(baseline[i], baseline[i + 1], baseline[i + 2])) ++baseline_magenta;
    }
    mpviz::destroy_renderer(base_r);
    ASSERT_GT(baseline_magenta, 0u) << "bowl-only baseline should show its magenta sentinel";

    // Same setup, plus a known-size fallback ego box (build_ego_fallback,
    // no glTF file needed) at the SAME nonzero ego pose -- sitting well
    // inside the bowl's own small inner-ring radius, so it sits squarely
    // in front of a meaningful chunk of the magenta surface checked above.
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));
    ASSERT_TRUE(mpviz::set_bowl_visible(r, true));
    ASSERT_TRUE(mpviz::set_camera_frame(r, 0, cam_pixels.data(), w, h, /*frame_id=*/1));
    mpviz::set_ego_model(r, "/nonexistent/path.glb", {1.5, 1.5, 1.2});
    mpviz::set_scene(r, scene);
    std::vector<uint8_t> with_ego(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {with_ego.data(), 320, 240}));

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
    mpviz::destroy_renderer(r);
}

// ---- VM-092 (Task 3) Step 2: disable knob, shipped default off ----------

TEST(Bowl, SelfViewMasksOffByDefaultThenEnabledSuppressesOccludedCameraViaExistingSkyColorPath) {
    // Decision 4: with self_view_masks left at its shipped default
    // (false), build_bowl() bakes with a zero-extent EgoBox, so the
    // occlusion test never fires -- the bake behaves exactly as Task 2 left
    // it. Force-enabling it for this test only must suppress
    // the occluded camera's contribution; the "falls back to sky_color,
    // not left unshaded" half of Decision 4's requirement is a STRUCTURAL
    // property of bowl.mat's existing `if (wsum > 0.0) ... else skyColor`
    // fragment code (untouched by this task -- a vertex whose only
    // covering camera(s) all get zeroed by occlusion already has wsum==0,
    // same as a vertex outside every camera's FOV), so this test's pixel
    // checks are a smoke test on top of that, not the only proof of it.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};

    // Camera 0 sits off to one side (rig frame), looking across the bowl --
    // NOT overhead -- so a body-sized ego box between it and the far side
    // of the bowl casts a real shadow (col0=right=(0,-1,0),
    // col1=down=(0,0,-1), col2=fwd=(1,0,0): looking in +x, same
    // column-extraction convention as OverheadCamera() above).
    mpviz::CameraExtrinsics ext{{0, 0, 1, -1, 0, 0, 0, -1, 0}, {-5.0, 0, 0.3}};
    mpviz::CameraIntrinsics in{250, 250, 320, 240, {0, 0, 0, 0, 0}};
    uint32_t w = 640, h = 480;
    mpviz::BowlConfig bc{};
    bc.camera_count = 1;
    bc.extrinsics = &ext;
    bc.intrinsics = &in;
    bc.cam_width = &w;
    bc.cam_height = &h;
    bc.bowl_R0 = 0.5;
    bc.bowl_k = 0.3;
    bc.bowl_Rmax = 4.0;
    bc.feather_margin = 0.0;  // isolate the weight question, no border feather
    bc.sky_color[0] = 0.05f;
    bc.sky_color[1] = 0.05f;
    bc.sky_color[2] = 0.05f;

    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (size_t i = 0; i < cam_pixels.size(); i += 3) {
        cam_pixels[i] = 255;
        cam_pixels[i + 1] = 0;
        cam_pixels[i + 2] = 255;
    }

    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    // A body-sized fallback box (no glTF needed), squarely between camera 0
    // and a real chunk of the bowl on the far side.
    mpviz::set_ego_model(r, "/nonexistent/path.glb", {1.6, 1.6, 0.7});
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));  // self_view_masks defaults false
    ASSERT_TRUE(mpviz::set_bowl_visible(r, true));
    ASSERT_TRUE(mpviz::set_camera_frame(r, 0, cam_pixels.data(), w, h, /*frame_id=*/1));

    std::vector<uint8_t> masks_off(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {masks_off.data(), 320, 240}));
    size_t magenta_off = 0;
    for (size_t i = 0; i < masks_off.size(); i += 3) {
        if (masks_off[i] > 150 && masks_off[i + 2] > 150) ++magenta_off;
    }
    ASSERT_GT(magenta_off, masks_off.size() / 3 / 100)
        << "sanity: the bowl should show its magenta sentinel with masks off";

    // Force-enable for this test only (shipped default stays false) and
    // re-bake -- set_self_view_masks() only stores the flag; build_bowl()
    // reads it at the NEXT set_bowl_config() call, same convention as
    // every other bake-time-only knob on this POD boundary.
    ASSERT_TRUE(mpviz::set_self_view_masks(r, true));
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));
    ASSERT_TRUE(mpviz::set_bowl_visible(r, true));
    ASSERT_TRUE(mpviz::set_camera_frame(r, 0, cam_pixels.data(), w, h, /*frame_id=*/2));

    std::vector<uint8_t> masks_on(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {masks_on.data(), 320, 240}));
    size_t magenta_on = 0;
    for (size_t i = 0; i < masks_on.size(); i += 3) {
        if (masks_on[i] > 150 && masks_on[i + 2] > 150) ++magenta_on;
    }
    EXPECT_LT(magenta_on, magenta_off)
        << "camera 0's own body between it and part of the bowl should have its contribution "
           "suppressed once self_view_masks is enabled (declared off by default, Decision 4)";

    size_t nonblack_pixels = 0;
    for (size_t i = 0; i < masks_on.size(); i += 3) {
        if (masks_on[i] > 5 || masks_on[i + 1] > 5 || masks_on[i + 2] > 5) ++nonblack_pixels;
    }
    EXPECT_GT(nonblack_pixels, masks_on.size() / 3 / 10)
        << "expected sky_color/theme background to still cover a meaningful fraction of the "
           "frame, not a mostly-black frame from uncovered/unshaded vertices";
    mpviz::destroy_renderer(r);
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
    // >10% of the frame is this test's acceptance bar (measured ~40% with
    // correct winding + rigPos wiring).
    EXPECT_GT(magenta_pixels, buf.size() / 3 / 10)
        << "expected the bowl's sampled surface to cover a meaningful fraction of the frame";
    mpviz::destroy_renderer(r);
}

TEST(Bowl, PerFragmentSamplingReadsTheCorrectPixelNotAMirroredOne) {
    // A UNIFORM magenta sentinel is blind to any position/orientation corruption (a
    // mirrored or offset rigPos samples magenta just as well as the correct
    // one). A camera texture split into four distinctly-colored quadrants
    // instead lets this test assert the on-screen left/right and
    // top/bottom ORDERING of sampled colors matches the overhead camera's
    // known orientation -- a mirrored-V (or any position corruption) would
    // scramble that ordering.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Overhead camera, straight down: col0=right=(1,0,0), col1=down=(0,-1,0)
    // (image +v/down is world -y), col2=fwd=(0,0,-1) (same convention as
    // OverheadCamera()/reproject.cu's CamDev). A pixel in the source
    // image's LEFT half (small xp) is world +x-ish (right = +x); a pixel in
    // the image's TOP half (small yp) is world +y-ish (image +v is world
    // -y, so image top/small-v is world +y).
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
    bc.feather_margin = 0.0;  // no feather -- isolate the position/UV question
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));
    ASSERT_TRUE(mpviz::set_bowl_visible(r, true));

    // Four quadrants, saturated + distinct on all 3 channels: TL=red,
    // TR=green, BL=blue, BR=yellow.
    std::vector<uint8_t> cam_pixels(static_cast<size_t>(w) * h * 3);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const bool left = x < w / 2;
            const bool top = y < h / 2;
            uint8_t* px = &cam_pixels[(static_cast<size_t>(y) * w + x) * 3];
            if (top && left) { px[0] = 255; px[1] = 0; px[2] = 0; }        // red
            else if (top && !left) { px[0] = 0; px[1] = 255; px[2] = 0; }  // green
            else if (!top && left) { px[0] = 0; px[1] = 0; px[2] = 255; }  // blue
            else { px[0] = 255; px[1] = 255; px[2] = 0; }                  // yellow
        }
    }
    ASSERT_TRUE(mpviz::set_camera_frame(r, 0, cam_pixels.data(), w, h, /*frame_id=*/1));

    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {buf.data(), 320, 240}));

    // Classify each on-screen pixel by its dominant sampled quadrant color
    // (exposure-compensated, so channels are boosted but the RED/GREEN/
    // BLUE/YELLOW signature survives: yellow needs high R AND G with low
    // B, the others need exactly one channel dominant).
    auto classify = [](uint8_t r8, uint8_t g8, uint8_t b8) -> char {
        const bool R = r8 > 120, G = g8 > 120, B = b8 > 120;
        if (R && G && !B) return 'Y';
        if (R && !G && !B) return 'R';
        if (!R && G && !B) return 'G';
        if (!R && !G && B) return 'B';
        return '.';
    };
    // Bucket by on-screen quadrant (this test's virtual camera looks down
    // at an angle, but the bowl sits centered in frame -- screen-left/
    // right and screen-top/bottom still partition it meaningfully).
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
    // The world's left/right camera-quadrant split (red|blue on world +x,
    // green|yellow on world -x -- right=(1,0,0) maps image-left to world
    // +x) must land predominantly on ONE screen side, not scrambled evenly
    // across both -- a mirrored or offset rigPos (the bug this test targets)
    // would scatter red/blue and green/yellow pixels roughly evenly on both
    // screen halves instead of separating them.
    EXPECT_GT(left_red + left_blue, 50) << "expected red/blue (image-left) quadrants on screen";
    EXPECT_GT(right_green + right_yellow, 50)
        << "expected green/yellow (image-right) quadrants on screen";
    EXPECT_LT(wrong_left_is_greenish, (left_red + left_blue) / 2)
        << "too much green/yellow leaking onto the red/blue screen side -- position/orientation "
           "corruption suspected";
    EXPECT_LT(wrong_right_is_reddish, (right_green + right_yellow) / 2)
        << "too much red/blue leaking onto the green/yellow screen side -- position/orientation "
           "corruption suspected";
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

TEST(Bowl, TwoCamerasWithNonzeroSlotIndexBothAppearInFrame) {
    // Every GPU render test above configures camera_count == 1, so only the
    // `idxA == 0` unrolled camera block in bowl.mat's fragment shader has
    // ever executed on a real GPU -- the bake-side tests above that use 2-3
    // cameras (BowlMeshBake.*) are CPU-only and never reach the shader. Two
    // overhead cameras straddling the bowl (camera 1 offset to +x, camera 0
    // to -x) makes camera 1 the per-triangle winner (nonzero index_a) over
    // the +x half of the bowl -- proving a non-zero slot's uniforms/sampler
    // are wired to the shader block that reads them, not just slot 0's.
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, -6, 6}, {0, 0, 0}, 70.0};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    // Same overhead R (col0=right=(1,0,0), col1=down=(0,-1,0), col2=fwd=
    // (0,0,-1)) as OverheadCamera() above, offset in t along x so each
    // camera's alignment (straight-down-ness) peaks on its own side of the
    // bowl.
    mpviz::CameraExtrinsics exts[2] = {
        {{1, 0, 0, 0, -1, 0, 0, 0, -1}, {-2.0, 0, 10.0}},
        {{1, 0, 0, 0, -1, 0, 0, 0, -1}, {2.0, 0, 10.0}},
    };
    mpviz::CameraIntrinsics ins[2] = {
        {300, 300, 160, 120, {0, 0, 0, 0, 0}},
        {300, 300, 160, 120, {0, 0, 0, 0, 0}},
    };
    uint32_t widths[2] = {320, 320};
    uint32_t heights[2] = {240, 240};
    mpviz::BowlConfig bc{};
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
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));
    ASSERT_TRUE(mpviz::set_bowl_visible(r, true));

    // Distinct saturated colors: camera 0 = pure red, camera 1 = pure green.
    std::vector<uint8_t> red(static_cast<size_t>(widths[0]) * heights[0] * 3);
    std::vector<uint8_t> green(static_cast<size_t>(widths[1]) * heights[1] * 3);
    for (size_t i = 0; i < red.size(); i += 3) { red[i] = 255; red[i + 1] = 0; red[i + 2] = 0; }
    for (size_t i = 0; i < green.size(); i += 3) { green[i] = 0; green[i + 1] = 255; green[i + 2] = 0; }
    ASSERT_TRUE(mpviz::set_camera_frame(r, 0, red.data(), widths[0], heights[0], /*frame_id=*/1));
    ASSERT_TRUE(mpviz::set_camera_frame(r, 1, green.data(), widths[1], heights[1], /*frame_id=*/1));

    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {buf.data(), 320, 240}));

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
    mpviz::destroy_renderer(r);
}
