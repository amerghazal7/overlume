// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// bowl_mesh.cpp — see bowl_mesh.hpp. Pure host math, no Filament/ROS/CUDA
// includes -- portable and GPU-free-testable per Task 2 Step 3.
#include "bowl_mesh.hpp"

#include "bowl_projection.hpp"

#include <algorithm>
#include <cmath>

namespace overlume::bowl {

namespace {

// Cosmetic render-only z-lift: the bowl's flat inner floor (r <= R0) is
// EXACTLY coplanar with the base ground patch (ground_grid.cpp's own quad,
// renderer.cpp's r->ground) and the OGM ground-grid layers (which lift
// themselves by kGradientZLiftM/kDynamicZLiftM for the identical reason,
// ground_grid.cpp) -- without a lift here the two z-fight, and which one
// wins the depth test becomes an unpredictable per-pixel coin flip.
// Applied only to the MESH's rendered/sampled position (this is
// bowl_mesh.cpp, not bowl_projection.hpp's BowlSurfacePoint, which stays
// an exact, unlifted CUDA-parity port -- Task 5's lidar colorization must
// never see this cosmetic offset). Small enough (1 cm) to be a negligible
// reprojection error at any real camera distance.
constexpr double kBowlZLiftM = 0.01;

struct LogicalVertex {
    overlume::Vec3 position;
    // Raw per-camera alignment^2 coverage weight (0 if uncovered),
    // camera_count entries -- retained per logical vertex so the
    // per-triangle pair decision below can look up any vertex's weight for
    // any candidate camera without re-projecting. Border feather is NOT
    // included here -- bowl.mat computes it per-fragment.
    std::vector<float> camWeight;
};

// Slab-method segment/AABB intersection (Kay/Kajiya): does the CLOSED
// segment [from, to] cross the axis-aligned box [center-half, center+half]
// somewhere strictly ahead of `from`? Per-axis, intersect the segment's
// parametric interval with that axis's slab, then intersect all three
// axis-intervals together. A near-zero direction component on some axis
// (segment parallel to that axis's slab faces) is handled as "the
// segment's constant coordinate on that axis must already lie inside the
// slab" rather than a divide-by-zero branch.
//
// A segment that STARTS inside the box is a special case, not a variant of
// the general test: tmin would stay clamped at 0 and the interval would
// always be non-empty, reporting every such segment as occluded -- exactly
// wrong for a camera mounted inside its own ego box (every deployed camera
// in this rig is), which would then read as self-occluded against every
// vertex it looks at. A segment starting inside a convex box is never
// occluded by it -- a convex box can't be re-entered once exited, so a
// segment leaving from inside it is never occluded BY it; check that first
// and return false rather than folding it into the slab math.
bool SegmentIntersectsAabb(const overlume::Vec3& from, const overlume::Vec3& to,
                           const overlume::Vec3& center, const overlume::Vec3& half) {
    const double d[3] = {to.x - from.x, to.y - from.y, to.z - from.z};
    const double o[3] = {from.x - center.x, from.y - center.y, from.z - center.z};
    const double h[3] = {half.x, half.y, half.z};
    if (std::abs(o[0]) <= h[0] && std::abs(o[1]) <= h[1] && std::abs(o[2]) <= h[2]) {
        return false;
    }
    double tmin = 0.0, tmax = 1.0;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-12) {
            if (o[i] < -h[i] || o[i] > h[i]) return false;
            continue;
        }
        double t1 = (-h[i] - o[i]) / d[i];
        double t2 = (h[i] - o[i]) / d[i];
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    return true;
}

}  // namespace

BowlMesh BakeBowlMesh(const BowlMeshParams& mesh_params, double bowl_R0, double bowl_k,
                      double bowl_Rmax, uint32_t camera_count,
                      const overlume::CameraExtrinsics* extrinsics,
                      const overlume::CameraIntrinsics* intrinsics, const uint32_t* cam_width,
                      const uint32_t* cam_height, const EgoBox& ego_box) {
    BowlMesh mesh;
    // Zero-extent on every axis is EgoBox's own "no ego configured"
    // convention -- skip the per-camera segment test entirely rather than
    // run it against a box no camera could ever be meaningfully inside or
    // outside of.
    const bool ego_active = ego_box.half_extents.x > 0.0 || ego_box.half_extents.y > 0.0 ||
                            ego_box.half_extents.z > 0.0;
    const uint32_t rings = std::max<uint32_t>(mesh_params.radial_rings, 1);
    const uint32_t segs = std::max<uint32_t>(mesh_params.theta_segments, 3);

    // Inner radius: avoid a degenerate zero-radius ring at the bowl's
    // center (the robot itself occupies that area; Task 3's ego mesh
    // depth-composites over it). A small fraction of R0, floored, is
    // enough -- this mesh only needs to approximate the surface shape.
    const double r_inner = std::max(0.05 * bowl_R0, 1e-3);
    const double r_outer = std::max(bowl_Rmax, r_inner + 1e-3);

    std::vector<LogicalVertex> grid(static_cast<size_t>(rings + 1) * segs);
    for (uint32_t ring = 0; ring <= rings; ++ring) {
        const double r = r_inner + (r_outer - r_inner) * (static_cast<double>(ring) / rings);
        for (uint32_t seg = 0; seg < segs; ++seg) {
            const double theta = 2.0 * M_PI * static_cast<double>(seg) / segs;
            LogicalVertex& lv = grid[static_cast<size_t>(ring) * segs + seg];
            lv.position = BowlSurfacePoint(bowl_R0, bowl_k, bowl_Rmax, theta, r);
            lv.position.z += kBowlZLiftM;
            lv.camWeight.assign(camera_count, 0.0f);
            for (uint32_t c = 0; c < camera_count; ++c) {
                float u = 0.0f, v = 0.0f;
                if (!ProjectToCameraUv(extrinsics[c], intrinsics[c], cam_width[c], cam_height[c],
                                       lv.position, &u, &v)) {
                    continue;
                }
                // Alignment^2 coverage only -- border feather is computed
                // per-fragment by bowl.mat instead (a vertex-baked feather
                // doesn't match a per-fragment-sampled UV's own pixel-exact
                // border).
                const float align = CameraAlignment(extrinsics[c], lv.position);
                float weight = align * align;
                if (ego_active) {
                    const overlume::Vec3 cam_pos{extrinsics[c].t[0], extrinsics[c].t[1],
                                                 extrinsics[c].t[2]};
                    if (SegmentIntersectsAabb(cam_pos, lv.position, ego_box.center,
                                              ego_box.half_extents)) {
                        weight = 0.0f;
                    }
                }
                lv.camWeight[c] = weight;
            }
        }
    }

    auto emit_triangle = [&](uint32_t i0, uint32_t i1, uint32_t i2) {
        const LogicalVertex* corners[3] = {&grid[i0], &grid[i1], &grid[i2]};

        // Per-triangle camera-triplet decision (Decision 3's construction
        // rule, extended to 3 slots per Decision resolution 2's "2-3
        // contributing cameras per fragment" and VM-091 gate close-out
        // finding 7): sum each camera's weight across the triangle's three
        // corners, keep the top 3. A triangle with fewer than 3 cameras
        // carrying any weight gets the missing slot(s) mirror index_a with
        // coverage forced to 0 below (never double-counts one camera across
        // slots). Ties resolve to the lowest-index camera examined first
        // (strict `>` never displaces a first-seen max) -- same documented
        // behavior the 2-slot version had, now extended one slot further.
        uint32_t pairA = 0, pairB = 0, pairC = 0;
        float bestA = -1.0f, bestB = -1.0f, bestC = -1.0f;
        for (uint32_t c = 0; c < camera_count; ++c) {
            float sum = 0.0f;
            for (int t = 0; t < 3; ++t) sum += corners[t]->camWeight[c];
            if (sum > bestA) {
                bestC = bestB;
                pairC = pairB;
                bestB = bestA;
                pairB = pairA;
                bestA = sum;
                pairA = c;
            } else if (sum > bestB) {
                bestC = bestB;
                pairC = pairB;
                bestB = sum;
                pairB = c;
            } else if (sum > bestC) {
                bestC = sum;
                pairC = c;
            }
        }
        const bool has_second = camera_count > 1 && pairB != pairA;
        const bool has_third = camera_count > 2 && pairC != pairA && pairC != pairB;

        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int t = 0; t < 3; ++t) {
            BowlVertex bv;
            bv.position = corners[t]->position;
            bv.index_a = pairA;
            bv.coverage_a = corners[t]->camWeight[pairA];
            bv.index_b = has_second ? pairB : pairA;
            bv.coverage_b = has_second ? corners[t]->camWeight[pairB] : 0.0f;
            bv.index_c = has_third ? pairC : pairA;
            bv.coverage_c = has_third ? corners[t]->camWeight[pairC] : 0.0f;
            mesh.vertices.push_back(bv);
        }
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 1);
        mesh.indices.push_back(base + 2);
    };

    // Winding: e1 = P01-P00 is +theta, e2 = P10-P00 is +r, and
    // theta_hat x r_hat = -z_hat -- so (i00,i01,i10) has its normal
    // pointing DOWN/outward, back-facing the bowl's interior (matinfo
    // confirms this material's raster state is `Culling: back`, and
    // RenderableManager's `.culling(false)` in bowl.cpp is frustum
    // culling, not face culling). (i00,i10,i01)/(i01,i10,i11) instead keeps
    // the interior surface front-facing; no `doubleSided` override (Task 3
    // depth-composites the ego mesh over the bowl and wants correct
    // facing, not a double-sided patch).
    for (uint32_t ring = 0; ring < rings; ++ring) {
        for (uint32_t seg = 0; seg < segs; ++seg) {
            const uint32_t seg1 = (seg + 1) % segs;
            const uint32_t i00 = ring * segs + seg;
            const uint32_t i01 = ring * segs + seg1;
            const uint32_t i10 = (ring + 1) * segs + seg;
            const uint32_t i11 = (ring + 1) * segs + seg1;
            emit_triangle(i00, i10, i01);
            emit_triangle(i01, i10, i11);
        }
    }
    return mesh;
}

}  // namespace overlume::bowl
