// bowl_mesh.cpp — see bowl_mesh.hpp. Pure host math, no Filament/ROS/CUDA
// includes -- portable and GPU-free-testable per Task 2 Step 3.
#include "bowl_mesh.hpp"

#include "bowl_projection.hpp"

#include <algorithm>
#include <cmath>

namespace mpviz::bowl {

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
    mpviz::Vec3 position;
    // Raw per-camera weight (feather * alignment^2, 0 if uncovered),
    // camera_count entries -- retained per logical vertex so the
    // per-triangle pair decision below can look up any vertex's weight for
    // any candidate camera without re-projecting.
    std::vector<float> camWeight;
};

}  // namespace

BowlMesh BakeBowlMesh(const BowlMeshParams& mesh_params, double bowl_R0, double bowl_k,
                      double bowl_Rmax, uint32_t camera_count,
                      const mpviz::CameraExtrinsics* extrinsics,
                      const mpviz::CameraIntrinsics* intrinsics, const uint32_t* cam_width,
                      const uint32_t* cam_height, double feather_margin) {
    BowlMesh mesh;
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
                const float xp = u * static_cast<float>(cam_width[c]);
                const float yp = v * static_cast<float>(cam_height[c]);
                const float feather =
                    BorderFeather(xp, yp, cam_width[c], cam_height[c], feather_margin);
                const float align = CameraAlignment(extrinsics[c], lv.position);
                lv.camWeight[c] = feather * align * align;
            }
        }
    }

    auto emit_triangle = [&](uint32_t i0, uint32_t i1, uint32_t i2) {
        const LogicalVertex* corners[3] = {&grid[i0], &grid[i1], &grid[i2]};

        // Per-triangle camera-pair decision (Decision 3's construction
        // rule): sum each camera's weight across the triangle's three
        // corners, keep the top 2. A triangle with fewer than 2 cameras
        // carrying any weight gets index_b == index_a with weight_b
        // forced to 0 below (never double-counts one camera as both
        // slots).
        uint32_t pairA = 0, pairB = 0;
        float bestA = -1.0f, bestB = -1.0f;
        for (uint32_t c = 0; c < camera_count; ++c) {
            float sum = 0.0f;
            for (int t = 0; t < 3; ++t) sum += corners[t]->camWeight[c];
            if (sum > bestA) {
                bestB = bestA;
                pairB = pairA;
                bestA = sum;
                pairA = c;
            } else if (sum > bestB) {
                bestB = sum;
                pairB = c;
            }
        }
        const bool has_second = camera_count > 1 && pairB != pairA;

        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int t = 0; t < 3; ++t) {
            BowlVertex bv;
            bv.position = corners[t]->position;
            bv.index_a = pairA;
            bv.weight_a = corners[t]->camWeight[pairA];
            bv.index_b = has_second ? pairB : pairA;
            bv.weight_b = has_second ? corners[t]->camWeight[pairB] : 0.0f;
            mesh.vertices.push_back(bv);
        }
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 1);
        mesh.indices.push_back(base + 2);
    };

    for (uint32_t ring = 0; ring < rings; ++ring) {
        for (uint32_t seg = 0; seg < segs; ++seg) {
            const uint32_t seg1 = (seg + 1) % segs;
            const uint32_t i00 = ring * segs + seg;
            const uint32_t i01 = ring * segs + seg1;
            const uint32_t i10 = (ring + 1) * segs + seg;
            const uint32_t i11 = (ring + 1) * segs + seg1;
            emit_triangle(i00, i01, i10);
            emit_triangle(i01, i11, i10);
        }
    }
    return mesh;
}

}  // namespace mpviz::bowl
