// bowl_mesh.hpp — VM-091 (unified-engine migration Task 2). Portable
// (no Filament/GL types, GPU-free-testable) bowl mesh generator + the
// per-vertex weight/camera-slot-index bake, split out of bowl.cpp for the
// same reason bowl_projection.hpp is its own TU (Task 2 Step 3: "extract
// the CPU-only bake step as a separately-callable function from the
// Filament-entity-creation step" -- bowl.cpp/bowl.hpp pull in
// renderer_internal.hpp/<filament/...> and so are never included by
// tests/*.cpp; this header has neither, so test_bowl.cpp includes it
// directly).
//
// Per Decision resolution 2 (2026-09-11, per-fragment sampling): this bake
// does NOT produce a UV -- the fragment shader recomputes it every frame
// from the rasterized world position via the per-camera K/plumb_bob
// uniforms and the ego-motion-delta uniform (bowl.mat/bowl.cpp). What this
// bake DOES produce, unchanged by that resolution, is each vertex's up to
// THREE covering cameras' (Decision resolution 2's own text: "2-3
// contributing cameras per fragment") baked ALIGNMENT-SQUARED coverage
// weight and camera-slot index -- Decision 3's own construction rule:
// camera-slot assignment is uniform per TRIANGLE (all three vertices carry
// the identical (index_a, index_b, index_c) triple, coverage 0 where a
// slot's camera doesn't cover that vertex), because CUSTOM0/CUSTOM1's index
// components are linearly interpolated by the rasterizer and an index that
// varies within one triangle would read as fractional garbage. This bake
// enforces that rule by never sharing a logical grid vertex across
// triangles -- each of a triangle's three corners gets its own duplicated
// BowlVertex record, its coverage_a/b/c computed against that ONE
// triangle's own camera-triplet decision (ponytail: bowl tessellation only
// needs to approximate the surface shape, per Decision resolution 2 -- the
// extra vertex-buffer memory this trades for is cheaper than detecting
// cross-triangle agreement first).
//
// The border feather is NOT baked here -- a bake step happens once per
// vertex while the fragment shader samples per-fragment, so a vertex-baked
// feather would produce a hard step at each camera's exact pixel-bounds
// cutoff instead of reproject.cu's smoothstep falloff. bowl.mat computes
// the feather itself, per fragment, from its
// `featherMargin` parameter (bowl.cpp pushes it from BowlConfig::
// feather_margin); this bake's coverage_a/coverage_b carry ONLY
// alignment^2 (bowl_projection::CameraAlignment squared), the part that
// only needs vertex-density fidelity, not the part that needs per-pixel
// fidelity.
#pragma once

#include "overlume/scene.h"

#include <cstdint>
#include <vector>

namespace overlume::bowl {

struct BowlVertex {
    overlume::Vec3 position;  // rig frame -- the vertex shader/fragment
                            // shader recompute UV from this per-camera.
    // Alignment^2 coverage weight only (border feather is per-fragment now,
    // bowl.mat's featherMargin parameter -- see this header's comment).
    float coverage_a = 0.0f;
    float coverage_b = 0.0f;
    // Third camera slot (VM-091 gate close-out, finding 7): Decision
    // resolution 2 states "2-3 contributing cameras per fragment" -- bowl.mat
    // had shipped only 2 (CUSTOM0), one short of that range. coverage_c/
    // index_c carry the third slot via a second interpolant (CUSTOM1).
    float coverage_c = 0.0f;
    uint32_t index_a = 0;
    uint32_t index_b = 0;
    uint32_t index_c = 0;
};

struct BowlMesh {
    std::vector<BowlVertex> vertices;
    std::vector<uint32_t> indices;  // triangle list, 3 entries per triangle
};

// Tessellation knobs -- only need to approximate the bowl SURFACE SHAPE
// (Decision resolution 2: UV fidelity is bounded by per-fragment sampling,
// not by mesh density), so these stay modest constants rather than a new
// config surface.
struct BowlMeshParams {
    uint32_t theta_segments = 64;
    uint32_t radial_rings = 24;
};

// VM-092 (Task 3, Decision 4): the ego's axis-aligned bounding box, RIG
// FRAME -- the same frame this bake operates in (Decision 3's frame
// convention), so it composes directly with BowlVertex::position and
// CameraExtrinsics::t with no transform. A zero-extent box on every axis
// (the default) means "no ego configured" -- BakeBowlMesh treats that as a
// no-op, same "missing data does nothing" convention as the rest of this
// POD boundary.
struct EgoBox {
    overlume::Vec3 center{0.0, 0.0, 0.0};
    overlume::Vec3 half_extents{0.0, 0.0, 0.0};
};

// Generates the bowl's radial-grid topology (rig frame, robot at origin,
// theta in [0, 2*pi), r from a small inner radius up to bowl_Rmax) and
// bakes each vertex's up-to-3 covering cameras' ALIGNMENT-SQUARED coverage
// weight (bowl_projection::CameraAlignment, squared -- border feather is
// NOT baked, see this header's comment) + camera-slot index, enforcing the
// per-triangle-uniform-index construction rule above. `camera_count` <=
// kMaxBowlCameras; `extrinsics`/`intrinsics`/`cam_width`/`cam_height` are
// `camera_count`-entry arrays, same contract as BowlConfig.
//
// `ego_box` (Decision 4): an analytic self-view occlusion test folded into
// this same per-vertex loop, before the per-triangle top-3 selection above
// -- a camera's weight for a vertex is zeroed here if the straight segment
// from that camera's rig-frame position to the vertex crosses ego_box, so
// an occluded camera never wins a slot a genuinely visible one could have
// taken. Occlusion MUST be resolved before ranking, or an occluded camera
// burns a slot: applying the test after selection would only zero an
// already-chosen slot's coverage, never let a genuinely visible camera take
// it instead. A cheap box/segment test, not a full per-camera depth-render
// pass (the fidelity/complexity simplification Decision 4 names explicitly). Defaults
// to a zero-extent box, this bake's own "no ego configured" no-op
// convention.
BowlMesh BakeBowlMesh(const BowlMeshParams& mesh_params, double bowl_R0, double bowl_k,
                      double bowl_Rmax, uint32_t camera_count,
                      const overlume::CameraExtrinsics* extrinsics,
                      const overlume::CameraIntrinsics* intrinsics, const uint32_t* cam_width,
                      const uint32_t* cam_height, const EgoBox& ego_box = EgoBox{});

}  // namespace overlume::bowl
