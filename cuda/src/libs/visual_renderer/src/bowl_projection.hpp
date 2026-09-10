// bowl_projection.hpp — VM-091 (unified-engine migration Task 2). Portable
// (no Filament/GL types, GPU-free-testable) pinhole + plumb_bob projection
// and bowl-surface math, ported verbatim from
// cuda/src/libs/rendering_reprojector/src/kernels/reproject.cu's own
// bowl_kernel per-camera loop (read in full before porting; NOT re-derived
// from a generic distortion reference, per Task 2 Step 0's own instruction --
// Task 5's lidar colorization adapter reuses ProjectToCameraUv, so it must
// stay bit-consistent with what the bowl itself does).
//
// mpviz::CameraExtrinsics.R is row-major; per rendering_reprojector's own
// types.hpp doc, "R columns = (right, down, fwd)" -- this file reads R's
// COLUMNS as that basis (right = column 0, down = column 1, fwd = column
// 2), exactly like reproject.cu's CamDev.
//
// Used by both bowl.cpp's per-vertex weight/camera-index bake (this task)
// and Task 5's lidar-colorization adapter -- one implementation, two call
// sites, per Decision 3's "same toolchain, one copy not two" precedent.
#pragma once

#include "visual_renderer/scene.h"

#include <cstdint>

namespace mpviz::bowl {

// RIG-frame point -> this camera's pixel UV [0,1]^2 (u = xp/width, v =
// yp/height -- the same "whole image maps to (0,0)..(1,1)" convention
// ground_grid.cpp's quad UV already uses), applying plumb_bob distortion
// forward exactly as reproject.cu's bowl_kernel does. Returns false if the
// point is behind the camera (z <= 1e-9), outside the plumb_bob polynomial's
// calibrated field (r2 > 3.0, same guard as the CUDA kernel), or projects
// outside the camera's pixel bounds -- the caller then treats this
// camera/vertex pair as zero weight (Decision 3/4).
bool ProjectToCameraUv(const mpviz::CameraExtrinsics& ext, const mpviz::CameraIntrinsics& in,
                       uint32_t width, uint32_t height, mpviz::Vec3 rig_point, float* out_u,
                       float* out_v);

// Bowl surface point at (theta, r): flat floor inside R0, parabolic wall
// k*(r-R0)^2 between R0 and Rmax, clamped flat beyond Rmax -- ported
// verbatim from reproject.cu's kernels/surface.cuh (bowl_height/bowl_g),
// which is itself types.hpp's own documented BowlParams parity. theta is
// measured about +Z (rig frame), r is radial distance from the rig origin.
mpviz::Vec3 BowlSurfacePoint(double bowl_R0, double bowl_k, double bowl_Rmax, double theta,
                             double r);

// Feather weight from a pixel's distance to its image border -- ported
// verbatim from blend.cuh's border_feather/smoothstep01 (reproject.cu's own
// seam-blend weight). `margin` is in source-image PIXELS, same unit as
// BowlConfig::feather_margin and reproject.cu's own field.
float BorderFeather(float xp, float yp, uint32_t width, uint32_t height, double margin);

// Alignment weight: how squarely a camera faces a rig-frame point, clamped
// to [0,1] -- ported verbatim from reproject.cu's bowl_kernel
// (`align = clamp(dot(normalize(rel), fwd), 0, 1)`). The bowl bake and
// Task 5's colorization both square this (align*align) per Decision 3.
float CameraAlignment(const mpviz::CameraExtrinsics& ext, mpviz::Vec3 rig_point);

}  // namespace mpviz::bowl
