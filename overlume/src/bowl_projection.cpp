// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "bowl_projection.hpp"

#include <cmath>

namespace overlume::bowl {

namespace {

struct Vec3d {
    double x, y, z;
};

Vec3d sub(const Vec3d& a, const Vec3d& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
double dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3d column(const double R[9], int j) { return {R[j], R[3 + j], R[6 + j]}; }

}

bool ProjectToCameraUv(const overlume::CameraExtrinsics& ext, const overlume::CameraIntrinsics& in,
                       uint32_t width, uint32_t height, overlume::Vec3 rig_point, float* out_u,
                       float* out_v) {
    const Vec3d right = column(ext.R, 0);
    const Vec3d down = column(ext.R, 1);
    const Vec3d fwd = column(ext.R, 2);
    const Vec3d t{ext.t[0], ext.t[1], ext.t[2]};
    const Vec3d P{rig_point.x, rig_point.y, rig_point.z};

    const Vec3d rel = sub(P, t);
    const double z = dot(fwd, rel);
    if (z <= 1e-9) return false;

    double xn = dot(right, rel) / z;
    double yn = dot(down, rel) / z;

    const double k1 = in.dist[0], k2 = in.dist[1], p1 = in.dist[2], p2 = in.dist[3],
                 k3 = in.dist[4];
    if (k1 != 0.0 || k2 != 0.0 || p1 != 0.0 || p2 != 0.0 || k3 != 0.0) {
        const double r2 = xn * xn + yn * yn;
        if (r2 > 3.0) return false;
        const double radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        const double xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn);
        const double yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn;
        xn = xd;
        yn = yd;
    }

    const double xp = in.fx * xn + in.cx;
    const double yp = in.fy * yn + in.cy;
    if (xp < 0.0 || xp > static_cast<double>(width) - 1.0 || yp < 0.0 ||
        yp > static_cast<double>(height) - 1.0) {
        return false;
    }

    *out_u = static_cast<float>(xp / static_cast<double>(width));
    *out_v = static_cast<float>(yp / static_cast<double>(height));
    return true;
}

overlume::Vec3 BowlSurfacePoint(double bowl_R0, double bowl_k, double bowl_Rmax, double theta,
                                double r) {
    const double d = std::fmin(std::fmax(r - bowl_R0, 0.0), bowl_Rmax - bowl_R0);
    const double z = bowl_k * d * d;
    return {r * std::cos(theta), r * std::sin(theta), z};
}

float BorderFeather(float xp, float yp, uint32_t width, uint32_t height, double margin) {
    const double d = std::fmin(std::fmin(xp, (static_cast<double>(width) - 1.0) - xp),
                               std::fmin(yp, (static_cast<double>(height) - 1.0) - yp));
    if (margin <= 0.0) return d >= 0.0 ? 1.0f : 0.0f;
    double x = d / margin;
    x = std::fmin(std::fmax(x, 0.0), 1.0);
    return static_cast<float>(x * x * (3.0 - 2.0 * x));
}

float CameraAlignment(const overlume::CameraExtrinsics& ext, overlume::Vec3 rig_point) {
    const Vec3d fwd = column(ext.R, 2);
    const Vec3d t{ext.t[0], ext.t[1], ext.t[2]};
    const Vec3d rel = sub(Vec3d{rig_point.x, rig_point.y, rig_point.z}, t);
    const double len = std::sqrt(dot(rel, rel));
    if (len <= 1e-12) return 0.0f;
    const double align = dot(rel, fwd) / len;
    return static_cast<float>(std::fmin(std::fmax(align, 0.0), 1.0));
}

}
