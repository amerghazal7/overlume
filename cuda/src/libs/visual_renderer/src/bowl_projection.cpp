// bowl_projection.cpp — see bowl_projection.hpp. Pure host math, no
// Filament/ROS/CUDA includes -- portable and GPU-free-testable per Task 2
// Step 0.
#include "bowl_projection.hpp"

#include <cmath>

namespace mpviz::bowl {

namespace {

struct Vec3d {
    double x, y, z;
};

Vec3d sub(const Vec3d& a, const Vec3d& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
double dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// R is row-major 3x3; column j is (R[j], R[3+j], R[6+j]) -- reproject.cu's
// own "R columns = (right,down,fwd)" convention (types.hpp).
Vec3d column(const double R[9], int j) { return {R[j], R[3 + j], R[6 + j]}; }

}  // namespace

bool ProjectToCameraUv(const mpviz::CameraExtrinsics& ext, const mpviz::CameraIntrinsics& in,
                       uint32_t width, uint32_t height, mpviz::Vec3 rig_point, float* out_u,
                       float* out_v) {
    const Vec3d right = column(ext.R, 0);
    const Vec3d down = column(ext.R, 1);
    const Vec3d fwd = column(ext.R, 2);
    const Vec3d t{ext.t[0], ext.t[1], ext.t[2]};
    const Vec3d P{rig_point.x, rig_point.y, rig_point.z};

    const Vec3d rel = sub(P, t);
    const double z = dot(fwd, rel);
    if (z <= 1e-9) return false;  // behind the camera

    double xn = dot(right, rel) / z;
    double yn = dot(down, rel) / z;

    const double k1 = in.dist[0], k2 = in.dist[1], p1 = in.dist[2], p2 = in.dist[3],
                 k3 = in.dist[4];
    if (k1 != 0.0 || k2 != 0.0 || p1 != 0.0 || p2 != 0.0 || k3 != 0.0) {
        // plumb_bob (OpenCV) forward distortion on normalized coords --
        // ported verbatim from reproject.cu's bowl_kernel. The polynomial
        // only holds inside the calibrated field (r2 <= 3, ~60 deg
        // off-axis); beyond that it can fold points back into frame, so
        // reject rather than distort garbage.
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

    // u/v span the whole image (0,0)..(1,1), same convention
    // ground_grid.cpp's quad UV already uses -- not (width-1)-normalized,
    // so a principal-point pixel (cx,cy) lands at exactly (0.5,0.5) rather
    // than a width-dependent near-0.5 value.
    *out_u = static_cast<float>(xp / static_cast<double>(width));
    *out_v = static_cast<float>(yp / static_cast<double>(height));
    return true;
}

mpviz::Vec3 BowlSurfacePoint(double bowl_R0, double bowl_k, double bowl_Rmax, double theta,
                             double r) {
    // Ported verbatim from reproject.cu's kernels/surface.cuh:
    // bowl_height(r) = k * clamp(r - R0, 0, Rmax - R0)^2 -- flat floor
    // inside R0, parabolic wall between R0 and Rmax, flat again (capped)
    // beyond Rmax.
    const double d = std::fmin(std::fmax(r - bowl_R0, 0.0), bowl_Rmax - bowl_R0);
    const double z = bowl_k * d * d;
    return {r * std::cos(theta), r * std::sin(theta), z};
}

float BorderFeather(float xp, float yp, uint32_t width, uint32_t height, double margin) {
    // Ported verbatim from blend.cuh's border_feather/smoothstep01.
    const double d = std::fmin(std::fmin(xp, (static_cast<double>(width) - 1.0) - xp),
                                std::fmin(yp, (static_cast<double>(height) - 1.0) - yp));
    if (margin <= 0.0) return d >= 0.0 ? 1.0f : 0.0f;
    double x = d / margin;
    x = std::fmin(std::fmax(x, 0.0), 1.0);
    return static_cast<float>(x * x * (3.0 - 2.0 * x));
}

float CameraAlignment(const mpviz::CameraExtrinsics& ext, mpviz::Vec3 rig_point) {
    const Vec3d fwd = column(ext.R, 2);
    const Vec3d t{ext.t[0], ext.t[1], ext.t[2]};
    const Vec3d rel = sub(Vec3d{rig_point.x, rig_point.y, rig_point.z}, t);
    const double len = std::sqrt(dot(rel, rel));
    if (len <= 1e-12) return 0.0f;
    const double align = dot(rel, fwd) / len;
    return static_cast<float>(std::fmin(std::fmax(align, 0.0), 1.0));
}

}  // namespace mpviz::bowl
