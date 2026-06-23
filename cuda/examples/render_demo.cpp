/**
 * @file render_demo.cpp
 * @brief Headless render-to-file demo using micropilot_rendering::reprojector.
 *
 * Swap the PPM write for cv::VideoWriter to produce a video — kept out to
 * avoid an OpenCV dependency.  This demonstrates headless (no GL/window/display)
 * render-to-file, consumer mode 2.
 *
 * The demo:
 *   - Builds 4 ring cameras with distinct constant-color images (no fixture needed).
 *   - Sweeps the virtual camera through a short arc (5 poses).
 *   - Calls render_bowl for each pose.
 *   - Writes output as numbered binary PPM files: frame_000.ppm, frame_001.ppm, ...
 *
 * Build (against the installed tree, no GL/ROS/window):
 *   cmake -S . -B build_demo \
 *     -Dmicropilot_rendering_DIR=<install>/lib/cmake/micropilot_rendering
 *   cmake --build build_demo
 *
 * Run:
 *   ./build_demo/render_demo [output_dir]
 *   (output_dir defaults to ./demo_frames)
 */

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rendering_reprojector/reprojector.hpp"
#include "rendering_reprojector/types.hpp"

using micropilot::rendering::BowlParams;
using micropilot::rendering::CameraParams;
using micropilot::rendering::Reprojector;

// ---------------------------------------------------------------------------
// Minimal math helpers (no external deps)
// ---------------------------------------------------------------------------
static std::array<float, 3> cross3(std::array<float, 3> a,
                                   std::array<float, 3> b)
{
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

static float dot3(std::array<float, 3> a, std::array<float, 3> b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static std::array<float, 3> norm3(std::array<float, 3> v)
{
    float d = std::sqrt(dot3(v, v));
    if (d < 1e-12f) return v;
    return {v[0] / d, v[1] / d, v[2] / d};
}

static std::array<float, 3> sub3(std::array<float, 3> a,
                                  std::array<float, 3> b)
{
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

/**
 * Build a CameraParams using look_at geometry (CV convention: +Z fwd, +X right,
 * +Y down).  eye and target are world-frame positions.
 */
static CameraParams make_lookat_cam(std::array<float, 3> eye,
                                    std::array<float, 3> target,
                                    float fx, float fy,
                                    float cx, float cy,
                                    int w, int h)
{
    std::array<float, 3> up{0.f, 0.f, 1.f};
    std::array<float, 3> fwd = norm3(sub3(target, eye));
    std::array<float, 3> right = norm3(cross3(fwd, up));
    if (dot3(right, right) < 1e-10f)
    {
        up = {0.f, 1.f, 0.f};
        right = norm3(cross3(fwd, up));
    }
    std::array<float, 3> down = cross3(fwd, right);  // already unit if fwd,right unit

    // R column-major as stored row-major: R = [right | down | fwd] (columns)
    // Stored row-major (row i = R[i,:]):
    //   row0 = [right.x, down.x, fwd.x]
    //   row1 = [right.y, down.y, fwd.y]
    //   row2 = [right.z, down.z, fwd.z]
    CameraParams cam{};
    // K row-major
    cam.K[0] = fx; cam.K[1] = 0.f; cam.K[2] = cx;
    cam.K[3] = 0.f; cam.K[4] = fy; cam.K[5] = cy;
    cam.K[6] = 0.f; cam.K[7] = 0.f; cam.K[8] = 1.f;
    // R row-major = R^T of column-stacked matrix
    cam.R[0] = right[0]; cam.R[1] = down[0]; cam.R[2] = fwd[0];
    cam.R[3] = right[1]; cam.R[4] = down[1]; cam.R[5] = fwd[1];
    cam.R[6] = right[2]; cam.R[7] = down[2]; cam.R[8] = fwd[2];
    cam.t[0] = eye[0]; cam.t[1] = eye[1]; cam.t[2] = eye[2];
    cam.width  = w;
    cam.height = h;
    return cam;
}

// ---------------------------------------------------------------------------
// PPM writer (P6, binary)
// ---------------------------------------------------------------------------
static void write_ppm(const std::string& path, const float* rgba,
                      int width, int height)
{
    // Convert float [0,1] to uint8 for PPM
    std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 3);
    for (int i = 0; i < width * height; ++i)
    {
        auto clamp = [](float v) -> unsigned char {
            int vi = static_cast<int>(v * 255.f + 0.5f);
            return static_cast<unsigned char>(vi < 0 ? 0 : vi > 255 ? 255 : vi);
        };
        pixels[i * 3 + 0] = clamp(rgba[i * 4 + 0]);
        pixels[i * 3 + 1] = clamp(rgba[i * 4 + 1]);
        pixels[i * 3 + 2] = clamp(rgba[i * 4 + 2]);
    }

    std::ofstream f(path, std::ios::binary);
    if (!f.is_open())
        throw std::runtime_error("Cannot open " + path + " for writing");
    // PPM header
    std::string hdr = "P6\n" + std::to_string(width) + " " +
                      std::to_string(height) + "\n255\n";
    f.write(hdr.c_str(), static_cast<std::streamsize>(hdr.size()));
    f.write(reinterpret_cast<const char*>(pixels.data()),
            static_cast<std::streamsize>(pixels.size()));
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    const char* out_dir = (argc > 1) ? argv[1] : "demo_frames";

    // Create output directory (best-effort — mkdir may fail if it exists)
    std::string mkdir_cmd = std::string("mkdir -p ") + out_dir;
    std::system(mkdir_cmd.c_str());  // NOLINT

    // ---- Virtual camera size -------------------------------------------
    const int OUT_W = 96, OUT_H = 72;

    // ---- 4 ring cameras, 90 deg apart, each with a distinct solid color ---
    // Colors: red, green, blue, yellow
    const int CAM_W = 64, CAM_H = 48;
    const float radius = 0.25f, mount_h = 0.55f;
    const float tilt_deg = 10.0f;
    const float tilt_rad = tilt_deg * 3.14159265f / 180.f;

    // Intrinsics for hfov=85 deg at 64x48
    const float hfov_rad = 85.f * 3.14159265f / 180.f;
    const float fx = (CAM_W / 2.f) / std::tan(hfov_rad / 2.f);
    const float fy = fx;
    const float cx = CAM_W / 2.f, cy = CAM_H / 2.f;

    const int N_CAM = 4;
    // Solid colors per camera (R,G,B in [0,1])
    const float cam_colors[N_CAM][3] = {
        {0.9f, 0.2f, 0.2f},  // red
        {0.2f, 0.8f, 0.2f},  // green
        {0.2f, 0.3f, 0.9f},  // blue
        {0.9f, 0.8f, 0.1f},  // yellow
    };

    std::vector<CameraParams> cams;
    std::vector<float> images_nhwc(N_CAM * CAM_H * CAM_W * 3, 0.f);

    for (int i = 0; i < N_CAM; ++i)
    {
        float theta = 2.f * 3.14159265f * i / N_CAM;
        float ct = std::cos(theta), st = std::sin(theta);

        // Camera forward: outward + downward tilt
        std::array<float, 3> fwd_raw{
            ct * std::cos(tilt_rad),
            st * std::cos(tilt_rad),
            -std::sin(tilt_rad)};
        float fwd_len = std::sqrt(fwd_raw[0] * fwd_raw[0] +
                                  fwd_raw[1] * fwd_raw[1] +
                                  fwd_raw[2] * fwd_raw[2]);
        std::array<float, 3> fwd{fwd_raw[0] / fwd_len,
                                  fwd_raw[1] / fwd_len,
                                  fwd_raw[2] / fwd_len};
        std::array<float, 3> world_up{0.f, 0.f, -1.f};
        std::array<float, 3> right = norm3(cross3(world_up, fwd));
        std::array<float, 3> down = cross3(fwd, right);

        CameraParams cam{};
        cam.K[0] = fx; cam.K[1] = 0.f; cam.K[2] = cx;
        cam.K[3] = 0.f; cam.K[4] = fy; cam.K[5] = cy;
        cam.K[6] = 0.f; cam.K[7] = 0.f; cam.K[8] = 1.f;
        cam.R[0] = right[0]; cam.R[1] = down[0]; cam.R[2] = fwd[0];
        cam.R[3] = right[1]; cam.R[4] = down[1]; cam.R[5] = fwd[1];
        cam.R[6] = right[2]; cam.R[7] = down[2]; cam.R[8] = fwd[2];
        cam.t[0] = radius * ct;
        cam.t[1] = radius * st;
        cam.t[2] = mount_h;
        cam.width  = CAM_W;
        cam.height = CAM_H;
        cams.push_back(cam);

        // Fill image with solid color
        size_t img_start = static_cast<size_t>(i) * CAM_H * CAM_W * 3;
        for (int px = 0; px < CAM_H * CAM_W; ++px)
        {
            images_nhwc[img_start + px * 3 + 0] = cam_colors[i][0];
            images_nhwc[img_start + px * 3 + 1] = cam_colors[i][1];
            images_nhwc[img_start + px * 3 + 2] = cam_colors[i][2];
        }
    }

    // ---- Build reprojector -------------------------------------------------
    Reprojector rep(OUT_W, OUT_H);
    rep.set_cameras(cams);
    rep.upload_images(images_nhwc.data(), N_CAM, CAM_H, CAM_W);

    BowlParams bowl{6.0f, 0.08f, 20.0f};

    // ---- Virtual camera intrinsics (hfov=70, 96x72) -----------------------
    const float vhfov_rad = 70.f * 3.14159265f / 180.f;
    const float vfx = (OUT_W / 2.f) / std::tan(vhfov_rad / 2.f);
    const float vfy = vfx;
    const float vcx = OUT_W / 2.f, vcy = OUT_H / 2.f;

    // ---- Sweep: arc from rear-left to rear-right at fixed height ----------
    const int N_FRAMES = 5;
    std::vector<float> out(OUT_W * OUT_H * 4, 0.f);
    int written = 0;

    for (int fi = 0; fi < N_FRAMES; ++fi)
    {
        // Virtual camera orbits at radius 3m, height 2m, arc from -30 to +30 deg
        float ang_deg = -30.f + fi * (60.f / (N_FRAMES - 1));
        float ang = ang_deg * 3.14159265f / 180.f;
        std::array<float, 3> eye{
            3.f * std::cos(ang + 3.14159265f),   // behind the rig
            3.f * std::sin(ang + 3.14159265f),
            2.f};
        std::array<float, 3> target{0.f, 0.f, 0.f};
        CameraParams vcam = make_lookat_cam(eye, target, vfx, vfy, vcx, vcy,
                                            OUT_W, OUT_H);

        rep.render_bowl(vcam, bowl, out.data());

        // Write PPM
        char fname[256];
        std::snprintf(fname, sizeof(fname), "%s/frame_%03d.ppm", out_dir, fi);
        write_ppm(fname, out.data(), OUT_W, OUT_H);
        ++written;
        std::printf("  Wrote %s\n", fname);
    }

    std::printf("render_demo: wrote %d frames to %s/\n", written, out_dir);
    return 0;
}
