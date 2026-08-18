// api.h — POD boundary. No standard-library container/string/smart-pointer
// types cross this header.
//
// Frozen for all Visual Mode epics (docs/superpowers/plans/2026-08-18-visual-mode.md,
// Epic 0 Task 2). This header is the only thing the ROS-side (gcc/libstdc++)
// node includes from this clang/libc++ library — everything crossing it must
// stay POD/fixed-width/raw-pointer so the two C++ runtimes never touch at an
// ABI boundary. Checked by scripts/check_pod_header.sh.
#pragma once

#include <cstdint>
#include <cstddef>

namespace mpviz {

struct CameraPose {
    double eye[3];
    double target[3];
    double vfov_deg;
};

struct RenderConfig {
    uint32_t width;
    uint32_t height;
    uint8_t quality;  // 0=low, 1=med, 2=high
};

struct FrameView {
    uint8_t* rgb;  // caller-owned buffer, w*h*3 bytes
    uint32_t width;
    uint32_t height;
};

class VisualRenderer;  // opaque

VisualRenderer* create_renderer(const RenderConfig&);  // nullptr on failure
void destroy_renderer(VisualRenderer*);
bool render_frame(VisualRenderer*, const CameraPose&, FrameView out);

}  // namespace mpviz
