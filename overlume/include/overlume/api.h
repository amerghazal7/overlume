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

namespace overlume {

struct CameraPose {
    double eye[3];
    double target[3];
    double vfov_deg;
};

struct RenderConfig {
    uint32_t width;
    uint32_t height;
    uint8_t quality;  // 0=low, 1=med, 2=high
    // Both fields below are nullable, caller-owned, and borrowed ONLY for
    // the duration of the create_renderer() call they're passed to —
    // create_renderer copies each into internal owned string storage on
    // VisualRenderer before returning, and never retains the raw pointer
    // past that call (Epic 1 Task 2 Step 7 /
    // docs/superpowers/plans/2026-08-18-visual-mode-epic1.md "Interfaces
    // frozen this epic"). This matters because set_theme() (Epic 1 Task 3)
    // re-reads the retained copy on every future call, long after this
    // call's raw pointer is gone.
    const char* theme_assets_dir;  // dir containing *.yaml theme files;
                                    // null -> compiled-in default dir
    const char* initial_theme;     // theme name (yaml stem); null -> "dark_adas"
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

}  // namespace overlume
