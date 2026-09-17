// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// api.h — POD boundary. No standard-library container/string/smart-pointer
// types cross this header.
//
// Frozen for all Visual Mode epics (docs/plans/2026-08-18-visual-mode.md,
// Epic 0 Task 2). This header is the only thing the ROS-side (gcc/libstdc++)
// node includes from this clang/libc++ library — everything crossing it must
// stay POD/fixed-width/raw-pointer so the two C++ runtimes never touch at an
// ABI boundary. Checked by scripts/check_pod_header.sh.

/// @file
/// @brief POD-only renderer lifecycle: construction config, camera pose,
/// frame output, and the three free functions that create/destroy/render.
///
/// No standard-library container/string/smart-pointer type crosses this
/// header — see this file's own top comment and scripts/check_pod_header.sh.
#pragma once

#include <cstdint>
#include <cstddef>

namespace overlume {

/// @brief A camera position, look-at target, and field of view for one
/// render_frame() call.
struct CameraPose {
    double eye[3];     ///< Camera position, map frame, xyz.
    double target[3];  ///< Look-at point, map frame, xyz.
    double vfov_deg;   ///< Vertical field of view, degrees.
};

/// @brief Renderer construction parameters, passed to create_renderer().
struct RenderConfig {
    uint32_t width;   ///< Output frame width, pixels.
    uint32_t height;  ///< Output frame height, pixels.
    uint8_t quality;  ///< 0=low, 1=med, 2=high.
    // Both fields below are nullable, caller-owned, and borrowed ONLY for
    // the duration of the create_renderer() call they're passed to —
    // create_renderer copies each into internal owned string storage on
    // VisualRenderer before returning, and never retains the raw pointer
    // past that call (Epic 1 Task 2 Step 7 /
    // docs/plans/2026-08-18-visual-mode-epic1.md "Interfaces
    // frozen this epic"). This matters because set_theme() (Epic 1 Task 3)
    // re-reads the retained copy on every future call, long after this
    // call's raw pointer is gone.
    /// Directory containing `*.yaml` theme files; null -> compiled-in
    /// default dir. Borrowed only for the duration of create_renderer() —
    /// see this struct's own comment above.
    const char* theme_assets_dir;
    /// Theme name (yaml stem); null -> `"dark_adas"`. Borrowed only for the
    /// duration of create_renderer() — see this struct's own comment above.
    const char* initial_theme;
};

/// @brief Caller-owned output buffer for render_frame().
struct FrameView {
    uint8_t* rgb;     ///< Caller-owned buffer, `width*height*3` bytes.
    uint32_t width;   ///< Buffer width, pixels; must match the renderer.
    uint32_t height;  ///< Buffer height, pixels; must match the renderer.
};

/// @brief Opaque renderer handle. Never dereferenced by a caller — obtained
/// from create_renderer() and passed to every other entry point in this
/// header and in scene.h.
class VisualRenderer;  // opaque

/// @brief Creates a renderer for the given configuration.
/// @return A new renderer, or `nullptr` on failure.
VisualRenderer* create_renderer(const RenderConfig&);  // nullptr on failure
/// @brief Destroys a renderer created by create_renderer().
void destroy_renderer(VisualRenderer*);
/// @brief Renders one frame from the given camera pose into `out`.
/// @param out Caller-owned output buffer; must match the renderer's
/// configured width/height.
/// @return `true` on success.
bool render_frame(VisualRenderer*, const CameraPose&, FrameView out);

}  // namespace overlume
