// Stub implementation for Epic 0 Task 1 (build-integration proof only).
// Real Filament backend lands in Epic 0 Task 2 — this file exists solely so
// the `visual_renderer` static-library target has a translation unit to
// compile, proving the clang/libc++ toolchain + Filament include path work
// end to end before any real rendering logic is written (TDD: prove the
// build, then fill it in).
#include "visual_renderer/api.h"

// Pulled in (but unused) to prove the Filament SDK headers resolve and
// compile clean under this directory's enforced clang++/libc++ flags.
#include <filament/Engine.h>

namespace mpviz {

class VisualRenderer {};

VisualRenderer* create_renderer(const RenderConfig&) {
    return nullptr;
}

void destroy_renderer(VisualRenderer* r) {
    delete r;
}

bool render_frame(VisualRenderer*, const CameraPose&, FrameView) {
    return false;
}

}  // namespace mpviz
