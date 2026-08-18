// version.cpp — the only translation unit backing the `visual_renderer`
// static library at the end of Epic 0 Task 1 (a CMake STATIC library needs
// at least one .cpp).
//
// Deliberately does NOT implement api.h and does NOT include any Filament
// header: that proof already lives in smoke/filament_link_probe.cpp (build
// integration + archive-link order) and scripts/check_pod_header.sh (header
// POD-cleanliness). Task 2 replaces this file with the real
// create_renderer/render_frame implementation — its Step 1 failing gtest
// depends on those symbols being genuinely undefined until then, which a
// stub definition here would have quietly defeated.
namespace mpviz {
int visual_renderer_build_marker() { return 1; }
}  // namespace mpviz
