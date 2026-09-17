// test_paths.hpp — Epic 1 Task 2 Step 5a. Turns the MPVIZ_TEST_DATA_DIR /
// MPVIZ_DEFAULT_THEME_DIR compile definitions (CMakeLists.txt's
// `foreach(_test_src ...)` loop) into the one symbol every test that needs
// the shipped theme assets references. Included by every test file in this
// and later tasks that references either.
#pragma once

#ifndef MPVIZ_DEFAULT_THEME_DIR
#error "MPVIZ_DEFAULT_THEME_DIR must be defined by CMakeLists.txt"
#endif
#ifndef MPVIZ_TEST_DATA_DIR
#error "MPVIZ_TEST_DATA_DIR must be defined by CMakeLists.txt"
#endif

namespace mpviz::testing {
inline constexpr const char* kThemeDir = MPVIZ_DEFAULT_THEME_DIR;
}  // namespace mpviz::testing
using mpviz::testing::kThemeDir;
