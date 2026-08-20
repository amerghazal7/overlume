// test_theme_parses.cpp — Epic 2 Task 1 (VM-020) Step 0.3.
//
// mpviz::theme_parses() exists so the two-yaml-cpp coexistence check (this
// archive's bundled, hidden-and-localized clang/libc++ yaml-cpp alongside
// the node's independently-built gcc/libstdc++ yaml_cpp_vendor, in ONE
// process) has a GPU-free entry point to call. The node-side half of that
// check -- calling mpviz_node::load_profile() and mpviz::theme_parses() in
// the same test binary -- is test/test_profile.cpp (node package, a later
// step); this file is the library-side half: proving theme_parses() itself
// needs no Engine/EGL/GPU, so it is even callable on a headless CI box in
// the first place. No renderer is created anywhere below -- that is the
// point being tested, not an omission.
#include "visual_renderer/scene.h"

#include "test_paths.hpp"

#include <gtest/gtest.h>

TEST(ThemeParses, LoadsAShippedThemeWithNoGpu) {
    // No mpviz::create_renderer() call anywhere in this test -- if this
    // needed a GPU it would have to GTEST_SKIP, defeating the whole reason
    // this entry point exists (Step 0's headless coexistence proof).
    EXPECT_TRUE(mpviz::theme_parses(kThemeDir, "dark_adas"));
    EXPECT_TRUE(mpviz::theme_parses(kThemeDir, "light_clay"));
}

TEST(ThemeParses, MissingThemeReturnsFalse) {
    EXPECT_FALSE(mpviz::theme_parses(kThemeDir, "no_such_theme"));
}

TEST(ThemeParses, NullOrEmptyArgsReturnFalse) {
    EXPECT_FALSE(mpviz::theme_parses(nullptr, "dark_adas"));
    EXPECT_FALSE(mpviz::theme_parses(kThemeDir, nullptr));
    EXPECT_FALSE(mpviz::theme_parses(nullptr, nullptr));
    EXPECT_FALSE(mpviz::theme_parses("", "dark_adas"));
    EXPECT_FALSE(mpviz::theme_parses(kThemeDir, ""));
}

TEST(ThemeParses, MalformedThemeYamlReturnsFalse) {
    // Reuses the sun_dir_a fixture dir's sibling fixture set is themed
    // correctly, so instead point at a directory that exists but holds no
    // <name>.yaml matching an invalid stem -- same "missing file" path
    // load_theme() already exercises, asserted here through the public
    // entry point rather than detail::load_theme directly (that's already
    // covered by test_theme.cpp; this file's job is only theme_parses()).
    EXPECT_FALSE(mpviz::theme_parses(kThemeDir, "definitely_not_a_theme_stem"));
}

TEST(ThemeParses, CallableRepeatedlyWithNoRendererEverCreated) {
    // Ordering-sensitive static state is the failure mode a single call
    // would miss (same reasoning as the node-side coexistence test calling
    // load_profile twice around the theme_parses call).
    EXPECT_TRUE(mpviz::theme_parses(kThemeDir, "dark_adas"));
    EXPECT_FALSE(mpviz::theme_parses(kThemeDir, "no_such_theme"));
    EXPECT_TRUE(mpviz::theme_parses(kThemeDir, "light_clay"));
}
