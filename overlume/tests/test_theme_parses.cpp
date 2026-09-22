// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/scene.h"

#include "test_paths.hpp"

#include <gtest/gtest.h>

TEST(ThemeParses, LoadsAShippedThemeWithNoGpu) {
    EXPECT_TRUE(overlume::theme_parses(kThemeDir, "dark_adas"));
    EXPECT_TRUE(overlume::theme_parses(kThemeDir, "light_clay"));
}

TEST(ThemeParses, MissingThemeReturnsFalse) {
    EXPECT_FALSE(overlume::theme_parses(kThemeDir, "no_such_theme"));
}

TEST(ThemeParses, NullOrEmptyArgsReturnFalse) {
    EXPECT_FALSE(overlume::theme_parses(nullptr, "dark_adas"));
    EXPECT_FALSE(overlume::theme_parses(kThemeDir, nullptr));
    EXPECT_FALSE(overlume::theme_parses(nullptr, nullptr));
    EXPECT_FALSE(overlume::theme_parses("", "dark_adas"));
    EXPECT_FALSE(overlume::theme_parses(kThemeDir, ""));
}

TEST(ThemeParses, MalformedThemeYamlReturnsFalse) {
    EXPECT_FALSE(overlume::theme_parses(kThemeDir, "definitely_not_a_theme_stem"));
}

TEST(ThemeParses, CallableRepeatedlyWithNoRendererEverCreated) {
    EXPECT_TRUE(overlume::theme_parses(kThemeDir, "dark_adas"));
    EXPECT_FALSE(overlume::theme_parses(kThemeDir, "no_such_theme"));
    EXPECT_TRUE(overlume::theme_parses(kThemeDir, "light_clay"));
}
