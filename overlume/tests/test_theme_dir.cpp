// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "theme_dir.hpp"
namespace fs = std::filesystem;
using overlume::detail::resolve_default_theme_dir;

TEST(ThemeDir, PrefersInstalledShareDirNextToModule) {
    const fs::path root =
        fs::weakly_canonical(fs::temp_directory_path()) / "overlume_theme_dir_test";
    fs::remove_all(root);
    fs::create_directories(root / "lib");
    fs::create_directories(root / "share/overlume/themes");
    std::ofstream(root / "share/overlume/themes/dark_adas.yaml") << "x: 1\n";
    EXPECT_EQ(resolve_default_theme_dir((root / "lib/liboverlume.so.0").string(), "/nonexistent"),
              (root / "share/overlume/themes").string());
    fs::remove_all(root);
}

TEST(ThemeDir, FindsFrameworkResourcesNextToBinary) {
    // Apple framework layout: Overlume.framework/{Overlume,Resources/themes}.
    const fs::path root =
        fs::weakly_canonical(fs::temp_directory_path()) / "overlume_theme_dir_framework_test";
    fs::remove_all(root);
    fs::create_directories(root / "Overlume.framework/Resources/themes");
    std::ofstream(root / "Overlume.framework/Resources/themes/dark_adas.yaml") << "x: 1\n";
    EXPECT_EQ(
        resolve_default_theme_dir((root / "Overlume.framework/Overlume").string(), "/nonexistent"),
        (root / "Overlume.framework/Resources/themes").string());
    fs::remove_all(root);
}

TEST(ThemeDir, FollowsSymlinkedLibDir) {
    // Merged-/usr: the loader reports /lib/liboverlume.so.0 for /usr/lib/liboverlume.so.0.
    const fs::path root =
        fs::weakly_canonical(fs::temp_directory_path()) / "overlume_theme_dir_symlink_test";
    fs::remove_all(root);
    fs::create_directories(root / "usr/lib");
    fs::create_directories(root / "usr/share/overlume/themes");
    std::ofstream(root / "usr/share/overlume/themes/dark_adas.yaml") << "x: 1\n";
    fs::create_directory_symlink("usr/lib", root / "lib");
    EXPECT_EQ(resolve_default_theme_dir((root / "lib/liboverlume.so.0").string(), "/nonexistent"),
              fs::weakly_canonical(root / "usr/share/overlume/themes").string());
    fs::remove_all(root);
}

TEST(ThemeDir, FallsBackToCompiledDefaultWhenNoShareDir) {
    EXPECT_EQ(resolve_default_theme_dir("/definitely/not/here/lib/x.so", "/compiled/themes"),
              "/compiled/themes");
}

TEST(ThemeDir, EmptyModulePathUsesCompiledDefault) {
    EXPECT_EQ(resolve_default_theme_dir("", "/compiled/themes"), "/compiled/themes");
}
