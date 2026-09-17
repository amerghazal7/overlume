// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// test_paths.hpp — Epic 1 Task 2 Step 5a. Turns the OVERLUME_TEST_DATA_DIR /
// OVERLUME_DEFAULT_THEME_DIR compile definitions (CMakeLists.txt's
// `foreach(_test_src ...)` loop) into the one symbol every test that needs
// the shipped theme assets references. Included by every test file in this
// and later tasks that references either.
#pragma once

#ifndef OVERLUME_DEFAULT_THEME_DIR
#error "OVERLUME_DEFAULT_THEME_DIR must be defined by CMakeLists.txt"
#endif
#ifndef OVERLUME_TEST_DATA_DIR
#error "OVERLUME_TEST_DATA_DIR must be defined by CMakeLists.txt"
#endif

namespace overlume::testing {
inline constexpr const char* kThemeDir = OVERLUME_DEFAULT_THEME_DIR;
}  // namespace overlume::testing
using overlume::testing::kThemeDir;
