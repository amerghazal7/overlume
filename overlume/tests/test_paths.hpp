// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#ifndef OVERLUME_DEFAULT_THEME_DIR
#error "OVERLUME_DEFAULT_THEME_DIR must be defined by CMakeLists.txt"
#endif
#ifndef OVERLUME_TEST_DATA_DIR
#error "OVERLUME_TEST_DATA_DIR must be defined by CMakeLists.txt"
#endif

namespace overlume::testing {
inline constexpr const char* kThemeDir = OVERLUME_DEFAULT_THEME_DIR;
}
using overlume::testing::kThemeDir;
