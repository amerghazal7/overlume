// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#define OVERLUME_VERSION_MAJOR 0
#define OVERLUME_VERSION_MINOR 1
#define OVERLUME_VERSION_PATCH 0

namespace overlume {

#define OVERLUME_STR_(x) #x
#define OVERLUME_STR(x) OVERLUME_STR_(x)

inline constexpr const char* kVersionString = OVERLUME_STR(OVERLUME_VERSION_MAJOR) "." OVERLUME_STR(
    OVERLUME_VERSION_MINOR) "." OVERLUME_STR(OVERLUME_VERSION_PATCH);

}
