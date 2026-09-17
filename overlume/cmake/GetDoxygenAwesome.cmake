# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# GetDoxygenAwesome.cmake — fetches the pinned doxygen-awesome-css theme
# (jothepro/doxygen-awesome-css) used by the `docs` target's Doxyfile
# (HTML_EXTRA_STYLESHEET). Only included when OVERLUME_BUILD_DOCS is ON.
#
# Pin: v2.5.0 (https://github.com/jothepro/doxygen-awesome-css/releases).
# SHA256 computed directly against the tag's source tarball (same pin
# discipline as GetFilament.cmake/GetCesiumNative.cmake: a source-shaped
# CMake FetchContent, not a system package — this is a handful of static
# .css files, no build step, so FetchContent_MakeAvailable only populates
# it (no CMakeLists.txt in the tarball to add_subdirectory).

set(DOXYGEN_AWESOME_VERSION "2.5.0")

include(FetchContent)
FetchContent_Declare(
    doxygen_awesome
    URL "https://github.com/jothepro/doxygen-awesome-css/archive/refs/tags/v${DOXYGEN_AWESOME_VERSION}.tar.gz"
    URL_HASH SHA256=959b6e9369a0f5c7c0a9beba34b18a02bc5e788e68f6733975908f133a784dd5)
FetchContent_MakeAvailable(doxygen_awesome)

# doxygen_awesome_SOURCE_DIR (set by FetchContent_MakeAvailable) holds
# doxygen-awesome.css + doxygen-awesome-sidebar-only.css at its root.
set(DOXYGEN_AWESOME_CSS_DIR "${doxygen_awesome_SOURCE_DIR}")
