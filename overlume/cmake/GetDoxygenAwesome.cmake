# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set(DOXYGEN_AWESOME_VERSION "2.5.0")

include(FetchContent)
FetchContent_Declare(
    doxygen_awesome
    URL "https://github.com/jothepro/doxygen-awesome-css/archive/refs/tags/v${DOXYGEN_AWESOME_VERSION}.tar.gz"
    URL_HASH SHA256=959b6e9369a0f5c7c0a9beba34b18a02bc5e788e68f6733975908f133a784dd5)
FetchContent_MakeAvailable(doxygen_awesome)

set(DOXYGEN_AWESOME_CSS_DIR "${doxygen_awesome_SOURCE_DIR}")
