# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# vcpkg chainload for the arm64-linux-clang-libcxx triplet: the aarch64 release
# cross toolchain, plus -fPIC (the port archives end up inside liboverlume.so).
include("${CMAKE_CURRENT_LIST_DIR}/toolchain-llvm-release-aarch64.cmake")
string(APPEND CMAKE_C_FLAGS_INIT " -fPIC")
string(APPEND CMAKE_CXX_FLAGS_INIT " -fPIC")
# Some ports build executables (draco, ktx tools): link the static aarch64 libc++
# instead of toolchain-llvm-release's -nostdlib++ (liboverlume gets it explicitly).
string(REPLACE " -nostdlib++" "" CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT}")
set(CMAKE_CXX_STANDARD_LIBRARIES "-lc++abi -lunwind -lpthread -ldl")
# With the sysroot as the only find root, vcpkg's own prefix (installed/<triplet>)
# is invisible to find_package (vcpkg.cmake's addition to CMAKE_FIND_ROOT_PATH
# does not survive here). The vcpkg toolchain is CMAKE_TOOLCHAIN_FILE for port builds.
if(CMAKE_TOOLCHAIN_FILE MATCHES "/scripts/buildsystems/vcpkg\\.cmake$" AND VCPKG_TARGET_TRIPLET)
    get_filename_component(_ov_vcpkg_root "${CMAKE_TOOLCHAIN_FILE}/../../.." ABSOLUTE)
    list(APPEND CMAKE_FIND_ROOT_PATH "${_ov_vcpkg_root}/installed/${VCPKG_TARGET_TRIPLET}")
endif()
