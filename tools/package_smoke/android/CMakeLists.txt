# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# NDK consumer of one ABI of the unpacked overlume-<ver>-android.zip (link-only check; the
# executables are not run). Uses ../main.cpp, the same consumer as the Linux smoke test.
#   cmake -S tools/package_smoke/android -B b -G Ninja --toolchain $ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake \
#         -DANDROID_ABI=<abi> -DANDROID_PLATFORM=26 -DANDROID_STL=c++_static -DOVERLUME_PREFIX=<unzipped>/<abi>
cmake_minimum_required(VERSION 3.16)
project(package_smoke_android LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(OVERLUME_PREFIX "" CACHE PATH "Unpacked ABI directory (has lib/cmake/overlume)")
list(APPEND CMAKE_FIND_ROOT_PATH "${OVERLUME_PREFIX}")   # the NDK toolchain restricts find_package to the sysroot
list(APPEND CMAKE_PREFIX_PATH "${OVERLUME_PREFIX}")
find_package(overlume 0.1 REQUIRED COMPONENTS static)
# On a device the themes live elsewhere than in the unpacked prefix on the build host.
set(OVERLUME_SMOKE_THEMES_DIR "${overlume_THEMES_DIR}" CACHE STRING "Themes dir baked into the static consumer")

add_executable(smoke_shared ../main.cpp)
target_compile_definitions(smoke_shared PRIVATE SMOKE_THEMES_DIR="${OVERLUME_SMOKE_THEMES_DIR}")
target_link_libraries(smoke_shared PRIVATE overlume::overlume)

add_executable(smoke_static ../main.cpp)
target_compile_definitions(smoke_static PRIVATE SMOKE_STATIC_LINK SMOKE_THEMES_DIR="${OVERLUME_SMOKE_THEMES_DIR}")
target_link_libraries(smoke_static PRIVATE overlume::overlume_static)
