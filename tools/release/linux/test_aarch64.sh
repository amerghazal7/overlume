#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Native aarch64 test run for the cross-built release. Runs as root inside an
# almalinux:8 arm64 container (so the glibc 2.28 floor is exercised for real),
# with the checkout mounted at /src, the test bundle unpacked into it
# (tools/release/make_test_bundle.sh) and the packages in /src/out.
#
#   docker run --rm -v "$PWD":/src -w /src almalinux:8 bash tools/release/linux/test_aarch64.sh
set -euo pipefail

CMAKE_VERSION=3.30.5
CMAKE_AARCH64_SHA256=da7dead2c92c1747b40d506d7f7d68590f5bab175316d2e7af73e48a2e417e48
build=/src/overlume/build-release-aarch64

[ "$(uname -m)" = aarch64 ] || { echo "FAIL: not an aarch64 host ($(uname -m))" >&2; exit 1; }

dnf -y install dnf-plugins-core epel-release >/dev/null
dnf config-manager --set-enabled powertools
# The build image's runtime set (Dockerfile) minus the toolchains.
dnf -y install binutils file which curl tar gzip \
    mesa-libEGL-devel mesa-libGL-devel mesa-dri-drivers >/dev/null
curl -fsSL -o /tmp/cmake.tgz \
    "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-aarch64.tar.gz"
echo "${CMAKE_AARCH64_SHA256}  /tmp/cmake.tgz" | sha256sum -c -
tar -xzf /tmp/cmake.tgz -C /usr/local --strip-components=1
rm /tmp/cmake.tgz

export EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1
/src/tools/release/check_glibc_floor.sh "$build/liboverlume.so.0"
for tgz in /src/out/overlume-*-linux-aarch64.tar.gz; do
    /src/tools/release/check_package_elf.sh "$tgz" AArch64
done
# FiftyObjectsSceneUpdateUnderTwoMilliseconds is a wall-clock budget that
# llvmpipe cannot meet (docs/status.md, known gap 11).
ctest --test-dir "$build" -L cpu --output-on-failure \
      -E FiftyObjectsSceneUpdateUnderTwoMilliseconds
