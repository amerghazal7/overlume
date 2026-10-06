#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# check_vcpkg_conan_linux_static.sh RENDERED_DIR PKG_DIR
# The Linux static product needs clang >= 18 + libc++, which the hosted ubuntu-22.04 runner lacks:
# run check_vcpkg_conan.sh's static variant inside ubuntu:24.04 (clang 18, libc++ 18 from apt).
# Needs docker, VCPKG_ROOT (bootstrapped on the host) and network (apt + PyPI).
set -euo pipefail

if [ $# -ne 2 ]; then echo "usage: $0 RENDERED_DIR PKG_DIR" >&2; exit 2; fi
: "${VCPKG_ROOT:?VCPKG_ROOT is not set}"
conan_version="${CONAN_VERSION:-2.31.2}"
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
rendered="$(cd "$1" && pwd)"; pkgs="$(cd "$2" && pwd)"

docker run --rm -e VCPKG_ROOT=/vcpkg -e DEBIAN_FRONTEND=noninteractive \
  -v "$repo":/repo:ro -v "$rendered":/rendered:ro -v "$pkgs":/pkgs:ro -v "$(cd "$VCPKG_ROOT" && pwd)":/vcpkg:ro \
  ubuntu:24.04 bash -c '
    set -euo pipefail
    apt-get update -qq > /dev/null
    apt-get install -y -qq --no-install-recommends clang libc++-dev libc++abi-dev cmake make libegl-dev libgles-dev git curl \
        ca-certificates zip unzip tar pkg-config python3 python3-venv > /dev/null
    python3 -m venv /opt/conan && /opt/conan/bin/pip install -q "conan=='"$conan_version"'"
    export PATH=/opt/conan/bin:$PATH
    /repo/tools/release/check_vcpkg_conan.sh /rendered /pkgs static'
