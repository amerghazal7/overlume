# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Portable Linux release build image: AlmaLinux 8 (glibc 2.28 floor) + the
# official LLVM 18.1.8 release toolchain at /opt/llvm. Filament is built from
# source by overlume/cmake/GetFilament.cmake inside this image.
#
#   docker build -t overlume-release-linux:x86_64 tools/release/linux
#   docker run --rm -v "$PWD":/src -w /src overlume-release-linux:x86_64 \
#     bash -c 'cmake --toolchain overlume/cmake/toolchain-llvm-release.cmake \
#                    -S overlume -B overlume/build-release-linux -G Ninja ...'   # build type defaults to Release
#
# No tokens belong in this image; none are read at build time.
FROM almalinux:8

ARG LLVM_VERSION=18.1.8
ARG CMAKE_VERSION=3.30.5

# x86_64 and aarch64 pins (SHA256 recorded from the upstream downloads).
ARG LLVM_X86_64_SHA256=54ec30358afcc9fb8aa74307db3046f5187f9fb89fb37064cdde906e062ebf36
ARG LLVM_AARCH64_SHA256=dcaa1bebbfbb86953fdfbdc7f938800229f75ad26c5c9375ef242edad737d999
ARG CMAKE_X86_64_SHA256=f747d9b23e1a252a8beafb4ed2bc2ddf78cff7f04a8e4de19f4ff88e9b51dc9d
ARG CMAKE_AARCH64_SHA256=da7dead2c92c1747b40d506d7f7d68590f5bab175316d2e7af73e48a2e417e48

RUN dnf -y install dnf-plugins-core epel-release \
 && dnf config-manager --set-enabled powertools \
 && dnf -y install \
      git git-lfs ninja-build python3.11 perl perl-IPC-Cmd make which file xz bzip2 \
      curl tar gzip zip unzip patch pkgconf-pkg-config \
      gcc gcc-c++ \
      ncurses-compat-libs \
      mesa-libEGL-devel mesa-libGL-devel mesa-libGLU-devel mesa-dri-drivers \
      libX11-devel libXi-devel libXext-devel libXrandr-devel libXxf86vm-devel libXcomposite-devel \
      rpm-build dpkg dpkg-dev \
 && dnf clean all
RUN alternatives --set python3 /usr/bin/python3.11 \
 && ln -s /usr/bin/ninja-build /usr/local/bin/ninja

# LLVM (clang, lld, libc++, libc++abi, libunwind, compiler-rt) in /opt/llvm.
RUN set -eux; arch="$(uname -m)"; \
    case "$arch" in \
      x86_64)  f="clang+llvm-${LLVM_VERSION}-x86_64-linux-gnu-ubuntu-18.04"; sha="$LLVM_X86_64_SHA256";; \
      aarch64) f="clang+llvm-${LLVM_VERSION}-aarch64-linux-gnu";             sha="$LLVM_AARCH64_SHA256";; \
      *) echo "unsupported arch $arch" >&2; exit 1;; \
    esac; \
    curl -fsSL -o /tmp/llvm.tar.xz \
      "https://github.com/llvm/llvm-project/releases/download/llvmorg-${LLVM_VERSION}/${f}.tar.xz"; \
    echo "${sha}  /tmp/llvm.tar.xz" | sha256sum -c -; \
    mkdir /opt/llvm; tar -xJf /tmp/llvm.tar.xz -C /opt/llvm --strip-components=1; \
    rm /tmp/llvm.tar.xz

# CMake from the official release tarball (the distro one is < 3.24).
RUN set -eux; arch="$(uname -m)"; \
    case "$arch" in \
      x86_64)  sha="$CMAKE_X86_64_SHA256";; \
      aarch64) sha="$CMAKE_AARCH64_SHA256";; \
    esac; \
    f="cmake-${CMAKE_VERSION}-linux-${arch}"; \
    curl -fsSL -o /tmp/cmake.tgz \
      "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${f}.tar.gz"; \
    echo "${sha}  /tmp/cmake.tgz" | sha256sum -c -; \
    tar -xzf /tmp/cmake.tgz -C /usr/local --strip-components=1; \
    rm /tmp/cmake.tgz

ENV OVERLUME_LLVM_ROOT=/opt/llvm \
    PATH=/opt/llvm/bin:$PATH \
    EGL_PLATFORM=surfaceless \
    LIBGL_ALWAYS_SOFTWARE=1

WORKDIR /src
