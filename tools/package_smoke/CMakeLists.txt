# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Standalone consumer of an installed overlume (find_package + pkg-config).
#   cmake -S tools/package_smoke -B b -DCMAKE_PREFIX_PATH=<prefix> [-DSMOKE_STATIC=ON]
cmake_minimum_required(VERSION 3.16)
project(package_smoke LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

option(SMOKE_STATIC "Build only the static-package consumer" OFF)
option(SMOKE_CONSUMER_YAML "Also link the consumer's own yaml-cpp (symbol-clash check)" ON)

if(SMOKE_STATIC)
    # clang++ -stdlib=libc++ consumer of the static component only.
    find_package(overlume 1.0 REQUIRED COMPONENTS static)
    add_executable(package_smoke_static main.cpp)
    target_compile_definitions(package_smoke_static PRIVATE SMOKE_STATIC_LINK
        SMOKE_THEMES_DIR="${overlume_THEMES_DIR}")
    target_link_libraries(package_smoke_static PRIVATE overlume::overlume_static)
    return()
endif()

set(SMOKE_YAML_TARGET "")
if(SMOKE_CONSUMER_YAML)
    find_package(yaml-cpp REQUIRED)
    if(TARGET yaml-cpp::yaml-cpp)
        set(SMOKE_YAML_TARGET yaml-cpp::yaml-cpp)
    else()
        set(SMOKE_YAML_TARGET yaml-cpp)   # yaml-cpp < 0.7
    endif()
endif()
if(NOT WIN32)
    find_package(PkgConfig REQUIRED)
endif()
find_package(overlume 1.0 REQUIRED)
add_executable(package_smoke main.cpp)
target_compile_definitions(package_smoke PRIVATE SMOKE_THEMES_DIR="${overlume_THEMES_DIR}")
target_link_libraries(package_smoke PRIVATE overlume::overlume ${SMOKE_YAML_TARGET})
if(SMOKE_CONSUMER_YAML)
    target_compile_definitions(package_smoke PRIVATE SMOKE_CONSUMER_YAML)
endif()

if(NOT WIN32)  # pkg-config is not shipped on Windows
    pkg_check_modules(OV REQUIRED IMPORTED_TARGET overlume)
    add_executable(package_smoke_pc main.cpp)
    target_compile_definitions(package_smoke_pc PRIVATE SMOKE_THEMES_DIR="${overlume_THEMES_DIR}")
    target_link_libraries(package_smoke_pc PRIVATE PkgConfig::OV ${SMOKE_YAML_TARGET})
    if(SMOKE_CONSUMER_YAML)
        target_compile_definitions(package_smoke_pc PRIVATE SMOKE_CONSUMER_YAML)
    endif()
    if(APPLE)
        # pkg-config gives -L/-l only; make the dylib loadable without DYLD_LIBRARY_PATH.
        target_link_options(package_smoke_pc PRIVATE "-Wl,-rpath,${OV_LIBDIR}")
    endif()
endif()
