#!/bin/bash
# $1 = build type (Debug|Release). Debug enables tests.
BUILD_TYPE=$1
if [ -z "$BUILD_TYPE" ]; then
    echo "Usage: $0 <Debug|Release>"; exit 1
fi
if [ "$BUILD_TYPE" == "Debug" ]; then
    BUILD_ARGS="-DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON"
elif [ "$BUILD_TYPE" == "Release" ]; then
    BUILD_ARGS="-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
else
    echo "Unknown build type '$BUILD_TYPE'."; exit 1
fi
cd ../../
BUILD_DIR="build"
[ -d "$BUILD_DIR" ] || mkdir "$BUILD_DIR"
cd $BUILD_DIR
PYBIND11_DIR=$(python3 -c "import pybind11; print(pybind11.get_cmake_dir())" 2>/dev/null || true)
if [ -n "$PYBIND11_DIR" ]; then
    BUILD_ARGS="$BUILD_ARGS -Dpybind11_DIR=$PYBIND11_DIR"
fi
cmake $BUILD_ARGS ..
cmake --build . --parallel $(($(nproc)/2))
cmake --install .
