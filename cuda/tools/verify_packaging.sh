#!/usr/bin/env bash
# verify_packaging.sh
# ---------------------------------------------------------------------------
# Proves that find_package(micropilot_rendering) works against the install
# tree with NO other CMake hints (no build-tree path, no PREFIX_PATH).
#
# It reuses cuda/examples/ (which already uses find_package) as the consumer.
# A throwaway build directory is created, built, and cleaned up on exit.
#
# Usage:
#   bash cuda/tools/verify_packaging.sh             (auto-locates install)
#   bash cuda/tools/verify_packaging.sh /path/to/install/libs
#
# Exit codes: 0 = PASS, 1 = FAIL
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CUDA_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Install prefix: default to cuda/install/libs
if [[ $# -ge 1 ]]; then
    INSTALL_PREFIX="$1"
else
    INSTALL_PREFIX="${CUDA_DIR}/install/libs"
fi

CMAKE_DIR="${INSTALL_PREFIX}/lib/cmake/micropilot_rendering"
EXAMPLES_DIR="${CUDA_DIR}/examples"
TMP_BUILD="$(mktemp -d)"

cleanup() {
    rm -rf "${TMP_BUILD}"
}
trap cleanup EXIT

echo "=== verify_packaging ==="
echo "install prefix : ${INSTALL_PREFIX}"
echo "cmake dir      : ${CMAKE_DIR}"
echo "consumer       : ${EXAMPLES_DIR}"
echo "build dir      : ${TMP_BUILD}"
echo ""

# 1. Sanity-check the install tree
if [[ ! -f "${CMAKE_DIR}/micropilot_renderingConfig.cmake" ]]; then
    echo "FAIL: micropilot_renderingConfig.cmake not found at ${CMAKE_DIR}"
    echo "      Did you run: cd cuda/scripts/libs_build && ./libs_build.sh Debug ?"
    exit 1
fi

# 2. Configure the consumer with ONLY the cmake dir hint — no other paths
echo "--- cmake configure ---"
cmake -S "${EXAMPLES_DIR}" -B "${TMP_BUILD}" \
    -Dmicropilot_rendering_DIR="${CMAKE_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    2>&1

echo ""
echo "--- cmake build ---"
cmake --build "${TMP_BUILD}" --parallel "$(nproc)" 2>&1

# 3. Run the resulting binary (smoke-run: --help / exit 0)
BINARY="${TMP_BUILD}/render_demo"
if [[ ! -x "${BINARY}" ]]; then
    echo ""
    echo "FAIL: render_demo binary not found at ${BINARY}"
    exit 1
fi

# render_demo needs no args to run (writes to ./demo_frames which mktemp dir will hold)
echo ""
echo "--- run binary ---"
cd "${TMP_BUILD}"
"${BINARY}" "${TMP_BUILD}/demo_frames" 2>&1 | head -10

echo ""
echo "PASS: find_package(micropilot_rendering) + build + run succeeded."
exit 0
