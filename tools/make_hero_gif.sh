#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Regenerates the README hero: docs/assets/hero.gif (animated) and
# docs/assets/hero.png (poster still, also used as the social preview) and
# docs/assets/hero-light.png (same frame in the light_clay theme).
#
#   tools/make_hero_gif.sh
#
# Builds overlume/tools/render_hero (headless EGL, no Cesium, no network),
# renders a seamlessly periodic frame sequence, then encodes it with ffmpeg's
# two-pass palettegen/paletteuse. Needs: cmake, ffmpeg, a GPU/EGL context.
#
# Knobs (environment): GIF_WIDTH (880), FPS (8), FRAMES (88), POSTER_IDX (0; frame 0 is the busiest
# moment, render_hero --phase 0.25),
# GIF_COLORS (192), GIF_DITHER (bayer scale 3; sierra2_4a is nicer but ~2x the bytes), DENOISE (14), CAPTIONS (1; needs the DejaVu Sans Bold font or FONT=path),
# BUILD_DIR, KEEP_FRAMES_DIR (render here and keep the PNGs).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

GIF_WIDTH="${GIF_WIDTH:-880}"
FPS="${FPS:-8}"
FRAMES="${FRAMES:-88}"
POSTER_IDX="${POSTER_IDX:-0}"
GIF_COLORS="${GIF_COLORS:-192}"
GIF_DITHER="${GIF_DITHER:-bayer:bayer_scale=3}"
DENOISE="${DENOISE:-14}"
CAPTIONS="${CAPTIONS:-1}"
FONT="${FONT:-/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf}"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/overlume/build}"
OUT_GIF="${REPO_ROOT}/docs/assets/hero.gif"
OUT_PNG="${REPO_ROOT}/docs/assets/hero.png"
OUT_LIGHT="${REPO_ROOT}/docs/assets/hero-light.png"
MAX_BYTES=$((8 * 1024 * 1024))
TARGET_BYTES=$((7 * 1024 * 1024))

command -v ffmpeg >/dev/null || { echo "FAIL: ffmpeg not found" >&2; exit 1; }
command -v cmake >/dev/null || { echo "FAIL: cmake not found" >&2; exit 1; }

TOOLCHAIN="${REPO_ROOT}/overlume/cmake/toolchain-clang-libcxx.cmake"
if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
    if ! command -v clang++ >/dev/null; then
        "${REPO_ROOT}/overlume/scripts/setup_toolchain_cesium.sh"
    fi
    cmake --toolchain "${TOOLCHAIN}" -S overlume -B "${BUILD_DIR}" \
        -DOVERLUME_ENABLE_CESIUM=OFF -DCMAKE_BUILD_TYPE=Release
fi
cmake --build "${BUILD_DIR}" -j --target render_hero

WORK="${KEEP_FRAMES_DIR:-$(mktemp -d)}"
mkdir -p "${WORK}"
if [[ -z "${KEEP_FRAMES_DIR:-}" ]]; then
    trap 'rm -rf "${WORK}"' EXIT
fi
rm -f "${WORK}"/frame_*.png

"${BUILD_DIR}/render_hero" --out "${WORK}" --width 1280 --height 720 \
    --frames "${FRAMES}" --fps "${FPS}" --quality 2 >/dev/null

# Poster: raw render, no captions. Light theme: one separate still.
cp "${WORK}/$(printf 'frame_%04d.png' "${POSTER_IDX}")" "${OUT_PNG}"
LIGHT_DIR="${WORK}/light"
mkdir -p "${LIGHT_DIR}"
"${BUILD_DIR}/render_hero" --out "${LIGHT_DIR}" --width 1280 --height 720 --frames "${FRAMES}" \
    --fps "${FPS}" --quality 2 --theme light_clay --only "${POSTER_IDX}" >/dev/null
cp "${LIGHT_DIR}/$(printf 'frame_%04d.png' "${POSTER_IDX}")" "${OUT_LIGHT}"

# Filter chain: light temporal/spatial denoise (the big GIF-size win), scale,
# then optional feature captions. Segment times are fractions of the loop so
# they follow FRAMES/FPS
# (no theme beat any more). The loop is exactly periodic, so there is no
# seam blend.
DUR="$(awk -v n="${FRAMES}" -v f="${FPS}" 'BEGIN { printf "%.3f", n / f }')"
# The denoiser has frame-to-frame state, so frame 0 would look different from the rest at the wrap.
# Feed it 3 lead-in frames (the loop tail) and trim them off before the captions.
SEQ="${WORK}/seq"
mkdir -p "${SEQ}"
for k in 0 1 2; do ln -sf "${WORK}/$(printf 'frame_%04d.png' $((FRAMES - 3 + k)))" "${SEQ}/s_$(printf '%04d' "${k}").png"; done
for ((k = 0; k < FRAMES; k++)); do ln -sf "${WORK}/$(printf 'frame_%04d.png' "${k}")" "${SEQ}/s_$(printf '%04d' $((k + 3))).png"; done
VF="hqdn3d=${DENOISE}:${DENOISE}:0:0,scale=${GIF_WIDTH}:-2:flags=lanczos,trim=start_frame=3,setpts=PTS-STARTPTS"
if [[ "${CAPTIONS}" == "1" && -f "${FONT}" ]]; then
    seg() { # seg <from_frac> <to_frac> <text> <x> <y>   (fractions may be expressions of DUR)
        VF+=",drawtext=fontfile=${FONT}:text='$3':fontsize=h*0.040:fontcolor=white"
        VF+=":x=$4:y=$5"
        VF+=":enable='gte(t,$1)*lt(t,$2)'"
    }
    # One fixed-size pill (invisible text of the longest caption) under every caption.
    # Captions hard-cut (no fade), so the pill is never empty, including frame 0.
    VF+=",drawtext=fontfile=${FONT}:text='Real-time Filament  ·  POD-only C++ API  ·  ROS 2 node':fontsize=h*0.040"
    VF+=":fontcolor=white@0:box=1:boxcolor=black@0.45:boxborderw=10:x=16:y=14"
    cap() { seg "$1" "$2" "$3" '16' '14'; }  # top-left: never over the ego
    q="$(awk -v d="${DUR}" 'BEGIN { printf "%.3f %.3f %.3f", d * 0.25, d * 0.5, d * 0.75 }')"
    read -r q1 q2 q3 <<<"${q}"
    cap 0 "${q1}" 'Tracked objects  ·  predicted paths  ·  alerts'
    cap "${q1}" "${q2}" 'HD map  ·  path ribbons  ·  occupancy grid'
    cap "${q2}" "${q3}" 'Markers  ·  waypoints  ·  trajectory carpet'
    cap "${q3}" "${DUR}" 'Real-time Filament  ·  POD-only C++ API  ·  ROS 2 node'
fi

PALETTE="${WORK}/palette.png"
ffmpeg -v error -y -framerate "${FPS}" -i "${SEQ}/s_%04d.png" \
    -vf "${VF},palettegen=stats_mode=diff:max_colors=${GIF_COLORS}" "${PALETTE}"
ffmpeg -v error -y -framerate "${FPS}" -i "${SEQ}/s_%04d.png" -i "${PALETTE}" \
    -lavfi "${VF}[x];[x][1:v]paletteuse=dither=${GIF_DITHER}" \
    -loop 0 "${OUT_GIF}"

BYTES="$(stat -c %s "${OUT_GIF}")"
echo "hero.gif: ${BYTES} bytes ($((BYTES / 1024)) KiB), hero.png: $(stat -c %s "${OUT_PNG}") bytes"
if ((BYTES > MAX_BYTES)); then
    echo "FAIL: hero.gif exceeds the 8 MB hard cap" >&2
    exit 1
fi
if ((BYTES > TARGET_BYTES)); then
    echo "WARN: hero.gif exceeds the 7 MB target; lower FPS/GIF_WIDTH/GIF_COLORS" >&2
fi
echo "PASS"
