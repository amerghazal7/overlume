#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# channel_smoke.sh repo SITE_DIR -- end-to-end check of the signed apt/yum
# repositories. Serves SITE_DIR with `python3 -m http.server` and, in
# ubuntu:22.04, debian:12, almalinux:8 and fedora:40 containers, adds the repo
# exactly as the README tells users to (apt: key in /etc/apt/keyrings +
# signed-by= source line; dnf: overlume.repo with its URL rewritten to the
# local server), installs `overlume`, builds tools/package_smoke against it and
# runs --expect-render. Then the tamper check: one flipped byte in InRelease
# must make `apt-get update` fail (the file is restored afterwards).
# Prints PASS/FAIL per step; exit 1 on any FAIL. Needs docker + network
# (image and distro package pulls). Override images with OVERLUME_SMOKE_IMAGES.
set -uo pipefail
[ $# -eq 2 ] && [ "$1" = repo ] || { echo "usage: $0 repo SITE_DIR" >&2; exit 2; }
site="$(cd "$2" && pwd)"
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
images=(ubuntu:22.04 debian:12 almalinux:8 fedora:40)
# shellcheck disable=SC2206
[ -n "${OVERLUME_SMOKE_IMAGES:-}" ] && images=($OVERLUME_SMOKE_IMAGES)
work="$(mktemp -d)"
port=$((20000 + RANDOM % 20000))
python3 -m http.server "$port" --bind 127.0.0.1 --directory "$site" > "$work/http.log" 2>&1 &
srv=$!
trap 'kill $srv 2>/dev/null; [ -f "$work/InRelease.orig" ] && cp "$work/InRelease.orig" "$site/apt/dists/stable/InRelease"; rm -rf "$work"' EXIT
base="http://127.0.0.1:$port"

cat > "$work/inner.sh" <<'INNER'
#!/usr/bin/env bash
set -u
image="$1"; mode="${2:-install}"; base="$3"
die() { echo "FAIL $image: $step"; exit 1; }
step=start; set -E; trap die ERR
run() { step="$1"; shift; if ! timeout 1500 "$@" > /tmp/step.log 2>&1; then tail -n 25 /tmp/step.log >&2; die; fi; }
case "$image" in ubuntu:*|debian:*) fam=deb;; *) fam=rpm;; esac

if [ "$fam" = deb ]; then
  export DEBIAN_FRONTEND=noninteractive
  run "apt-get update (distro)" apt-get update
  run "install curl" apt-get install -y --no-install-recommends ca-certificates curl
  # --- as the README instructs (URL = the local server) ---
  run "fetch key" bash -c "mkdir -p /etc/apt/keyrings && curl -fsSL $base/overlume-release.asc -o /etc/apt/keyrings/overlume.asc"
  echo "deb [signed-by=/etc/apt/keyrings/overlume.asc] $base/apt stable main" > /etc/apt/sources.list.d/overlume.list
  if [ "$mode" = tamper ]; then
    step="tampered InRelease must be refused"
    if apt-get update > /tmp/step.log 2>&1; then cat /tmp/step.log >&2; die; fi
    grep -Eq "not signed|BADSIG|NO_PUBKEY|invalid|signature" /tmp/step.log || { cat /tmp/step.log >&2; die; }
    echo "PASS $image: tampered InRelease refused"; exit 0
  fi
  run "apt-get update (overlume)" apt-get update
  run "install overlume + toolchain" apt-get install -y --no-install-recommends overlume g++ cmake make \
      pkg-config libyaml-cpp-dev libegl1 libegl-mesa0 libgl1-mesa-dri
else
  printf 'timeout=30\nminrate=100000\nretries=5\n' >> /etc/dnf/dnf.conf
  case "$image" in
    almalinux:8) run "enable powertools" bash -c 'dnf -y install dnf-plugins-core epel-release && dnf config-manager --set-enabled powertools';;
  esac
  run "install curl" dnf -y install curl
  # --- as the README instructs (URL rewritten to the local server) ---
  run "fetch overlume.repo" bash -c "curl -fsSL $base/overlume.repo -o /etc/yum.repos.d/overlume.repo \
      && sed -i 's#https://amerghazal7.github.io/overlume#$base#g' /etc/yum.repos.d/overlume.repo"
  run "install overlume + toolchain" dnf -y install overlume gcc-c++ cmake make pkgconf-pkg-config \
      yaml-cpp-devel mesa-libEGL mesa-dri-drivers
  step="repo key imported by dnf (gpgcheck + repo_gpgcheck active)"
  grep -q '^repo_gpgcheck=1' /etc/yum.repos.d/overlume.repo || die
fi
export EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1
run "configure consumer" env CXX=g++ cmake -S /smoke -B /tmp/b -DCMAKE_BUILD_TYPE=Release
run "build consumer" cmake --build /tmp/b
for t in package_smoke package_smoke_pc; do
  step="$t --expect-render"; out="$(/tmp/b/$t --expect-render 2>&1)" || { echo "$out" >&2; die; }
  case "$out" in *PASS*) ;; *) echo "$out" >&2; die;; esac
done
echo "PASS $image: installed from repo, renders"
INNER

dock() { # IMAGE MODE
  nice -n 15 ionice -c3 docker run --rm --network host -v "$repo/tools/package_smoke":/smoke:ro \
      -v "$work/inner.sh":/inner.sh:ro "$1" bash /inner.sh "$1" "$2" "$base" 2>&1
}

rc=0
pids=()
for img in "${images[@]}"; do
  dock "$img" install > "$work/${img//[:\/]/_}.log" &
  pids+=($!)
done
wait "${pids[@]}"
for img in "${images[@]}"; do
  log="$work/${img//[:\/]/_}.log"
  res="$(grep -E "^(PASS|FAIL) $img" "$log" | tail -n1)"
  [ -n "$res" ] || res="FAIL $img: no result (container error)"
  echo "$res"
  case "$res" in PASS*) ;; *) rc=1; tail -n 20 "$log" | sed 's/^/    | /';; esac
done

# Tamper check: flip one byte inside InRelease's signed text.
irel="$site/apt/dists/stable/InRelease"
cp "$irel" "$work/InRelease.orig"
python3 - "$irel" <<'PY'
import sys
p = sys.argv[1]; b = bytearray(open(p, "rb").read()); b[200] ^= 0x01; open(p, "wb").write(b)
PY
res="$(dock "ubuntu:22.04" tamper | grep -E '^(PASS|FAIL) ' | tail -n1)"
cp "$work/InRelease.orig" "$irel"; rm -f "$work/InRelease.orig"
[ -n "$res" ] || res="FAIL tamper: no result"
echo "$res"
case "$res" in PASS*) ;; *) rc=1;; esac
# Restored file must install again (proves the failure was the tamper, not the harness).
res="$(dock "ubuntu:22.04" install | grep -E '^(PASS|FAIL) ' | tail -n1)"
echo "restore check: ${res:-FAIL no result}"
case "$res" in PASS*) ;; *) rc=1;; esac
exit "$rc"
