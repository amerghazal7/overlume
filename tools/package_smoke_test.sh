#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# package_smoke_test.sh PKG_DIR -- clean-room install matrix for the Linux
# release packages. Every image installs the .deb/.rpm from PKG_DIR with its
# own distro toolchain and yaml-cpp, builds tools/package_smoke against it
# (find_package + pkg-config), renders under llvmpipe, then repeats without a
# GPU driver. Extra cases: static package (ubuntu:22.04, almalinux:9, clang +
# libc++), the static-with-gcc configure guard, relocated tar.gz, upgrade and
# clean removal. Prints one "PASS <image>" / "FAIL <image>: <step>" per image;
# exit 1 on any FAIL. Needs docker and network (image + distro package pulls).
set -uo pipefail

if [ $# -ne 1 ]; then echo "usage: $0 PKG_DIR" >&2; exit 2; fi
pkg_dir="$(cd "$1" && pwd)"
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
images=(ubuntu:20.04 ubuntu:22.04 ubuntu:24.04 debian:11 debian:12 almalinux:8 almalinux:9 fedora:40)
# Debugging aid: OVERLUME_SMOKE_IMAGES="almalinux:9 fedora:40" runs a subset.
# shellcheck disable=SC2206
[ -n "${OVERLUME_SMOKE_IMAGES:-}" ] && images=($OVERLUME_SMOKE_IMAGES)
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

# ---- per-image script (runs inside the container as root) -------------------
cat > "$work/inner.sh" <<'INNER'
#!/usr/bin/env bash
set -u
image="$1"
step="start"
die() { echo "FAIL $image: $step"; exit 1; }
trap 'die' ERR
set -E
run() { step="$1"; shift; if ! timeout 1500 "$@" > "/tmp/step.log" 2>&1; then tail -n 25 /tmp/step.log >&2; die; fi; }
case "$image" in
  ubuntu:*|debian:*) fam=deb;;
  *) fam=rpm;;
esac
want_static=0
case "$image" in ubuntu:24.04|fedora:40) want_static=1;; esac  # need libc++ >= 18: el9 ships none, ubuntu 22.04 has 14

install_pkgs() {
  if [ "$fam" = deb ]; then
    export DEBIAN_FRONTEND=noninteractive
    if [ "$image" = debian:11 ]; then
      # bullseye LTS ended 2026-08: its repositories moved to archive.debian.org.
      sed -i -e '/debian-security/d' -e 's#deb.debian.org/debian#archive.debian.org/debian#' /etc/apt/sources.list
      echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99archive
    fi
    run "apt-get update" apt-get update
    # The image carries security updates that left the archive; pin to the archived versions.
    [ "$image" = debian:11 ] && run "downgrade libc6 to the archived version" apt-get install -y \
        --allow-downgrades libc6=2.31-13+deb11u11 libc-bin=2.31-13+deb11u11 perl-base=5.32.1-4+deb11u3
    run "install shared + toolchain" apt-get install -y --allow-downgrades --no-install-recommends \
        /pkg/overlume_*.deb g++ cmake make pkg-config libyaml-cpp-dev libegl1 libegl-mesa0 \
        libgl1-mesa-dri ca-certificates
    [ "$want_static" = 1 ] && run "install static + clang" apt-get install -y --allow-downgrades --no-install-recommends \
        /pkg/overlume-static_*.deb clang libc++-dev libc++abi-dev
  else
    # Abort and retry stalled mirror connections instead of hanging.
    printf 'timeout=30\nminrate=100000\nretries=5\nfastestmirror=True\n' >> /etc/dnf/dnf.conf
    case "$image" in
      almalinux:8) run "enable powertools" bash -c 'dnf -y install dnf-plugins-core epel-release && dnf config-manager --set-enabled powertools';;
      almalinux:9) run "enable crb" bash -c 'dnf -y install dnf-plugins-core epel-release && dnf config-manager --set-enabled crb';;
    esac
    run "import release key" rpm --import /keys/overlume-release.asc
    for p in /pkg/overlume-[0-9]*.rpm /pkg/overlume-static-[0-9]*.rpm; do
      [ -e "$p" ] || continue
      step="rpm -K $(basename "$p")"
      out="$(rpm -K "$p" 2>&1)"
      case "$out" in *"digests signatures OK"*) ;; *) echo "$out" >&2; die;; esac
    done
    run "install shared + toolchain" dnf -y install /pkg/overlume-[0-9]*.rpm gcc-c++ cmake make \
        pkgconf-pkg-config yaml-cpp-devel mesa-libEGL mesa-dri-drivers
    [ "$want_static" = 1 ] && run "install static + clang" dnf -y install \
        /pkg/overlume-static-[0-9]*.rpm clang libcxx-devel libcxxabi-devel
  fi
  return 0
}

install_pkgs
export EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1

run "configure shared consumer" env CXX=g++ cmake -S /smoke -B /tmp/b -DCMAKE_BUILD_TYPE=Release
run "build shared consumer" cmake --build /tmp/b
for t in package_smoke package_smoke_pc; do
  step="$t --expect-render"; out="$(/tmp/b/$t --expect-render 2>&1)" || { echo "$out" >&2; die; }
  case "$out" in *PASS*) ;; *) echo "$out" >&2; die;; esac
done

if [ "$want_static" = 1 ]; then
  run "configure static consumer (clang+libc++)" env CXX=clang++ CXXFLAGS=-stdlib=libc++ \
      cmake -S /smoke -B /tmp/bs -DSMOKE_STATIC=ON -DCMAKE_BUILD_TYPE=Release
  run "build static consumer" cmake --build /tmp/bs
  step="package_smoke_static --expect-render"
  out="$(/tmp/bs/package_smoke_static --expect-render 2>&1)" || { echo "$out" >&2; die; }
  case "$out" in *PASS*) ;; *) echo "$out" >&2; die;; esac
  step="static guard (g++ must fail at configure)"
  if env CXX=g++ cmake -S /smoke -B /tmp/bg -DSMOKE_STATIC=ON > /tmp/guard.log 2>&1; then die; fi
  tr '\n' ' ' < /tmp/guard.log | tr -s ' ' | grep -q "requires clang++ with -stdlib=libc++" || { tail -n 15 /tmp/guard.log >&2; die; }
fi

# Distro libc++ older than 18 (ubuntu 22.04 ships 14) must be refused at configure time.
if [ "$image" = ubuntu:22.04 ]; then
  run "install static + old clang" apt-get install -y --no-install-recommends \
      /pkg/overlume-static_*.deb clang libc++-dev libc++abi-dev
  step="static guard (clang < 18 must fail at configure)"
  if env CXX=clang++ CXXFLAGS=-stdlib=libc++ cmake -S /smoke -B /tmp/bo -DSMOKE_STATIC=ON > /tmp/guard.log 2>&1; then die; fi
  tr '\n' ' ' < /tmp/guard.log | tr -s ' ' | grep -q "LLVM >= 18" || { tail -n 15 /tmp/guard.log >&2; die; }
fi

# No GPU driver: with libglvnd's EGL vendor list emptied, eglInitialize fails exactly as on
# a box without Mesa, and create_renderer must return nullptr, not abort. (Removing the
# Mesa packages is not equivalent: Mesa 25 keeps software rendering inside libegl-mesa0.)
for t in package_smoke package_smoke_pc; do
  step="$t --expect-no-gpu"
  out="$(__EGL_VENDOR_LIBRARY_FILENAMES=/nonexistent/none.json /tmp/b/$t --expect-no-gpu 2>&1)" || { echo "$out" >&2; die; }
  case "$out" in *PASS*) ;; *) echo "$out" >&2; die;; esac
done

# The shared .deb must register its soname (shlibs) and run ldconfig on install.
if [ "$fam" = deb ]; then
  step="deb ships shlibs"
  grep -q '^liboverlume 0 overlume (= ' /var/lib/dpkg/info/overlume.shlibs 2>/dev/null || die
  step="deb postinst runs ldconfig"
  grep -q ldconfig /var/lib/dpkg/info/overlume.postinst 2>/dev/null || die
fi

# Upgrade path (same version reinstalled in place), then clean removal.
if [ "$fam" = deb ]; then
  run "reinstall" dpkg -i /pkg/overlume_*.deb
  names=overlume; { [ "$want_static" = 1 ] || [ "$image" = ubuntu:22.04 ]; } && names="overlume-static overlume"
  run "remove packages" apt-get remove -y --purge $names
else
  run "reinstall" rpm -U --replacepkgs /pkg/overlume-[0-9]*.rpm
  names=overlume; [ "$want_static" = 1 ] && names="overlume-static overlume"
  run "remove packages" rpm -e $names
fi
step="removal left files behind"
left="$(ls -d /usr/include/overlume /usr/share/overlume /usr/lib/cmake/overlume /usr/lib/liboverlume* \
  /usr/lib/overlume /usr/share/pkgconfig/overlume.pc /etc/ld.so.conf.d/overlume.conf 2>/dev/null || true)"
[ -z "$left" ] || { echo "$left" >&2; die; }

# Relocated prefix from the tar.gz (ubuntu:22.04 only).
if [ "$image" = ubuntu:22.04 ]; then
  mkdir /opt/ov
  run "extract tar.gz" tar -xzf /pkg/overlume-*-linux-*.tar.gz -C /opt/ov --strip-components=1
  run "configure relocated consumer" env CXX=g++ cmake -S /smoke -B /tmp/br -DCMAKE_PREFIX_PATH=/opt/ov
  run "build relocated consumer" cmake --build /tmp/br
  step="relocated package_smoke --expect-render"
  out="$(LD_LIBRARY_PATH=/opt/ov/lib /tmp/br/package_smoke --expect-render 2>&1)" || { echo "$out" >&2; die; }
  case "$out" in *PASS*) ;; *) echo "$out" >&2; die;; esac
  # The relocated .pc must resolve ${pcfiledir}/../.. to /opt/ov (nothing is left in /usr).
  step="relocated pkg-config: prefix"
  [ "$(realpath "$(PKG_CONFIG_PATH=/opt/ov/share/pkgconfig pkg-config --variable=libdir overlume)")" = /opt/ov/lib ] || die
  step="relocated package_smoke_pc --expect-render"
  out="$(LD_LIBRARY_PATH=/opt/ov/lib /tmp/br/package_smoke_pc --expect-render 2>&1)" || { echo "$out" >&2; die; }
  case "$out" in *PASS*) ;; *) echo "$out" >&2; die;; esac
fi
echo "PASS $image"
INNER

# ---- drive the matrix (images in parallel) ----------------------------------
rc=0
for img in "${images[@]}"; do
  (
    log="$work/${img//[:\/]/_}.log"
    docker run --rm -v "$pkg_dir":/pkg:ro -v "$repo/tools/package_smoke":/smoke:ro \
         -v "$repo/packaging/keys":/keys:ro -v "$work/inner.sh":/inner.sh:ro \
         "$img" bash /inner.sh "$img" > "$log" 2>&1 || true
    grep -E "^(PASS|FAIL) $img" "$log" > "$log.result" || echo "FAIL $img: no result (container error)" > "$log.result"
  ) &
done
wait
for img in "${images[@]}"; do
  log="$work/${img//[:\/]/_}.log"
  res="$(cat "$log.result")"
  echo "$res"
  case "$res" in PASS*) ;; *) rc=1; tail -n 20 "$log" | sed 's/^/    | /';; esac
done
exit "$rc"
