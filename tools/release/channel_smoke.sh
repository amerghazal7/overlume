#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# channel_smoke.sh all RUN_ID [PAGES_RUN_ID]
#   End-to-end check of every README "Install" path that can run on Linux, against the artifacts of one
#   release.yml dry run (RUN_ID) and one pages.yml dry run (PAGES_RUN_ID; default: the newest successful
#   workflow_dispatch pages.yml run on the current branch; it must have been dispatched with run_id=RUN_ID,
#   the packages inside its site are compared byte for byte with RUN_ID's, a mismatch FAILs).
#   The README is the source of truth: the apt and dnf blocks are extracted from README.md and run verbatim
#   (minus sudo, URL pointed at the local server, -y added); for the other legs, the commands the smoke
#   depends on are grepped out of README.md and a drift FAILs. Downloads (gh, cached under ${OVERLUME_SMOKE_CACHE:-~/.cache/
#   overlume-channel-smoke}) and runs, as the README documents them:
#     sums     SHA256SUMS.asc verified against packaging/keys/overlume-release.asc in a fresh keyring,
#              then sha256sum -c --ignore-missing over the downloaded packages
#     archive  the Linux .tar.gz extracted to a prefix, consumer built with CMAKE_PREFIX_PATH (ubuntu:22.04)
#     repo     apt + dnf from the pages dry-run site (the `repo` mode below)
#     vcpkg    overlay port + Conan recipe: shared (host) and static (clang 18 + libc++, ubuntu:24.04)
#     android  NDK consumer of all four ABIs, Prefab CLI over the AAR, Maven bundle contents + signatures
#   macOS (brew, .pkg), iOS (SwiftPM), Windows (installer, zip, vcpkg, Conan) cannot run here; the CI jobs
#   package-macos / package-ios / package-windows / channels run those. Prints PASS/FAIL per step; exit 1
#   on any FAIL. Needs docker, gh (authenticated), java, ninja, cmake, conan 2, git, network.
#
# channel_smoke.sh repo SITE_DIR
#   Check of the signed apt/yum repositories alone. Serves SITE_DIR with `python3 -m http.server` and, in
# ubuntu:22.04, debian:12, almalinux:8 and fedora:40 containers, adds the repo
# exactly as the README tells users to (apt: key in /etc/apt/keyrings +
# signed-by= source line; dnf: overlume.repo with its URL rewritten to the
# local server), installs `overlume`, builds tools/package_smoke against it and
# runs --expect-render. Then the tamper check: one flipped byte in InRelease
# must make `apt-get update` fail (the file is restored afterwards).
# Prints PASS/FAIL per step; exit 1 on any FAIL. Needs docker + network
# (image and distro package pulls). Override images with OVERLUME_SMOKE_IMAGES.
# shellcheck disable=SC2329,SC2016  # functions run through step(); the container script is meant literally
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"

if [ "${1:-}" = all ]; then
  [ $# -ge 2 ] && [ $# -le 3 ] || { echo "usage: $0 all RUN_ID [PAGES_RUN_ID] | $0 repo SITE_DIR" >&2; exit 2; }
  run_id="$2"; pages_id="${3:-}"
  gh_repo="${OVERLUME_SMOKE_GH_REPO:-amerghazal7/overlume}"
  cache="${OVERLUME_SMOKE_CACHE:-$HOME/.cache/overlume-channel-smoke}"
  if [ -z "$pages_id" ]; then
    pages_id="$(gh run list -R "$gh_repo" --workflow pages.yml --branch "$(git -C "$repo" rev-parse --abbrev-ref HEAD)" \
        --event workflow_dispatch --status success --limit 1 --json databaseId -q '.[0].databaseId')"
    [ -n "$pages_id" ] || { echo "FAIL: no successful pages.yml run; pass PAGES_RUN_ID" >&2; exit 1; }
  fi
  rel="$cache/run-$run_id"; pg="$cache/run-$pages_id"
  work="$(mktemp -d)"; chmod 755 "$work"
  trap 'rm -rf "$work"' EXIT
  rc=0
  ok()   { echo "PASS $*"; }
  bad()  { echo "FAIL $*"; rc=1; }
  step() { local name="$1"; shift; if "$@" > "$work/step.log" 2>&1; then ok "$name"; else tail -n 30 "$work/step.log" | sed 's/^/    | /'; bad "$name"; fi; }
  fetch() { # RUN_ID DEST NAME...  (cached: a non-empty directory is reused)
    local id="$1" dest="$2"; shift 2
    for n in "$@"; do
      if [ -z "$(ls -A "$dest/$n" 2>/dev/null)" ]; then
        nice -n 15 gh run download "$id" -R "$gh_repo" -n "$n" -D "$dest/$n" > "$work/dl.log" 2>&1 \
          || { cat "$work/dl.log"; echo "FAIL download $n of run $id (expired?)"; exit 1; }
      fi
    done
  }
  echo "release run $run_id, pages run $pages_id, cache $cache"
  fetch "$run_id" "$rel" pkg-linux-x86_64 pkg-linux-aarch64 pkg-android maven-bundle release-sums channels-local
  fetch "$pages_id" "$pg" github-pages

  # ---- README drift: every command string the legs below hard-code must still be in README.md -------------
  readme_drift() {
    local s rc=0
    for s in '--overlay-ports=overlume-vcpkg-port/ports' \
             'conan create overlume-conan --version' '-o "overlume/*:shared=True"' '--build=missing' \
             'apt-get install libegl1 libgles2 libgl1' 'overlume::overlume_static' \
             'io.github.amerghazal7:overlume:' 'sha256sum -c --ignore-missing SHA256SUMS' 'find_package(overlume REQUIRED CONFIG)'; do
      grep -qF -- "$s" "$repo/README.md" || { echo "README.md no longer contains: $s"; rc=1; }
    done
    return "$rc"
  }
  step "README drift: strings the smoke depends on are still documented" readme_drift

  # ---- README assets: every release asset / download URL the README names exists in RUN_ID's SHA256SUMS -----
  readme_assets() {
    local v a rc=0 sums="$rel/release-sums/SHA256SUMS"
    v="$(sed -n 's/.*[ *]overlume-\(.*\)-linux-x86_64\.tar\.gz$/\1/p' "$sums")"
    [ -n "$v" ] || { echo "cannot derive the version from $sums"; return 1; }
    while read -r a; do
      grep -qE "^[0-9a-f]{64} [ *]$a\$" "$sums" || { echo "README names $a, not an asset of run $run_id"; rc=1; }
    done < <(grep -oE '[Oo]verlume[A-Za-z-]*-(<ver>|\$V)[A-Za-z0-9_.-]*\.(zip|exe|tar\.gz|pkg|aar|deb|rpm)' "$repo/README.md" | sed "s/<ver>/$v/; s/\\\$V/$v/" | sort -u)
    # every releases URL (host, org, repo, tag prefix) must be exact; every SHA256SUMS* name must be SHA256SUMS or SHA256SUMS.asc
    while read -r a; do
      [[ "$a" =~ ^https://github\.com/amerghazal7/overlume/releases(/download/v(<ver>|\$V))?$ ]] || { echo "README release URL is not .../releases or .../releases/download/v<ver>: $a"; rc=1; }
    done < <(grep -oE 'https://[^ )`"]*/releases[^ )`"]*' "$repo/README.md" | sed -E 's#/(overlume-|SHA256SUMS)[^/]*$##' | sort -u)
    while read -r a; do
      [[ "$a" == SHA256SUMS || "$a" == SHA256SUMS.asc ]] || { echo "README names $a, expected SHA256SUMS or SHA256SUMS.asc"; rc=1; }
    done < <(grep -oE 'SHA256SUMS[A-Za-z0-9_.-]*' "$repo/README.md" | sed -E 's/[.]+$//' | sort -u)
    while read -r a; do
      [[ "$a" == SHA256SUMS || "$a" == SHA256SUMS.asc ]] || grep -qE "^[0-9a-f]{64} [ *]$a\$" "$sums" || { echo "README downloads $a, not an asset of run $run_id"; rc=1; }
    done < <(grep -oE '(https://github\.com/amerghazal7/overlume/releases/download/v(<ver>|\$V)|\$B)/[^ )`"]+' "$repo/README.md" | sed -E 's#.*/##; s/<ver>/'"$v"'/g; s/\$V/'"$v"'/g' | sort -u)
    return "$rc"
  }
  step "README: every named release asset and download URL matches run $run_id" readme_assets

  # ---- sums: SHA256SUMS.asc against the committed key, fresh keyring; then the checksums ------------------
  verify_sums() {
    export GNUPGHOME="$work/gnupg"; mkdir -m 700 "$GNUPGHOME"
    gpg --batch --quiet --import "$repo/packaging/keys/overlume-release.asc" || return 1
    local fpr; fpr="$(gpg --batch --with-colons --list-keys | awk -F: '/^fpr:/ {print $10; exit}')"
    echo "signing key $fpr"
    gpg --batch --status-fd 1 --verify "$rel/release-sums/SHA256SUMS.asc" "$rel/release-sums/SHA256SUMS" 2>/dev/null \
        | grep -q "^\[GNUPG:\] VALIDSIG $fpr " || return 1
    mkdir "$work/all"
    for d in pkg-linux-x86_64 pkg-linux-aarch64 pkg-android; do
      for f in "$rel/$d"/*; do case "$f" in *SHA256SUMS-*) ;; *) ln -s "$f" "$work/all/";; esac; done
    done
    (cd "$work/all" && sha256sum -c --ignore-missing "$rel/release-sums/SHA256SUMS" | tee "$work/sums.out") || return 1
    [ "$(grep -c ': OK$' "$work/sums.out")" -ge 10 ]
  }
  step "sums: SHA256SUMS.asc verifies against packaging/keys/overlume-release.asc, sha256sum -c over the downloaded packages" verify_sums
  unset GNUPGHOME

  # ---- archive: tar.gz to a prefix, find_package via CMAKE_PREFIX_PATH ---------------------------------------
  archive_script='set -e
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq && apt-get install -y -qq --no-install-recommends g++ cmake make pkg-config libyaml-cpp-dev libegl1 libgles2 libgl1 libegl-mesa0 libgl1-mesa-dri >/dev/null
    mkdir /opt/o && tar -xzf /pkg/overlume-*-linux-x86_64.tar.gz -C /opt/o
    p=$(echo /opt/o/overlume-*-linux-x86_64)
    cmake -S /smoke -B /tmp/b -DCMAKE_PREFIX_PATH=$p -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build /tmp/b >/dev/null
    export EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 PKG_CONFIG_PATH=$p/lib/pkgconfig
    /tmp/b/package_smoke --expect-render | grep -q PASS'
  step "archive: tar.gz extracted to a prefix, find_package(overlume) consumer renders (ubuntu:22.04, gcc)" \
      nice -n 15 ionice -c3 docker run --rm --network host -v "$rel/pkg-linux-x86_64":/pkg:ro \
      -v "$repo/tools/package_smoke":/smoke:ro ubuntu:22.04 bash -c "$archive_script"

  # ---- repo: apt + dnf from the pages dry-run site ----------------------------------------------------------
  site="$work/site"; mkdir "$site"; tar -xf "$pg/github-pages/artifact.tar" -C "$site"
  pages_from_run() { # the site's packages must be the ones of RUN_ID
    local f n=0 other
    for f in "$rel"/pkg-linux-*/*.deb "$rel"/pkg-linux-*/*.rpm; do
      [ -e "$f" ] || continue
      case "$f" in *.deb) other="$site/apt/pool/main/$(basename "$f")";; *) other="$(find "$site/rpm" -name "$(basename "$f")" | head -n1)";; esac
      cmp -s "$f" "$other" || { echo "pages run not built from RUN_ID: $(basename "$f") differs or is missing"; return 1; }
      n=$((n + 1))
    done
    [ "$n" -ge 8 ]
  }
  step "pages run $pages_id was built from release run $run_id (every .deb/.rpm identical)" pages_from_run
  "$0" repo "$site" > "$work/repo.log" 2>&1; rrc=$?
  grep -E '^(PASS|FAIL|restore)' "$work/repo.log" | sed 's/^/repo: /'
  [ "$rrc" -eq 0 ] || { tail -n 30 "$work/repo.log" | sed 's/^/    | /'; rc=1; }

  # ---- vcpkg + Conan ----------------------------------------------------------------------------------------
  tools="$cache/tools"; mkdir -p "$tools"
  vc="$(sed -n 's/^ *VCPKG_COMMIT: *//p' "$repo/.github/workflows/release.yml" | head -n1)"
  cv="$(sed -n 's/^ *CONAN_VERSION: *"\(.*\)"/\1/p' "$repo/.github/workflows/release.yml" | head -n1)"
  cs="$(sed -n 's/^ *CONAN_SHA256_LINUX: *//p' "$repo/.github/workflows/release.yml" | head -n1)"
  setup_vcpkg() {
    if [ ! -x "$tools/vcpkg/vcpkg" ]; then
      rm -rf "$tools/vcpkg"; mkdir "$tools/vcpkg"; cd "$tools/vcpkg" || return 1
      git init -q && git remote add origin https://github.com/microsoft/vcpkg \
        && git fetch -q --depth 1 origin "$vc" && git checkout -q FETCH_HEAD || return 1
      [ "$(git rev-parse HEAD)" = "$vc" ] && ./bootstrap-vcpkg.sh -disableMetrics >/dev/null || return 1
    fi
    if [ ! -x "$tools/conan/bin/conan" ]; then
      rm -rf "$tools/conan"; mkdir "$tools/conan"
      curl -fsSL -o "$tools/conan.tgz" "https://github.com/conan-io/conan/releases/download/$cv/conan-$cv-linux-x86_64.tgz" || return 1
      echo "$cs  $tools/conan.tgz" | sha256sum -c - >/dev/null || return 1
      tar -xzf "$tools/conan.tgz" -C "$tools/conan"
    fi
  }
  step "vcpkg (pinned commit) and Conan (pinned checksum) tools" setup_vcpkg
  export VCPKG_ROOT="$tools/vcpkg" CONAN_BIN_DIR="$tools/conan/bin" PATH="$tools/conan/bin:$PATH"
  # channels-local is rendered for http://127.0.0.1:8000 (the run's own archives); pkg-linux-x86_64 holds them.
  step "vcpkg overlay port + Conan recipe, shared (Linux x86_64, gcc)" \
      nice -n 15 "$here/check_vcpkg_conan.sh" "$rel/channels-local" "$rel/pkg-linux-x86_64" shared
  step "vcpkg overlay port + Conan recipe, static (clang 18 + libc++, ubuntu:24.04)" \
      nice -n 15 ionice -c3 "$here/check_vcpkg_conan_linux_static.sh" "$rel/channels-local" "$rel/pkg-linux-x86_64"

  # ---- android ------------------------------------------------------------------------------------------------
  export ANDROID_NDK_HOME="${ANDROID_NDK_HOME:-$HOME/.cache/overlume-android/android-ndk-r27c}"
  if [ ! -d "$ANDROID_NDK_HOME" ]; then
    OVERLUME_ANDROID_HOME="$cache/android" "$repo/tools/android/setup_sdk.sh" ndk > /dev/null 2>&1
    export ANDROID_NDK_HOME="$cache/android/android-ndk-r27c"
  fi
  mkdir "$work/unz"; unzip -q "$rel"/pkg-android/overlume-*-android.zip -d "$work/unz"
  step "android: NDK consumer links shared + static for arm64-v8a armeabi-v7a x86 x86_64" \
      nice -n 15 "$repo/tools/android/check_consumer.sh" "$work/unz"
  step "android: Prefab CLI resolves the AAR (4 ABIs x c++_shared/c++_static)" \
      "$repo/tools/android/check_prefab.sh" "$(ls "$rel"/pkg-android/overlume-*-android.aar)"
  verify_maven() {
    export GNUPGHOME="$work/gnupg2"; mkdir -m 700 "$GNUPGHOME"
    gpg --batch --quiet --import "$repo/packaging/keys/overlume-release.asc" || return 1
    mkdir "$work/mvn"; unzip -q "$rel"/maven-bundle/*.zip -d "$work/mvn"
    local pom n=0 f
    pom="$(find "$work/mvn" -name 'overlume-*.pom' | head -n1)"; [ -n "$pom" ] || return 1
    grep -q '<groupId>io.github.amerghazal7</groupId>' "$pom" && grep -q '<artifactId>overlume</artifactId>' "$pom" || return 1
    [ -n "$(find "$work/mvn" -name 'overlume-*.aar' | head -n1)" ] || return 1
    while IFS= read -r f; do
      case "$f" in *.asc|*.md5|*.sha1) continue;; esac
      gpg --batch --verify "$f.asc" "$f" 2>/dev/null || { echo "bad signature: $f"; return 1; }
      [ "$(md5sum "$f" | cut -d' ' -f1)" = "$(cat "$f.md5")" ] || return 1
      n=$((n + 1))
    done < <(find "$work/mvn" -type f)
    [ "$n" -ge 3 ]  # pom, aar, sources/javadoc
  }
  step "android: Maven Central bundle holds io.github.amerghazal7:overlume (pom + aar), every file signed by the release key" verify_maven
  unset GNUPGHOME
  echo "not run here (covered by CI): brew + .pkg (package-macos), SwiftPM (package-ios), Windows installer/zip/vcpkg/Conan (package-windows, channels)"
  [ "$rc" -eq 0 ] && echo "ALL PASS" || echo "SOME FAILED"
  exit "$rc"
fi
[ $# -eq 2 ] && [ "$1" = repo ] || { echo "usage: $0 all RUN_ID [PAGES_RUN_ID] | $0 repo SITE_DIR" >&2; exit 2; }
site="$(cd "$2" && pwd)"
images=(ubuntu:22.04 debian:12 almalinux:8 fedora:40)
# shellcheck disable=SC2206
[ -n "${OVERLUME_SMOKE_IMAGES:-}" ] && images=($OVERLUME_SMOKE_IMAGES)
work="$(mktemp -d)"
port=$((20000 + RANDOM % 20000))
python3 -m http.server "$port" --bind 127.0.0.1 --directory "$site" > "$work/http.log" 2>&1 &
srv=$!
trap 'kill $srv 2>/dev/null; [ -f "$work/InRelease.orig" ] && cp "$work/InRelease.orig" "$site/apt/dists/stable/InRelease"; rm -rf "$work"' EXIT
base="http://127.0.0.1:$port"
readme="$repo/README.md"
# The README's own apt and dnf blocks, run as-is except: no sudo (containers run as root), the public URL points at
# the local server, -y for the installs. Optional lines ("overlume-static") are dropped.
block() { # MARKER-REGEX
  awk -v m="$1" '$0 ~ m {f=1} f && /^```bash/ {c=1; next} c && /^```/ {exit} c' "$readme" \
    | sed -e 's/  *#.*//' -e 's/sudo //' -e "s#https://amerghazal7.github.io/overlume#$base#g" \
          -e 's/apt-get install /apt-get install -y /' -e 's/dnf install /dnf -y install /' | grep -v 'overlume-static'
}
block '^\*\*Linux, apt' > "$work/readme_apt.sh"
block '^\*\*Linux, dnf' > "$work/readme_dnf.sh"
for f in apt dnf; do
  { grep -q "$base" "$work/readme_$f.sh" && grep -q install "$work/readme_$f.sh"; } \
    || { echo "FAIL README.md: the $f install block is missing or no longer points at amerghazal7.github.io/overlume"; exit 1; }
done

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
  if [ "$mode" = tamper ]; then
    step="tampered InRelease must be refused"
    # README block up to (excluding) the installs: its own `apt-get update` must refuse the tampered InRelease
    if bash -e <(grep -v 'apt-get install' /w/readme_apt.sh) > /tmp/step.log 2>&1; then cat /tmp/step.log >&2; die; fi
    grep -Eq "not signed|BADSIG|NO_PUBKEY|invalid|signature" /tmp/step.log || { cat /tmp/step.log >&2; die; }
    echo "PASS $image: tampered InRelease refused"; exit 0
  fi
  run "README apt block (key, source line, update, install overlume)" bash -e /w/readme_apt.sh
  run "install toolchain" apt-get install -y --no-install-recommends g++ cmake make \
      pkg-config libyaml-cpp-dev libegl1 libegl-mesa0 libgl1-mesa-dri
else
  printf 'timeout=30\nminrate=100000\nretries=5\n' >> /etc/dnf/dnf.conf
  case "$image" in
    almalinux:8) run "enable powertools" bash -c 'dnf -y install dnf-plugins-core epel-release && dnf config-manager --set-enabled powertools';;
  esac
  run "install curl" dnf -y install curl
  # README dnf block in two halves: the repo file download, then (after pointing the file's own baseurl/gpgkey at the
  # local server, which the published file cannot know) the install.
  run "README dnf block: fetch overlume.repo" bash -e <(grep -v 'dnf -y install' /w/readme_dnf.sh)
  run "point overlume.repo at the local server" sed -i "s#https://amerghazal7.github.io/overlume#$base#g" /etc/yum.repos.d/overlume.repo
  run "README dnf block: install overlume" bash -e <(grep 'dnf -y install' /w/readme_dnf.sh)
  run "install toolchain" dnf -y install gcc-c++ cmake make pkgconf-pkg-config \
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
      -v "$work/inner.sh":/inner.sh:ro -v "$work":/w:ro "$1" bash /inner.sh "$1" "$2" "$base" 2>&1
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
