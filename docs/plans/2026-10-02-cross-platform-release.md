# Cross-Platform Release Packaging Implementation Plan

> **For agentic workers:** executed as dynamic workflows per `AGENTS.md`
> (orchestrator on the session model, implementers on Sonnet, review gates on
> Opus, ≤2 fix rounds per task). Steps use checkbox (`- [ ]`) syntax.

**Goal:** Publishing a GitHub release (`v*` tag) builds, tests, signs and
publishes Overlume for every supported platform, installable through each
platform's native channel:

| Platform | Artifacts (release assets) | Install channel |
|---|---|---|
| Linux x86_64 / aarch64 | `.deb`, `.rpm`, `.tar.gz` (shared + static) | signed apt + dnf/yum repos on GitHub Pages |
| macOS universal2 (arm64+x86_64) | `.pkg`, `.tar.gz` (shared + static) | Homebrew tap `amerghazal7/homebrew-overlume` |
| iOS (device arm64, simulator arm64+x86_64) | `Overlume.xcframework.zip` (dynamic) + static xcframework | Swift Package Manager (see Decision D7) |
| Windows x64 / arm64 | NSIS `.exe`, `.zip` (shared + static) | vcpkg overlay port, Conan recipe |
| Android arm64-v8a, armeabi-v7a, x86_64, x86 | Prefab `.aar`, `.zip` (shared + static) | Maven Central `io.github.amerghazal7:overlume` |

Linux and macOS also get pkg-config; every platform gets `find_package(overlume)`.
The vcpkg port and Conan recipe cover Linux and macOS too.

**Architecture:** One CMake project, one `OBJECT` library feeding two link
products: `overlume` (static, unchanged for the ROS node/tests/gate) and
`overlume_shared` (shared, self-contained: Filament, yaml-cpp, cesium-native
and the C++ runtime baked in, only `overlume::*` exported). Both are
installed as separate components (`overlume` = shared runtime + headers +
assets, `static` = static archives). The headless EGL platform becomes one
of four compile-time back ends (EGL/desktop-GL on Linux, EGL/GLES on
Android, Metal on macOS and iOS, WGL/OpenGL on Windows). Release Linux
builds run in an AlmaLinux 8 container (glibc 2.28 floor) with the official
LLVM 18.1.8 release toolchain and Filament from source. CPack produces native
packages; `release.yml` runs a build matrix, then publish jobs per channel.

**Tech Stack:** CMake ≥ 3.24 + CPack (DEB, RPM, TGZ, productbuild, NSIS,
ZIP), Filament 1.56.5 (prebuilt mac/windows-x64/ios; source build for Linux
release builds, Linux aarch64, Windows arm64, Android), cesium-native 0.64.0
via its vcpkg, Android NDK r27c + Prefab, Xcode 15 `xcodebuild -create-xcframework`,
`reprepro` + `createrepo_c`, Sonatype Central Portal publisher API, GitHub
Actions hosted runners, Docker for clean-install smoke tests.

**Spec:** user request 2026-10-02 + decisions below.

## Decisions (2026-10-02, user)

- D1 Scope: **full port**, all platforms in the table above packaged.
- D2 Packaged artifact: **shared library** with deps baked in, POD API exported only;
  **plus a static package** on every platform.
- D3 **Cesium ON** in every release package.
- D4 **No accepted gaps**: old-glibc Linux (RHEL/Alma 8+), package
  repositories, signing, macOS x86_64, Windows arm64, all four Android ABIs,
  iOS — all in scope.
- D5 Channels: signed apt+yum on GitHub Pages, Homebrew tap (repo created
  2026-10-02), Maven Central, vcpkg overlay port + Conan recipe.
- D6 Signing: one GPG key (RSA-4096, fpr `89281DE03F68406F29C62CCF8A1D000F68FE6404`,
  expires 2029-10-01) signs apt/yum repos, rpm packages, Maven artifacts and
  `SHA256SUMS`. Apple Developer ID/notarisation and Windows Authenticode
  steps are implemented but run only when their secrets exist
  (`APPLE_*`, `WINDOWS_SIGNING_*`); absent secrets produce a `::warning::`,
  never a silent skip.
- D7 SwiftPM: manifest lives in **`amerghazal7/overlume-swift`** (created 2026-10-02);
  the release job commits `Package.swift` there and tags it `vX.Y.Z`.
- D8 CI iteration: pushing work branch `release-packaging` to origin and
  `workflow_dispatch` dry runs are authorised (2026-10-02).

## Credentials (names only; values never in repo, logs or transcripts)

| Secret (repo `amerghazal7/overlume`) | Used by |
|---|---|
| `OVERLUME_GPG_PRIVATE_KEY`, `OVERLUME_GPG_PASSPHRASE`, `OVERLUME_GPG_FINGERPRINT` | rpm signing, apt/yum repo signing, Maven `.asc`, `SHA256SUMS.asc` |
| `MAVEN_CENTRAL_USERNAME`, `MAVEN_CENTRAL_PASSWORD` | Central Portal publisher API (Bearer = base64 of `user:pass`) |
| `HOMEBREW_TAP_DEPLOY_KEY` | push formula to `amerghazal7/homebrew-overlume` (write deploy key) |
| `SWIFTPM_REPO_DEPLOY_KEY` | push `Package.swift` + tag to `amerghazal7/overlume-swift` (write deploy key) |
| `APPLE_DEVELOPER_ID_P12`, `APPLE_DEVELOPER_ID_P12_PASSWORD`, `APPLE_NOTARY_KEY_ID`, `APPLE_NOTARY_ISSUER_ID`, `APPLE_NOTARY_KEY_P8` | not yet provisioned |
| `WINDOWS_SIGNING_*` (Azure Trusted Signing or PFX) | not yet provisioned |

Local key material: `~/.config/overlume-release/` (mode 700; private key,
passphrase, revocation cert, tap deploy key). The public key is committed at
`packaging/keys/overlume-release.asc` (Task 1) and served from Pages.

## Global Constraints

- Public headers stay POD-only and append-only (ADR-0003/0004);
  `overlume/scripts/check_pod_header.sh` passes. No header changes in this plan.
- The **dev** build on Linux x86_64 (prebuilt Filament, `overlume/build`) is
  byte-identical: `tools/ci_visual_mode.sh` green with **no golden changes**
  after every task. A golden diff is a finding.
- `overlume/build/liboverlume.a`, its merge step and the ROS node's
  consumption in `ros/src/overlume_ros/CMakeLists.txt` stay as they are.
- `CESIUM_ION_TOKEN` / `MAPBOX_TOKEN` never enter any build, package, log or
  test. Smoke tests need no network after the package is installed.
- Secrets are referenced by name only; scripts print PASS/FAIL and HTTP codes,
  never secret values; `set -x` is forbidden in any step that sees a secret.
- Pins: Filament **1.56.5**, cesium-native **0.64.0**, LLVM **18.1.8**, NDK
  **r27c**, Android API **26** (armeabi-v7a/x86 too), macOS **13.0**, iOS
  **15.0**, Windows **10** / MSVC v143.
- Linux release packages: built in `almalinux:8` → glibc ≥ 2.28 (RHEL/Alma/
  Rocky 8+, Ubuntu 20.04+, Debian 11+, Fedora 36+).
- Package name `overlume`; static package `overlume-static`; Maven
  `io.github.amerghazal7:overlume`; version from `project(overlume VERSION …)`;
  SOVERSION = major.
- macOS/iOS/Windows/Android are exercised only on GitHub-hosted runners:
  those tasks iterate by pushing a work branch and running `release.yml`
  via `workflow_dispatch` with `dry_run=true` (builds + tests + signs with
  the real key; publishes nothing; Maven bundle validated then dropped).
- Every task ends with its runnable check + green gate, one commit per task.

## Review Focus

1. **gcc/libstdc++ consumer with its own yaml-cpp/spdlog** links and runs
   the shared lib with no symbol clash → Task 1 export check + Task 3 smoke.
2. **Installed package, `theme_assets_dir = nullptr`** finds installed
   themes (not the CI path, not the fallback) → Task 1 unit test + Task 3 smoke
   (incl. relocated `tar.gz` prefix).
3. **Row 0 is the top row on every backend** (EGL/GLES/Metal/WGL readback)
   → Task 4 orientation test, run in every platform job.
4. **No GPU on the target box** → `create_renderer` returns `nullptr`
   cleanly → smoke `--expect-no-gpu` on Linux, and on macOS/Windows when the
   runner lacks a device.
5. **A user following the README on a fresh machine** (`apt install overlume`
   after adding the repo key, `brew install`, Gradle dependency, vcpkg/Conan
   install) gets a working build → Task 12 end-to-end channel smoke, against
   the dry-run outputs served from a local HTTP server (apt/yum) and the
   validated-but-dropped Maven bundle's contents.
6. **Static package with the wrong toolchain** (gcc/libstdc++ on Linux)
   fails at CMake configure with a clear message, never at link/run time
   → Task 1 config guard + Task 3 smoke negative case.

---

## File map

| File | Task | Responsibility |
|---|---|---|
| `overlume/CMakeLists.txt` | 1,2,4–8 | `overlume_obj`, both products, install/export, per-platform switches |
| `overlume/cmake/overlume_exports.map` / `overlume_exports_apple.txt` | 1 / 7 | export lists |
| `overlume/cmake/overlumeConfig.cmake.in`, `overlume.pc.in` | 1 | `find_package` (components `shared`/`static`), pkg-config |
| `overlume/cmake/OverlumePackaging.cmake` | 3,6–8 | CPack per platform |
| `overlume/cmake/GetFilament.cmake` | 2,6–8 | prebuilt-or-source Filament |
| `overlume/cmake/GetCesiumNative.cmake`, `vcpkg-triplets/*` | 2,6–8 | per-target triplet |
| `overlume/cmake/toolchain-llvm-release.cmake`, `tools/release/linux/Dockerfile` | 2 | Alma 8 + LLVM 18.1.8 release toolchain |
| `overlume/src/theme_dir.{hpp,cpp}` | 1 | default theme dir |
| `overlume/src/platform.hpp`, `platform_{egl,metal,wgl}.cpp` | 4,6,7,8 | headless back ends |
| `overlume/scripts/check_shared_exports.{sh,ps1}` | 1,7,8 | export hygiene |
| `packaging/keys/overlume-release.asc` | 1 | public key |
| `tools/package_smoke/…`, `tools/package_smoke_test.sh` | 3 | clean-room consumer |
| `tools/android/*`, `packaging/maven/*` | 6 | AAR, POM, Central upload |
| `tools/apple/*`, `packaging/homebrew/overlume.rb.in`, `packaging/swiftpm/Package.swift.in` | 7 | universal2, xcframework, formula, SPM |
| `packaging/vcpkg/ports/overlume/*.in`, `packaging/conan/{conanfile.py,conandata.yml.in}` | 9 | overlay port, recipe |
| `tools/release/{build_apt_repo.sh,build_yum_repo.sh,sign_sums.sh}`, `.github/workflows/pages.yml` | 10 | signed repos + docs on Pages |
| `tools/release/channel_smoke.sh` | 12 | end-to-end channel smoke |
| `.github/workflows/release.yml` | 3,5–12 | matrix + publish jobs |
| `docs/runbooks/release.md`, `README.md`, `docs/status.md`, `CHANGELOG.md`, `NOTICE` | 12 | docs |

---

### Task 1: Shared + static products, install components, `find_package`/pkg-config

**Files:** Modify `overlume/CMakeLists.txt`, `overlume/src/renderer.cpp:99-101,720-725`.
Create `overlume/cmake/overlume_exports.map`, `overlume/cmake/overlumeConfig.cmake.in`,
`overlume/cmake/overlume.pc.in`, `overlume/src/theme_dir.{hpp,cpp}`,
`overlume/scripts/check_shared_exports.sh`, `overlume/tests/test_theme_dir.cpp`,
`packaging/keys/overlume-release.asc` (copy of `~/.config/overlume-release/overlume-release-public.asc`).

**Interfaces — Produces:**
- Targets `overlume_obj` (OBJECT), `overlume` (STATIC, unchanged output),
  `overlume_shared` (SHARED, `OUTPUT_NAME overlume`, `SOVERSION ${PROJECT_VERSION_MAJOR}`);
  in-tree alias `overlume::overlume` → static (tests/examples unchanged).
- Installed exports: `overlume::overlume` (shared, component `overlume`) and
  `overlume::overlume_static` (component `static`); `find_package(overlume COMPONENTS static)`
  loads the static targets; default loads shared. Variables `overlume_THEMES_DIR`, `overlume_MODELS_DIR`.
- `std::string overlume::detail::resolve_default_theme_dir(const std::string& module_path, const char* compiled_default);`
  `std::string overlume::detail::current_module_path();`
- Install layout (component `overlume`): `lib/liboverlume.so*` (Windows: `bin/overlume.dll`
  + `lib/overlume.lib`), `include/overlume/*.h`, `share/overlume/{themes,models}/`,
  `lib/cmake/overlume/`, `share/pkgconfig/overlume.pc`, `share/doc/overlume/{LICENSE,NOTICE,ATTRIBUTION.md}`.
  Component `static`: `lib/liboverlume.a` (the merged archive) + `lib/overlume/deps/*.a`
  (Filament archives, plus libc++/abi/unwind? **no** — the consumer's own
  libc++ provides them) + `lib/cmake/overlume/overlumeStaticTargets.cmake`.

- [ ] **Step 1: Failing test** `overlume/tests/test_theme_dir.cpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "theme_dir.hpp"
namespace fs = std::filesystem;
using overlume::detail::resolve_default_theme_dir;

TEST(ThemeDir, PrefersInstalledShareDirNextToModule) {
    const fs::path root = fs::temp_directory_path() / "overlume_theme_dir_test";
    fs::remove_all(root);
    fs::create_directories(root / "lib");
    fs::create_directories(root / "share/overlume/themes");
    std::ofstream(root / "share/overlume/themes/dark_adas.yaml") << "x: 1\n";
    EXPECT_EQ(resolve_default_theme_dir((root / "lib/liboverlume.so.0").string(), "/nonexistent"),
              (root / "share/overlume/themes").string());
    fs::remove_all(root);
}

TEST(ThemeDir, FallsBackToCompiledDefaultWhenNoShareDir) {
    EXPECT_EQ(resolve_default_theme_dir("/definitely/not/here/lib/x.so", "/compiled/themes"),
              "/compiled/themes");
}

TEST(ThemeDir, EmptyModulePathUsesCompiledDefault) {
    EXPECT_EQ(resolve_default_theme_dir("", "/compiled/themes"), "/compiled/themes");
}
```

- [ ] **Step 2:** build + `ctest -R ThemeDir` → FAIL (header missing).
- [ ] **Step 3: Implement** `theme_dir.cpp`: return
  `<dirname(module)>/../share/overlume/themes` (lexically normalised) if it is
  a directory with at least one `*.yaml`, else `compiled_default`. (Windows
  DLL in `bin/`, same rule. Apple frameworks/Android: callers pass the dir;
  the rule simply fails over.) `current_module_path()`: `dladdr` on POSIX,
  `GetModuleHandleExW(FROM_ADDRESS|UNCHANGED_REFCOUNT)` + `GetModuleFileNameW`
  on `_WIN32`, `""` on failure. `renderer.cpp:724` uses
  `detail::resolve_default_theme_dir(detail::current_module_path(), DEFAULT_THEME_ASSETS_DIR)`.
- [ ] **Step 4:** `ctest -R ThemeDir` → PASS.
- [ ] **Step 5: Restructure.** Baseline first:
  `nm --defined-only overlume/build/liboverlume.a | awk '{print $NF}' | sort > $SCRATCH/nm_before.txt`.
  `overlume_obj` carries every compile option/definition/include now on
  `overlume` (public include dir `PUBLIC` with build/install interfaces) and
  `PRIVATE Filament::filament yaml-cpp::yaml-cpp` for usage requirements.
  `overlume` = `add_library(overlume STATIC $<TARGET_OBJECTS:overlume_obj>)`
  with today's link lines held in `_overlume_private_link`; merge
  `POST_BUILD` passes `$<TARGET_OBJECTS:overlume_obj>`. After rebuild the
  `nm` listing diffs empty against the baseline.
- [ ] **Step 6: Shared target** (`option(OVERLUME_BUILD_SHARED … ON)`):

```cmake
add_library(overlume_shared SHARED $<TARGET_OBJECTS:overlume_obj> ${_overlume_overlume_stream_objects})
set_target_properties(overlume_shared PROPERTIES
    OUTPUT_NAME overlume VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME overlume)
target_include_directories(overlume_shared PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(overlume_shared PRIVATE ${_overlume_private_link})
if(CMAKE_SYSTEM_NAME MATCHES "Linux|Android")
    target_link_options(overlume_shared PRIVATE
        "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/cmake/overlume_exports.map"
        "-Wl,--exclude-libs,ALL" "-Wl,--no-undefined")
endif()
```

  `overlume/cmake/overlume_exports.map`:

```
{
  global: extern "C++" { overlume::*; };
  local: *;
};
```

- [ ] **Step 7: Export check** `overlume/scripts/check_shared_exports.sh NM READELF LIB`:

```bash
#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
set -euo pipefail
nm_tool="$1"; readelf_tool="$2"; lib="$3"
bad=$("$nm_tool" -D --defined-only -C "$lib" | awk '$2 ~ /^[TDBRVW]$/ {sub(/^[^ ]+ [^ ]+ /,""); print}' \
      | grep -vE '^overlume::|^(_init|_fini|_edata|_end|__bss_start)$' || true)
if [ -n "$bad" ]; then echo "FAIL: non-overlume exports:"; echo "$bad" | head -20; exit 1; fi
needed=$("$readelf_tool" -d "$lib" | awk '/NEEDED/ {print $NF}' | tr -d '[]')
echo "NEEDED: $(echo $needed)"
if echo "$needed" | grep -qE 'libc\+\+|libc\+\+abi|libunwind|libyaml-cpp|libstdc\+\+'; then
  echo "FAIL: runtime leaked into NEEDED"; exit 1; fi
echo PASS
```

  ctest `shared_exports` (label `cpu`). Revert check: drop the version
  script → FAIL (verify once, restore).
- [ ] **Step 8: Install + config.** `set(CMAKE_INSTALL_LIBDIR lib)` before
  `include(GNUInstallDirs)` (`# ponytail: no multiarch libdir; the .so is
  self-contained`). Component `overlume`: shared target via
  `install(TARGETS overlume_shared EXPORT overlumeTargets …)`, headers,
  themes, `assets/models/*.glb` + `environment/`, docs files. Component
  `static`: `install(FILES $<TARGET_FILE:overlume> …)` + Filament archives to
  `lib/overlume/deps/` + a hand-written `overlumeStaticTargets.cmake`
  (installed from `overlume/cmake/overlumeStaticTargets.cmake.in`) defining
  `overlume::overlume_static` as `IMPORTED STATIC` with
  `INTERFACE_LINK_LIBRARIES` = the deps archives in the same group order the
  build uses + platform libs. `overlumeConfig.cmake.in`:

```cmake
@PACKAGE_INIT@
set(overlume_THEMES_DIR "${PACKAGE_PREFIX_DIR}/@CMAKE_INSTALL_DATADIR@/overlume/themes")
set(overlume_MODELS_DIR "${PACKAGE_PREFIX_DIR}/@CMAKE_INSTALL_DATADIR@/overlume/models")
if("static" IN_LIST overlume_FIND_COMPONENTS)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
       (NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR NOT CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+"))
        set(overlume_FOUND FALSE)
        set(overlume_NOT_FOUND_MESSAGE
            "overlume static on Linux requires clang++ with -stdlib=libc++ (it embeds "
            "libc++-ABI objects). Use the shared library (default component) with gcc.")
        return()
    endif()
    include("${CMAKE_CURRENT_LIST_DIR}/overlumeStaticTargets.cmake")
    set(overlume_static_FOUND TRUE)
endif()
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/overlumeTargets.cmake")
    include("${CMAKE_CURRENT_LIST_DIR}/overlumeTargets.cmake")
    set(overlume_shared_FOUND TRUE)
endif()
check_required_components(overlume)
```

  `write_basic_package_version_file(… COMPATIBILITY SameMinorVersion)`.
  `overlume.pc.in`: `prefix=${pcfiledir}/../..`, `Libs: -L${libdir} -loverlume`,
  `Cflags: -I${includedir}`. FetchContent deps' own `install()` rules must
  not reach either component.
- [ ] **Step 9:** `cmake --install overlume/build --component overlume --prefix $SCRATCH/inst`
  and `--component static` into a second prefix; layouts match exactly;
  `readelf -d` SONAME `liboverlume.so.0`, no RUNPATH. Gate green, no golden change.
- [ ] **Step 10: Commit** `feat(build): installable shared + static liboverlume with find_package/pkg-config`.

### Task 2: Portable Linux release toolchain (Alma 8, LLVM 18.1.8 tarball, Filament from source)

**Files:** Create `tools/release/linux/Dockerfile`, `overlume/cmake/toolchain-llvm-release.cmake`,
`overlume/cmake/vcpkg-triplets/arm64-linux-clang-libcxx.cmake`; modify
`overlume/cmake/GetFilament.cmake`, `overlume/cmake/GetCesiumNative.cmake`, `overlume/CMakeLists.txt` (matc path).

**Interfaces — Produces** (contract for Tasks 3, 6–8): `GetFilament.cmake`
sets `FILAMENT_ROOT` (has `include/`), `FILAMENT_LIB_DIR`,
`FILAMENT_HOST_MATC` (host-runnable `matc`; the material loop uses it instead
of `${FILAMENT_ROOT}/bin/matc`), target `Filament::filament`.
`OVERLUME_FILAMENT_FROM_SOURCE` (BOOL; default `ON` when no prebuilt exists for
the target — Linux non-x86_64, Android, Windows arm64 — and forced `ON` by
the release container). Docker image `overlume-release-linux:<arch>` built
from the Dockerfile with `/opt/llvm` = LLVM 18.1.8.

- [ ] **Step 1: Dockerfile** `FROM almalinux:8`; `dnf install` git, git-lfs,
  cmake (≥ 3.24 from the official CMake release tarball, pinned + SHA256),
  ninja, python3.11, perl, `mesa-libEGL-devel mesa-libGL-devel`, zip, unzip,
  rpm-build, dpkg (EPEL) + `dpkg-dev`; LLVM from
  `https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-x86_64-linux-gnu-ubuntu-18.04.tar.xz`
  (aarch64: `clang+llvm-18.1.8-aarch64-linux-gnu.tar.xz`), SHA256 pinned
  (download once, record literal). Image must not contain tokens.
- [ ] **Step 2: Toolchain** `toolchain-llvm-release.cmake`: compilers from
  `/opt/llvm/bin`, same `-stdlib=libc++` flags as `toolchain-clang-libcxx.cmake`;
  `-static-libgcc`; the `libc++.a` probe in `CMakeLists.txt` resolves inside
  `/opt/llvm/lib/<triple>/` (adjust the probe to accept that layout; dev
  layout must still resolve as today).
- [ ] **Step 3: Filament source path** at **configure** time (the lib
  `GLOB` needs files): download `https://github.com/google/filament/archive/refs/tags/v1.56.5.tar.gz`
  (pin SHA256), `execute_process` configure/build/install into
  `${CMAKE_BINARY_DIR}/_deps/filament-src-install` with the parent's compilers
  and flags, `-DCMAKE_BUILD_TYPE=Release -DFILAMENT_SKIP_SAMPLES=ON
  -DFILAMENT_SUPPORTS_VULKAN=OFF -DFILAMENT_ENABLE_JAVA=OFF`, stamp-file
  guarded, `FATAL_ERROR` with the log path on failure. Cross targets
  (Android, iOS, Windows arm64) forward `CMAKE_TOOLCHAIN_FILE`/ABI vars and
  take `FILAMENT_HOST_MATC` from a host prebuilt (Linux x64 tarball or mac
  tarball).
- [ ] **Step 4:** triplet `arm64-linux-clang-libcxx`; `GetCesiumNative.cmake`
  selects by `CMAKE_SYSTEM_PROCESSOR`.
- [ ] **Step 5: Equivalence check** — in the container, build with
  `OVERLUME_FILAMENT_FROM_SOURCE=ON`, run cpu+gpu ctest (llvmpipe via
  `EGL_PLATFORM=surfaceless`) and the golden tests → green, **no golden
  change**; `objdump -T liboverlume.so.0 | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1`
  ≤ `GLIBC_2.28` (this is the runnable check that fails if the build ever
  slips to a newer glibc). Record build times.
  **Step 5 result (2026-10-02, fresh `HOME`, repo mounted read-only, x86_64
  image, 32 cores):** Filament 1.56.5 source build ~76 s; cesium/vcpkg
  ~4.5 min (configure total 365 s); overlume build 40 s; full `ctest` 210 s:
  294/295 pass (cpu 285 + gpu 10 labels, goldens included, no golden change
  possible on the read-only mount), 6 skipped (network/capture tests).
  The one failure, `Objects.FiftyObjectsSceneUpdateUnderTwoMilliseconds`, is
  deterministic (12.5 ms vs 2 ms budget, llvmpipe) and recorded as known gap
  11 in `docs/status.md`. glibc floor: `GLIBC_2.28`.
  **Task 2 amendment (2026-10-02, CI dry run 37022162508): Linux aarch64 is
  cross-compiled, not built natively.** The official LLVM 18.1.8 aarch64 tarball's
  `clang++` needs GLIBC_2.29/GLIBCXX_3.4.26, so it cannot even run in `almalinux:8`
  on an arm64 runner (a from-source native LLVM was rejected). User decision
  (option 2): build aarch64 on the x86_64 runner with the x86_64 `/opt/llvm`
  (`--target=aarch64-linux-gnu`, lld) against an AlmaLinux 8 aarch64 sysroot.
  `tools/release/linux/Dockerfile.cross-aarch64` (a layer on the unchanged x86_64
  image, whose Dockerfile still contains the now-unused aarch64 download branch)
  installs the sysroot with `dnf --installroot --forcearch=aarch64` (no emulation;
  `filesystem` first, scriptlets off, absolute symlinks made relative, package
  list recorded in `/opt/sysroot-aarch64.manifest`) and builds libc++/libc++abi/
  libunwind 18.1.8 (pinned `llvm-project-18.1.8.src.tar.xz`, SHA256 in the
  Dockerfile) into `/opt/llvm/lib/aarch64-unknown-linux-gnu/`, the tarball's own
  per-target layout, so the `libc++.a` probe resolves unchanged apart from passing
  `--target`. `toolchain-llvm-release-aarch64.cmake` only sets the triple and
  sysroot; all flags stay in `toolchain-llvm-release.cmake`. `GetFilament.cmake`
  cross mode first builds Filament's host tools (matc, resgen, ...; this writes the
  `ImportExecutables-Release.cmake` Filament's cross build includes) with the x86_64
  toolchain, then cross-builds the libraries; `FILAMENT_HOST_MATC` is that matc.
  Cesium/vcpkg: the `arm64-linux-clang-libcxx` triplet chainloads the cross
  toolchain (`vcpkg-llvm-release-aarch64-toolchain.cmake`, adds -fPIC and the vcpkg
  prefix to the find roots); the vcpkg host triplet is `x64-linux-clang-libcxx`.
- [ ] **Step 6:** dev gate green on the host. **Commit**
  `feat(build): Alma 8 release toolchain, Filament source build, glibc 2.28 floor`.

### Task 3: Linux packages, clean-room smoke matrix, release workflow skeleton

**Files:** Create `overlume/cmake/OverlumePackaging.cmake`, `tools/package_smoke/{CMakeLists.txt,main.cpp}`,
`tools/package_smoke_test.sh`, `tools/release/sign_sums.sh`; modify `overlume/CMakeLists.txt`
(include packaging last), `.github/workflows/release.yml`.

**Interfaces — Consumes:** Task 1 components + config, Task 2 image.
**Produces:** `cpack` → `overlume_<ver>_<arch>.deb`, `overlume-static_<ver>_<arch>.deb`,
`overlume-<ver>-1.<arch>.rpm`, `overlume-static-<ver>-1.<arch>.rpm`,
`overlume-<ver>-linux-<arch>.tar.gz` (both components);
`package_smoke --expect-render|--expect-no-gpu` prints `PASS`/`FAIL: <reason>`;
`tools/package_smoke_test.sh PKG_DIR` exit 0/1; `tools/release/sign_sums.sh DIR NAME`
writes `SHA256SUMS-<NAME>.txt` + detached `.asc` (GPG from env
`OVERLUME_GPG_PRIVATE_KEY`/`_PASSPHRASE`, imported into a temp `GNUPGHOME`,
deleted on exit). Workflow jobs `create`, `package` (matrix), later publish jobs `needs: package`.

- [ ] **Step 1: Smoke consumer** — `tools/package_smoke/main.cpp` uses only
  public headers (pattern `examples/01_hello_frame.cpp`), `theme_assets_dir = nullptr`,
  renders one frame, asserts non-uniform image and that the installed theme
  was used: background matches `dark_adas.yaml`'s clear colour
  (`SMOKE_THEMES_DIR` from `${overlume_THEMES_DIR}`, short line scan, ±2) and
  differs from `kFallbackTheme()`'s (constant copied with a pointer to its
  definition in `overlume/src/`; if equal, pick another element that differs
  and say which in the report). Includes `<yaml-cpp/yaml.h>` and requires
  `YAML::Load("a: 1")["a"].as<int>() == 1`. `--expect-no-gpu`: `create_renderer`
  returns `nullptr` without aborting. `CMakeLists.txt`: targets
  `package_smoke` (shared, `find_package(overlume 0.1 REQUIRED)`),
  `package_smoke_pc` (`pkg_check_modules(OV REQUIRED IMPORTED_TARGET overlume)`),
  and, when `-DSMOKE_STATIC=ON`, `package_smoke_static`
  (`find_package(overlume 0.1 REQUIRED COMPONENTS static)`).
- [ ] **Step 2: Packaging config** `OverlumePackaging.cmake`:

```cmake
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
set(CPACK_PACKAGE_NAME overlume)
set(CPACK_PACKAGE_VENDOR "Amer Ghazal")
set(CPACK_PACKAGE_CONTACT "Amer Ghazal <amer.ghazal@micropolis.ae>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Overlume real-time robot-scene rendering library")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/amerghazal7/overlume")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE")
string(TOLOWER "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}" _ov_plat)
set(CPACK_PACKAGE_FILE_NAME "overlume-${PROJECT_VERSION}-${_ov_plat}")
set(CPACK_STRIP_FILES ON)
set(CPACK_COMPONENTS_ALL overlume static)
set(CPACK_COMPONENT_STATIC_DEPENDS overlume)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CPACK_GENERATOR "DEB;RPM;TGZ")
    set(CPACK_PACKAGING_INSTALL_PREFIX /usr)
    set(CPACK_DEB_COMPONENT_INSTALL ON)
    set(CPACK_DEBIAN_OVERLUME_PACKAGE_NAME overlume)
    set(CPACK_DEBIAN_STATIC_PACKAGE_NAME overlume-static)
    set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
    set(CPACK_DEBIAN_ENABLE_COMPONENT_DEPENDS ON)
    set(CPACK_DEBIAN_PACKAGE_SECTION libs)
    set(CPACK_RPM_COMPONENT_INSTALL ON)
    set(CPACK_RPM_OVERLUME_PACKAGE_NAME overlume)
    set(CPACK_RPM_STATIC_PACKAGE_NAME overlume-static)
    set(CPACK_RPM_FILE_NAME RPM-DEFAULT)
    set(CPACK_RPM_PACKAGE_LICENSE "Apache-2.0")
    set(CPACK_RPM_PACKAGE_AUTOREQ ON)
    set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION
        /usr/lib/cmake /usr/share/pkgconfig /usr/share/doc)
    set(CPACK_ARCHIVE_COMPONENT_INSTALL OFF)  # one tar.gz with both components
endif()
# Tasks 6-8 append ANDROID / APPLE / WIN32 branches here.
include(CPack)
```

  Verify `tar tzf` paths are prefix-relative (`overlume-…/lib/…`); if TGZ
  inherits `/usr`, generate it with a second `cpack -G TGZ -D CPACK_PACKAGING_INSTALL_PREFIX=`
  call. `dpkg -c` lists only Task 1 paths. rpm signing: after `cpack`,
  `rpm --addsign` with the imported key (`%_gpg_name` = fingerprint), verified
  with `rpm -K`.
- [ ] **Step 3: Docker matrix** `tools/package_smoke_test.sh PKG_DIR`
  (shellcheck-clean). Images: `ubuntu:20.04`, `ubuntu:22.04`, `ubuntu:24.04`,
  `debian:11`, `debian:12`, `almalinux:8`, `almalinux:9`, `fedora:40`. Per image:
  install the shared package + `g++ cmake pkg-config` + distro yaml-cpp dev +
  Mesa EGL/DRI (deb: `./overlume_*.deb libyaml-cpp-dev libegl1 libegl-mesa0 libgl1-mesa-dri`;
  rpm: `./overlume-[0-9]*.rpm gcc-c++ cmake pkgconf-pkg-config yaml-cpp-devel mesa-libEGL mesa-dri-drivers`,
  EPEL/PowerTools enabled on Alma for yaml-cpp); rpm images run `rpm -K` on
  the package with the public key imported (`rpm --import packaging/keys/overlume-release.asc`)
  → must report `digests signatures OK`. Build both shared smoke targets with
  `CXX=g++`, run `EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 … --expect-render`;
  remove Mesa DRI and run `--expect-no-gpu`. On `ubuntu:22.04` and
  `almalinux:9` also install `overlume-static` + distro `clang`/`libc++-dev`
  (Alma: LLVM toolset) and build `package_smoke_static` with
  `CXX=clang++ CXXFLAGS=-stdlib=libc++` → `--expect-render`; and configure the
  static target with `g++` → configure must FAIL with the guard message
  (grep for it). Reinstall (upgrade path), remove both packages, assert
  `/usr/include/overlume`, `/usr/share/overlume`, `/usr/lib/cmake/overlume`,
  `/usr/lib/liboverlume*` gone. On `ubuntu:22.04` extract the `tar.gz` to
  `/opt/ov`, build with `-DCMAKE_PREFIX_PATH=/opt/ov`, run with
  `LD_LIBRARY_PATH=/opt/ov/lib` → `--expect-render` (relocated prefix). One
  `PASS <image>` / `FAIL <image>: <step>` line each; exit 1 on any FAIL.
- [ ] **Step 4:** in the Task 2 container `cpack`, then on the host
  `tools/package_smoke_test.sh <dir>` → all PASS. Revert check: drop
  `--exclude-libs,ALL` and the version script → `shared_exports` FAILs
  (record whether the yaml-cpp coexistence smoke also does); restore.
- [ ] **Step 5: `release.yml` skeleton.** Rename job `release` → `create`
  (body unchanged; tag push only). Add `workflow_dispatch` input `dry_run`
  (boolean, default `true`). Top-level `permissions: contents: read`.
  Job `package`:

```yaml
  package:
    needs: create
    if: ${{ !cancelled() && (needs.create.result == 'success' || github.event_name == 'workflow_dispatch') }}
    permissions:
      contents: write
    strategy:
      fail-fast: false
      matrix:
        include:
          - { name: linux-x86_64,  os: ubuntu-22.04,     container_arch: x86_64 }
          - { name: linux-aarch64, os: ubuntu-22.04-arm, container_arch: aarch64 }
    runs-on: ${{ matrix.os }}
    env:
      DRY_RUN: ${{ github.event_name == 'workflow_dispatch' && inputs.dry_run }}
    steps:
      # checkout (lfs); build/cache the Task 2 image (actions/cache on a docker save
      # tarball keyed by hashFiles('tools/release/linux/Dockerfile'));
      # docker run: configure -DOVERLUME_ENABLE_CESIUM=ON -DOVERLUME_BUILD_DOCS=OFF
      #   -DOVERLUME_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
      #   -DOVERLUME_FILAMENT_FROM_SOURCE=ON, build, ctest -L cpu, cpack, rpm --addsign;
      # cache overlume/build/_deps (filament-src-install + vcpkg) keyed on
      #   hashFiles('overlume/cmake/**', 'tools/release/linux/Dockerfile') + arch;
      # tools/package_smoke_test.sh (host docker);
      # tools/release/sign_sums.sh out ${{ matrix.name }};
      # actions/upload-artifact name=pkg-${{ matrix.name }};
      # if DRY_RUN != 'true': gh release upload "$GITHUB_REF_NAME" out/* --clobber
```

  Secrets are passed only to the signing steps via `env:`.
- [ ] **Step 6:** `actionlint` (pinned binary in the scratchpad) clean; gate green.
  **Task 3 result (2026-10-02, x86_64 only; aarch64 is exercised by CI):**
  clean-room matrix 8/8 PASS (ubuntu 20.04/22.04/24.04, debian 11/12, alma 8/9,
  fedora 40; `rpm -K` reports `digests signatures OK`; relocated tar.gz, upgrade,
  clean removal, no-GPU and both static guards included); `cpu` ctest 285/285 in
  the container (minus known gap 11); `shared_exports` and glibc floor 2.28 pass.
  Revert check: dropping the version script and `--exclude-libs` fails
  `shared_exports` (libcrypto/libssl symbols leak) and the yaml-cpp coexistence
  smoke also fails (renderer run aborts on ubuntu:24.04 and debian:12).
  Deviations from the text above, each found by the matrix: (1) smoke cannot tell
  the installed theme from `kFallbackTheme()` by pixels (identical palette), so
  it asserts `theme_assets_loaded()` plus default-dir frame == explicit-dir frame;
  (2) `.pc` installs to `share/pkgconfig` (Fedora/Alma pkg-config does not search
  `/usr/lib/pkgconfig`), the rpm registers `/usr/lib` via `ld.so.conf.d` (RHEL's
  loader ignores it); (3) `shlibdeps` is replaced by explicit Depends/Requires
  (no dpkg database in Alma) and adds `libgl1`/`libGL.so.1` (Filament dlopen()s
  it); (4) theme lookup canonicalises the module path (`/lib` -> `/usr/lib`);
  (5) static consumers: ubuntu:24.04 + fedora:40, not ubuntu:22.04 + alma:9
  (alma 9 ships no libc++; ubuntu 22.04's libc++ 14 lacks `__cxa_init_primary_exception`)
  and the config guard now requires LLVM >= 18, with a 22.04 negative case;
  a statically linked executable has no `share/` beside it, so the static smoke
  passes the installed theme dir explicitly; (6) no-GPU is `__EGL_VENDOR_LIBRARY_FILENAMES`
  pointing nowhere (Mesa 25 keeps swrast inside libegl-mesa0, so removing
  packages is not equivalent); (7) `sign_rpms.sh` and `OverlumeCPackOptions.cmake.in`
  added (rpmsign wrapper; tar.gz prefix), `rpm-sign` added to the Dockerfile,
  and debian:11 is pointed at archive.debian.org with libc6 pinned.
  **Step 7 dry runs:** 37022162508 (x86 green); 37037401298 (x86 green; aarch64
  cross build + native cpu tests green; the fedora:40 static rpm failure was fixed
  by 62595c4); 37045928609 (pending).
  **Task 3 amendment (cross-compiled aarch64, 2026-10-02).** The `package` matrix
  keeps only `linux-x86_64`. aarch64 is two jobs: `package-linux-aarch64-build`
  (ubuntu-22.04, cross image, read-only token: build, `check_glibc_floor.sh` and
  `check_shared_exports.sh` with llvm-objdump/nm, cpack, `check_package_elf.sh`,
  rpm signing, `make_test_bundle.sh`) and `package-linux-aarch64` (ubuntu-22.04-arm:
  `tools/release/linux/test_aarch64.sh` in `almalinux:8`, the cpu ctest set from the
  bundle with `gtest_discover_tests(DISCOVERY_MODE PRE_TEST)` plus the glibc and
  machine checks, then `package_smoke_test.sh`, then `sign_sums.sh` and the tag
  upload). Cross-only changes: CPack deb/rpm architecture are set explicitly
  (arm64/aarch64), `CMAKE_STRIP/OBJDUMP/OBJCOPY` are llvm-*, `CMAKE_NM/READELF`
  are bare names (resolved on the arm64 host), `<triple>-clang` symlinks give
  bare-compiler probes (KTX's CPU check) the right target, and the vcpkg chainload
  appends the vcpkg prefix to the find roots and links the static libc++ for port
  executables. Local proof on x86_64: full cross build, 3 package kinds x2,
  glibc floor `GLIBC_2.28`, `check_shared_exports` PASS, all shipped ELF AArch64,
  `rpmsign` on the aarch64 rpms with a throwaway key; dev gate green. Native
  execution is proven only by CI (see `docs/status.md`, known gap 12).
- [ ] **Step 7:** push work branch, `gh workflow run release.yml --ref <branch> -f dry_run=true`,
  both Linux jobs green. **Commit**
  `feat(release): signed deb/rpm/tgz (shared+static) for x86_64/aarch64, smoke-tested`.

### Task 4: Headless platform seam (no behaviour change on Linux)

**Files:** Create `overlume/src/platform.hpp`, `overlume/src/platform_egl.cpp`,
`overlume/tests/test_readback_orientation.cpp`; modify `overlume/src/renderer.cpp`
(move `HeadlessEglPlatform`, now lines 117–240), `overlume/src/renderer_internal.hpp:47,114`,
`overlume/CMakeLists.txt`.

**Interfaces — Produces:**

```cpp
// overlume/src/platform.hpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
#pragma once
#include <filament/Engine.h>
namespace overlume::detail {
struct HeadlessPlatform {
    filament::backend::Platform* platform = nullptr;  // owned; nullptr = Filament default
    filament::Engine::Backend backend = filament::Engine::Backend::OPENGL;
};
// Compile-time-selected headless back end. Never aborts; on failure
// Engine::Builder::build() returns nullptr exactly as today.
HeadlessPlatform make_headless_platform();
void destroy_headless_platform(HeadlessPlatform&);  // after Engine::destroy
}  // namespace overlume::detail
```

  CMake: `OVERLUME_PLATFORM` = `egl` (Linux, Android), `metal` (Darwin, iOS),
  `wgl` (Windows); `list(FILTER OVERLUME_SOURCES EXCLUDE REGEX "/platform_[a-z]+\\.cpp$")`
  then append `src/platform_${OVERLUME_PLATFORM}.cpp`. `OVERLUME_PLATFORM_LIBS`
  (Linux: `EGL`) replaces the bare `EGL` in the test link line.

- [ ] **Step 1: Orientation test** (label `gpu`): renderer as in
  `test_hello_frame.cpp`, camera above ground looking at the horizon; row 1
  mean ≠ row `height-2` mean and row 1 matches the theme sky/clear colour
  (±8). Prove it bites: flip rows in `render_frame`'s readback copy → FAIL; restore → PASS.
- [ ] **Step 2:** move the EGL class verbatim into `platform_egl.cpp`
  (`overlume::detail`), `make_headless_platform()` → `{new HeadlessEglPlatform(), OPENGL}`;
  `renderer_internal.hpp` holds `detail::HeadlessPlatform platform;`;
  `create_renderer` uses `.backend(p.backend).platform(p.platform)`;
  `destroy_renderer` calls `destroy_headless_platform`.
- [ ] **Step 3:** gate green, goldens untouched; orientation PASS.
- [ ] **Step 4: Commit** `refactor(renderer): headless platform seam ahead of per-OS back ends`.

### Task 5: Portability switches shared by every non-Linux target

**Files:** Modify `overlume/CMakeLists.txt`, `overlume/cmake/GetCesiumNative.cmake`.

- [ ] **Step 1:** clang / `-stdlib=libc++` / `libc++.a` probes and the static
  libc++ embedding run only `if(CMAKE_SYSTEM_NAME STREQUAL "Linux")`.
  GNU-ld-only flags (`--start-group/--end-group`, `--allow-multiple-definition`,
  `--exclude-libs`, `--no-undefined`) only `if(CMAKE_SYSTEM_NAME MATCHES "Linux|Android")`.
  `merge_yamlcpp.sh` runs on Linux and Android (it uses `ar/ld -r/objcopy/nm`,
  all present in the NDK as `llvm-*`); on Apple/Windows the static product
  is produced without the rename step, and the static package documents that
  consumers must not also link their own yaml-cpp/spdlog/cesium there — the
  config file emits `message(WARNING …)` once when the static component is
  loaded on those platforms.
- [ ] **Step 2:** tests/tools/examples link `overlume::overlume` and
  `${OVERLUME_PLATFORM_LIBS}`; test data dir from a cache var
  `OVERLUME_TEST_DATA_DIR_RUNTIME` (default = source dir; Android sets the
  device path).
- [ ] **Step 3:** `GetCesiumNative.cmake` maps target → overlay triplet:
  `x64|arm64-linux-clang-libcxx`, `arm64|arm|x64|x86-android-overlume`,
  `arm64|x64-osx-overlume`, `arm64-ios-overlume`, `arm64|x64-ios-simulator-overlume`,
  `x64|arm64-windows-overlume` (static libs, dynamic CRT on Windows,
  `VCPKG_OSX_DEPLOYMENT_TARGET 13.0`, iOS 15.0, Android API 26).
- [ ] **Step 4:** Linux dev gate green, no golden change (pure guard
  changes). **Commit** `build: gate Linux-only toolchain/link logic; per-target cesium triplets`.

### Task 6: Android (arm64-v8a, armeabi-v7a, x86_64, x86) + Maven Central

**Files:** Create `overlume/cmake/vcpkg-triplets/{arm64,arm,x64,x86}-android-overlume.cmake`,
`tools/android/{build_all_abis.sh,build_aar.sh,run_tests_on_emulator.sh}`,
`tools/android/prefab/{prefab.json,module.json,abi.json.in,AndroidManifest.xml}`,
`packaging/maven/overlume.pom.in`, `tools/release/publish_maven_central.sh`,
`tools/package_smoke/android/{CMakeLists.txt,main.cpp}`; modify `overlume/src/platform_egl.cpp`,
`overlume/cmake/OverlumePackaging.cmake`, `.github/workflows/release.yml`.

**Produces:** `overlume-<ver>-android.aar` (Prefab: `prefab/modules/overlume/libs/android.<abi>/{liboverlume.so,abi.json}`,
`prefab/modules/overlume/include/`; plus module `overlume_static` with
`liboverlume.a` + deps, `"static": true`), `overlume-<ver>-android.zip`
(`<abi>/{lib,include,share,lib/cmake}`, both components).

- [ ] **Step 1:** `build_all_abis.sh` configures each ABI with
  `-DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=<abi> -DANDROID_PLATFORM=26 -DANDROID_STL=c++_static`.
- [ ] **Step 2:** Filament per ABI via the Task 2 source path; host
  `matc` from the Linux x64 prebuilt; materials `-p mobile -a opengl`.
- [ ] **Step 3:** `platform_egl.cpp` `#if defined(__ANDROID__)`:
  `eglBindAPI(EGL_OPENGL_ES_API)`, `EGL_RENDERABLE_TYPE EGL_OPENGL_ES3_BIT`,
  context `EGL_CONTEXT_CLIENT_VERSION 3`, no `bluegl`. Non-Android
  preprocessed output unchanged (diff empty). `OVERLUME_PLATFORM_LIBS`:
  `EGL GLESv3 log android`.
- [ ] **Step 4:** `run_tests_on_emulator.sh BUILD_DIR`: `adb push` test
  binaries + assets + fixtures to `/data/local/tmp/ov/`, run every `cpu`
  test + `ReadbackOrientation.*` + `HelloFrame.*`, exit 1 on any failure.
  CI: `reactivecircus/android-emulator-runner@v2`, API 30, run twice —
  `arch: x86_64` and `arch: x86` (`-no-window -gpu swiftshader_indirect`).
  arm64-v8a/armeabi-v7a: build + `check_shared_exports.sh` with NDK
  `llvm-nm`/`llvm-readelf` (no hosted ARM emulator); the task report says so.
- [ ] **Step 5:** `build_aar.sh` zips the Prefab layout (schema v2,
  `abi.json` `{"abi":"<abi>","api":26,"ndk":27,"stl":"c++_static","static":false}`),
  `AndroidManifest.xml` `package="io.github.amerghazal7.overlume"`, no Gradle.
  NDK consumer check: `tools/package_smoke/android` built with the NDK for
  each ABI against the unpacked zip's `<abi>/lib/cmake` → links.
- [ ] **Step 6: Maven Central.** `overlume.pom.in`: groupId
  `io.github.amerghazal7`, artifactId `overlume`, packaging `aar`, name,
  description, url, Apache-2.0 license, developer, scm — every field Central
  requires. Bundle layout `io/github/amerghazal7/overlume/<ver>/` with
  `overlume-<ver>.aar`, `.pom`, `-sources.jar` (headers + README) and
  `-javadoc.jar` (README pointing at the Pages API docs) — each with `.asc`
  (GPG detached, armored), `.md5`, `.sha1`. `publish_maven_central.sh BUNDLE_ZIP MODE`
  (`MODE` = `validate` | `publish`): `POST https://central.sonatype.com/api/v1/publisher/upload?publishingType=USER_MANAGED|AUTOMATIC&name=overlume-<ver>`
  with `Authorization: Bearer $(printf '%s:%s' "$MAVEN_CENTRAL_USERNAME" "$MAVEN_CENTRAL_PASSWORD" | base64 -w0)`,
  poll `POST /api/v1/publisher/status?id=…` until `VALIDATED`/`PUBLISHED`
  or `FAILED` (print the `errors` JSON — it contains no secrets), and in
  `validate` mode `DELETE /api/v1/publisher/deployment/<id>` afterwards.
  Prints HTTP codes and states only. Dry runs use `validate`; tag pushes `publish`.
  **Prerequisite (user):** public key on `keyserver.ubuntu.com` and
  `keys.openpgp.org`; validation fails with a signature error until it is.
- [ ] **Step 7:** CI job `android` (ubuntu-22.04, `nttld/setup-ndk` r27c):
  all four ABIs, emulator tests, `cpack -G ZIP` per ABI merged into
  the final zip, AAR, Maven bundle, `sign_sums.sh`, upload artifact; job
  `publish-maven` (`needs: [package-android]`, runs `validate` on dry runs,
  `publish` on tags).
- [ ] **Step 8:** dry-run green incl. Maven `VALIDATED`; Linux gate green.
  **Commit** `feat(android): 4-ABI NDK build, GLES back end, Prefab AAR on Maven Central`.

### Task 7: Apple — macOS universal2 + iOS XCFramework, Homebrew, SwiftPM

**Files:** Create `overlume/src/platform_metal.cpp`, `overlume/cmake/overlume_exports_apple.txt`,
`overlume/cmake/vcpkg-triplets/{arm64,x64}-osx-overlume.cmake`, `…/arm64-ios-overlume.cmake`,
`…/{arm64,x64}-ios-simulator-overlume.cmake`, `tools/apple/{build_macos_universal.sh,build_xcframework.sh,sign_and_notarize.sh}`,
`packaging/homebrew/overlume.rb.in`, `tools/release/publish_homebrew.sh`,
`packaging/swiftpm/Package.swift.in`, `tools/release/publish_swiftpm.sh`;
modify `GetFilament.cmake` (mac + ios prebuilt tarballs, SHA256 pinned),
`overlume/scripts/check_shared_exports.sh` (Darwin branch), `OverlumePackaging.cmake`, `release.yml`.

- [x] **Step 1: Metal back end** `platform_metal.cpp`: `{nullptr, Backend::METAL}`
  (Filament default `PlatformMetal`, headless `createSwapChain(w,h,CONFIG_READABLE)`).
  Materials on Apple: `matc -a metal -p desktop` (macOS) / `-p mobile` (iOS).
  `OVERLUME_PLATFORM_LIBS`: frameworks `Metal QuartzCore CoreVideo IOSurface Foundation`
  (+ `Cocoa` macOS, `UIKit` iOS).
- [x] **Step 2:** If ld64 reports duplicate symbols (cesium/spdlog archives
  reached twice), dedupe the archive list at its source and name it in the
  report; never `-multiply_defined`.
- [x] **Step 3: Exports** `-Wl,-exported_symbols_list,…/overlume_exports_apple.txt`
  (lines `__ZN8overlume*`, `__ZNK8overlume*`); `MACOSX_RPATH ON`,
  `INSTALL_NAME_DIR @rpath`. Darwin branch of `check_shared_exports.sh`:
  `nm -gU -C` exports all `overlume::`; `otool -L` lists only
  `/usr/lib/lib{c++.1,System.B,z.1,objc.A}.dylib` and `/System/Library/Frameworks/*`.
- [x] **Step 4: macOS universal2** `build_macos_universal.sh`: arm64 and
  x86_64 builds (separate build dirs: cesium vcpkg is per-arch), then
  `lipo -create` the dylib and each static archive; `lipo -verify_arch arm64 x86_64`
  is the check. CPack `productbuild;TGZ` from a staging install of the
  merged tree, prefix `/usr/local`. Tests: `ctest -L cpu` on both arches
  (`macos-14` arm64 natively, x86_64 under Rosetta), orientation +
  hello-frame GPU tests on arm64. If the hosted runner has no usable Metal
  device, run smoke `--expect-no-gpu` and emit `::warning::no Metal device`
  — never a silent skip; the report states which happened.
- [x] **Step 5: iOS** `build_xcframework.sh`: device (`arm64`, iOS 15.0)
  and simulator (`arm64;x86_64`, lipo'd) builds with
  `-DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=iphoneos|iphonesimulator`.
  Shared → `Overlume.framework` (`FRAMEWORK TRUE`, `MACOSX_FRAMEWORK_IDENTIFIER io.github.amerghazal7.overlume`,
  headers in `Headers/`, themes/models in `Resources/`), combined with
  `xcodebuild -create-xcframework -framework … -framework … -output Overlume.xcframework`;
  static → `OverlumeStatic.xcframework` from `-library liboverlume.a -headers include`
  (Filament/cesium archives merged in with `libtool -static` so it is one
  library per slice). Check: `xcrun simctl` boots an iPhone simulator and a
  tiny XCTest host (`tools/apple/ios_smoke/`, Swift calling a 10-line C++
  shim over `create_renderer`/`render_frame`) passes
  (`xcodebuild test -destination 'platform=iOS Simulator,name=iPhone 15'`),
  or reports `nullptr` → the report states which.
- [x] **Step 6: Signing** `sign_and_notarize.sh`: if `APPLE_DEVELOPER_ID_P12`
  is set → temp keychain, `codesign --timestamp --options runtime` the dylib
  and framework, `productsign` the `.pkg`, `xcrun notarytool submit --wait`
  with the API key, `xcrun stapler staple`; else `::warning::Apple signing
  secrets not configured; packages unsigned`. Works either way; the check is
  `pkgutil --check-signature` (signed) or the warning (unsigned).
- [x] **Step 7: Homebrew** `overlume.rb.in`: binary formula over the macOS
  universal `tar.gz` (`url`, `sha256`, `version`, `license "Apache-2.0"`,
  `depends_on macos: :ventura`, `install` copies the tree into `prefix`,
  `test do` compiles a 5-line program against `overlume::overlume` via
  `find_package` and runs it with `--expect-no-gpu` semantics: just links
  and calls `create_renderer` with a 0×0 config → `nullptr`).
  `publish_homebrew.sh VERSION SHA256 URL`: renders the formula, clones the
  tap with `HOMEBREW_TAP_DEPLOY_KEY` (ssh-agent, key never echoed), commits
  `Formula/overlume.rb`, pushes (tag runs only). Dry run: `brew install
  --formula ./overlume.rb` against the dry-run tarball served locally
  (`url "file://…"`), then `brew test overlume`.
- [x] **Step 8: SwiftPM** — manifest in `amerghazal7/overlume-swift` (D7), pushed with `SWIFTPM_REPO_DEPLOY_KEY` and tagged `vX.Y.Z` (Package.swift with
  `binaryTarget(name: "Overlume", url: <release asset>, checksum: <swift package compute-checksum>)`,
  `platforms: [.iOS(.v15), .macOS(.v13)]`). `publish_swiftpm.sh` renders and
  pushes it (tag runs only); dry run: `swift package resolve` + `swift build`
  of a consumer against a locally served zip.
- [x] **Step 9:** CI jobs `macos` and `ios` (`macos-14`, Xcode 15.4),
  `publish-homebrew`, `publish-swiftpm`; dry-run green; Linux gate green.
  **Commit** `feat(apple): Metal back end, macOS universal2 pkg + Homebrew, iOS XCFramework + SwiftPM`.

**Task 7 results (2026-10-05, complete; CI dry run 37345214541 on b21c55d, fully green).**
Proved on CI: all three iOS slices and both macOS arches build, link, install and pass the Mach-O export
check; macOS cpu tests pass on both arches; `package-macos` ran the universal merge, pkg/tar.gz, the
unsigned warning path, the relocated tar.gz smoke (arm64 + x86_64 under Rosetta), the pkg install + smoke,
and `brew install` + `brew test` from a local tap; `package-ios` assembled the xcframeworks (fat simulator
slices, export check per slice), zipped them, and passed the SwiftPM consumer XCTest in an iPhone simulator
plus `publish_swiftpm.sh check`; `sign-apple` took the no-secrets warning path. `publish-homebrew` and
`publish-swiftpm` were correctly skipped (dry run), so their real push branches have never executed. The
Android, Linux x86_64/aarch64 legs stayed green. `release.yml` now has a concurrency group so a new
dispatch on the same ref cancels the previous dry run (a dispatch on a tag ref queues behind the tag run instead of cancelling it).
Deviations: (1) Filament's prebuilt mac SDK is arm64-only and the iOS SDK has no arm64 simulator slice, so
macOS x86_64 and every iOS slice build Filament from source (host tools = the mac SDK's arm64 binaries);
the arm64 install therefore carries different static-archive numbering plus `bluegl`/`bluevk`, which the
universal merge reconciles by name (x86_64 numbering; arm64-only archives ship fat with an empty x86_64
slice, appended to `overlumeStaticTargets.cmake`); (2) Apple-only `-Werror` is removed from Filament's own
targets (Xcode 15.4 SDK deprecations) and Filament's iOS toolchain is patched to
`-mios-simulator-version-min` for simulator slices; (3) the .pkg/.tar.gz are assembled with
pkgbuild/productbuild/tar from lipo'd installs, not CPack; (4) the xcframework headers are flat
(`<Overlume/api.h>`); (5) the runners' CMake 4 needs `CMAKE_POLICY_VERSION_MINIMUM=3.5` for yaml-cpp
0.8.0; (6) static xcframework archives are merged per arch with `libtool -static -arch_only`.
**Known limitation (not fixed): no Metal frame renders on CI.** The hosted macOS runners' paravirtual GPU
(`Apple Paravirtual device`, also behind the iOS simulator's GPU) cannot drive Filament's Metal driver
(`newArgumentEncoderWithLayout:` is missing and aborts). `create_renderer` returns nullptr there (device
name probe in `platform_metal.cpp`), the macOS gpu-labelled tests (incl. `ReadbackOrientation.Row0IsTopOfImage`)
are not run (workflow `::warning::`), the macOS smoke runs `--expect-no-gpu` (`::warning::`) and the iOS
simulator smoke skips its render (`::warning::`). Metal rendering, including row-0-is-top, needs a one-off
run on a real Mac (or a self-hosted macOS runner; `OVERLUME_SMOKE_RENDER=1` re-enables the simulator render)
before the first tagged release.

### Task 8: Windows x64 + arm64 (MSVC, WGL/OpenGL)

**Files:** Create `overlume/src/platform_wgl.cpp`, `overlume/cmake/vcpkg-triplets/{x64,arm64}-windows-overlume.cmake`,
`overlume/scripts/check_shared_exports.ps1`, `tools/windows/sign.ps1`; modify `GetFilament.cmake`
(`filament-windows.tgz` `/MD` release libs for x64; arm64 via the source
path with the MSVC arm64 toolset, host `matc` from the x64 tarball),
`overlume/CMakeLists.txt`, `OverlumePackaging.cmake`, `release.yml`.

- [ ] **Step 1:** MSVC v143, `CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL`,
  `/utf-8 /permissive- /Zc:__cplusplus /EHsc`. Compile fixes in `src/` with
  portable code; `#ifdef _WIN32` only around OS API calls.
- [ ] **Step 2:** `platform_wgl.cpp`: `{nullptr, Backend::OPENGL}` (Filament's
  `PlatformWGL`, headless swap chain); libs `opengl32 gdi32 user32`.
- [ ] **Step 3:** `WINDOWS_EXPORT_ALL_SYMBOLS ON` on `overlume_shared`
  (`# ponytail: exports overlume's own objects only, never Filament/cesium
  archives; an OVERLUME_API header macro is the upgrade if the export table
  ever matters`). `check_shared_exports.ps1` (`dumpbin /exports`, fail on
  `filament|YAML|spdlog|Cesium`) is the Windows `shared_exports` ctest.
- [ ] **Step 4:** CPack `NSIS;ZIP` per arch (`CPACK_NSIS_MODIFY_PATH ON`,
  components `overlume` (required) + `static` (optional checkbox)).
- [ ] **Step 5: Signing** `tools/windows/sign.ps1`: if `WINDOWS_SIGNING_*`
  set → `signtool sign /fd sha256 /tr http://timestamp.digicert.com /td sha256`
  (or Azure Trusted Signing action) on `overlume.dll` and the installer;
  else `::warning::`; check `signtool verify /pa` when signed.
- [ ] **Step 6:** CI matrix `windows-x64` (`windows-2022`) and
  `windows-arm64` (`windows-11-arm`): build, `ctest -L cpu`, GPU tests with
  Mesa llvmpipe `opengl32.dll` (`pal1000/mesa-dist-win`, pinned + SHA256;
  x64 and arm64 builds), NSIS silent install `/S /D=C:\overlume`, smoke
  consumer via `-DCMAKE_PREFIX_PATH=C:\overlume` with `yaml-cpp` from vcpkg.
- [ ] **Step 7:** dry-run green; Linux gate green. **Commit**
  `feat(windows): MSVC/WGL build, x64+arm64 NSIS/zip packages`.

### Task 9: vcpkg overlay port + Conan recipe

**Files:** Create `packaging/vcpkg/ports/overlume/{vcpkg.json.in,portfile.cmake.in,usage}`,
`packaging/conan/{conanfile.py,conandata.yml.in,test_package/{conanfile.py,CMakeLists.txt,main.cpp}}`,
`tools/release/render_vcpkg_conan.sh`; modify `release.yml`.

**Produces:** release assets `overlume-<ver>-vcpkg-port.zip`
(`ports/overlume/…`) and `overlume-<ver>-conan-recipe.zip`, rendered with
the SHA512/SHA256 of every platform archive of that release.

- [ ] **Step 1: vcpkg port** — binary port: `portfile.cmake` picks the
  release archive by `VCPKG_TARGET_IS_WINDOWS/OSX/LINUX` + `VCPKG_TARGET_ARCHITECTURE`
  (`x64-windows`, `arm64-windows`, `x64-linux`, `arm64-linux`,
  `x64-osx`/`arm64-osx` → universal), `vcpkg_download_distfile` with SHA512,
  extracts, installs `include`, `lib`/`bin`, `share/overlume`; static
  triplets (`*-static`, `*-static-md`) install the `static` component
  instead; `vcpkg_cmake_config_fixup(PACKAGE_NAME overlume CONFIG_PATH lib/cmake/overlume)`;
  `set(VCPKG_POLICY_DLLS_WITHOUT_LIBS …)` only if the linter needs it; `usage`
  shows `find_package(overlume CONFIG REQUIRED)`. Unsupported triplets fail
  with a clear message.
- [ ] **Step 2: Conan 2 recipe** `conanfile.py`: `package_type` from
  `options.shared` (default `True`), `settings` os/arch, `source()` none,
  `build()` downloads + checks the archive from `conandata.yml`
  (`sources[version][os][arch]` url+sha256), `package()` copies the tree,
  `package_info()` sets `cmake_file_name "overlume"`, `cmake_target_name "overlume::overlume"`
  (static: `overlume::overlume_static` + `system_libs`/`frameworks`).
  `test_package` builds and links the 5-line consumer.
- [ ] **Step 3:** `render_vcpkg_conan.sh RELEASE_DIR VERSION` fills both
  templates from the `SHA256SUMS-*` files (computing SHA512 for vcpkg).
- [ ] **Step 4: Checks** in a `channels` job matrix (ubuntu-22.04,
  macos-14, windows-2022), on dry runs pointed at the run's own artifacts
  via a local `http.server`: `vcpkg install overlume --overlay-ports=…` +
  build the consumer with the vcpkg toolchain; `conan create packaging/conan --version <ver>`
  (runs `test_package`) for `-o shared=True` and `False`.
- [ ] **Step 5:** dry-run green. **Commit** `feat(release): vcpkg overlay port and Conan recipe generated per release`.

### Task 10: Signed apt + yum repositories on GitHub Pages (merged with API docs)

**Files:** Create `tools/release/build_apt_repo.sh`, `tools/release/build_yum_repo.sh`,
`.github/workflows/pages.yml`; delete the deploy job from `.github/workflows/docs.yml`
(it keeps building docs on PRs as a check); modify `release.yml` (trigger `pages.yml`
via `workflow_call` after publish).

**Produces:** `https://amerghazal7.github.io/overlume/` serving docs (unchanged
URLs), `apt/` (`dists/stable/{InRelease,Release,Release.gpg}`, `main/binary-{amd64,arm64}/`,
`pool/`), `rpm/{x86_64,aarch64}/repodata/repomd.xml{,.asc}` + packages,
`overlume-release.asc`, `overlume.repo`.

- [ ] **Step 1:** Pages is stateless: `pages.yml` (on push to main, on
  `workflow_call`, on `workflow_dispatch`) builds docs, then
  `gh release download` the `.deb`/`.rpm` assets of the newest **N**
  releases, where N is the largest count keeping the site < 900 MB (Pages
  limit 1 GB; the script computes N from asset sizes and prints it; ≥ 1
  enforced, else FAIL). Older versions stay on the Releases page.
- [ ] **Step 2:** `build_apt_repo.sh` with `reprepro` (`Codename: stable`,
  `Architectures: amd64 arm64`, `Components: main`, `SignWith: <fingerprint>`)
  → `InRelease` + `Release.gpg`. `build_yum_repo.sh` with `createrepo_c` per
  arch, `gpg --detach-sign --armor repodata/repomd.xml`, and an
  `overlume.repo` (`gpgcheck=1`, `repo_gpgcheck=1`,
  `gpgkey=https://amerghazal7.github.io/overlume/overlume-release.asc`).
  GPG imported into a temp `GNUPGHOME` from secrets, removed on exit.
- [ ] **Step 3: Check** — `tools/release/channel_smoke.sh repo SITE_DIR`
  serves the built site with `python3 -m http.server` and, in
  `ubuntu:22.04`, `debian:12`, `almalinux:8`, `fedora:40` containers, adds
  the repo exactly as the README will say (apt: keyring to
  `/etc/apt/keyrings/overlume.asc`, `signed-by=` source line; dnf:
  `overlume.repo` with the URL rewritten to the local server), installs
  `overlume`, runs the smoke `--expect-render`. Tamper check: flip a byte
  in `InRelease` → `apt-get update` must fail; restore.
- [ ] **Step 4:** docs URLs unchanged (`tools/check_docs_links.py` clean;
  `curl -s -o /dev/null -w '%{http_code}'` on the live docs index after the
  first deploy prints 200). **Commit**
  `feat(release): signed apt/yum repos on GitHub Pages alongside API docs`.

### Task 11: Release orchestration and integrity

**Files:** Modify `.github/workflows/release.yml`, `tools/release/sign_sums.sh`.

- [ ] **Step 1:** Job graph: `create` → `package-linux` (x86_64, aarch64),
  `package-android`, `package-macos`, `package-ios`, `package-windows`
  (x64, arm64) → `sums` (merges all `SHA256SUMS-*` into `SHA256SUMS`,
  signs `SHA256SUMS.asc`, uploads) → publish jobs `publish-maven`,
  `publish-homebrew`, `publish-swiftpm`, `channels` (vcpkg/Conan render +
  checks), `pages` (`workflow_call`). Publish jobs run only when every
  package job succeeded (no partial release); a failed tag run leaves the
  release as **draft**: `create` makes it `--draft`, a final `finalize` job
  flips it to published after all publish jobs pass.
- [ ] **Step 2:** Concurrency group `release-${{ github.ref }}`; every
  third-party action pinned by commit SHA; `timeout-minutes` per job.
- [ ] **Step 3: Check** — `gh workflow run release.yml -f dry_run=true` on
  the work branch: every job green, Maven `VALIDATED` then dropped,
  `gpg --verify SHA256SUMS.asc SHA256SUMS` OK with the public key, and
  `sha256sum -c SHA256SUMS` OK over all downloaded artifacts.
- [ ] **Step 4: Commit** `ci(release): draft-until-green release graph with signed checksums`.

### Task 12: Docs, notices, end-to-end channel smoke

**Files:** Create `docs/runbooks/release.md`, `tools/release/channel_smoke.sh`
(extend from Task 10); modify `README.md` (Install section per platform/channel),
`docs/README.md`, `docs/status.md`, `CHANGELOG.md`, `NOTICE`.

- [ ] **Step 1: NOTICE** — every library now shipped inside the binaries,
  verified against pinned sources: libc++/libc++abi/libunwind
  (`Apache-2.0 WITH LLVM-exception`), spdlog, fmt, and the cesium vcpkg
  closure (from `vcpkg_installed/<triplet>/share/*/copyright`, per platform).
- [ ] **Step 2: README Install** — apt (keyring + `signed-by` line), dnf
  (`overlume.repo`), `brew install amerghazal7/overlume/overlume`, SwiftPM
  snippet, Gradle `implementation("io.github.amerghazal7:overlume:<ver>")` +
  `buildFeatures { prefab = true }` + `find_package(overlume REQUIRED CONFIG)`,
  vcpkg overlay port and Conan commands, Windows installer, manual archives;
  consumer CMake snippet; static-component note (Linux: clang+libc++).
- [ ] **Step 3: runbook** `release.md`: bump `version.h` + `project()` →
  CHANGELOG → tag/push → watch → verify; dry-run rehearsal; rerun a failed
  platform; key rotation (new key, update secrets, re-publish
  `overlume-release.asc`, users re-import); Apple/Windows signing secret
  provisioning steps; Maven namespace/keyserver prerequisites.
- [ ] **Step 4:** `channel_smoke.sh all RUN_ID` downloads one dry-run's
  artifacts and runs every README install path that can run on Linux
  (apt, dnf, vcpkg, Conan, Android NDK consumer) exactly as documented;
  macOS/Windows/iOS paths run in the Task 9/7 CI jobs. All PASS.
- [ ] **Step 5:** `python3 tools/check_docs_links.py` clean; gate green;
  status ledger updated with the first full dry run's URL. **Commit**
  `docs(release): install guide per channel, release runbook, third-party notices`.

## Open items owned by the user (tracked in `docs/status.md`)

- Public key sent to keyserver.ubuntu.com and keys.openpgp.org by the user
  2026-10-02; keys.openpgp.org serves it (HTTP 200), keyserver.ubuntu.com
  still 404 at last check (propagation) — re-check before Task 6's first
  Maven validation. keys.openpgp.org UID shows only after the emailed link is clicked.
- Apple Developer ID + notary API key secrets (until then: unsigned + warning).
- Windows code-signing secrets (until then: unsigned + warning).
