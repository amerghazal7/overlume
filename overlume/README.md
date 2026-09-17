# overlume

`overlume`'s ROS-free rendering library. Compiles
clang/libc++ (Filament's prebuilt SDK requires it); the ROS node that
eventually links it stays gcc/libstdc++ — the two never mix at an ABI
boundary, which is why the public API (`include/overlume/api.h`) is
POD-only (checked by `scripts/check_pod_header.sh`). See
`docs/design/2026-08-18-visual-mode-design.md` §2 and the plan's
Epic 0 Task 1 notes for the full rationale.

## Build

Three commands, from a clean shell, no manual exports:

```bash
scripts/setup_toolchain_cesium.sh
cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .
cmake --build build && ctest --test-dir build --output-on-failure
```

- **`scripts/setup_toolchain_cesium.sh`** — root-less bootstrap of
  clang-18/libc++-18 from apt.llvm.org (no `apt install`, no root) into
  `${XDG_CACHE_HOME:-$HOME/.cache}/overlume-toolchain-cesium`. Idempotent: exits
  immediately if that prefix already has a working clang++. Skip this step
  entirely if `clang++` on `PATH` already has a co-located libc++ (a normal
  root-installed `clang` + `libc++-dev`). (VM-061 Step 6, user decision
  2026-09-15: this is now the PRIMARY toolchain, not just the cesium/vcpkg
  one — `scripts/setup_toolchain.sh`'s older clang-14 prefix is no longer
  used here; clang-18 is required because cesium-native's vcpkg dependency
  ada-url needs `std::ranges::replace`, which libc++-14/-15 don't
  implement.)
- **`cmake --toolchain cmake/toolchain-clang-libcxx.cmake`** — selects that
  clang++ (or the PATH one) and bakes in `-stdlib=libc++`. `GetFilament.cmake`
  then fetches the pinned Filament 1.56.5 prebuilt SDK (sha256-verified) on
  first configure.
- **`cmake --build && ctest`** — builds the static `overlume` lib and
  runs 2 tests: `check_pod_header` (the public header stays POD-only) and
  `filament_link_probe` (a real executable that links `overlume` and
  calls into Filament, proving the ~30-archive link graph resolves).

The result needs no `LD_LIBRARY_PATH` or rpath to run: `overlume`
statically links libc++/libc++abi/libunwind (see `CMakeLists.txt`), so
binaries are self-contained even off this dev box.

## Why clang/libc++, why pinned at 1.56.5, why static

See the dated comments in `cmake/GetFilament.cmake` and
`docs/plans/2026-08-18-visual-mode.md` (Epic 0 Task 1) for the
full history: newer Filament releases (~v1.57+) require glibc ≥2.38 and don't
link on this project's Ubuntu 22.04 (glibc 2.35) boxes; the archive list
needs `-Wl,--start-group/--end-group` because Filament ships no CMake package
config for its ~30 mutually-cross-referencing static libs; and static-linking
the C++ runtime avoids needing a hand-exported `LD_LIBRARY_PATH` for the
toolchain's co-located `libc++.so.1`/`libunwind.so.1` (a real, previously-hit
failure mode). One extra landmine worth knowing up front: this box also has
an unrelated, ABI-incompatible system package that happens to also be named
`libunwind` (the nongnu backtrace library) — `CMakeLists.txt` resolves the
correct (LLVM) one as a sibling of `libc++.a`, not via a bare `-lunwind`.
