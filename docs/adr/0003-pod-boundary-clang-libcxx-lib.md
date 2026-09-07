# ADR-0003: clang/libc++ static library behind a POD-only public API

**Status:** Accepted (2026-08-18, recorded 2026-09-07)
**Re-argued in:** spec §2; master plan Global Constraints, Epic 0 Task 1
Fix-ups 1–3, Task 3 Deviation; Epic 2 Task 1 Step 0.

## Context

Filament's prebuilt SDK is built with clang + libc++. The ROS 2 Humble workspace
builds with gcc + libstdc++. Mixing the two runtimes across a `std::` type is
undefined behavior.

## Decision

- `visual_renderer` compiles as a self-contained **clang/libc++ static archive**
  with libc++/libc++abi/libunwind linked statically (`-nostdlib++` + explicit
  archives). A root-less toolchain is bootstrapped by `scripts/setup_toolchain.sh`
  and selected by `cmake/toolchain-clang-libcxx.cmake`.
- The public headers (`include/visual_renderer/*.h`) contain **only** fixed-width
  scalars, POD structs, raw pointers and `const char*`. `scripts/check_pod_header.sh`
  enforces this as a ctest.
- The gcc node consumes the archive as a hand-declared `IMPORTED STATIC` target.
  libc++'s `std::__1` inline namespace prevents symbol collisions with libstdc++.
- Third-party libraries used on both sides (yaml-cpp, googletest) are built twice,
  once per toolchain; a GPU-free test (`theme_parses`) proves coexistence.

## Consequences

- No ABI hazard as long as the header check holds; the node stays on stock gcc.
- Every public entry point must be C-like; strings are borrowed `const char*` with
  documented lifetimes; arrays are pointer + count.
- Two toolchains in one repo; build documentation and CI must carry both.

## Revisit when

- Filament is built from source with gcc (ADR-0001 fallback), which removes the
  runtime split, **or** the header check becomes a recurring obstacle to a
  feature (then consider a C ABI with opaque handles rather than relaxing it).
