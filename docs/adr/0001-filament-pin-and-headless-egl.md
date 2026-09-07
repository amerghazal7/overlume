# ADR-0001: Filament 1.56.5 prebuilt SDK + hand-rolled headless EGL platform

**Status:** Accepted (2026-08-19, recorded 2026-09-07)
**Re-argued in:** master plan Epic 0 Task 1 Step 3 and Task 2 Deviation 1; spec §2.

## Context

Visual mode renders with Google Filament to a headless swapchain. The prebuilt
Linux SDK is the fast path, but two facts constrain the choice:

- Releases newer than ~1.56.x are linked against glibc ≥ 2.38 (`__isoc23_sscanf`
  symbols) and do not link on this project's Ubuntu 22.04 / glibc 2.35 boxes.
  Non-monotonic across patch releases (1.56.5 clean, 1.56.8 not).
- The prebuilt `libbackend.a` ships only `PlatformGLX` (needs `$DISPLAY`) and
  `PlatformNoop`. `PlatformEGLHeadless` is declared but not compiled in.

## Decision

- Pin **Filament 1.56.5** (sha256 recorded in `cmake/GetFilament.cmake`).
- Implement a small `HeadlessEglPlatform` in `renderer.cpp` (raw EGL pbuffer
  context) and hand-declare the two `bluegl::bind()/unbind()` entry points needed
  to populate the driver's GL function table, instead of building Filament from
  source.
- Source build of Filament (documented in `GetFilament.cmake`) is the fallback,
  not the default.

## Consequences

- Fast, reproducible build; no X server needed; proven by `filament_link_probe`
  and `test_hello_frame` running with `DISPLAY` unset.
- Two private-ABI couplings: the mangled names of `bluegl::bind/unbind` and the
  `OpenGLPlatform` virtual interface. Both can change silently on any Filament
  bump; the link probe catches the first, the hello-frame test the second.
- Stuck on 1.56.5 until the robot image moves to glibc ≥ 2.38 or Filament is
  built from source.

## Revisit when

- The robot OS image changes glibc, **or** a Filament feature/fix newer than
  1.56.5 is needed, **or** a driver change breaks the pbuffer path. At that point
  switch to the source build (which also makes `PlatformEGLHeadless` available and
  retires the hand-rolled platform).
