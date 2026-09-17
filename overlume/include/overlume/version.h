// version.h — POD-only, additive (ADR-0004: a new header bumps
// kSceneVersion nothing; this one carries release numbering, not scene.h/
// api.h layout). Overlume's release version as compile-time constants —
// the three component macros are hand-written here, and the same number `project(overlume VERSION ...)` in
// overlume/CMakeLists.txt declares. That CMakeLists.txt has a configure-time
// check (right after its `project()` call) that parses the three #defines
// below and fails the configure step if they don't match
// PROJECT_VERSION_{MAJOR,MINOR,PATCH} — the simplest honest mechanism that
// keeps the two from silently drifting apart, with no generated header and
// no runtime static_assert needed (nothing in src/ or examples/ includes
// this header today, so a compiled check would never actually run).

/// @file
/// @brief Overlume's release version, as macros and a dotted string.
#pragma once

/// @brief Major version component (semver). Bumped on a breaking API change.
#define OVERLUME_VERSION_MAJOR 0
/// @brief Minor version component (semver). Bumped on an additive,
/// backward-compatible change.
#define OVERLUME_VERSION_MINOR 1
/// @brief Patch version component (semver). Bumped on a fix with no API
/// change.
#define OVERLUME_VERSION_PATCH 0

namespace overlume {

/// @cond INTERNAL
#define OVERLUME_STR_(x) #x
#define OVERLUME_STR(x) OVERLUME_STR_(x)
/// @endcond

/// @brief Overlume's release version as a dotted string, e.g. `"0.1.0"`.
///
/// Derived from #OVERLUME_VERSION_MAJOR / #OVERLUME_VERSION_MINOR /
/// #OVERLUME_VERSION_PATCH by stringification, so it cannot drift from
/// them; the three macros are checked against `project(overlume VERSION ...)`
/// in overlume/CMakeLists.txt at configure time — see this file's top comment.
inline constexpr const char* kVersionString = OVERLUME_STR(OVERLUME_VERSION_MAJOR) "." OVERLUME_STR(
    OVERLUME_VERSION_MINOR) "." OVERLUME_STR(OVERLUME_VERSION_PATCH);

}  // namespace overlume
