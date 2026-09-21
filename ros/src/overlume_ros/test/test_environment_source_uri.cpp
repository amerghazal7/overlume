// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

/** @file test_environment_source_uri.cpp
 *  @brief Unit coverage for compose_environment_source_uri() -- VM-064 gate
 *  round 1 finding: a non-empty environment_tile_cache_dir used to be
 *  appended unconditionally, so composing it against the google preset's
 *  "...&cache=off" produced a second "cache=" key that parse_ion_spec()'s
 *  last-wins loop resolved to the DIR, silently defeating Decision 14 Step
 *  2(b)'s compliance lever. Exercises the header directly; no ROS/renderer
 *  runtime needed.
 */

#include <gtest/gtest.h>

#include "overlume_ros/environment_source_uri.hpp"

namespace overlume::ros {
namespace {

TEST(ComposeEnvironmentSourceUri, EmptySourceUriUsesChunksDirVerbatim) {
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "", ""), "/baked/chunks");
}

// 2026-09-21: compose_environment_source_uri() gained a 4th `follow_terrain`
// param (default false, see its own header comment) -- every ion:// URI now
// also gets an explicit follow_terrain=on/off key, so these pre-existing
// pinned strings (the 3-arg overload omits the new param) grew that key
// too. See the FollowTerrain* tests below for coverage of the key itself.
TEST(ComposeEnvironmentSourceUri, PlainIonUriWithCacheDirAppendsCache) {
    EXPECT_EQ(compose_environment_source_uri("", "ion://96188", "/mnt/data/tiles"),
              "ion://96188?cache=/mnt/data/tiles&follow_terrain=off");
}

TEST(ComposeEnvironmentSourceUri, ChunksDirFallbackAppendedWhenNotAlreadyPresent) {
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "ion://96188", ""),
              "ion://96188?fallback=/baked/chunks&follow_terrain=off");
}

// The actual gate finding: the google preset's URI already carries
// "cache=off" (Decision 14 Step 2b) -- a non-empty tile-cache-dir param must
// NOT add a second cache= key, which parse_ion_spec()'s last-wins loop would
// resolve to the dir, overwriting "off".
TEST(ComposeEnvironmentSourceUri, GooglePresetCacheOffSurvivesNonEmptyCacheDirParam) {
    const std::string google_preset_uri = "ion://2275207?materials=original&cache=off";
    EXPECT_EQ(compose_environment_source_uri("", google_preset_uri, "/mnt/data/tiles"),
              google_preset_uri + "&follow_terrain=off");
}

// Same duplicate-key hazard for fallback=.
TEST(ComposeEnvironmentSourceUri, ExplicitFallbackInUriSurvivesNonEmptyChunksDir) {
    const std::string uri_with_fallback = "ion://96188?fallback=/explicit/dir";
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", uri_with_fallback, ""),
              uri_with_fallback + "&follow_terrain=off");
}

// 2026-09-21 ("option 2"): follow_terrain=on|off is appended to an ion://
// URI only, only when absent, and defaults false (every pre-existing call
// in this file, above, is unaffected by the new 4th param's default).
TEST(ComposeEnvironmentSourceUri, FollowTerrainAppendedWhenAbsent) {
    EXPECT_EQ(compose_environment_source_uri("", "ion://96188", "", /*follow_terrain=*/true),
              "ion://96188?follow_terrain=on");
    EXPECT_EQ(compose_environment_source_uri("", "ion://96188", "", /*follow_terrain=*/false),
              "ion://96188?follow_terrain=off");
}

TEST(ComposeEnvironmentSourceUri, FollowTerrainNotDuplicatedWhenPresent) {
    const std::string uri_with_follow_terrain = "ion://96188?follow_terrain=off";
    EXPECT_EQ(compose_environment_source_uri("", uri_with_follow_terrain, "",
                                             /*follow_terrain=*/true),
              uri_with_follow_terrain);
}

TEST(ComposeEnvironmentSourceUri, FollowTerrainNeverAppendedToPlainDirectoryUri) {
    // environment_source_uri empty -> environment_chunks_dir_ verbatim (a
    // plain baked-chunk directory, no query string at all) -- true here
    // must have no effect.
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "", "", /*follow_terrain=*/true),
              "/baked/chunks");
}

TEST(ComposeEnvironmentSourceUri, FollowTerrainCombinesWithCacheAndFallback) {
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "ion://96188", "/mnt/data/tiles",
                                             /*follow_terrain=*/true),
              "ion://96188?cache=/mnt/data/tiles&fallback=/baked/chunks&follow_terrain=on");
}

}  // namespace
}  // namespace overlume::ros

// fallback_dir_from_source_uri(): the STREAMING_FALLBACK WARN names the dir
// actually in effect (final review 2026-09-17, finding #17). Reads the
// COMPOSED string, so an inline fallback= wins exactly as compose does.
TEST(FallbackDirFromSourceUri, NoKeyIsEmpty) {
    EXPECT_EQ(overlume::ros::fallback_dir_from_source_uri("ion://96188?cache=/c"), "");
    EXPECT_EQ(overlume::ros::fallback_dir_from_source_uri(""), "");
}

TEST(FallbackDirFromSourceUri, LastKeyRunsToEnd) {
    EXPECT_EQ(
        overlume::ros::fallback_dir_from_source_uri("ion://96188?cache=/c&fallback=/baked/dir"),
        "/baked/dir");
}

TEST(FallbackDirFromSourceUri, MiddleKeyStopsAtAmpersand) {
    EXPECT_EQ(
        overlume::ros::fallback_dir_from_source_uri("ion://96188?fallback=/explicit&cache=off"),
        "/explicit");
}

TEST(FallbackDirFromSourceUri, InlineFallbackWinsThroughCompose) {
    const std::string composed = overlume::ros::compose_environment_source_uri(
        "/chunks", "ion://96188?fallback=/explicit", "");
    EXPECT_EQ(overlume::ros::fallback_dir_from_source_uri(composed), "/explicit");
}

// 2026-09-21: ground_bias= rides on the library default (0.3 m) and is only
// composed when the node param differs from it.
TEST(ComposeEnvironmentSourceUri, GroundBiasAppendedOnlyWhenNotDefault) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3),
              "ion://96188?follow_terrain=on");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.5),
              "ion://96188?follow_terrain=on&ground_bias=0.5");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188?ground_bias=0", "",
                                                            true, 0.5),
              "ion://96188?ground_bias=0&follow_terrain=on");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked", "", "", true, 0.5), "/baked");
}

// 2026-09-21 live finding: replaces_ground= rides on the library default
// (true) and is only composed (as "off") when the node param differs from
// it -- same "only when it differs from default" shape as ground_bias=
// above, so every pre-existing pinned string in this file (all omitting
// the 6th param) stays unchanged.
TEST(ComposeEnvironmentSourceUri, ReplacesGroundAppendedOnlyWhenFalse) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, true),
              "ion://96188?follow_terrain=on");
    EXPECT_EQ(
        overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, false),
        "ion://96188?follow_terrain=on&replaces_ground=off");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188?replaces_ground=off",
                                                            "", true, 0.3, false),
              "ion://96188?replaces_ground=off&follow_terrain=on");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked", "", "", true, 0.3, false),
              "/baked");
}

// 2026-09-21 multi-point plane fit: max_tilt_deg= rides on the library
// default (2.0) and is only composed when the node param differs from it --
// same "only when it differs from default" shape as ground_bias=/
// replaces_ground= above, so every pre-existing pinned string in this file
// (all omitting the 7th param) stays unchanged.
TEST(ComposeEnvironmentSourceUri, MaxTiltDegAppendedOnlyWhenNotDefault) {
    EXPECT_EQ(
        overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, true, 2.0),
        "ion://96188?follow_terrain=on");
    EXPECT_EQ(
        overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, true, 5.0),
        "ion://96188?follow_terrain=on&max_tilt_deg=5");
    EXPECT_EQ(
        overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, true, 0.0),
        "ion://96188?follow_terrain=on&max_tilt_deg=0");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188?max_tilt_deg=0", "",
                                                            true, 0.3, true, 5.0),
              "ion://96188?max_tilt_deg=0&follow_terrain=on");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked", "", "", true, 0.3, true, 5.0),
              "/baked");
}
