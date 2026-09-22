// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <gtest/gtest.h>

#include "overlume_ros/environment_source_uri.hpp"

namespace overlume::ros {
namespace {

TEST(ComposeEnvironmentSourceUri, EmptySourceUriUsesChunksDirVerbatim) {
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "", ""), "/baked/chunks");
}

TEST(ComposeEnvironmentSourceUri, PlainIonUriWithCacheDirAppendsCache) {
    EXPECT_EQ(compose_environment_source_uri("", "ion://96188", "/mnt/data/tiles"),
              "ion://96188?cache=/mnt/data/tiles&follow_terrain=off");
}

TEST(ComposeEnvironmentSourceUri, ChunksDirFallbackAppendedWhenNotAlreadyPresent) {
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "ion://96188", ""),
              "ion://96188?fallback=/baked/chunks&follow_terrain=off");
}

TEST(ComposeEnvironmentSourceUri, GooglePresetCacheOffSurvivesNonEmptyCacheDirParam) {
    const std::string google_preset_uri = "ion://2275207?materials=original&cache=off";
    EXPECT_EQ(compose_environment_source_uri("", google_preset_uri, "/mnt/data/tiles"),
              google_preset_uri + "&follow_terrain=off");
}

TEST(ComposeEnvironmentSourceUri, ExplicitFallbackInUriSurvivesNonEmptyChunksDir) {
    const std::string uri_with_fallback = "ion://96188?fallback=/explicit/dir";
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", uri_with_fallback, ""),
              uri_with_fallback + "&follow_terrain=off");
}

TEST(ComposeEnvironmentSourceUri, FollowTerrainAppendedWhenAbsent) {
    EXPECT_EQ(compose_environment_source_uri("", "ion://96188", "", true),
              "ion://96188?follow_terrain=on");
    EXPECT_EQ(compose_environment_source_uri("", "ion://96188", "", false),
              "ion://96188?follow_terrain=off");
}

TEST(ComposeEnvironmentSourceUri, FollowTerrainNotDuplicatedWhenPresent) {
    const std::string uri_with_follow_terrain = "ion://96188?follow_terrain=off";
    EXPECT_EQ(compose_environment_source_uri("", uri_with_follow_terrain, "", true),
              uri_with_follow_terrain);
}

TEST(ComposeEnvironmentSourceUri, FollowTerrainNeverAppendedToPlainDirectoryUri) {
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "", "", true), "/baked/chunks");
}

TEST(ComposeEnvironmentSourceUri, FollowTerrainCombinesWithCacheAndFallback) {
    EXPECT_EQ(
        compose_environment_source_uri("/baked/chunks", "ion://96188", "/mnt/data/tiles", true),
        "ion://96188?cache=/mnt/data/tiles&fallback=/baked/chunks&follow_terrain=on");
}

}  // namespace
}  // namespace overlume::ros

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

TEST(ComposeEnvironmentSourceUri, BrightnessAppendedOnlyWhenNotDefault) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, true,
                                                            2.0, 1.0),
              "ion://96188?follow_terrain=on");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, true,
                                                            2.0, 1.8),
              "ion://96188?follow_terrain=on&brightness=1.8");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188?brightness=2.5", "",
                                                            true, 0.3, true, 2.0, 1.8),
              "ion://96188?brightness=2.5&follow_terrain=on");
    EXPECT_EQ(
        overlume::ros::compose_environment_source_uri("/baked", "", "", true, 0.3, true, 2.0, 1.8),
        "/baked");
}
