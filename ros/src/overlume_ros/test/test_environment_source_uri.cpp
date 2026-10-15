// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include <gtest/gtest.h>

#include "overlume_ros/environment_source_uri.hpp"

#include "overlume/scene.h"

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

}
}

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

TEST(ComposeEnvironmentSourceUri, RadiusAppendedOnlyWhenNotDefault) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, true,
                                                            2.0, 1.0, 700.0),
              "ion://96188?follow_terrain=on");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188", "", true, 0.3, true,
                                                            2.0, 1.0, 250.0),
              "ion://96188?follow_terrain=on&radius=250");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "ion://96188?radius=50", "", true,
                                                            0.3, true, 2.0, 1.0, 250.0),
              "ion://96188?radius=50&follow_terrain=on");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked", "", "", true, 0.3, true, 2.0,
                                                            1.0, 250.0),
              "/baked");
}

TEST(ComposeEnvironmentSourceUri, MapboxGetsFallbackButNeverCache) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked/chunks", "mapbox://buildings",
                                                            "/mnt/data/tiles"),
              "mapbox://buildings?fallback=/baked/chunks");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri(
                  "/baked/chunks", "mapbox://buildings?materials=original", "/mnt/data/tiles"),
              "mapbox://buildings?materials=original&fallback=/baked/chunks");
}

TEST(ComposeEnvironmentSourceUri, MapboxNeverGetsTheIonOnlyKeys) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "mapbox://buildings", "", true, 0.5,
                                                            false, 5.0),
              "mapbox://buildings");
}

TEST(ComposeEnvironmentSourceUri, MapboxGetsBrightnessAndRadiusWhenNotDefault) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "mapbox://buildings", "", true, 0.3,
                                                            true, 2.0, 1.0, 700.0),
              "mapbox://buildings");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri(
                  "/baked", "mapbox://buildings", "/tiles", true, 0.5, false, 5.0, 1.8, 250.0),
              "mapbox://buildings?fallback=/baked&brightness=1.8&radius=250");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "mapbox://buildings?radius=300", "",
                                                            true, 0.3, true, 2.0, 1.8, 250.0),
              "mapbox://buildings?radius=300&brightness=1.8");
}

TEST(ComposeEnvironmentSourceUri, IonUriWithEveryKnobSetIsUnchanged) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked", "ion://96188", "/tiles", true,
                                                            0.5, false, 5.0, 1.8, 250.0),
              "ion://96188?cache=/tiles&fallback=/baked&follow_terrain=on&ground_bias=0.5&"
              "replaces_ground=off&max_tilt_deg=5&brightness=1.8&radius=250");
}

TEST(MapboxAttributionKind, TruthTableOverStateVisibilityParameterAndScheme) {
    using overlume::EnvironmentSourceState;
    using overlume::ros::AttributionKind;
    struct Case {
        const char* uri;
        AttributionKind when_streaming;
    };
    const Case cases[] = {
        {"", AttributionKind::NONE},
        {"/baked/chunks", AttributionKind::NONE},
        {"ion://96188", AttributionKind::NONE},
        {"ion://2275207?materials=original&cache=off", AttributionKind::GOOGLE},
        {"mapbox://buildings", AttributionKind::MAPBOX},
        {"mapbox://buildings?materials=original", AttributionKind::MAPBOX},
    };
    const EnvironmentSourceState states[] = {
        EnvironmentSourceState::NONE, EnvironmentSourceState::BAKED,
        EnvironmentSourceState::STREAMING, EnvironmentSourceState::STREAMING_FALLBACK};
    for (const Case& c : cases) {
        for (const EnvironmentSourceState state : states) {
            for (const bool visible : {false, true}) {
                for (const bool enabled : {false, true}) {
                    const AttributionKind expected =
                        visible && enabled && state == EnvironmentSourceState::STREAMING
                            ? c.when_streaming
                            : AttributionKind::NONE;
                    EXPECT_EQ(
                        overlume::ros::environment_attribution_kind(visible, enabled, c.uri, state),
                        expected)
                        << "uri '" << c.uri << "' state " << static_cast<int>(state) << " visible "
                        << visible << " enabled " << enabled;
                }
            }
        }
    }
}

namespace {
// The two layer parameters, with every other node knob at its default.
std::string compose_layers(const std::string& uri, const std::string& terrain,
                           const std::string& imagery) {
    return overlume::ros::compose_environment_source_uri(
        "", uri, "", false, overlume::ros::kGroundBiasDefaultM,
        overlume::ros::kReplacesGroundDefault, overlume::ros::kMaxTiltDegDefault,
        overlume::ros::kBrightnessDefault, overlume::ros::kTileRadiusDefaultM, terrain, imagery);
}
}  // namespace

TEST(ComposeEnvironmentSourceUri, MapboxTerrainEnableAddsTheLayerOnce) {
    EXPECT_EQ(compose_layers("mapbox://buildings", "enable", ""), "mapbox://buildings,terrain");
    EXPECT_EQ(compose_layers("mapbox://buildings?materials=original", "enable", ""),
              "mapbox://buildings,terrain?materials=original");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain", "enable", ""),
              "mapbox://buildings,terrain");
    EXPECT_EQ(compose_layers("mapbox://terrain,buildings?imagery=satellite", "enable", ""),
              "mapbox://terrain,buildings?imagery=satellite");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked", "mapbox://buildings",
                                                            "/tiles", true, 0.5, false, 5.0, 1.8,
                                                            250.0, "enable", "satellite"),
              "mapbox://buildings,terrain?fallback=/baked&brightness=1.8&radius=250&"
              "imagery=satellite");
}

TEST(ComposeEnvironmentSourceUri, MapboxTerrainDisableOrEmptyLeavesTheLayerList) {
    EXPECT_EQ(compose_layers("mapbox://buildings", "disable", ""), "mapbox://buildings");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain", "disable", ""),
              "mapbox://buildings,terrain");
    EXPECT_EQ(compose_layers("mapbox://buildings", "", ""), "mapbox://buildings");
}

TEST(ComposeEnvironmentSourceUri, MapboxImageryAppendedOnlyWhenTheUriLacksIt) {
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain", "", "satellite"),
              "mapbox://buildings,terrain?imagery=satellite");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain", "", "disable"),
              "mapbox://buildings,terrain?imagery=off");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain?imagery=satellite", "", "disable"),
              "mapbox://buildings,terrain?imagery=satellite");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain?imagery=off", "", "satellite"),
              "mapbox://buildings,terrain?imagery=off");
}

TEST(ComposeEnvironmentSourceUri, IonTerrainAndImageryAppendedOnlyWhenTheUriLacksThem) {
    EXPECT_EQ(compose_layers("ion://96188", "ion:1", "ion:2"),
              "ion://96188?follow_terrain=off&terrain=1&imagery=2");
    EXPECT_EQ(compose_layers("ion://96188", "disable", "disable"),
              "ion://96188?follow_terrain=off&terrain=off&imagery=off");
    EXPECT_EQ(compose_layers("ion://96188?terrain=1&imagery=2", "disable", "disable"),
              "ion://96188?terrain=1&imagery=2&follow_terrain=off");
    EXPECT_EQ(compose_layers("ion://96188?imagery=3", "ion:1", "ion:2"),
              "ion://96188?imagery=3&follow_terrain=off&terrain=1");
}

TEST(ComposeEnvironmentSourceUri, ValuesForTheOtherSchemeOrNoSchemeAreIgnored) {
    EXPECT_EQ(compose_layers("mapbox://buildings", "ion:1", "ion:2"), "mapbox://buildings");
    EXPECT_EQ(compose_layers("ion://96188", "enable", "satellite"),
              "ion://96188?follow_terrain=off");
    EXPECT_EQ(compose_layers("ion://96188", "1", "ion:"), "ion://96188?follow_terrain=off");
    EXPECT_EQ(compose_layers("ion://96188", "ion:-1", "ion:2x"), "ion://96188?follow_terrain=off");
    EXPECT_EQ(compose_layers("mapbox://buildings", "on", "off"), "mapbox://buildings");
}

TEST(ComposeEnvironmentSourceUri, IonTerrainKeyIsNotFollowTerrain) {
    EXPECT_EQ(compose_layers("ion://96188?follow_terrain=on", "ion:1", ""),
              "ion://96188?follow_terrain=on&terrain=1");
}

TEST(ComposeEnvironmentSourceUri, EmptyLayerParametersAndBakedSourcesAppendNothing) {
    EXPECT_EQ(compose_layers("ion://96188", "", ""), "ion://96188?follow_terrain=off");
    EXPECT_EQ(compose_layers("/baked/chunks", "enable", "satellite"), "/baked/chunks");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri(
                  "/baked/chunks", "", "", false, 0.3, true, 2.0, 1.0, 700.0, "ion:1", "ion:2"),
              "/baked/chunks");
}

TEST(ComposeEnvironmentSourceUri, IonUriWithEveryKnobAndBothLayersKeepsTheOrder) {
    EXPECT_EQ(
        overlume::ros::compose_environment_source_uri("/baked", "ion://96188", "/tiles", true, 0.5,
                                                      false, 5.0, 1.8, 250.0, "ion:1", "ion:2"),
        "ion://96188?cache=/tiles&fallback=/baked&follow_terrain=on&ground_bias=0.5&"
        "replaces_ground=off&max_tilt_deg=5&brightness=1.8&radius=250&terrain=1&imagery=2");
}

TEST(EnvironmentLayerWarning, NamesAValueTheConfiguredSourceIgnores) {
    using overlume::ros::environment_layer_warning;
    EXPECT_EQ(
        environment_layer_warning("environment_terrain", "ion:1", "enable", "mapbox://buildings"),
        "environment_terrain 'ion:1' is ignored for environment source 'mapbox://buildings' "
        "(accepted: \"\", disable, enable for mapbox://, ion:<asset id> for ion://)");
    EXPECT_EQ(
        environment_layer_warning("environment_imagery", "satellite", "satellite", "ion://96188"),
        "environment_imagery 'satellite' is ignored for environment source 'ion://96188' "
        "(accepted: \"\", disable, satellite for mapbox://, ion:<asset id> for ion://)");
    EXPECT_EQ(environment_layer_warning("environment_imagery", "sat", "satellite", ""),
              "environment_imagery 'sat' is ignored for environment source '' "
              "(accepted: \"\", disable, satellite for mapbox://, ion:<asset id> for ion://)");
}

TEST(EnvironmentLayerWarning, SilentForAValueThatApplies) {
    using overlume::ros::environment_layer_warning;
    EXPECT_EQ(
        environment_layer_warning("environment_terrain", "enable", "enable", "mapbox://buildings"),
        "");
    EXPECT_EQ(environment_layer_warning("environment_terrain", "ion:1", "enable", "ion://96188"),
              "");
    EXPECT_EQ(
        environment_layer_warning("environment_imagery", "disable", "satellite", "ion://96188"),
        "");
    EXPECT_EQ(environment_layer_warning("environment_imagery", "", "satellite", "ion://96188"), "");
    EXPECT_EQ(environment_layer_warning("environment_terrain", "ion:1", "enable", "/baked/chunks"),
              "");
    EXPECT_EQ(environment_layer_warning("environment_imagery", "satellite", "satellite", ""), "");
}

TEST(MapboxAttributionKind, CreditsMakeAnIonSourceOtherThanGoogleCredits) {
    using overlume::EnvironmentSourceState;
    using overlume::ros::AttributionKind;
    using overlume::ros::environment_attribution_kind;
    const auto streaming = EnvironmentSourceState::STREAMING;
    EXPECT_EQ(environment_attribution_kind(true, true, "ion://96188?terrain=1&imagery=2", streaming,
                                           true),
              AttributionKind::CREDITS);
    EXPECT_EQ(environment_attribution_kind(true, true, "ion://96188?terrain=1&imagery=2", streaming,
                                           false),
              AttributionKind::NONE);
    EXPECT_EQ(environment_attribution_kind(true, true, "ion://2275207?materials=original",
                                           streaming, true),
              AttributionKind::GOOGLE);
    EXPECT_EQ(
        environment_attribution_kind(true, true, "mapbox://buildings,terrain", streaming, true),
        AttributionKind::MAPBOX);
    EXPECT_EQ(environment_attribution_kind(true, true, "/baked/chunks", streaming, true),
              AttributionKind::NONE);
    EXPECT_EQ(environment_attribution_kind(true, true, "ion://96188", EnvironmentSourceState::BAKED,
                                           true),
              AttributionKind::NONE);
    EXPECT_EQ(environment_attribution_kind(false, true, "ion://96188", streaming, true),
              AttributionKind::NONE);
    EXPECT_EQ(environment_attribution_kind(true, false, "ion://96188", streaming, true),
              AttributionKind::NONE);
}

TEST(EnvironmentAttributionText, WithoutCreditsEachKindDrawsTodaysLine) {
    using overlume::ros::AttributionKind;
    using overlume::ros::environment_attribution_text;
    EXPECT_EQ(environment_attribution_text(AttributionKind::MAPBOX, ""),
              overlume::ros::kMapboxAttributionText);
    EXPECT_EQ(environment_attribution_text(AttributionKind::GOOGLE, ""),
              "3D Tiles data (c) Google");
    EXPECT_EQ(environment_attribution_text(AttributionKind::CREDITS, ""), "");
    EXPECT_EQ(environment_attribution_text(AttributionKind::NONE, ""), "");
}

TEST(EnvironmentAttributionText, CreditsFollowTheBaseTextAndPrecedeImproveThisMap) {
    using overlume::ros::AttributionKind;
    using overlume::ros::environment_attribution_text;
    const std::string bing = "Earthstar Geographics; \xC2\xA9 2026 Microsoft Corporation";
    EXPECT_EQ(environment_attribution_text(AttributionKind::MAPBOX, "\xC2\xA9 Maxar"),
              "\xC2\xA9 Mapbox \xC2\xA9 OpenStreetMap \xC2\xA9 Maxar Improve this map");
    EXPECT_EQ(environment_attribution_text(AttributionKind::GOOGLE, bing),
              "3D Tiles data (c) Google Earthstar Geographics \xC2\xA9 2026 Microsoft Corporation");
    EXPECT_EQ(environment_attribution_text(AttributionKind::CREDITS, bing),
              "Earthstar Geographics \xC2\xA9 2026 Microsoft Corporation");
    EXPECT_EQ(environment_attribution_text(AttributionKind::NONE, bing), "");
}

TEST(EnvironmentAttributionText, EmptyCreditsBetweenSeparatorsAreSkipped) {
    EXPECT_EQ(overlume::ros::environment_attribution_text(overlume::ros::AttributionKind::CREDITS,
                                                          "A; ; B; "),
              "A B");
}
