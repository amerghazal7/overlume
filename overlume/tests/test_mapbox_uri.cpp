// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_loader.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

using overlume::mapbox::MapboxSpecResult;
using overlume::mapbox::parse_mapbox_spec;

bool mentions(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

}

TEST(MapboxUri, DefaultsWithOrWithoutATrailingQuestionMark) {
    for (const char* spec : {"buildings", "buildings?"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        ASSERT_TRUE(r.spec) << spec << ": " << r.error;
        EXPECT_FALSE(r.spec->materials_original) << spec;
        EXPECT_EQ(r.spec->radius_m, 500.0) << spec;
        EXPECT_EQ(r.spec->radius_m, overlume::mapbox::kDefaultRadiusM) << spec;
        EXPECT_EQ(r.spec->brightness, 1.0) << spec;
        EXPECT_EQ(r.spec->fallback_dir, "") << spec;
        EXPECT_EQ(r.error, "") << spec;
        EXPECT_EQ(r.warning, "") << spec;
    }
}

TEST(MapboxUri, EachKeySetsItsField) {
    const MapboxSpecResult r = parse_mapbox_spec(
        "buildings?materials=original&radius=250.5&brightness=1.75&fallback=/srv/overlume/chunks");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_TRUE(r.spec->materials_original);
    EXPECT_EQ(r.spec->radius_m, 250.5);
    EXPECT_EQ(r.spec->brightness, 1.75);
    EXPECT_EQ(r.spec->fallback_dir, "/srv/overlume/chunks");
    EXPECT_EQ(r.warning, "");
    const MapboxSpecResult clay = parse_mapbox_spec("buildings?materials=clay");
    ASSERT_TRUE(clay.spec) << clay.error;
    EXPECT_FALSE(clay.spec->materials_original);
    EXPECT_EQ(clay.warning, "");
}

TEST(MapboxUri, BadNumbersRefuseTheUriNamingTheKey) {
    const char* const cases[][2] = {
        {"buildings?radius=0", "radius"},
        {"buildings?radius=-5", "radius"},
        {"buildings?radius=abc", "radius"},
        {"buildings?radius=12m", "radius"},
        {"buildings?radius=inf", "radius"},
        {"buildings?radius=nan", "radius"},
        {"buildings?radius=", "radius"},
        {"buildings?brightness=0", "brightness"},
        {"buildings?brightness=-1", "brightness"},
        {"buildings?brightness=dim", "brightness"},
        {"buildings?brightness=1.5x", "brightness"},
        {"buildings?brightness=", "brightness"},
    };
    for (const auto& [spec, key] : cases) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, key)) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, CacheAcceptsOnlyOff) {
    const MapboxSpecResult off = parse_mapbox_spec("buildings?cache=off");
    EXPECT_TRUE(off.spec) << off.error;
    for (const char* spec : {"buildings?cache=/var/cache/overlume",
                             "buildings?cache=", "buildings?cache=on", "buildings?cache=OFF"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, "cache")) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, LayerListInAnyOrderSetsTheLayers) {
    struct Case {
        const char* spec;
        bool buildings, terrain;
        double radius_m;
    };
    for (const Case& c :
         {Case{"buildings", true, false, 500.0}, Case{"terrain", false, true, 500.0},
          Case{"buildings,terrain", true, true, 500.0},
          Case{"terrain,buildings", true, true, 500.0},
          Case{"terrain,buildings?radius=250", true, true, 250.0},
          Case{"terrain?", false, true, 500.0}}) {
        const MapboxSpecResult r = parse_mapbox_spec(c.spec);
        ASSERT_TRUE(r.spec) << c.spec << ": " << r.error;
        EXPECT_EQ(r.spec->buildings, c.buildings) << c.spec;
        EXPECT_EQ(r.spec->terrain, c.terrain) << c.spec;
        EXPECT_FALSE(r.spec->imagery_satellite) << c.spec << ": imagery defaults to off";
        EXPECT_EQ(r.spec->radius_m, c.radius_m) << c.spec << ": the list ends at the first '?'";
        EXPECT_EQ(r.error, "") << c.spec;
    }
    const MapboxSpecResult key = parse_mapbox_spec("buildings?terrain=on");
    ASSERT_TRUE(key.spec) << key.error;
    EXPECT_FALSE(key.spec->terrain) << "layers come from the list; terrain= is an ion:// key";
}

TEST(MapboxUri, EmptyOrUnknownLayerNamesRefuseQuotingTheName) {
    const char* const cases[][2] = {
        {"", "''"},
        {"?radius=5", "''"},
        {"buildings,", "''"},
        {",terrain", "''"},
        {"buildings,,terrain", "''"},
        {"building", "'building'"},
        {"Buildings", "'Buildings'"},
        {"buildings/", "'buildings/'"},
        {"buildings, terrain", "' terrain'"},
        {"terrain,roads", "'roads'"},
        {"buildings;terrain", "'buildings;terrain'"},
    };
    for (const auto& [spec, name] : cases) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << "'" << spec << "'";
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << "'" << spec << "': " << r.error;
        EXPECT_TRUE(mentions(r.error, "'buildings' and 'terrain'"))
            << "'" << spec << "': " << r.error;
        EXPECT_TRUE(mentions(r.error, std::string("not ") + name))
            << "'" << spec << "': " << r.error;
        EXPECT_FALSE(mentions(r.error, "refused")) << "the loader's log line says refused";
    }
}

TEST(MapboxUri, RepeatedLayerNamesRefuse) {
    const char* const cases[][2] = {
        {"buildings,buildings", "'buildings'"},
        {"terrain,terrain", "'terrain'"},
        {"buildings,terrain,buildings", "'buildings'"},
        {"terrain,buildings,terrain?imagery=satellite", "'terrain'"},
    };
    for (const auto& [spec, name] : cases) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << spec << ": " << r.error;
        // The contract's one text for empty, repeated and unknown names.
        EXPECT_TRUE(mentions(r.error, "'buildings' and 'terrain'")) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, std::string("not ") + name)) << spec << ": " << r.error;
        EXPECT_FALSE(mentions(r.error, "refused")) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, ImagerySatelliteNeedsTheTerrainLayer) {
    for (const char* spec : {"terrain?imagery=satellite", "buildings,terrain?imagery=satellite",
                             "terrain,buildings?materials=original&imagery=satellite"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        ASSERT_TRUE(r.spec) << spec << ": " << r.error;
        EXPECT_TRUE(r.spec->imagery_satellite) << spec;
    }
    for (const char* spec :
         {"buildings?imagery=satellite", "buildings?imagery=off&imagery=satellite"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, "imagery")) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, "'terrain'")) << spec << ": " << r.error;
        EXPECT_FALSE(mentions(r.error, "refused")) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, ImageryOffNeedsNoTerrainAndTheLastValueWins) {
    for (const char* spec :
         {"buildings?imagery=off", "buildings,terrain?imagery=off",
          "terrain?imagery=satellite&imagery=off", "buildings?imagery=satellite&imagery=off"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        ASSERT_TRUE(r.spec) << spec << ": " << r.error;
        EXPECT_FALSE(r.spec->imagery_satellite) << spec;
        EXPECT_EQ(r.warning, "") << spec;
    }
}

TEST(MapboxUri, UnknownImageryValuesRefuseNamingTheKey) {
    for (const char* spec :
         {"terrain?imagery=", "terrain?imagery=bogus", "terrain?imagery=Satellite",
          "terrain?imagery=on", "terrain?imagery=2", "terrain?imagery=bogus&imagery=satellite"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, "imagery")) << spec << ": " << r.error;
        EXPECT_FALSE(mentions(r.error, "refused")) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, IonOnlyAndUnknownKeysAreIgnored) {
    // Values parse_ion_spec would refuse: ignoring a key must not validate it.
    const MapboxSpecResult r = parse_mapbox_spec(
        "buildings?follow_terrain=maybe&ground_bias=x&replaces_ground=2&max_tilt_deg=-1"
        "&max_cache_items=many&zoom=17&=5");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_EQ(r.spec->radius_m, 500.0);
    EXPECT_EQ(r.spec->brightness, 1.0);
    EXPECT_FALSE(r.spec->materials_original);
    EXPECT_EQ(r.spec->fallback_dir, "");
    EXPECT_EQ(r.warning, "");
}

TEST(MapboxUri, LastValueOfARepeatedKeyWins) {
    const MapboxSpecResult r = parse_mapbox_spec(
        "buildings?radius=100&brightness=2&radius=250&materials=original&materials=clay"
        "&fallback=/srv/a&fallback=/srv/b");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_EQ(r.spec->radius_m, 250.0);
    EXPECT_EQ(r.spec->brightness, 2.0);
    EXPECT_FALSE(r.spec->materials_original);
    EXPECT_EQ(r.spec->fallback_dir, "/srv/b");
    EXPECT_FALSE(parse_mapbox_spec("buildings?radius=0&radius=250").spec)
        << "a bad value refuses the URI even when a later one is good";
}

TEST(MapboxUri, PairsWithoutEqualsAreIgnored) {
    const MapboxSpecResult r = parse_mapbox_spec("buildings?radius&materials&brightness=2&&cache");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_EQ(r.spec->radius_m, 500.0);
    EXPECT_FALSE(r.spec->materials_original);
    EXPECT_EQ(r.spec->brightness, 2.0);
    EXPECT_EQ(r.warning, "");
}

TEST(MapboxUri, UnknownMaterialsWarnsAndMeansClay) {
    const MapboxSpecResult r = parse_mapbox_spec("buildings?materials=bogus");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_FALSE(r.spec->materials_original);
    EXPECT_TRUE(mentions(r.warning, "materials")) << r.warning;
    EXPECT_TRUE(mentions(r.warning, "bogus")) << r.warning;
    EXPECT_EQ(r.error, "");
    const MapboxSpecResult last = parse_mapbox_spec("buildings?materials=original&materials=bogus");
    ASSERT_TRUE(last.spec) << last.error;
    EXPECT_FALSE(last.spec->materials_original) << "the unknown value, given last, means clay";
}

TEST(MapboxUri, EmptyFallbackMeansNoFallback) {
    const MapboxSpecResult alone = parse_mapbox_spec("buildings?fallback=");
    ASSERT_TRUE(alone.spec) << alone.error;
    EXPECT_EQ(alone.spec->fallback_dir, "");
    const MapboxSpecResult cleared =
        parse_mapbox_spec("buildings?fallback=/srv/overlume/chunks&fallback=");
    ASSERT_TRUE(cleared.spec) << cleared.error;
    EXPECT_EQ(cleared.spec->fallback_dir, "") << "the empty value, given last, clears it";
}
