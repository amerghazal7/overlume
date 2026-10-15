// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_mvt.hpp"

#include "mapbox_mvt_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace mvt = overlume::testing::mvt;
using overlume::mapbox::decode_mvt_building_layer;
using overlume::mapbox::MvtBuilding;
using overlume::mapbox::MvtBuildingLayer;
using overlume::mapbox::MvtPoint;
using overlume::mapbox::MvtPolygon;
using Ring = std::vector<std::array<int32_t, 2>>;

MvtBuildingLayer decode(const std::vector<uint8_t>& tile) {
    return decode_mvt_building_layer(tile.data(), tile.size());
}

std::vector<uint8_t> bytes(const std::string& s) { return {s.begin(), s.end()}; }

mvt::Layer building_layer(std::vector<mvt::Feature> features) {
    mvt::Layer layer;
    layer.name = "building";
    layer.features = std::move(features);
    return layer;
}

// Clockwise on screen (y down), so a positive surveyor's area: an exterior ring.
Ring square(int32_t x, int32_t y, int32_t size) {
    return {{x, y}, {x + size, y}, {x + size, y + size}, {x, y + size}};
}

// Wound the other way: a hole.
Ring square_hole(int32_t x, int32_t y, int32_t size) {
    return {{x, y}, {x, y + size}, {x + size, y + size}, {x + size, y}};
}

Ring as_ring(const std::vector<MvtPoint>& points) {
    Ring ring;
    for (const MvtPoint& p : points) ring.push_back({p.x, p.y});
    return ring;
}

// The one building of a tile whose single feature (a square, no id) carries only `key` = `value`.
MvtBuilding with_tag(const std::string& key, mvt::Value value) {
    mvt::Feature f;
    f.rings = {square(0, 0, 10)};
    f.tags = {{key, std::move(value)}};
    const MvtBuildingLayer out = decode(mvt::encode_tile({building_layer({f})}));
    EXPECT_TRUE(out.ok);
    EXPECT_EQ(out.buildings.size(), 1u);
    return out.buildings.empty() ? MvtBuilding{} : out.buildings[0];
}

uint32_t cmd(uint32_t id, uint32_t count) { return mvt::detail::command(id, count); }

uint32_t zz(int32_t v) { return mvt::detail::zigzag32(v); }

// A 10 x 10 exterior ring as raw geometry: MoveTo, LineTo(3), ClosePath.
const std::vector<uint32_t> kSquare10 = {cmd(1, 1), zz(0),  zz(0),   cmd(2, 3), zz(10),   zz(0),
                                         zz(0),     zz(10), zz(-10), zz(0),     cmd(7, 1)};

template <typename T>
std::vector<T> with_tail(std::vector<T> head, const std::vector<T>& tail) {
    head.insert(head.end(), tail.begin(), tail.end());
    return head;
}

// A raw POLYGON Feature message, for tags and geometry the encoder never writes. Tag indices are
// 64-bit, so a test can write one past 32 bits.
std::vector<uint8_t> raw_feature(const std::vector<uint64_t>& tags,
                                 const std::vector<uint32_t>& geometry) {
    std::vector<uint8_t> packed_tags;
    for (const uint64_t t : tags) mvt::detail::put_varint(packed_tags, t);
    std::vector<uint8_t> f;
    mvt::detail::put_bytes(f, 2, packed_tags);
    mvt::detail::put_tag(f, 3, 0);
    mvt::detail::put_varint(f, 3);
    mvt::detail::put_packed(f, 4, geometry);
    return f;
}

// A raw building layer: `features` as given, the one key "height" and the one value `value`.
std::vector<uint8_t> raw_building_layer(
    const std::vector<std::vector<uint8_t>>& features,
    const std::vector<uint8_t>& value = mvt::dbl(10.0).message) {
    std::vector<uint8_t> layer;
    mvt::detail::put_string(layer, 1, "building");
    for (const std::vector<uint8_t>& f : features) mvt::detail::put_bytes(layer, 2, f);
    mvt::detail::put_string(layer, 3, "height");
    mvt::detail::put_bytes(layer, 4, value);
    return layer;
}

std::vector<uint8_t> raw_tile(const std::vector<std::vector<uint8_t>>& layers) {
    std::vector<uint8_t> tile;
    for (const std::vector<uint8_t>& layer : layers) mvt::detail::put_bytes(tile, 3, layer);
    return tile;
}

uint32_t le32(const std::vector<uint8_t>& b, size_t at) {
    return uint32_t{b[at]} | uint32_t{b[at + 1]} << 8 | uint32_t{b[at + 2]} << 16 |
           uint32_t{b[at + 3]} << 24;
}

// Reads a gzip stream of stored deflate blocks back. false unless every part is well formed.
bool read_gzip_stored(const std::vector<uint8_t>& gz, std::vector<uint8_t>& raw, uint32_t& crc,
                      uint32_t& isize) {
    const std::vector<uint8_t> header = {0x1f, 0x8b, 0x08, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0xff};
    if (gz.size() < header.size() || !std::equal(header.begin(), header.end(), gz.begin()))
        return false;
    size_t p = header.size();
    for (bool last = false; !last;) {
        if (p + 5 > gz.size() || (gz[p] & 0xfe) != 0) return false;  // BTYPE 00 only
        last = (gz[p] & 1) != 0;
        const size_t len = gz[p + 1] | gz[p + 2] << 8;
        const size_t nlen = gz[p + 3] | gz[p + 4] << 8;
        if ((len ^ nlen) != 0xffff || p + 5 + len > gz.size()) return false;
        raw.insert(raw.end(), gz.begin() + p + 5, gz.begin() + p + 5 + len);
        p += 5 + len;
    }
    if (p + 8 != gz.size()) return false;
    crc = le32(gz, p);
    isize = le32(gz, p + 4);
    return true;
}

}

TEST(MapboxMvt, SquareBuildingKeepsItsIdRingAndFields) {
    const uint64_t id = (uint64_t{1} << 62) + 7;  // a nine-byte varint
    mvt::Layer layer = building_layer(
        {mvt::building(id, {square(100, 200, 50)}, 35.0, 3.0, true, false, "building:part")});
    layer.extent = 8192;
    const MvtBuildingLayer out = decode(mvt::encode_tile({layer}));
    ASSERT_TRUE(out.ok);
    EXPECT_EQ(out.extent, 8192u);
    EXPECT_EQ(out.skipped_features, 0u);
    ASSERT_EQ(out.buildings.size(), 1u);
    const MvtBuilding& b = out.buildings[0];
    EXPECT_EQ(b.id, std::optional<uint64_t>(id));
    ASSERT_EQ(b.polygons.size(), 1u);
    EXPECT_EQ(as_ring(b.polygons[0].outer), square(100, 200, 50));
    EXPECT_TRUE(b.polygons[0].holes.empty());
    EXPECT_EQ(b.height, std::optional<double>(35.0));
    EXPECT_EQ(b.min_height, std::optional<double>(3.0));
    EXPECT_EQ(b.extrude, std::optional<bool>(true));
    EXPECT_EQ(b.underground, std::optional<bool>(false));
    EXPECT_EQ(b.type, "building:part");
}

TEST(MapboxMvt, CourtyardHoleBelongsToItsExteriorInVersions1And2) {
    for (const uint32_t version : {1u, 2u}) {
        mvt::Layer layer =
            building_layer({mvt::building(7, {square(0, 0, 100), square_hole(30, 30, 40)}, 20.0)});
        layer.version = version;
        const MvtBuildingLayer out = decode(mvt::encode_tile({layer}));
        ASSERT_TRUE(out.ok) << "version " << version;
        ASSERT_EQ(out.buildings.size(), 1u) << "version " << version;
        ASSERT_EQ(out.buildings[0].polygons.size(), 1u) << "version " << version;
        const MvtPolygon& polygon = out.buildings[0].polygons[0];
        EXPECT_EQ(as_ring(polygon.outer), square(0, 0, 100)) << "version " << version;
        ASSERT_EQ(polygon.holes.size(), 1u) << "version " << version;
        EXPECT_EQ(as_ring(polygon.holes[0]), square_hole(30, 30, 40)) << "version " << version;
    }
}

TEST(MapboxMvt, MultipolygonStartsAPolygonAtEachExterior) {
    const MvtBuildingLayer out = decode(mvt::encode_tile({building_layer({mvt::building(
        8, {square(0, 0, 10), square(50, 50, 30), square_hole(60, 60, 10)}, 12.0)})}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u);
    const std::vector<MvtPolygon>& polygons = out.buildings[0].polygons;
    ASSERT_EQ(polygons.size(), 2u);
    EXPECT_EQ(as_ring(polygons[0].outer), square(0, 0, 10));
    EXPECT_TRUE(polygons[0].holes.empty());
    EXPECT_EQ(as_ring(polygons[1].outer), square(50, 50, 30));
    ASSERT_EQ(polygons[1].holes.size(), 1u);
    EXPECT_EQ(as_ring(polygons[1].holes[0]), square_hole(60, 60, 10));
}

TEST(MapboxMvt, HeightFromEveryNumericValueType) {
    EXPECT_EQ(with_tag("height", mvt::dbl(828.5)).height, std::optional<double>(828.5));
    EXPECT_EQ(with_tag("height", mvt::flt(12.25f)).height, std::optional<double>(12.25));
    EXPECT_EQ(with_tag("height", mvt::i64(-3)).height, std::optional<double>(-3.0))
        << "a negative int64 is a ten-byte varint";
    EXPECT_EQ(with_tag("height", mvt::u64(40)).height, std::optional<double>(40.0));
    EXPECT_EQ(with_tag("height", mvt::s64(-7)).height, std::optional<double>(-7.0));
    EXPECT_EQ(with_tag("min_height", mvt::s64(212)).min_height, std::optional<double>(212.0));
    EXPECT_EQ(with_tag("min_height", mvt::flt(0.5f)).min_height, std::optional<double>(0.5));
    // Not a number, or not finite: absent, so the loader skips the feature instead of building NaN.
    EXPECT_FALSE(with_tag("height", mvt::str("15")).height);
    EXPECT_FALSE(with_tag("height", mvt::boolean(true)).height);
    EXPECT_FALSE(with_tag("height", mvt::dbl(std::nan(""))).height);
    EXPECT_FALSE(with_tag("height", mvt::dbl(HUGE_VAL)).height);
    EXPECT_FALSE(with_tag("type", mvt::str("roof")).height);
}

TEST(MapboxMvt, FlagsFromStringsOrBoolsAndTypeFromAString) {
    EXPECT_EQ(with_tag("extrude", mvt::str("true")).extrude, std::optional<bool>(true));
    EXPECT_EQ(with_tag("extrude", mvt::str("false")).extrude, std::optional<bool>(false));
    EXPECT_EQ(with_tag("extrude", mvt::boolean(true)).extrude, std::optional<bool>(true));
    EXPECT_EQ(with_tag("extrude", mvt::boolean(false)).extrude, std::optional<bool>(false));
    EXPECT_EQ(with_tag("underground", mvt::str("true")).underground, std::optional<bool>(true));
    EXPECT_EQ(with_tag("underground", mvt::boolean(false)).underground, std::optional<bool>(false));
    EXPECT_FALSE(with_tag("extrude", mvt::str("yes")).extrude);
    EXPECT_FALSE(with_tag("extrude", mvt::u64(1)).extrude);
    EXPECT_FALSE(with_tag("underground", mvt::str("TRUE")).underground);
    EXPECT_EQ(with_tag("type", mvt::str("roof")).type, "roof");
    EXPECT_EQ(with_tag("type", mvt::u64(3)).type, "");
    EXPECT_FALSE(with_tag("type", mvt::str("roof")).id) << "the feature has no id field";
}

TEST(MapboxMvt, OnlyTheLayerNamedBuildingIsDecoded) {
    mvt::Layer road;
    road.name = "road";
    road.features = {mvt::building(1, {square(0, 0, 10)}, 5.0)};
    mvt::Layer near_miss = road;
    near_miss.name = "buildings";
    const MvtBuildingLayer out = decode(mvt::encode_tile(
        {road, building_layer({mvt::building(2, {square(20, 20, 10)}, 9.0)}), near_miss}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u);
    EXPECT_EQ(out.buildings[0].id, std::optional<uint64_t>(2));
    EXPECT_EQ(out.skipped_features, 0u);
}

TEST(MapboxMvt, TileWithoutABuildingLayerIsOkAndEmpty) {
    mvt::Layer water;
    water.name = "water";
    water.features = {mvt::building(1, {square(0, 0, 4096)}, 0.0)};
    const MvtBuildingLayer sea = decode(mvt::encode_tile({water}));
    EXPECT_TRUE(sea.ok);
    EXPECT_TRUE(sea.buildings.empty());
    EXPECT_EQ(sea.skipped_features, 0u);
    const MvtBuildingLayer empty = decode_mvt_building_layer(nullptr, 0);
    EXPECT_TRUE(empty.ok) << "an empty body is a tile with no layers";
    EXPECT_TRUE(empty.buildings.empty());
}

TEST(MapboxMvt, NonPolygonFeaturesAreSkippedAndCounted) {
    mvt::Feature line = mvt::building(1, {square(0, 0, 10)}, 5.0);
    line.geom_type = 2;  // LINESTRING
    mvt::Feature unknown = mvt::building(2, {square(0, 0, 10)}, 5.0);
    unknown.geom_type = 0;  // UNKNOWN
    const MvtBuildingLayer out = decode(mvt::encode_tile(
        {building_layer({line, unknown, mvt::building(3, {square(20, 20, 10)}, 6.0)})}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u);
    EXPECT_EQ(out.buildings[0].id, std::optional<uint64_t>(3));
    EXPECT_EQ(out.skipped_features, 2u);
}

TEST(MapboxMvt, HoleBeforeAnyExteriorSkipsTheFeature) {
    const MvtBuildingLayer out = decode(mvt::encode_tile(
        {building_layer({mvt::building(1, {square_hole(30, 30, 40), square(0, 0, 100)}, 20.0),
                         mvt::building(2, {square(0, 0, 10)}, 5.0)})}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u);
    EXPECT_EQ(out.buildings[0].id, std::optional<uint64_t>(2));
    EXPECT_EQ(out.skipped_features, 1u);
}

TEST(MapboxMvt, DegenerateRingsAndRepeatedVerticesAreDropped) {
    // The first vertex repeated at the end and (10, 0) written twice: both repeats go.
    const Ring closed = {{0, 0}, {10, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};
    const Ring line = {{0, 0}, {10, 0}, {20, 0}};  // collinear: zero area
    const Ring two = {{5, 5}, {6, 6}};             // fewer than three distinct vertices
    const MvtBuildingLayer out = decode(mvt::encode_tile({building_layer(
        {mvt::building(1, {line, closed, two}, 8.0), mvt::building(2, {two, line}, 8.0)})}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u) << "a feature left without a ring is skipped";
    EXPECT_EQ(out.skipped_features, 1u);
    EXPECT_EQ(out.buildings[0].id, std::optional<uint64_t>(1));
    ASSERT_EQ(out.buildings[0].polygons.size(), 1u);
    EXPECT_EQ(as_ring(out.buildings[0].polygons[0].outer), square(0, 0, 10));
    EXPECT_TRUE(out.buildings[0].polygons[0].holes.empty());
}

TEST(MapboxMvt, MalformedFeaturesAreSkippedWhileTheRestDecode) {
    // Every case but the LineTo count is valid apart from one defect: with the check for that
    // defect removed, it would decode.
    const std::vector<uint8_t> good = raw_feature({0, 0}, kSquare10);
    const std::vector<std::vector<uint8_t>> features = {
        good,
        raw_feature({0, 1}, kSquare10),                  // value index out of range
        raw_feature({1, 0}, kSquare10),                  // key index out of range
        raw_feature({0, uint64_t{1} << 32}, kSquare10),  // a value index past 32 bits
        raw_feature({0}, kSquare10),                     // odd tag count
        // An 11-byte varint id, then a valid feature.
        with_tail({0x08, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00}, good),
        // A valid feature, then an id varint that still continues at its tenth byte.
        with_tail(good, {0x08, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}),
        // A valid feature, then wire type 7.
        with_tail(good, {0x0f}),
        // A valid feature, then a geometry length past the feature's end.
        with_tail(good, {0x22, 0x05, 0x09}),
        // A LineTo count of 2^29 - 1 with one point behind it.
        raw_feature({0, 0}, {cmd(1, 1), zz(0), zz(0), cmd(2, (1u << 29) - 1), zz(1), zz(1)}),
        // LineTo before MoveTo, then a valid ring.
        raw_feature({0, 0}, {cmd(2, 1), zz(1), zz(1), cmd(1, 1), zz(-1), zz(-1), cmd(2, 3), zz(10),
                             zz(0), zz(0), zz(10), zz(-10), zz(0), cmd(7, 1)}),
        // A MoveTo of two points, otherwise a closed square.
        raw_feature({0, 0}, {cmd(1, 2), zz(0), zz(0), zz(10), zz(0), cmd(2, 2), zz(0), zz(10),
                             zz(-10), zz(0), cmd(7, 1)}),
        // A MoveTo while a ring is open, then a valid ring.
        raw_feature({0, 0},
                    with_tail({cmd(1, 1), zz(5), zz(5), cmd(2, 1), zz(1), zz(0)}, kSquare10)),
        // A valid ring, then one that is never closed.
        raw_feature({0, 0}, with_tail(kSquare10, {cmd(1, 1), zz(20), zz(20), cmd(2, 2), zz(9),
                                                  zz(0), zz(0), zz(9)})),
        // ClosePath without a ring, then a valid ring.
        raw_feature({0, 0}, with_tail({cmd(7, 1)}, kSquare10)),
        // A ClosePath count of 2 on an otherwise valid square.
        raw_feature({0, 0}, {cmd(1, 1), zz(0), zz(0), cmd(2, 3), zz(10), zz(0), zz(0), zz(10),
                             zz(-10), zz(0), cmd(7, 2)}),
        // A square whose LineTo has the unknown command id 4.
        raw_feature({0, 0}, {cmd(1, 1), zz(0), zz(0), cmd(4, 3), zz(10), zz(0), zz(0), zz(10),
                             zz(-10), zz(0), cmd(7, 1)}),
        // The cursor leaves int32; wrapped, the ring would be a positive exterior.
        raw_feature({0, 0}, {cmd(1, 1), zz(2147483647), zz(0), cmd(2, 2), zz(1), zz(0), zz(0),
                             zz(-1), cmd(7, 1)}),
        good,
    };
    const MvtBuildingLayer out = decode(raw_tile({raw_building_layer(features)}));
    ASSERT_TRUE(out.ok) << "feature errors never fail the tile";
    ASSERT_EQ(out.buildings.size(), 2u);
    EXPECT_EQ(out.skipped_features, features.size() - 2);
    for (const MvtBuilding& b : out.buildings) EXPECT_EQ(b.height, std::optional<double>(10.0));
}

TEST(MapboxMvt, BrokenFramingFailsTheTile) {
    const std::vector<uint8_t> good = raw_feature({0, 0}, kSquare10);
    // Wire types 3 (a group start) and 7, each followed by 0x00, which a length-delimited read
    // would take as an empty payload: only the wire type is wrong.
    const std::vector<uint8_t> group = with_tail(raw_building_layer({good}), {0x0b, 0x00});
    const std::vector<uint8_t> bad_value = raw_building_layer({good}, {0x0f, 0x00});
    std::vector<uint8_t> unnamed = {0x0f, 0x00};  // before the layer's name
    mvt::detail::put_string(unnamed, 1, "road");
    std::vector<uint8_t> road;
    mvt::detail::put_string(road, 1, "road");
    road.push_back(0x0f);  // wire type 7 after the name: never read
    // A valid building layer, then broken tile framing.
    const std::vector<uint8_t> trailing = with_tail(raw_tile({raw_building_layer({good})}), {0x0f});

    EXPECT_TRUE(decode(raw_tile({raw_building_layer({good})})).ok);
    for (const std::vector<uint8_t>& tile :
         {raw_tile({group}), raw_tile({bad_value}), raw_tile({unnamed, raw_building_layer({good})}),
          bytes("<!DOCTYPE html><html><body>Service unavailable</body></html>"),
          bytes("{\"message\":\"error\"}"), raw_tile({raw_building_layer({good}), unnamed}),
          raw_tile({raw_building_layer({good}), group}), trailing}) {
        const MvtBuildingLayer out = decode(tile);
        EXPECT_FALSE(out.ok) << tile.size() << " bytes";
        EXPECT_TRUE(out.buildings.empty()) << "nothing in a malformed tile is usable";
        EXPECT_EQ(out.skipped_features, 0u) << tile.size() << " bytes";
    }
    const MvtBuildingLayer skipped = decode(raw_tile({road, raw_building_layer({good})}));
    EXPECT_TRUE(skipped.ok) << "other layers are skipped by length";
    EXPECT_EQ(skipped.buildings.size(), 1u);
}

TEST(MapboxMvt, ZeroExtentFailsTheTile) {
    mvt::Layer layer = building_layer({mvt::building(1, {square(0, 0, 10)}, 5.0)});
    layer.extent = 0;
    const MvtBuildingLayer out = decode(mvt::encode_tile({layer}));
    EXPECT_FALSE(out.ok) << "tile units are divided by the extent";
    EXPECT_TRUE(out.buildings.empty());
    std::vector<uint8_t> wide = raw_building_layer({raw_feature({0, 0}, kSquare10)});
    mvt::detail::put_tag(wide, 5, 0);
    mvt::detail::put_varint(wide, uint64_t{1} << 32);
    EXPECT_FALSE(decode(raw_tile({wide})).ok) << "an extent of 2^32 would wrap to 0";
}

TEST(MapboxMvt, Standard3dFilterNeedsExtrudeAndNotUnderground) {
    struct Row {
        std::optional<bool> extrude, underground;
        bool passes;
    };
    const Row rows[] = {
        {true, false, true},          {true, std::nullopt, false},
        {true, true, false},          {false, false, false},
        {false, std::nullopt, false}, {false, true, false},
        {std::nullopt, false, false}, {std::nullopt, std::nullopt, false},
        {std::nullopt, true, false},
    };
    const auto text = [](const std::optional<bool>& v) {
        return v ? (*v ? "true" : "false") : "missing";
    };
    for (const Row& row : rows) {
        MvtBuilding b;
        b.extrude = row.extrude;
        b.underground = row.underground;
        EXPECT_EQ(overlume::mapbox::passes_standard_3d_filter(b), row.passes)
            << "extrude " << text(row.extrude) << ", underground " << text(row.underground);
    }
}

TEST(MapboxMvt, EveryPrefixOfAValidTileDecodesSafely) {
    mvt::Layer water;
    water.name = "water";
    water.features = {mvt::building(1, {square(0, 0, 4096)}, 0.0)};
    const mvt::Layer buildings =
        building_layer({mvt::building(2, {square(0, 0, 100), square_hole(30, 30, 40)}, 20.0),
                        mvt::building(3, {square(200, 200, 10), square(300, 300, 10)}, 30.0, 5.0),
                        mvt::building(4, {square(-33, -33, 50)}, 9.0, 0.0, false)});
    const std::vector<uint8_t> tile = mvt::encode_tile({water, buildings});
    const size_t water_end = mvt::encode_tile({water}).size();
    ASSERT_EQ(decode(tile).buildings.size(), 3u);
    // Each prefix is a view into the full buffer: a reader that ran past `size` would find the
    // real bytes behind the cut and decode buildings from them.
    for (size_t n = 0; n < tile.size(); ++n) {
        const MvtBuildingLayer out = decode_mvt_building_layer(tile.data(), n);
        EXPECT_EQ(out.ok, n == 0 || n == water_end) << "prefix of " << n << " bytes";
        EXPECT_TRUE(out.buildings.empty()) << "prefix of " << n << " bytes";
        EXPECT_EQ(out.skipped_features, 0u) << "prefix of " << n << " bytes";
    }
}

TEST(MapboxMvt, RandomAndMutatedBuffersDecodeSafely) {
    // No sanitizer runs in the gate: this proves no crash and the output invariants on every input.
    const std::vector<uint8_t> valid = mvt::encode_tile({building_layer(
        {mvt::building(2, {square(0, 0, 100), square_hole(30, 30, 40)}, 20.0),
         mvt::building(3, {square(200, 200, 10), square(300, 300, 10)}, 30.0, 5.0)})});
    uint32_t seed = 20261014u;
    const auto next = [&seed] {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    size_t buildings_seen = 0;
    for (int i = 0; i < 2000; ++i) {
        std::vector<uint8_t> buf;
        if (i % 2 == 0) {  // 0..512 random bytes
            buf.resize(next() % 513);
            for (uint8_t& byte : buf) byte = static_cast<uint8_t>(next());
        } else {  // the valid tile with one to four bytes overwritten
            buf = valid;
            for (uint32_t k = 1 + next() % 4; k > 0; --k)
                buf[next() % buf.size()] = static_cast<uint8_t>(next());
        }
        const MvtBuildingLayer out = decode(buf);
        if (!out.ok) {
            EXPECT_TRUE(out.buildings.empty()) << "buffer " << i;
            EXPECT_EQ(out.skipped_features, 0u) << "buffer " << i;
        }
        EXPECT_GT(out.extent, 0u) << "buffer " << i;
        for (const MvtBuilding& b : out.buildings) {
            ++buildings_seen;
            EXPECT_FALSE(b.polygons.empty()) << "buffer " << i;
            for (const MvtPolygon& polygon : b.polygons) {
                EXPECT_GE(polygon.outer.size(), 3u) << "buffer " << i;
                for (const auto& hole : polygon.holes) EXPECT_GE(hole.size(), 3u) << "buffer " << i;
            }
            if (b.height) EXPECT_TRUE(std::isfinite(*b.height)) << "buffer " << i;
        }
    }
    EXPECT_GT(buildings_seen, 0u) << "the mutations must reach the feature decoder";
}

TEST(MapboxMvt, GzipStoredWritesAValidStoredBlockStream) {
    const std::vector<uint8_t> nine = bytes("123456789");
    const std::vector<uint8_t> gz = mvt::gzip_stored(nine);
    ASSERT_GE(gz.size(), 3u);
    EXPECT_EQ(gz[0], 0x1f);
    EXPECT_EQ(gz[1], 0x8b);
    EXPECT_EQ(gz[2], 0x08);
    std::vector<uint8_t> raw;
    uint32_t crc = 0, isize = 0;
    ASSERT_TRUE(read_gzip_stored(gz, raw, crc, isize));
    EXPECT_EQ(raw, nine);
    EXPECT_EQ(crc, 0xcbf43926u) << "CRC-32's published check value for \"123456789\"";
    EXPECT_EQ(isize, 9u);

    std::vector<uint8_t> big(70000);  // two blocks: 65,535 + 4,465 bytes
    for (size_t i = 0; i < big.size(); ++i) big[i] = static_cast<uint8_t>(i * 7 + 3);
    raw.clear();
    ASSERT_TRUE(read_gzip_stored(mvt::gzip_stored(big), raw, crc, isize));
    EXPECT_EQ(raw, big);
    EXPECT_EQ(isize, 70000u);

    raw.clear();
    ASSERT_TRUE(read_gzip_stored(mvt::gzip_stored({}), raw, crc, isize));
    EXPECT_TRUE(raw.empty());
    EXPECT_EQ(crc, 0u);
    EXPECT_EQ(isize, 0u);
}
