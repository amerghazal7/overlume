// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace {

namespace mb = overlume::mapbox;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
// The operating area (spec section 2, D3), with a heading and a non-zero ellipsoidal height so
// neither can hide behind a zero.
constexpr overlume::GeoAnchor kAnchor{25.0803, 55.3910, 0.35, 12.0};

// Spherical Web Mercator written out independently of the library: fractional tile x, y at zoom z.
std::array<double, 2> mercator_tile_xy(double lon_deg, double lat_deg, uint32_t z) {
    const double n = std::ldexp(1.0, static_cast<int>(z));
    return {(lon_deg + 180.0) / 360.0 * n,
            (1.0 - std::asinh(std::tan(lat_deg * kDegToRad)) / kPi) / 2.0 * n};
}

// Its inverse: (u, v) in units of tile t, `extent` per edge.
mb::LonLatDeg mercator_lonlat(const mb::TileId& t, double u, double v, double extent) {
    const double n = std::ldexp(1.0, static_cast<int>(t.z));
    return {(t.x + u / extent) / n * 360.0 - 180.0,
            std::atan(std::sinh(kPi * (1.0 - 2.0 * (t.y + v / extent) / n))) / kDegToRad};
}

mb::TileId anchor_tile() {
    return mb::tile_containing(kAnchor.origin_lon_deg, kAnchor.origin_lat_deg, mb::kLandmarkZoom);
}

}

TEST(MapboxTileFrame, XyzLonLatRoundTripAtBothDemoSites) {
    struct Site {
        const char* name;
        double lon, lat;
        uint32_t x, y;
    };
    // Downtown Dubai's z14 tile is 10707/7006 (spec section 3).
    const Site sites[] = {{"Downtown Dubai", 55.2744, 25.1972, 10707, 7006},
                          {"operating area", 55.3910, 25.0803, 10712, 7012}};
    for (const Site& s : sites) {
        const std::array<double, 2> f = mercator_tile_xy(s.lon, s.lat, mb::kLandmarkZoom);
        EXPECT_EQ(static_cast<uint32_t>(std::floor(f[0])), s.x) << s.name;
        EXPECT_EQ(static_cast<uint32_t>(std::floor(f[1])), s.y) << s.name;

        const mb::TileId t = mb::tile_containing(s.lon, s.lat, mb::kLandmarkZoom);
        EXPECT_EQ(t.z, mb::kLandmarkZoom) << s.name;
        EXPECT_EQ(t.x, s.x) << s.name;
        EXPECT_EQ(t.y, s.y) << s.name;

        const mb::LonLatDeg back =
            mb::tile_point_to_lonlat(t, (f[0] - t.x) * mb::kLandmarkExtent,
                                     (f[1] - t.y) * mb::kLandmarkExtent, mb::kLandmarkExtent);
        EXPECT_NEAR(back.lon, s.lon, 1e-9) << s.name;
        EXPECT_NEAR(back.lat, s.lat, 1e-9) << s.name;

        const mb::LonLatRect b = mb::tile_bounds(t);
        EXPECT_LT(b.west, s.lon) << s.name;
        EXPECT_GT(b.east, s.lon) << s.name;
        EXPECT_LT(b.south, s.lat) << s.name;
        EXPECT_GT(b.north, s.lat) << s.name;
    }
}

TEST(MapboxTileFrame, TileContainingClampsToTheMercatorSquare) {
    const uint32_t last = (1u << mb::kLandmarkZoom) - 1;
    EXPECT_EQ(mb::tile_containing(0.0, 89.9, mb::kLandmarkZoom).y, 0u);
    EXPECT_EQ(mb::tile_containing(0.0, -89.9, mb::kLandmarkZoom).y, last);
    EXPECT_EQ(mb::tile_containing(180.0, 0.0, mb::kLandmarkZoom).x, last);
    EXPECT_EQ(mb::tile_containing(-180.0, 0.0, mb::kLandmarkZoom).x, 0u);
    // Beyond the square (and NaN) the clamp must not cast a negative double to unsigned.
    EXPECT_EQ(mb::tile_containing(-190.0, 0.0, mb::kLandmarkZoom).x, 0u) << "west of the square";
    EXPECT_EQ(mb::tile_containing(std::nan(""), 0.0, mb::kLandmarkZoom).x, 0u) << "NaN";

    const mb::LonLatRect world = mb::tile_bounds({0, 0, 0});
    EXPECT_NEAR(world.west, -180.0, 1e-9);
    EXPECT_NEAR(world.east, 180.0, 1e-9);
    EXPECT_NEAR(world.north, 85.0511287798066, 1e-9);
    EXPECT_NEAR(world.south, -85.0511287798066, 1e-9);
}

TEST(MapboxTileFrame, FootprintTilesAreTheSixteenZ16SquaresRowMajor) {
    const mb::TileId t = anchor_tile();
    const std::array<mb::TileId, 16> sub = mb::footprint_tiles_of(t);
    for (uint32_t j = 0; j < 4; ++j) {
        for (uint32_t i = 0; i < 4; ++i) {
            const mb::TileId& s = sub[j * 4 + i];
            EXPECT_EQ(s.z, mb::kFootprintZoom);
            EXPECT_EQ(s.x, 4 * t.x + i) << "i=" << i << " j=" << j;
            EXPECT_EQ(s.y, 4 * t.y + j) << "i=" << i << " j=" << j;
        }
    }
    const mb::LonLatRect whole = mb::tile_bounds(t);
    const mb::LonLatRect nw = mb::tile_bounds(sub[0]), se = mb::tile_bounds(sub[15]);
    EXPECT_NEAR(nw.west, whole.west, 1e-12);
    EXPECT_NEAR(nw.north, whole.north, 1e-12);
    EXPECT_NEAR(se.east, whole.east, 1e-12);
    EXPECT_NEAR(se.south, whole.south, 1e-12);
}

TEST(MapboxTileFrame, TileCentreIsTheFrameOriginWithEastNorthUpAxes) {
    const mb::TileId t = anchor_tile();
    const mb::TileFrame f = mb::make_tile_frame(t, kAnchor.origin_height_m);
    const auto at = [&](double u, double v, double z_m) {
        return mb::project_to_frame(f, t, u, v, mb::kLandmarkExtent, z_m);
    };
    for (const double c : at(4096.0, 4096.0, 0.0)) {
        EXPECT_NEAR(c, 0.0, 1e-6) << "the frame sits at the tile centre, origin_height_m up";
    }
    const std::array<double, 3> east = at(4196.0, 4096.0, 0.0);
    EXPECT_GT(east[0], 20.0);
    EXPECT_LT(std::abs(east[1]), 1e-3 * east[0]);
    const std::array<double, 3> north = at(4096.0, 3996.0, 0.0);
    EXPECT_GT(north[1], 20.0);
    EXPECT_LT(std::abs(north[0]), 1e-3 * north[1]);
    const std::array<double, 3> up = at(4096.0, 4096.0, 30.0);
    EXPECT_NEAR(up[0], 0.0, 1e-6);
    EXPECT_NEAR(up[1], 0.0, 1e-6);
    EXPECT_NEAR(up[2], 30.0, 1e-6);
}

TEST(MapboxTileFrame, PlacementChainMatchesTheStreamProbes) {
    const mb::TileId t = anchor_tile();
    struct Point {
        mb::TileId src;
        double u, v, extent;
    };
    // Three points of the z14 tile, and one of a z16 square, as the extrusions use them.
    const Point points[] = {{t, 0.0, 0.0, mb::kLandmarkExtent},
                            {t, 4096.0, 4096.0, mb::kLandmarkExtent},
                            {t, 7000.0, 1500.0, mb::kLandmarkExtent},
                            {mb::footprint_tiles_of(t)[5], 1000.0, 3000.0, 4096.0}};
    for (const Point& p : points) {
        for (const double z_m : {0.0, 30.0}) {
            double got[3] = {};
            ASSERT_TRUE(overlume::testing::mapbox_tile_point_to_map_probe(
                kAnchor, p.src.z, p.src.x, p.src.y, p.u, p.v, p.extent, z_m, got));
            const mb::LonLatDeg ll = mercator_lonlat(p.src, p.u, p.v, p.extent);
            const double alt = kAnchor.origin_height_m + z_m;
            double x = 0.0, y = 0.0, zTangent = 0.0, zRaw = 0.0, zCorrected = 0.0;
            ASSERT_TRUE(overlume::testing::ecef_to_map_probe(
                kAnchor.origin_lat_deg, kAnchor.origin_lon_deg, kAnchor.heading_rad,
                kAnchor.origin_height_m, ll.lat, ll.lon, alt, &x, &y, &zTangent));
            ASSERT_TRUE(overlume::testing::ecef_height_correction_probe(
                kAnchor.origin_lat_deg, kAnchor.origin_lon_deg, kAnchor.heading_rad,
                kAnchor.origin_height_m, ll.lat, ll.lon, alt, &zRaw, &zCorrected));
            EXPECT_NEAR(got[0], x, 0.02) << "z" << p.src.z << " u=" << p.u << " v=" << p.v;
            EXPECT_NEAR(got[1], y, 0.02) << "z" << p.src.z << " u=" << p.u << " v=" << p.v;
            EXPECT_NEAR(got[2], zCorrected, 0.02) << "u=" << p.u << " v=" << p.v;
            EXPECT_NEAR(got[2], z_m, 0.02) << "bases land on map z = 0 and roofs at their height";
        }
    }
}

TEST(MapboxTileFrame, LandmarkNodeAffineMapTracksTheExactChain) {
    const mb::TileId t = anchor_tile();
    const mb::TileFrame f = mb::make_tile_frame(t, kAnchor.origin_height_m);
    const double tx = 5000.0, ty = 3000.0;
    const double s =
        mb::mapbox_node_scale(mb::tile_point_to_lonlat(t, tx, ty, mb::kLandmarkExtent).lat);
    const std::optional<mb::Mat4d> a =
        mb::landmark_node_to_frame(f, {s, 0, 0, 0, 0, -s, 0, 0, 0, 0, 1, 0, tx, ty, 0, 1});
    ASSERT_TRUE(a.has_value());
    const mb::Mat4d& m = *a;
    // Heights run along the ellipsoid normal, so the z column is the anchor's normal exactly; here
    // it leans 6e-5 off the frame's z axis.
    const std::array<double, 3> ground =
        mb::project_to_frame(f, t, tx, ty, mb::kLandmarkExtent, 0.0);
    const std::array<double, 3> raised =
        mb::project_to_frame(f, t, tx, ty, mb::kLandmarkExtent, 1.0);
    for (size_t r = 0; r < 3; ++r) {
        EXPECT_NEAR(m[8 + r], raised[r] - ground[r], 1e-6) << "z column, row " << r;
    }
    for (const double dx : {-250.0, 0.0, 250.0}) {
        for (const double dy : {-250.0, 0.0, 250.0}) {
            for (const double dz : {0.0, 300.0}) {
                // Mapbox's matrix: local x east, y north, z up → tile units with v pointing south.
                const std::array<double, 3> exact =
                    mb::project_to_frame(f, t, tx + s * dx, ty - s * dy, mb::kLandmarkExtent, dz);
                const double ex = m[0] * dx + m[4] * dy + m[8] * dz + m[12] - exact[0];
                const double ey = m[1] * dx + m[5] * dy + m[9] * dz + m[13] - exact[1];
                const double ez = m[2] * dx + m[6] * dy + m[10] * dz + m[14] - exact[2];
                // 2.24 cm at worst, at a top corner: an affine map cannot follow the ellipsoid's
                // sag (9.8 mm at 354 m) or its up direction's tilt (1.7 cm at 300 m).
                EXPECT_LT(std::sqrt(ex * ex + ey * ey + ez * ez), 0.025)
                    << "dx=" << dx << " dy=" << dy << " dz=" << dz;
            }
        }
    }
}

TEST(MapboxTileFrame, LandmarkNodeRejectsUnexpectedMatrices) {
    const mb::TileId t = anchor_tile();
    const mb::TileFrame f = mb::make_tile_frame(t, kAnchor.origin_height_m);
    const double tx = 5000.0, ty = 3000.0;
    const double s =
        mb::mapbox_node_scale(mb::tile_point_to_lonlat(t, tx, ty, mb::kLandmarkExtent).lat);
    const mb::Mat4d good = {s, 0, 0, 0, 0, -s, 0, 0, 0, 0, 1, 0, tx, ty, 0, 1};
    EXPECT_TRUE(mb::landmark_node_to_frame(f, good).has_value());

    // 0.1 rad about z: the diagonal stays inside the 1% scale band (cos 0.1 = 0.995), so only the
    // rotation check can reject it.
    mb::Mat4d rotated = good;
    rotated[0] = s * std::cos(0.1);
    rotated[1] = s * std::sin(0.1);
    rotated[4] = s * std::sin(0.1);
    rotated[5] = -s * std::cos(0.1);
    EXPECT_FALSE(mb::landmark_node_to_frame(f, rotated).has_value()) << "rotated";

    mb::Mat4d scaled = good;
    scaled[0] *= 1.05;
    scaled[5] *= 1.05;
    EXPECT_FALSE(mb::landmark_node_to_frame(f, scaled).has_value()) << "scale 5% off";

    mb::Mat4d stretched = good;
    stretched[5] *= 1.001;
    EXPECT_FALSE(mb::landmark_node_to_frame(f, stretched).has_value()) << "x and y scales differ";

    mb::Mat4d lifted = good;
    lifted[14] = 5.0;
    EXPECT_FALSE(mb::landmark_node_to_frame(f, lifted).has_value()) << "z translation";

    mb::Mat4d infinite = good;
    infinite[12] = INFINITY;
    EXPECT_FALSE(mb::landmark_node_to_frame(f, infinite).has_value()) << "non-finite translation";

    // The Web Mercator scale on Mapbox's 6,371,008.8 m mean-radius sphere, not WGS84's 6,378,137 m
    // equatorial radius (0.11% apart).
    const double want = 8192.0 * 16384.0 / (2.0 * kPi * 6371008.8 * std::cos(25.0 * kDegToRad));
    EXPECT_NEAR(mb::mapbox_node_scale(25.0), want, want * 1e-12);
}

TEST(MapboxTileFrame, PlacementProbeTakesZ14ToZ30Only) {
    const mb::TileId t = anchor_tile();
    double got[3] = {};
    EXPECT_FALSE(overlume::testing::mapbox_tile_point_to_map_probe(
        kAnchor, 13, t.x >> 1, t.y >> 1, 0.0, 0.0, mb::kLandmarkExtent, 0.0, got));
    EXPECT_FALSE(overlume::testing::mapbox_tile_point_to_map_probe(kAnchor, 31, 0, 0, 0.0, 0.0,
                                                                   mb::kLandmarkExtent, 0.0, got));
    EXPECT_TRUE(overlume::testing::mapbox_tile_point_to_map_probe(kAnchor, 14, t.x, t.y, 0.0, 0.0,
                                                                  mb::kLandmarkExtent, 0.0, got));
}
