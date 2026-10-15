// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_glb_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace glb = overlume::testing::glb;
namespace mb = overlume::mapbox;
using overlume::testing::MapboxLandmarkProbe;

// The test site's z14 tile (test_mapbox_tile_frame.cpp pins it), anchored 12 m up.
constexpr uint32_t kX14 = 10707, kY14 = 7006;
constexpr double kOriginHeightM = 12.0;
// One fixture building after reading: a box of 24 vertices (per-face normals) and 36 u16
// indices, with POSITION, NORMAL and TEXCOORD_0 as floats (12 + 12 + 8 bytes per vertex).
constexpr uint64_t kBuildingBytes = 24 * (12 + 12 + 8) + 36 * 2;

glb::LandmarkSpec landmark(uint64_t id, double tx, double ty) {
    glb::LandmarkSpec l;
    l.id = id;
    l.tx = tx;
    l.ty = ty;
    l.lat_deg =
        mb::tile_point_to_lonlat({mb::kLandmarkZoom, kX14, kY14}, tx, ty, mb::kLandmarkExtent).lat;
    return l;
}

// Ids above 2^63, as Mapbox's are: they only survive as strings.
std::vector<glb::LandmarkSpec> two_landmarks() {
    return {landmark(18000000000000000001ull, 3000.0, 3000.0),
            landmark(18000000000000000002ull, 5200.0, 4800.0)};
}

MapboxLandmarkProbe probe(const std::vector<uint8_t>& bytes, bool keep_materials) {
    return overlume::testing::probe_mapbox_landmark_tile(bytes, kX14, kY14, kOriginHeightM,
                                                         keep_materials);
}

uint32_t u32_at(const std::vector<uint8_t>& b, size_t at) {
    return static_cast<uint32_t>(b[at]) | static_cast<uint32_t>(b[at + 1]) << 8 |
           static_cast<uint32_t>(b[at + 2]) << 16 | static_cast<uint32_t>(b[at + 3]) << 24;
}

// GLB layout: a 12-byte header, the JSON chunk (8-byte header + data), the BIN chunk.
size_t bin_chunk_header_at(const std::vector<uint8_t>& b) { return 20 + u32_at(b, 12); }

// The fixture with its JSON chunk edited (the fixture writes compact JSON, so substrings are
// exact) and the GLB lengths rewritten.
std::vector<uint8_t> with_json_edits(
    const std::vector<uint8_t>& b, const std::vector<std::pair<std::string, std::string>>& edits) {
    std::string json(b.begin() + 20, b.begin() + bin_chunk_header_at(b));
    for (const auto& [from, to] : edits) {
        const size_t at = json.find(from);
        EXPECT_NE(at, std::string::npos) << from;
        if (at != std::string::npos) json.replace(at, from.size(), to);
    }
    while (json.size() % 4 != 0) json.push_back(' ');
    const std::vector<uint8_t> bin(b.begin() + bin_chunk_header_at(b), b.end());
    std::vector<uint8_t> out;
    const auto put = [&out](size_t v) {
        for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put(0x46546C67);  // "glTF"
    put(2);
    put(12 + 8 + json.size() + bin.size());
    put(json.size());
    put(0x4E4F534A);  // "JSON"
    out.insert(out.end(), json.begin(), json.end());
    out.insert(out.end(), bin.begin(), bin.end());
    return out;
}

// The fixture with EXT_meshopt_compression renamed KHR_meshopt_compression throughout, which
// cesium 0.64 does not decode. The names are the same length, so the GLB keeps its layout.
std::vector<uint8_t> with_khr_meshopt(std::vector<uint8_t> b) {
    const std::string from = "EXT_meshopt_compression", to = "KHR_meshopt_compression";
    std::string json(b.begin() + 20, b.begin() + bin_chunk_header_at(b));
    for (size_t at = json.find(from); at != std::string::npos; at = json.find(from, at)) {
        json.replace(at, from.size(), to);
    }
    std::copy(json.begin(), json.end(), b.begin() + 20);
    return b;
}

// Every view in buffer 0 as written: no meshopt, no quantization.
glb::GlbOptions plain_options() {
    glb::GlbOptions opts;
    opts.meshopt = false;
    opts.quantize = false;
    return opts;
}

}

TEST(MapboxLandmarks, FixtureVertexStreamsUseCodecV0) {
    const std::vector<uint8_t> bytes = glb::build_landmark_glb(two_landmarks(), {});
    // The BIN chunk opens with building 0's POSITION stream.
    const size_t first = bin_chunk_header_at(bytes) + 8;
    ASSERT_GT(bytes.size(), first);
    EXPECT_EQ(static_cast<int>(bytes[first]), 0xa0)
        << "vertex codec v0, the version EXT_meshopt_compression specifies";
}

TEST(MapboxLandmarks, ClayKeepsOnlyTheDefaultSceneBuildings) {
    const MapboxLandmarkProbe p = probe(glb::build_landmark_glb(two_landmarks(), {}), false);
    ASSERT_TRUE(p.ok) << p.error;
    EXPECT_EQ(p.buildings_kept, 2u);
    EXPECT_EQ(p.nodes_skipped, 0u);
    EXPECT_EQ(p.scenes, 1u) << "the LOD scene goes";
    EXPECT_EQ(p.nodes, 2u) << "footprint and LOD nodes go";
    EXPECT_EQ(p.meshes, 2u);
    EXPECT_EQ(p.materials, 2u) << "materials stay; clay replaces them at upload";
    EXPECT_EQ(p.images, 0u);
    EXPECT_EQ(p.textures, 0u);
    EXPECT_EQ(p.samplers, 0u);
    // Per building: POSITION, NORMAL, TEXCOORD_0 and indices, each in its own view and buffer.
    EXPECT_EQ(p.accessors, 8u);
    EXPECT_EQ(p.buffer_views, 8u);
    EXPECT_EQ(p.buffers, 8u);
    EXPECT_EQ(p.footprint_triangles, 4u);
    EXPECT_FALSE(p.meshopt_extension_left);
    EXPECT_EQ(p.buffer_bytes, 2 * kBuildingBytes)
        << "only the buildings' decoded streams remain: no compressed source, fallback, "
           "quantized, footprint, LOD or image bytes";
}

TEST(MapboxLandmarks, OriginalKeepsTheOcclusionTextureAndCompactsBuffers) {
    const std::vector<uint8_t> bytes = glb::build_landmark_glb(two_landmarks(), {});
    const MapboxLandmarkProbe clay = probe(bytes, false);
    const MapboxLandmarkProbe original = probe(bytes, true);
    ASSERT_TRUE(original.ok) << original.error;
    EXPECT_EQ(original.buildings_kept, 2u);
    EXPECT_EQ(original.images, 1u);
    EXPECT_EQ(original.textures, 1u);
    EXPECT_EQ(original.samplers, 1u);
    EXPECT_EQ(original.materials, 2u);
    EXPECT_EQ(original.buffer_views, clay.buffer_views + 1) << "plus the JPEG's view";
    EXPECT_EQ(original.buffers, clay.buffers + 1) << "plus buffer 0, cut down to the JPEG";
    const uint64_t buffer0 = original.buffer_bytes - clay.buffer_bytes;
    const uint64_t jpeg = glb::detail::occlusion_jpeg().size();
    EXPECT_GE(buffer0, jpeg) << "buffer 0 keeps the JPEG";
    EXPECT_LE(buffer0, jpeg + 7)
        << "and is cut down to it: compactBuffers shrinks the gap ahead in multiples of 8 bytes";
}

TEST(MapboxLandmarks, KeptNodesPlaceTheirAnchorInTheTileFrame) {
    const std::vector<glb::LandmarkSpec> specs = two_landmarks();
    const MapboxLandmarkProbe p = probe(glb::build_landmark_glb(specs, {}), false);
    ASSERT_TRUE(p.ok) << p.error;
    ASSERT_EQ(p.node_matrices.size(), specs.size());
    const mb::TileId t{mb::kLandmarkZoom, kX14, kY14};
    const mb::TileFrame f = mb::make_tile_frame(t, kOriginHeightM);
    for (size_t i = 0; i < specs.size(); ++i) {
        const std::array<double, 16>& m = p.node_matrices[i];
        const std::array<double, 3> want =
            mb::project_to_frame(f, t, specs[i].tx, specs[i].ty, mb::kLandmarkExtent, 0.0);
        EXPECT_NEAR(m[12], want[0], 0.01) << "node " << i;
        EXPECT_NEAR(m[13], want[1], 0.01) << "node " << i;
        EXPECT_NEAR(m[14], want[2], 0.01) << "node " << i;
        EXPECT_EQ(m[15], 1.0) << "node " << i;
    }
}

TEST(MapboxLandmarks, NodeWithUnexpectedMatrixIsSkipped) {
    glb::GlbOptions opts;
    opts.bad_matrix_on_first = true;
    const MapboxLandmarkProbe p = probe(glb::build_landmark_glb(two_landmarks(), opts), false);
    ASSERT_TRUE(p.ok) << p.error;
    EXPECT_EQ(p.nodes_skipped, 1u);
    EXPECT_EQ(p.buildings_kept, 1u);
    EXPECT_EQ(p.nodes, 1u);
    EXPECT_EQ(p.meshes, 1u);
    EXPECT_EQ(p.footprint_triangles, 4u) << "the footprints keep their own, valid matrices";
}

TEST(MapboxLandmarks, CorruptMeshoptStreamFailsTheWholeTile) {
    glb::GlbOptions opts;
    opts.corrupt_meshopt = true;
    const MapboxLandmarkProbe p = probe(glb::build_landmark_glb(two_landmarks(), opts), false);
    EXPECT_FALSE(p.ok);
    EXPECT_NE(p.error.find("meshopt"), std::string::npos) << p.error;
    EXPECT_EQ(p.nodes, 0u) << "no half-decoded landmark reaches the renderer";
    EXPECT_EQ(p.meshes, 0u);
    EXPECT_EQ(p.buffers, 0u);
    EXPECT_EQ(p.footprint_triangles, 0u);
}

// Spec section 12: a switch to KHR_meshopt_compression shows as zero landmarks, not a crash.
TEST(MapboxLandmarks, KhrMeshoptCompressionFailsCleanly) {
    const MapboxLandmarkProbe p =
        probe(with_khr_meshopt(glb::build_landmark_glb(two_landmarks(), {})), false);
    EXPECT_FALSE(p.ok);
    EXPECT_NE(p.error.find("KHR_meshopt_compression"), std::string::npos) << p.error;
    EXPECT_EQ(p.nodes, 0u);
    EXPECT_EQ(p.buffers, 0u);
    EXPECT_EQ(p.footprint_triangles, 0u);
}

// Inputs that pass the reader and the extension checks, then fail the pruned model's checks.
TEST(MapboxLandmarks, InconsistentKeptModelFailsCleanly) {
    struct Case {
        std::vector<uint8_t> glb;
        bool keep_materials;
        const char* what;
    };
    const Case cases[] = {
        {with_json_edits(with_khr_meshopt(glb::build_landmark_glb(two_landmarks(), {})),
                         {{"\"extensionsRequired\":[\"KHR_meshopt_compression\",",
                           "\"extensionsRequired\":["}}),
         false, "KHR_meshopt_compression used but not required: views in the empty fallback"},
        {with_json_edits(
             glb::build_landmark_glb(two_landmarks(), plain_options()),
             {{"\"byteOffset\":0,\"byteLength\":", "\"byteOffset\":0,\"byteLength\":9"}}),
         false, "a bufferView running past the end of buffer 0"},
        {with_json_edits(
             glb::build_landmark_glb(two_landmarks(), {}),
             {{"\"occlusionTexture\":{\"index\":0", "\"occlusionTexture\":{\"index\":1"}}),
         true, "a material naming texture 1 of 1"},
    };
    for (const Case& c : cases) {
        const MapboxLandmarkProbe p = probe(c.glb, c.keep_materials);
        EXPECT_FALSE(p.ok) << c.what;
        EXPECT_FALSE(p.error.empty()) << c.what;
        EXPECT_EQ(p.nodes, 0u) << c.what;
        EXPECT_EQ(p.footprint_triangles, 0u) << c.what;
    }
}

TEST(MapboxLandmarks, NonGlbAndTruncatedInputFailCleanly) {
    const std::vector<uint8_t> good = glb::build_landmark_glb(two_landmarks(), {});
    const std::string html = "<html><body>502 Bad Gateway</body></html>";
    const std::string json = "{\"message\":\"not a tile\"}";
    std::vector<uint8_t> version3 = good;  // a GLB header with an unknown container version
    version3[4] = 3;
    const std::vector<std::vector<uint8_t>> inputs = {
        {},
        std::vector<uint8_t>(html.begin(), html.end()),
        std::vector<uint8_t>(json.begin(), json.end()),
        version3,
        std::vector<uint8_t>(good.begin(), good.begin() + 30),
        std::vector<uint8_t>(good.begin(), good.begin() + good.size() / 2),
    };
    for (const std::vector<uint8_t>& input : inputs) {
        const MapboxLandmarkProbe p = probe(input, false);
        EXPECT_FALSE(p.ok) << input.size() << " bytes";
        EXPECT_FALSE(p.error.empty()) << input.size() << " bytes";
        EXPECT_EQ(p.nodes, 0u) << input.size() << " bytes";
    }

    // Buffer 0 claiming more bytes than the BIN chunk holds: cesium returns the model and an
    // error, and the error alone fails the tile.
    const MapboxLandmarkProbe inflated = probe(
        with_json_edits(glb::build_landmark_glb(two_landmarks(), plain_options()),
                        {{"\"buffers\":[{\"byteLength\":", "\"buffers\":[{\"byteLength\":9"}}),
        false);
    EXPECT_FALSE(inflated.ok);
    EXPECT_NE(inflated.error.find("binary chunk"), std::string::npos) << inflated.error;
    EXPECT_EQ(inflated.nodes, 0u);
}

TEST(MapboxLandmarks, SceneIsDefaultSceneThenSceneThenFirst) {
    const std::vector<uint8_t> base = glb::build_landmark_glb(two_landmarks(), {});
    // Scene 0 holds the buildings and their footprints; scene 1 the LOD nodes, no footprints.
    const MapboxLandmarkProbe unnamed = probe(
        with_json_edits(base, {{"\"name\":\"Default Scene\",", ""}, {"\"name\":\"LOD\",", ""}}),
        false);
    ASSERT_TRUE(unnamed.ok) << unnamed.error;
    EXPECT_EQ(unnamed.buildings_kept, 2u) << "no names, no scene: scene 0";
    EXPECT_EQ(unnamed.footprint_triangles, 4u) << "no names, no scene: scene 0";

    const MapboxLandmarkProbe renamed =
        probe(with_json_edits(base, {{"\"name\":\"Default Scene\"", "\"name\":\"Other\""},
                                     {"\"name\":\"LOD\"", "\"name\":\"Default Scene\""}}),
              false);
    ASSERT_TRUE(renamed.ok) << renamed.error;
    EXPECT_EQ(renamed.buildings_kept, 2u) << "\"Default Scene\" wins at index 1";
    EXPECT_EQ(renamed.footprint_triangles, 0u) << "\"Default Scene\" wins at index 1";

    const MapboxLandmarkProbe pointed =
        probe(with_json_edits(base, {{"\"name\":\"Default Scene\",", ""},
                                     {"\"name\":\"LOD\",", ""},
                                     {"{\"asset\"", "{\"scene\":1,\"asset\""}}),
              false);
    ASSERT_TRUE(pointed.ok) << pointed.error;
    EXPECT_EQ(pointed.buildings_kept, 2u) << "no names: the top-level scene, 1";
    EXPECT_EQ(pointed.footprint_triangles, 0u) << "no names: the top-level scene, 1";
}

TEST(MapboxLandmarks, UncompressedUnquantizedVariantReadsTheSame) {
    const MapboxLandmarkProbe a = probe(glb::build_landmark_glb(two_landmarks(), {}), false);
    const MapboxLandmarkProbe b =
        probe(glb::build_landmark_glb(two_landmarks(), plain_options()), false);
    ASSERT_TRUE(a.ok) << a.error;
    ASSERT_TRUE(b.ok) << b.error;
    EXPECT_EQ(b.buildings_kept, a.buildings_kept);
    EXPECT_EQ(b.scenes, a.scenes);
    EXPECT_EQ(b.nodes, a.nodes);
    EXPECT_EQ(b.meshes, a.meshes);
    EXPECT_EQ(b.accessors, a.accessors);
    EXPECT_EQ(b.materials, a.materials);
    EXPECT_EQ(b.images, a.images);
    EXPECT_EQ(b.footprint_triangles, a.footprint_triangles);
    EXPECT_FALSE(b.meshopt_extension_left);
    ASSERT_EQ(b.node_matrices.size(), a.node_matrices.size());
    for (size_t i = 0; i < a.node_matrices.size(); ++i) {
        for (size_t k = 0; k < 16; ++k) {
            EXPECT_NEAR(b.node_matrices[i][k], a.node_matrices[i][k], 1e-9) << i << "," << k;
        }
    }
}

TEST(MapboxLandmarks, InsideAnyFootprintTruthTable) {
    using Triangles = std::vector<std::array<double, 6>>;
    const std::array<double, 6> ccw{0, 0, 10, 0, 0, 10};
    const std::array<double, 6> cw{0, 0, 0, 10, 10, 0};
    const std::array<double, 6> distant{100, 100, 110, 100, 100, 110};
    const std::array<double, 6> point{5, 5, 5, 5, 5, 5};
    struct Case {
        Triangles triangles;
        double x, y;
        bool inside;
        const char* what;
    };
    const Case cases[] = {
        {{ccw}, 2, 2, true, "inside a counter-clockwise triangle"},
        {{cw}, 2, 2, true, "inside a clockwise triangle"},
        {{ccw}, 6, 6, false, "beyond the hypotenuse"},
        {{ccw}, -1, 2, false, "west of the triangle"},
        {{ccw}, 5, 0, true, "on an edge"},
        {{ccw}, 5, 5, true, "on the edge two triangles of a square share"},
        {{ccw}, 0, 0, true, "on a corner"},
        {{ccw, distant}, 102, 102, true, "inside the second triangle"},
        {{ccw, distant}, 50, 50, false, "between the triangles"},
        {{point}, 5, 5, false, "a degenerate triangle covers nothing"},
        {{point}, 70, -3, false, "a degenerate triangle covers nothing"},
        {{}, 0, 0, false, "no footprints"},
    };
    for (const Case& c : cases) {
        EXPECT_EQ(overlume::testing::mapbox_inside_any_footprint_probe(c.triangles, c.x, c.y),
                  c.inside)
            << c.what;
    }
}
