// test_gltf_normals.cpp — overlume::ensure_flat_normals() (gltf_normals.hpp), the
// load-time fix for environment geometry with no vertex normals. No
// Filament/gtfio types here (same tests/*.cpp boundary as every other file
// in this directory -- see environment_test_hooks.hpp) -- these check the
// byte/JSON level directly against the real committed fixtures.
#include "gltf_normals.hpp"

#include "test_paths.hpp"

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

std::vector<uint8_t> read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    EXPECT_TRUE(f) << "missing fixture: " << path;
    const std::streamsize size = f.tellg();
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    f.seekg(0);
    f.read(reinterpret_cast<char*>(bytes.data()), size);
    return bytes;
}

uint32_t read_u32(const std::vector<uint8_t>& b, size_t off) {
    uint32_t v = 0;
    std::memcpy(&v, b.data() + off, 4);
    return v;
}

// b3dm envelope (28-byte header) -> the embedded .glb payload, same layout
// StreamingEnvironmentSource's real tiles arrive in.
std::vector<uint8_t> glb_from_b3dm(const std::vector<uint8_t>& b3dm) {
    const uint32_t ftJson = read_u32(b3dm, 12);
    const uint32_t ftBin = read_u32(b3dm, 16);
    const uint32_t btJson = read_u32(b3dm, 20);
    const uint32_t btBin = read_u32(b3dm, 24);
    const size_t glbOffset = 28 + ftJson + ftBin + btJson + btBin;
    return std::vector<uint8_t>(b3dm.begin() + static_cast<long>(glbOffset), b3dm.end());
}

// Splits a .glb blob's JSON chunk out (independent of gltf_normals.cpp's
// own parser -- duplicated here deliberately so a bug in one doesn't hide
// behind the other).
YAML::Node parse_glb_json(const std::vector<uint8_t>& glb) {
    size_t off = 12;
    while (off + 8 <= glb.size()) {
        const uint32_t len = read_u32(glb, off);
        const uint32_t type = read_u32(glb, off + 4);
        if (type == 0x4E4F534A) {  // "JSON"
            std::string text(reinterpret_cast<const char*>(glb.data() + off + 8), len);
            return YAML::Load(text);
        }
        off += 8 + len;
    }
    return {};
}

int count_primitives_without_normal(const YAML::Node& gltf) {
    int missing = 0;
    for (const YAML::Node& mesh : gltf["meshes"]) {
        for (const YAML::Node& prim : mesh["primitives"]) {
            if (!prim["attributes"]["NORMAL"]) ++missing;
        }
    }
    return missing;
}

// Splits a .glb blob's BIN chunk out -- same independent-of-gltf_normals.cpp
// duplication reasoning as parse_glb_json() above.
std::vector<uint8_t> parse_glb_bin(const std::vector<uint8_t>& glb) {
    size_t off = 12;
    while (off + 8 <= glb.size()) {
        const uint32_t len = read_u32(glb, off);
        const uint32_t type = read_u32(glb, off + 4);
        if (type == 0x004E4942) {  // "BIN\0"
            return std::vector<uint8_t>(glb.begin() + static_cast<long>(off + 8),
                                         glb.begin() + static_cast<long>(off + 8 + len));
        }
        off += 8 + len;
    }
    return {};
}

// Reads a VEC3 FLOAT accessor's raw values straight out of `bin`, assuming
// buffer 0 / no stride (true of every accessor ensure_flat_normals() itself
// appends) -- deliberately NOT reusing gltf_normals.cpp's own
// read_float3_accessor() (that would let a bug in one hide behind the
// other, same reasoning as parse_glb_json() above).
std::vector<float> read_vec3_accessor(const YAML::Node& gltf, const std::vector<uint8_t>& bin,
                                       int accessorIdx) {
    const YAML::Node acc = gltf["accessors"][accessorIdx];
    const int bvIdx = acc["bufferView"].as<int>();
    const YAML::Node bv = gltf["bufferViews"][bvIdx];
    const size_t bvOffset = bv["byteOffset"] ? bv["byteOffset"].as<size_t>() : 0;
    const size_t accOffset = acc["byteOffset"] ? acc["byteOffset"].as<size_t>() : 0;
    const size_t count = acc["count"].as<size_t>();
    std::vector<float> out(count * 3);
    std::memcpy(out.data(), bin.data() + bvOffset + accOffset, count * 3 * sizeof(float));
    return out;
}

// Minimal single-chunk GLB (JSON only, no BIN) -- enough to drive
// ensure_flat_normals() through its early-return guards without needing a
// real mesh.
std::vector<uint8_t> build_json_only_glb(const std::string& json) {
    auto pad4 = [](size_t n) { return (4 - (n % 4)) % 4; };
    std::string padded = json;
    padded.append(pad4(padded.size()), ' ');
    const uint32_t jsonLen = static_cast<uint32_t>(padded.size());
    const uint32_t total = 12 + 8 + jsonLen;
    std::vector<uint8_t> out;
    auto put_u32 = [&out](uint32_t v) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
        out.insert(out.end(), p, p + 4);
    };
    put_u32(0x46546C67);  // "glTF"
    put_u32(2);
    put_u32(total);
    put_u32(jsonLen);
    put_u32(0x4E4F534A);  // "JSON"
    out.insert(out.end(), padded.begin(), padded.end());
    return out;
}

const std::string kBakedChunk = std::string(OVERLUME_TEST_DATA_DIR) +
    "/tests/fixtures/environment_test_town_0/chunks/chunk_-1_-1.glb";
const std::string kIonTile = std::string(OVERLUME_TEST_DATA_DIR) +
    "/tests/fixtures/environment_ion_fixture_0/tile_a.b3dm";

}  // namespace

TEST(GltfNormals, AddsNormalToChunkMissingIt) {
    const std::vector<uint8_t> original = read_file(kBakedChunk);
    YAML::Node beforeJson = parse_glb_json(original);
    ASSERT_GT(count_primitives_without_normal(beforeJson), 0)
        << "fixture precondition: chunk_-1_-1.glb is expected to carry POSITION only";

    const std::vector<uint8_t> patched = overlume::ensure_flat_normals(original);
    EXPECT_NE(patched.size(), original.size()) << "normals should have been appended";

    YAML::Node afterJson = parse_glb_json(patched);
    EXPECT_EQ(count_primitives_without_normal(afterJson), 0);

    // Finding #12: metadata-only checks below (componentType/type/count)
    // would stay green even if compute_flat_normals() returned its
    // degenerate (0,0,1) fallback for every vertex -- read the actual
    // appended floats back out of the patched GLB and assert they're real
    // computed normals, not the fallback.
    const std::vector<uint8_t> patchedBin = parse_glb_bin(patched);
    bool anyNonDegenerate = false;
    for (const YAML::Node& mesh : afterJson["meshes"]) {
        for (const YAML::Node& prim : mesh["primitives"]) {
            const int normalIdx = prim["attributes"]["NORMAL"].as<int>();
            const int posIdx = prim["attributes"]["POSITION"].as<int>();
            const YAML::Node normalAcc = afterJson["accessors"][normalIdx];
            const YAML::Node posAcc = afterJson["accessors"][posIdx];
            EXPECT_EQ(normalAcc["componentType"].as<int>(), 5126);
            EXPECT_EQ(normalAcc["type"].as<std::string>(), "VEC3");
            EXPECT_EQ(normalAcc["count"].as<int>(), posAcc["count"].as<int>());

            const std::vector<float> normals = read_vec3_accessor(afterJson, patchedBin, normalIdx);
            for (size_t v = 0; v + 2 < normals.size(); v += 3) {
                const float len = std::sqrt(normals[v] * normals[v] + normals[v + 1] * normals[v + 1] +
                                             normals[v + 2] * normals[v + 2]);
                EXPECT_NEAR(len, 1.0f, 1e-3f) << "appended normal at vertex " << (v / 3) << " isn't unit length";
                const bool isDegenerateFallback = std::abs(normals[v]) < 1e-6f &&
                                                   std::abs(normals[v + 1]) < 1e-6f &&
                                                   std::abs(normals[v + 2] - 1.0f) < 1e-6f;
                if (!isDegenerateFallback) anyNonDegenerate = true;
            }
        }
    }
    EXPECT_TRUE(anyNonDegenerate) << "every appended normal is the (0,0,1) degenerate fallback -- "
                                     "compute_flat_normals() may not be computing real normals";
}

// Finding #12 (optional synthetic case, cheap): buffers[0].uri set means
// buffer 0 is NOT the embedded BIN chunk (an external/data: URI instead) --
// gltf_normals.cpp's own guard (`if (gltf["buffers"][0]["uri"]) return
// glb_bytes;`) must bail out before ever indexing into `bin` with that
// buffer's accessors, which would otherwise read unrelated/absent bytes and
// synthesize garbage normals. No real mesh data needed: the guard fires
// before any primitive is even inspected.
TEST(GltfNormals, BuffersWithUriAreLeftUnchanged) {
    const std::string json =
        R"({"asset":{"version":"2.0"},)"
        R"("buffers":[{"byteLength":0,"uri":"external.bin"}],)"
        R"("bufferViews":[],"accessors":[],)"
        R"("meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":0,"mode":4}]}]})";
    const std::vector<uint8_t> glb = build_json_only_glb(json);
    const std::vector<uint8_t> patched = overlume::ensure_flat_normals(glb);
    EXPECT_EQ(patched, glb) << "a buffers[0].uri (external/data-URI) source must be left "
                               "byte-for-byte unchanged, not have normals synthesized from "
                               "unrelated/absent BIN bytes";
}

// Running the fix twice must be a no-op the second time -- every primitive
// already has NORMAL after the first pass, so this also proves the "already
// has normals" path is left untouched (bytes identical, not just
// equivalent).
TEST(GltfNormals, SecondPassIsANoOp) {
    const std::vector<uint8_t> original = read_file(kBakedChunk);
    const std::vector<uint8_t> oncePatched = overlume::ensure_flat_normals(original);
    const std::vector<uint8_t> twicePatched = overlume::ensure_flat_normals(oncePatched);
    EXPECT_EQ(oncePatched, twicePatched);
}

// The committed streaming fixture already carries NORMAL (measured
// directly, not assumed) -- this is the real "already has normals" case the
// load-time hook must leave untouched.
TEST(GltfNormals, StreamedTileWithNormalAlreadyIsUntouched) {
    const std::vector<uint8_t> glb = glb_from_b3dm(read_file(kIonTile));
    YAML::Node beforeJson = parse_glb_json(glb);
    ASSERT_EQ(count_primitives_without_normal(beforeJson), 0)
        << "fixture precondition: tile_a.b3dm is expected to already carry NORMAL";

    const std::vector<uint8_t> patched = overlume::ensure_flat_normals(glb);
    EXPECT_EQ(patched, glb) << "geometry that already has normals must pass through byte-for-byte";
}

TEST(GltfNormals, MalformedInputIsReturnedUnchanged) {
    const std::vector<uint8_t> empty;
    EXPECT_EQ(overlume::ensure_flat_normals(empty), empty);

    const std::vector<uint8_t> garbage{1, 2, 3, 4, 5};
    EXPECT_EQ(overlume::ensure_flat_normals(garbage), garbage);
}
