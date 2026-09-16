// test_gltf_normals.cpp — mpviz::ensure_flat_normals() (gltf_normals.hpp), the
// load-time fix for environment geometry with no vertex normals. No
// Filament/gtfio types here (same tests/*.cpp boundary as every other file
// in this directory -- see environment_test_hooks.hpp) -- these check the
// byte/JSON level directly against the real committed fixtures.
#include "gltf_normals.hpp"

#include "test_paths.hpp"

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

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

const std::string kBakedChunk = std::string(MPVIZ_TEST_DATA_DIR) +
    "/tests/fixtures/environment_test_town_0/chunks/chunk_-1_-1.glb";
const std::string kIonTile = std::string(MPVIZ_TEST_DATA_DIR) +
    "/tests/fixtures/environment_ion_fixture_0/tile_a.b3dm";

}  // namespace

TEST(GltfNormals, AddsNormalToChunkMissingIt) {
    const std::vector<uint8_t> original = read_file(kBakedChunk);
    YAML::Node beforeJson = parse_glb_json(original);
    ASSERT_GT(count_primitives_without_normal(beforeJson), 0)
        << "fixture precondition: chunk_-1_-1.glb is expected to carry POSITION only";

    const std::vector<uint8_t> patched = mpviz::ensure_flat_normals(original);
    EXPECT_NE(patched.size(), original.size()) << "normals should have been appended";

    YAML::Node afterJson = parse_glb_json(patched);
    EXPECT_EQ(count_primitives_without_normal(afterJson), 0);

    for (const YAML::Node& mesh : afterJson["meshes"]) {
        for (const YAML::Node& prim : mesh["primitives"]) {
            const int normalIdx = prim["attributes"]["NORMAL"].as<int>();
            const int posIdx = prim["attributes"]["POSITION"].as<int>();
            const YAML::Node normalAcc = afterJson["accessors"][normalIdx];
            const YAML::Node posAcc = afterJson["accessors"][posIdx];
            EXPECT_EQ(normalAcc["componentType"].as<int>(), 5126);
            EXPECT_EQ(normalAcc["type"].as<std::string>(), "VEC3");
            EXPECT_EQ(normalAcc["count"].as<int>(), posAcc["count"].as<int>());
        }
    }
}

// Running the fix twice must be a no-op the second time -- every primitive
// already has NORMAL after the first pass, so this also proves the "already
// has normals" path is left untouched (bytes identical, not just
// equivalent).
TEST(GltfNormals, SecondPassIsANoOp) {
    const std::vector<uint8_t> original = read_file(kBakedChunk);
    const std::vector<uint8_t> oncePatched = mpviz::ensure_flat_normals(original);
    const std::vector<uint8_t> twicePatched = mpviz::ensure_flat_normals(oncePatched);
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

    const std::vector<uint8_t> patched = mpviz::ensure_flat_normals(glb);
    EXPECT_EQ(patched, glb) << "geometry that already has normals must pass through byte-for-byte";
}

TEST(GltfNormals, MalformedInputIsReturnedUnchanged) {
    const std::vector<uint8_t> empty;
    EXPECT_EQ(mpviz::ensure_flat_normals(empty), empty);

    const std::vector<uint8_t> garbage{1, 2, 3, 4, 5};
    EXPECT_EQ(mpviz::ensure_flat_normals(garbage), garbage);
}
