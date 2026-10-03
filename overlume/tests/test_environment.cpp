// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "golden.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

const std::string kTestTownDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";

constexpr overlume::Vec3 kChunk0Center{-128.0, -128.0, 0.0};
constexpr overlume::Vec3 kFarFromTown{100000.0, 100000.0, 0.0};

constexpr overlume::Vec3 kBuildingsCentroid{-109.2, -17.1, 3.0};

}

TEST(Environment, SetEnvironmentSourceWithMissingDirIsNonFatal) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    EXPECT_FALSE(overlume::set_environment_source(r, "/nonexistent/path", a));

    std::vector<uint8_t> buf(320 * 240 * 3);
    EXPECT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    overlume::destroy_renderer(r);
}

TEST(Environment, NullSourceUriIsANoOpNotAConfigError) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_FALSE(overlume::set_environment_source(r, nullptr, overlume::GeoAnchor{}));
    EXPECT_FALSE(overlume::set_environment_source(r, "", overlume::GeoAnchor{}));
    overlume::destroy_renderer(r);
}

TEST(Environment, ChunksWithinLoadRadiusOfEgoAreRenderedAsThemedClay) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kChunk0Center;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u);
    overlume::destroy_renderer(r);
}

TEST(Environment, ChunksFarFromEgoAreNotLoaded) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFarFromTown;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 0u);
    overlume::destroy_renderer(r);
}

TEST(Environment, HysteresisBandKeepsChunksLoadedThenUnloadsAndReloads) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));

    std::vector<uint8_t> buf(320 * 240 * 3);
    overlume::SceneGraph s{};
    s.ego.valid = 1;

    const overlume::Vec3 base{kChunk0Center.x, kChunk0Center.y - 150.0, kChunk0Center.z};

    s.ego.position = base;
    overlume::set_scene(r, s);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    ASSERT_EQ(overlume::testing::environment_loaded_chunk_count(r), 1u);

    s.ego.position = {base.x + 350.0, base.y, base.z};
    overlume::set_scene(r, s);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 1u);

    s.ego.position = {base.x + 500.0, base.y, base.z};
    overlume::set_scene(r, s);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 0u);

    s.ego.position = base;
    overlume::set_scene(r, s);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 1u);

    overlume::destroy_renderer(r);
}

namespace {
size_t count_differing_bytes(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b,
                             int tolerance = 1) {
    size_t n = 0;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
        if (std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i])) > tolerance) ++n;
    }
    return n;
}
}

TEST(Environment, SetEnvironmentVisibleNullRendererIsFalse) {
    EXPECT_FALSE(overlume::set_environment_visible(nullptr, true));
    EXPECT_FALSE(overlume::set_environment_visible(nullptr, false));
}

TEST(Environment, SetEnvironmentVisibleFalseHidesLoadedChunksWithoutTearingDown) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kChunk0Center;
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{kBuildingsCentroid.x + 50, kBuildingsCentroid.y - 70, 40},
                              {kBuildingsCentroid.x, kBuildingsCentroid.y, kBuildingsCentroid.z},
                              60.0};
    const size_t nBytes = 320u * 240u * 3u;
    std::vector<uint8_t> visibleBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(r, pose, {visibleBuf.data(), 320, 240}));
    const uint64_t loadedBefore = overlume::testing::environment_loaded_chunk_count(r);
    ASSERT_GT(loadedBefore, 0u);
    ASSERT_EQ(overlume::testing::environment_scene_membership_count(r), loadedBefore);
    const double ssimVisible = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/environment_test_town_dark_adas.png",
        OVERLUME_TMP_DIR "/environment_visible_before_hide_actual.png");
    ASSERT_GT(ssimVisible, overlume::testing::kSsimMin);

    ASSERT_TRUE(overlume::set_environment_visible(r, false));
    std::vector<uint8_t> hiddenBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(r, pose, {hiddenBuf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), loadedBefore);
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r), 0u);
    const size_t diffHidden = count_differing_bytes(visibleBuf, hiddenBuf);
    EXPECT_GT(diffHidden, nBytes / 20) << "buildings still visible in the rendered frame after "
                                          "set_environment_visible(r, false) (only "
                                       << diffHidden << "/" << nBytes << " bytes changed)";

    ASSERT_TRUE(overlume::set_environment_visible(r, true));
    std::vector<uint8_t> shownAgainBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(r, pose, {shownAgainBuf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), loadedBefore);
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r), loadedBefore);
    const size_t diffShownAgain = count_differing_bytes(shownAgainBuf, visibleBuf);
    EXPECT_LT(diffShownAgain, nBytes / 20)
        << "re-showing produced a different frame than before hiding (" << diffShownAgain << "/"
        << nBytes << " bytes changed beyond dithering-level noise)";
    overlume::destroy_renderer(r);
}

TEST(Environment, ChunkLoadedWhileHiddenDoesNotPopIntoView) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));
    ASSERT_TRUE(overlume::set_environment_visible(r, false));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kChunk0Center;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{kBuildingsCentroid.x + 50, kBuildingsCentroid.y - 70, 40},
                              {kBuildingsCentroid.x, kBuildingsCentroid.y, kBuildingsCentroid.z},
                              60.0};
    const size_t nBytes = 320u * 240u * 3u;
    std::vector<uint8_t> hiddenBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(r, pose, {hiddenBuf.data(), 320, 240}));
    ASSERT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u)
        << "loading itself must still happen while hidden";
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r), 0u)
        << "a chunk loaded while hidden was added to the Filament scene anyway";

    auto* rNoEnv = overlume::create_renderer(cfg);
    ASSERT_NE(rNoEnv, nullptr);
    overlume::set_scene(rNoEnv, s);
    std::vector<uint8_t> noEnvBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(rNoEnv, pose, {noEnvBuf.data(), 320, 240}));
    overlume::destroy_renderer(rNoEnv);
    const size_t diffFromNoEnv = count_differing_bytes(hiddenBuf, noEnvBuf);
    EXPECT_LT(diffFromNoEnv, nBytes / 100)
        << "a chunk loaded while hidden popped into view (" << diffFromNoEnv << "/" << nBytes
        << " bytes differ from a renderer with no environment source at all)";

    ASSERT_TRUE(overlume::set_environment_visible(r, true));
    std::vector<uint8_t> shownBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(r, pose, {shownBuf.data(), 320, 240}));
    const size_t diffShown = count_differing_bytes(hiddenBuf, shownBuf);
    EXPECT_GT(diffShown, nBytes / 20) << "showing again produced no visible change (only "
                                      << diffShown << "/" << nBytes << " bytes changed)";
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r),
              overlume::testing::environment_loaded_chunk_count(r));
    overlume::destroy_renderer(r);
}

TEST(Environment, SetEnvironmentVisibleFalseBeforeArmingHidesNewlyOpenedSource) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ASSERT_TRUE(overlume::environment_visible(r))
        << "default must be true before anyone touches it";

    ASSERT_TRUE(overlume::set_environment_visible(r, false));
    ASSERT_FALSE(overlume::environment_visible(r));
    ASSERT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::NONE);

    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));
    EXPECT_FALSE(overlume::environment_visible(r));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kChunk0Center;
    overlume::set_scene(r, s);
    overlume::CameraPose pose{{kBuildingsCentroid.x + 50, kBuildingsCentroid.y - 70, 40},
                              {kBuildingsCentroid.x, kBuildingsCentroid.y, kBuildingsCentroid.z},
                              60.0};
    const size_t nBytes = 320u * 240u * 3u;
    std::vector<uint8_t> hiddenBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(r, pose, {hiddenBuf.data(), 320, 240}));
    ASSERT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u)
        << "loading itself must still happen while hidden";
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r), 0u)
        << "a source armed while environmentVisible was already false came up visible anyway";

    ASSERT_TRUE(overlume::set_environment_visible(r, true));
    std::vector<uint8_t> shownBuf(nBytes);
    ASSERT_TRUE(overlume::render_frame(r, pose, {shownBuf.data(), 320, 240}));
    const size_t diffShown = count_differing_bytes(hiddenBuf, shownBuf);
    EXPECT_GT(diffShown, nBytes / 20) << "showing produced no visible change (only " << diffShown
                                      << "/" << nBytes << " bytes changed)";
    EXPECT_EQ(overlume::testing::environment_scene_membership_count(r),
              overlume::testing::environment_loaded_chunk_count(r));
    overlume::destroy_renderer(r);
}

TEST(Environment, InvalidEgoFreezesLoadedChunkCount) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{0, -8, 3}, {0, 0, 0.5}, 60};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kChunk0Center;
    overlume::set_scene(r, s);
    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    const uint64_t loadedBefore = overlume::testing::environment_loaded_chunk_count(r);
    ASSERT_GT(loadedBefore, 0u);

    s.ego.valid = 0;
    s.ego.position = kFarFromTown;
    overlume::set_scene(r, s);
    ASSERT_TRUE(overlume::render_frame(r, pose, {buf.data(), 320, 240}));
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), loadedBefore);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentGolden, TestTown_DarkAdas) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), a));

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kChunk0Center;
    overlume::set_scene(r, s);

    overlume::CameraPose pose{{kBuildingsCentroid.x + 50, kBuildingsCentroid.y - 70, 40},
                              {kBuildingsCentroid.x, kBuildingsCentroid.y, kBuildingsCentroid.z},
                              60.0};
    std::vector<uint8_t> warm(320u * 240u * 3u);
    ASSERT_TRUE(overlume::render_frame(r, pose, {warm.data(), 320, 240}));
    ASSERT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u);

    double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/environment_test_town_dark_adas.png",
        OVERLUME_TMP_DIR "/environment_test_town_dark_adas_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentPerf, RenderMsDeltaWithTestTownLoaded) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    overlume::CameraPose pose{{kChunk0Center.x + 40, kChunk0Center.y - 60, 35},
                              {kChunk0Center.x, kChunk0Center.y, 2},
                              60.0};
    std::vector<uint8_t> buf(320u * 240u * 3u);
    constexpr int kFrames = 30;

    auto measure = [&](bool withEnvironment) -> double {
        auto* r = overlume::create_renderer(cfg);
        if (!r) return -1.0;
        if (withEnvironment) {
            overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
            overlume::set_environment_source(r, kTestTownDir.c_str(), a);
        }
        overlume::SceneGraph s{};
        s.ego.valid = 1;
        s.ego.position = kChunk0Center;
        overlume::set_scene(r, s);
        overlume::render_frame(r, pose, {buf.data(), 320, 240});
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < kFrames; ++i) {
            overlume::render_frame(r, pose, {buf.data(), 320, 240});
        }
        const auto t1 = std::chrono::steady_clock::now();
        overlume::destroy_renderer(r);
        return std::chrono::duration<double, std::milli>(t1 - t0).count() / kFrames;
    };

    const double withEnv = measure(true);
    if (withEnv < 0.0) GTEST_SKIP() << "no GPU/EGL";
    const double withoutEnv = measure(false);
    std::cerr << "[EnvironmentPerf] mean render_ms without=" << withoutEnv << " with=" << withEnv
              << " delta=" << (withEnv - withoutEnv) << " ms\n";
    SUCCEED();
}
