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

namespace overlume::ros
{
namespace
{

TEST(ComposeEnvironmentSourceUri, EmptySourceUriUsesChunksDirVerbatim)
{
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "", ""), "/baked/chunks");
}

TEST(ComposeEnvironmentSourceUri, PlainIonUriWithCacheDirAppendsCache)
{
    EXPECT_EQ(compose_environment_source_uri("", "ion://96188", "/mnt/data/tiles"),
              "ion://96188?cache=/mnt/data/tiles");
}

TEST(ComposeEnvironmentSourceUri, ChunksDirFallbackAppendedWhenNotAlreadyPresent)
{
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", "ion://96188", ""),
              "ion://96188?fallback=/baked/chunks");
}

// The actual gate finding: the google preset's URI already carries
// "cache=off" (Decision 14 Step 2b) -- a non-empty tile-cache-dir param must
// NOT add a second cache= key, which parse_ion_spec()'s last-wins loop would
// resolve to the dir, overwriting "off".
TEST(ComposeEnvironmentSourceUri, GooglePresetCacheOffSurvivesNonEmptyCacheDirParam)
{
    const std::string google_preset_uri = "ion://2275207?materials=original&cache=off";
    EXPECT_EQ(compose_environment_source_uri("", google_preset_uri, "/mnt/data/tiles"),
              google_preset_uri);
}

// Same duplicate-key hazard for fallback=.
TEST(ComposeEnvironmentSourceUri, ExplicitFallbackInUriSurvivesNonEmptyChunksDir)
{
    const std::string uri_with_fallback = "ion://96188?fallback=/explicit/dir";
    EXPECT_EQ(compose_environment_source_uri("/baked/chunks", uri_with_fallback, ""),
              uri_with_fallback);
}

}  // namespace
}  // namespace overlume::ros

// fallback_dir_from_source_uri(): the STREAMING_FALLBACK WARN names the dir
// actually in effect (final review 2026-09-17, finding #17). Reads the
// COMPOSED string, so an inline fallback= wins exactly as compose does.
TEST(FallbackDirFromSourceUri, NoKeyIsEmpty)
{
    EXPECT_EQ(overlume::ros::fallback_dir_from_source_uri("ion://96188?cache=/c"), "");
    EXPECT_EQ(overlume::ros::fallback_dir_from_source_uri(""), "");
}

TEST(FallbackDirFromSourceUri, LastKeyRunsToEnd)
{
    EXPECT_EQ(overlume::ros::fallback_dir_from_source_uri(
                  "ion://96188?cache=/c&fallback=/baked/dir"),
              "/baked/dir");
}

TEST(FallbackDirFromSourceUri, MiddleKeyStopsAtAmpersand)
{
    EXPECT_EQ(overlume::ros::fallback_dir_from_source_uri(
                  "ion://96188?fallback=/explicit&cache=off"),
              "/explicit");
}

TEST(FallbackDirFromSourceUri, InlineFallbackWinsThroughCompose)
{
    const std::string composed = overlume::ros::compose_environment_source_uri(
        "/chunks", "ion://96188?fallback=/explicit", "");
    EXPECT_EQ(overlume::ros::fallback_dir_from_source_uri(composed), "/explicit");
}
