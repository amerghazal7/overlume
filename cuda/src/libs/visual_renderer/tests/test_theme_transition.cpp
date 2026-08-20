// test_theme_transition.cpp — Epic 1 Task 3 (VM-014): animated theme
// toggle (docs/superpowers/plans/2026-08-18-visual-mode-epic1.md).
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"

#include "theme.hpp"
#include "theme_transition.hpp"

#include "golden.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace {

double MaxAbsDiff(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    double maxDiff = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        maxDiff = std::max(maxDiff, std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i])));
    }
    return maxDiff;
}

}  // namespace

TEST(OklabHelpers, RoundTripIsIdentity) {
    // oklab_to_linear_srgb(linear_srgb_to_oklab(c)) must recover `c` --
    // verifies the published matrices used here are genuine inverses of
    // each other (catches a transposed/mistyped matrix element, which a
    // "does it look about right" reading of blend() output alone wouldn't).
    const mpviz::detail::Float3 colors[] = {
        {0.05f, 0.06f, 0.08f},  // dark_adas.palette.ground
        {0.82f, 0.80f, 0.76f},  // light_clay.palette.ground
        {1.0f, 1.0f, 1.0f},
        {0.0f, 0.0f, 0.0f},
    };
    for (const auto& c : colors) {
        const mpviz::detail::Oklab lab = mpviz::detail::linear_srgb_to_oklab(c);
        const mpviz::detail::Float3 back = mpviz::detail::oklab_to_linear_srgb(lab);
        EXPECT_NEAR(back.r, c.r, 1e-4f);
        EXPECT_NEAR(back.g, c.g, 1e-4f);
        EXPECT_NEAR(back.b, c.b, 1e-4f);
    }
}

TEST(ThemeTransition, MidpointBlend_IsBetweenEndpointsInOklab) {
    // dark_adas.ground vs light_clay.ground blended at w=0.5 must land near
    // the Oklab midpoint of the two, NOT the naive-sRGB-lerp midpoint (the
    // two differ measurably for colors this far apart -- that's the whole
    // reason spec §4.3 calls out Oklab specifically).
    const std::optional<mpviz::detail::Theme> dark =
        mpviz::detail::load_theme(kThemeDir, "dark_adas");
    const std::optional<mpviz::detail::Theme> light =
        mpviz::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());

    const mpviz::detail::Float3& a = dark->palette.ground;
    const mpviz::detail::Float3& b = light->palette.ground;
    const mpviz::detail::Float3 oklabBlend = mpviz::detail::blend_color(a, b, 0.5f);
    const mpviz::detail::Float3 naiveLerp{(a.r + b.r) / 2.0f, (a.g + b.g) / 2.0f,
                                           (a.b + b.b) / 2.0f};

    // The two must differ measurably -- if this ever came back ~equal, either
    // blend_color() silently degenerated into a plain component lerp, or the
    // two ground colors stopped being far enough apart to tell the
    // difference (neither should happen for the shipped themes).
    const double delta = std::abs(oklabBlend.r - naiveLerp.r) +
                          std::abs(oklabBlend.g - naiveLerp.g) +
                          std::abs(oklabBlend.b - naiveLerp.b);
    EXPECT_GT(delta, 0.01)
        << "Oklab-space blend and naive sRGB lerp landed on ~the same color -- "
           "blend_color() isn't actually blending in Oklab space.";

    // Self-consistency: the result's Oklab lightness must sit between the
    // two endpoints' lightness (a genuinely "between" blend, not something
    // that overshot/undershot due to a sign error in the matrices).
    const float La = mpviz::detail::linear_srgb_to_oklab(a).L;
    const float Lb = mpviz::detail::linear_srgb_to_oklab(b).L;
    const float Lmid = mpviz::detail::linear_srgb_to_oklab(oklabBlend).L;
    EXPECT_GE(Lmid, std::min(La, Lb) - 1e-4f);
    EXPECT_LE(Lmid, std::max(La, Lb) + 1e-4f);
}

TEST(ThemeTransition, Smoothstep_EasesInAndOut) {
    // smoothstep's defining property: weight(t) at t=0.1/0.9 sits closer to
    // the endpoints (0/1) than a linear ramp (which would give exactly
    // 0.1/0.9) would put it.
    EXPECT_LT(mpviz::detail::smoothstep01(0.1f), 0.1f);
    EXPECT_GT(mpviz::detail::smoothstep01(0.9f), 0.9f);
    EXPECT_FLOAT_EQ(mpviz::detail::smoothstep01(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(mpviz::detail::smoothstep01(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(mpviz::detail::smoothstep01(0.5f), 0.5f);  // symmetric at the midpoint
}

TEST(ThemeTransition, DeterministicClock_MatchesTargetAtDuration) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};  // medium, same as Task 2's goldens
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);                     // t=0, still dark_adas
    mpviz::set_theme(r, "light_clay", 0.0, 0.8);     // begin transition at t=0

    mpviz::CameraPose kFixedPose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};

    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);
    // golden at t=0.0 (still ~dark_adas -- first tick of the transition).
    EXPECT_GT(mpviz::testing::render_and_compare(
                  r, kFixedPose, MPVIZ_TEST_DATA_DIR "/tests/goldens/transition_t0.png",
                  "/tmp/transition_t0_actual.png"),
              0.98);

    scene.sim_time_sec = 0.4;
    mpviz::set_scene(r, scene);
    // golden at t=0.4s (~50% blended -- its own committed midpoint golden,
    // not compared against either endpoint).
    EXPECT_GT(mpviz::testing::render_and_compare(
                  r, kFixedPose, MPVIZ_TEST_DATA_DIR "/tests/goldens/transition_t0_4.png",
                  "/tmp/transition_t0_4_actual.png"),
              0.98);

    scene.sim_time_sec = 0.8;
    mpviz::set_scene(r, scene);
    // golden at t=0.8s (fully light_clay -- transition_sec elapsed).
    EXPECT_GT(mpviz::testing::render_and_compare(
                  r, kFixedPose, MPVIZ_TEST_DATA_DIR "/tests/goldens/transition_t0_8.png",
                  "/tmp/transition_t0_8_actual.png"),
              0.98);

    mpviz::destroy_renderer(r);
}

TEST(ThemeTransition, RetargetMidFlight_StartsFromCurrentBlendNotEndpoint) {
    // set_theme(light_clay) at t=0, advance to sim_time_sec=0.4 (mid-blend,
    // ~50% through the 0.8s transition) and render -- this is the frame we
    // compare against. Then, WITHOUT advancing the clock, call set_theme()
    // again (retarget back to dark_adas) and render the SAME sim_time_sec
    // again: with a correct retarget-from-current-blend implementation,
    // `t` for the new transition is (0.4-0.4)/duration == 0, so
    // blend(from, to, 0) == `from` exactly -- `from` being the snapshot of
    // the pre-retarget blend -- and the two frames must match. A buggy
    // implementation that snapped `from` to either endpoint (dark_adas or
    // light_clay) instead of the actual current blend would render a
    // visibly different frame here.
    constexpr uint32_t kWidth = 320, kHeight = 240;
    mpviz::RenderConfig cfg{kWidth, kHeight, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }

    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);
    mpviz::set_theme(r, "light_clay", 0.0, 0.8);

    scene.sim_time_sec = 0.4;
    mpviz::set_scene(r, scene);
    std::vector<uint8_t> beforeRetarget(static_cast<size_t>(kWidth) * kHeight * 3);
    mpviz::FrameView beforeView{beforeRetarget.data(), kWidth, kHeight};
    ASSERT_TRUE(mpviz::render_frame(r, pose, beforeView));

    // Retarget mid-flight, same instant on the clock.
    ASSERT_TRUE(mpviz::set_theme(r, "dark_adas", 0.4, 0.8));

    std::vector<uint8_t> afterRetarget(static_cast<size_t>(kWidth) * kHeight * 3);
    mpviz::FrameView afterView{afterRetarget.data(), kWidth, kHeight};
    ASSERT_TRUE(mpviz::render_frame(r, pose, afterView));

    // At w=0 every color field goes through one extra Oklab encode/decode
    // round trip (blend_color(from, to, 0) == oklab_to_linear_srgb(encode(
    // from)), not literally `from`) — OklabHelpers.RoundTripIsIdentity above
    // bounds that round trip to ~1e-4 in linear color space, which the ACES
    // tonemap + 8-bit quantization can turn into a few discrete levels here
    // and there. sun.intensity/ibl.intensity themselves are NOT a source of
    // extra noise here (Task 3 Step 3's geometric-intensity fix): lerpf_
    // geometric(a, b, 0) == a exactly (pow(x, 0) == 1 for any finite x>0),
    // bit-for-bit identical to the pre-retarget value, same as the old
    // linear lerp was at w=0. What DID move is the mid-transition operating
    // point the geometric fix targets on purpose (dimmer than the old
    // linear blend's overshoot -- that's the whole point), which shifts
    // where on the ACES tonemap curve this round trip's ~1e-4 color noise
    // lands: mean abs diff is essentially unchanged (0.465/255, was
    // ~0.46/255 under the linear intensity lerp), but the single worst
    // pixel now lands 11/255 instead of ~2/255 (measured on this build) --
    // real, but nowhere near what a genuine snap-to-endpoint bug would
    // produce (dark_adas vs. light_clay differ by ~100+ mean luminance
    // levels, spread across nearly every pixel, not one outlier pixel).
    // 24.0 keeps comfortable margin above the measured 11 while still
    // catching a real snap with wide margin.
    EXPECT_LT(MaxAbsDiff(beforeRetarget, afterRetarget), 24.0)
        << "retargeting set_theme() mid-flight visibly snapped the frame -- "
           "the new transition's `from` isn't the current blend.";

    mpviz::destroy_renderer(r);
}

TEST(ThemeTransition, MidTransition_LuminanceDoesNotOvershootEndpoints) {
    // Review finding (Epic1 Task2/3 gate, MAJOR 1): sun.intensity/ibl.
    // intensity were plain linearly lerped across ~19-29x ranges (dark_adas
    // 480000/256000 lux -> light_clay 25000/8750 lux) while palette albedo
    // brightens at the same time -- illumination x albedo is a PRODUCT, so a
    // linear-lerp-of-illumination blend overshoots past both endpoints
    // around t~0.5-0.65 (measured: frame-mean luminance 43 -> peak 205 ->
    // settles at 175 -- a "dim to night" toggle visibly flashes brighter
    // than day mid-transition). The fix interpolates intensity scalars
    // GEOMETRICALLY (log-space) instead, which by construction can't
    // overshoot either endpoint. This test renders t=0.5 and t=0.6 (of the
    // 0.8s transition, i.e. sim_time_sec 0.4/0.48) and asserts each stays
    // within [min(endpoint means)-3, max(endpoint means)+3] -- must FAIL
    // against a linear intensity lerp (peak ~205 vs endpoint max ~175), PASS
    // once the lerp is geometric.
    constexpr uint32_t kWidth = 320, kHeight = 240;
    mpviz::RenderConfig cfg{kWidth, kHeight, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};

    // Endpoint means come straight from the already-committed t0/t0_8
    // goldens (t=0.0 == still dark_adas, t=0.8 == fully settled light_clay)
    // -- no re-render needed, and this stays correct even if those two
    // goldens are regenerated later for an unrelated reason.
    const mpviz::testing::FrameStats endpointA = mpviz::testing::analyze_png(
        MPVIZ_TEST_DATA_DIR "/tests/goldens/transition_t0.png");
    const mpviz::testing::FrameStats endpointB = mpviz::testing::analyze_png(
        MPVIZ_TEST_DATA_DIR "/tests/goldens/transition_t0_8.png");
    ASSERT_GT(endpointA.mean, 0.0) << "couldn't load transition_t0.png golden";
    ASSERT_GT(endpointB.mean, 0.0) << "couldn't load transition_t0_8.png golden";
    const double loBound = std::min(endpointA.mean, endpointB.mean) - 3.0;
    const double hiBound = std::max(endpointA.mean, endpointB.mean) + 3.0;

    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);
    mpviz::set_theme(r, "light_clay", 0.0, 0.8);

    const double sampleTsOfDuration[] = {0.5, 0.6};  // t=0.5, t=0.6 of the 0.8s transition
    for (double tOfDuration : sampleTsOfDuration) {
        scene.sim_time_sec = tOfDuration * 0.8;
        mpviz::set_scene(r, scene);
        const std::string outPath =
            "/tmp/transition_mid_t" + std::to_string(tOfDuration) + "_actual.png";
        mpviz::testing::render_and_compare(r, pose, "/nonexistent/no_such_golden.png",
                                            outPath.c_str());
        const mpviz::testing::FrameStats stats = mpviz::testing::analyze_png(outPath.c_str());
        EXPECT_GE(stats.mean, loBound)
            << "t=" << tOfDuration << " mean " << stats.mean << " undershot endpoints ["
            << endpointA.mean << ", " << endpointB.mean << "]";
        EXPECT_LE(stats.mean, hiBound)
            << "t=" << tOfDuration << " mean " << stats.mean << " overshot endpoints ["
            << endpointA.mean << ", " << endpointB.mean
            << "] -- illumination x albedo blew out mid-transition (linear intensity lerp?)";
    }

    mpviz::destroy_renderer(r);
}

TEST(ThemeTransition, UnknownThemeName_ReturnsFalseAndLeavesActiveThemeUnchanged) {
    mpviz::RenderConfig cfg{320, 240, /*quality=*/1, kThemeDir, "dark_adas"};
    mpviz::VisualRenderer* r = mpviz::create_renderer(cfg);
    if (r == nullptr) {
        GTEST_SKIP() << "no GPU/EGL";
    }
    mpviz::SceneGraph scene{};
    scene.sim_time_sec = 0.0;
    mpviz::set_scene(r, scene);

    EXPECT_FALSE(mpviz::set_theme(r, "no_such_theme", 0.0, 0.8));

    // No transition should have started -- rendering right away must still
    // match the dark_adas golden (Task 2's own empty-world golden), same
    // pose/scene as that test.
    mpviz::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    EXPECT_GT(mpviz::testing::render_and_compare(
                  r, pose, MPVIZ_TEST_DATA_DIR "/tests/goldens/empty_world_dark_adas.png",
                  "/tmp/set_theme_unknown_actual.png"),
              0.98);
    mpviz::destroy_renderer(r);
}
