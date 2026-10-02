// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal
//
// Clean-room consumer of an INSTALLED overlume package. Uses only the public
// headers and passes theme_assets_dir = nullptr, so a pass proves the
// installed share/overlume/themes was found next to the library.
//
//   package_smoke --expect-render    renders a frame, checks image + theme
//   package_smoke --expect-no-gpu    create_renderer() must return nullptr
//
// Prints PASS or "FAIL: <reason>"; exit 0 / 1.

#include <overlume/api.h>
#include <overlume/scene.h>

#ifdef SMOKE_CONSUMER_YAML
#include <yaml-cpp/yaml.h>
#endif

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifndef SMOKE_THEMES_DIR
#error "SMOKE_THEMES_DIR must be defined by CMakeLists.txt (${overlume_THEMES_DIR})"
#endif

namespace {

#ifdef SMOKE_STATIC_LINK
// Linked statically, the "module" is this executable, which has no share/ next to it.
const char* const kDefaultThemeDir = SMOKE_THEMES_DIR;
#else
const char* const kDefaultThemeDir = nullptr;  // installed-package default
#endif

constexpr uint32_t kW = 320;
constexpr uint32_t kH = 240;

int fail(const char* why) {
    std::printf("FAIL: %s\n", why);
    return 1;
}

enum class Render { kOk, kNoRenderer, kFrameFailed };

// Renders one frame; `theme_dir` nullptr = the installed-package default.
Render render(const char* theme_dir, const overlume::CameraPose& pose, std::vector<uint8_t>& rgb,
              bool* theme_loaded) {
    overlume::RenderConfig config{};
    config.width = kW;
    config.height = kH;
    config.quality = 1;
    config.theme_assets_dir = theme_dir;
    config.initial_theme = "dark_adas";
    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) return Render::kNoRenderer;
    *theme_loaded = overlume::theme_assets_loaded(renderer);
    rgb.assign(static_cast<size_t>(kW) * kH * 3, 0);
    overlume::FrameView view{rgb.data(), kW, kH};
    const bool ok = overlume::render_frame(renderer, pose, view);
    overlume::destroy_renderer(renderer);
    return ok ? Render::kOk : Render::kFrameFailed;
}

}  // namespace

int main(int argc, char** argv) {
    const bool expect_render = argc > 1 && std::strcmp(argv[1], "--expect-render") == 0;
    const bool expect_no_gpu = argc > 1 && std::strcmp(argv[1], "--expect-no-gpu") == 0;
    if (!expect_render && !expect_no_gpu) return fail("usage: package_smoke --expect-render|--expect-no-gpu");

    // The distro's own yaml-cpp must coexist with the one baked into liboverlume.
#ifdef SMOKE_CONSUMER_YAML
    if (YAML::Load("a: 1")["a"].as<int>() != 1) return fail("consumer yaml-cpp broken");
#endif

    overlume::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;

    std::vector<uint8_t> rgb;
    bool loaded = false;
    const Render first = render(kDefaultThemeDir, pose, rgb, &loaded);
    if (expect_no_gpu) {
        // Only a nullptr from create_renderer() is the graceful no-GPU answer.
        if (first == Render::kNoRenderer) {
            std::printf("PASS\n");
            return 0;
        }
        return fail(first == Render::kOk ? "a frame rendered but no GPU was expected"
                                         : "render_frame failed (create_renderer should have returned nullptr)");
    }
    if (first == Render::kNoRenderer) return fail("create_renderer returned nullptr (no GPU/EGL?)");
    if (first == Render::kFrameFailed) return fail("render_frame failed");

    // The installed themes must be found with theme_assets_dir = nullptr.
    // (dark_adas.yaml's colours equal the compiled-in fallback theme's, so the
    // pixels cannot tell the two apart; theme_assets_loaded() can.)
    if (!loaded) return fail("installed theme dir not found (compiled-in fallback theme in use)");

    bool varied = false;
    for (size_t i = 3; i < rgb.size() && !varied; ++i) varied = rgb[i] != rgb[i % 3];
    if (!varied) return fail("frame is uniform");

    // Same frame through the explicit ${overlume_THEMES_DIR}: the default
    // resolution must have picked that directory.
    std::vector<uint8_t> ref;
    if (render(SMOKE_THEMES_DIR, pose, ref, &loaded) != Render::kOk || !loaded)
        return fail("explicit installed theme dir did not load");
    int worst = 0;
    for (size_t i = 0; i < rgb.size(); ++i) worst = std::max(worst, std::abs(int(rgb[i]) - int(ref[i])));
    if (worst > 2) return fail("default-theme frame differs from the explicit installed-theme frame");

    std::printf("PASS\n");
    return 0;
}
