// 03_themes.cpp — loads both shipped themes, starts a set_theme() transition
// between them, and renders using the DETERMINISTIC clock the API exposes
// (SceneGraph::sim_time_sec) rather than wall-clock time. Public headers
// only: <overlume/api.h>, <overlume/scene.h>.
//
// set_theme()'s contract (scene.h): eases every themed token from whatever
// is active toward `theme_name`, starting at `at_sec` and finishing
// `transition_sec` seconds later — both measured against the caller's own
// clock (SceneGraph::sim_time_sec), never wall time. That's what makes a
// mid-transition frame reproducible: publish the same sim_time_sec twice
// and you get the same blend twice, in a batch job or a unit test alike.
#include <overlume/api.h>
#include <overlume/scene.h>

#include "common.hpp"

#include <cstdio>
#include <vector>

namespace {

bool render_at(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
                const overlume::RenderConfig& config, double sim_time_sec, std::vector<uint8_t>& rgb) {
    // Re-publishing the scene at each sim_time_sec is what actually moves
    // set_theme()'s ease forward — render_frame() alone reads whatever
    // scene was last published (freeze-frame, see 02_scene_population.cpp),
    // so sim_time_sec has to travel through a set_scene() call.
    overlume::SceneGraph scene{};
    scene.sim_time_sec = sim_time_sec;
    scene.ego.valid = 1;
    overlume::set_scene(r, scene);
    overlume::FrameView view{rgb.data(), config.width, config.height};
    return overlume::render_frame(r, pose, view);
}

}  // namespace

int main(int argc, char** argv) {
    const overlume_examples::ExampleArgs args =
        overlume_examples::parse_args(argc, argv, "03_themes.png");

    // theme_parses() is a GPU-free check: it only exercises the yaml-cpp
    // parse path (create_renderer()'s own first step), no Filament/EGL
    // involved — useful here to prove both shipped themes are readable
    // before spending a renderer on them.
    for (const char* name : {"dark_adas", "light_clay"}) {
        const bool parses = overlume::theme_parses(args.theme_dir.c_str(), name);
        std::printf("theme_parses(%s, \"%s\") = %s\n", args.theme_dir.c_str(), name,
                    parses ? "true" : "false");
    }

    overlume::RenderConfig config{};
    config.width = 640;
    config.height = 480;
    config.quality = 1;
    config.theme_assets_dir = args.theme_dir.c_str();
    config.initial_theme = "dark_adas";

    overlume::VisualRenderer* renderer = overlume::create_renderer(config);
    if (renderer == nullptr) {
        std::fprintf(stderr, "03_themes: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }

    const overlume::CameraPose pose{{0.0, -8.0, 4.0}, {0.0, 0.0, 0.0}, 60.0};
    std::vector<uint8_t> rgb(static_cast<size_t>(config.width) * config.height * 3);

    // t=0: still fully dark_adas (nothing has been requested to change yet).
    if (!render_at(renderer, pose, config, 0.0, rgb)) {
        std::fprintf(stderr, "03_themes: render_frame() at t=0 failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }

    // Start a 0.8s transition to light_clay, beginning at sim_time_sec=0.0.
    if (!overlume::set_theme(renderer, "light_clay", /*at_sec=*/0.0, /*transition_sec=*/0.8)) {
        std::fprintf(stderr, "03_themes: set_theme() returned false unexpectedly\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }

    // t=0.4: exactly half-way through the 0.8s ease — every themed token
    // (palette, HUD colors, sun/IBL) sits at its Oklab-blended midpoint.
    if (!render_at(renderer, pose, config, 0.4, rgb)) {
        std::fprintf(stderr, "03_themes: render_frame() mid-transition failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }
    const bool wroteMid =
        overlume_examples::write_png(args.output_path, config.width, config.height, rgb.data());

    // t=0.8: transition_sec has fully elapsed — fully light_clay now.
    // Rendered here to prove the ease actually completes; not written out
    // (this example writes one image, the mid-transition frame above,
    // which is the interesting one to look at).
    if (!render_at(renderer, pose, config, 0.8, rgb)) {
        std::fprintf(stderr, "03_themes: render_frame() at t=0.8 failed\n");
        overlume::destroy_renderer(renderer);
        return 1;
    }
    std::printf("rendered t=0.0 (dark_adas), t=0.4 (mid-blend, written above), "
                "t=0.8 (fully light_clay)\n");

    overlume::destroy_renderer(renderer);
    return wroteMid ? 0 : 1;
}
