// hello_frame.cpp — Epic 0 Task 2 manual/visual proof
// (docs/superpowers/plans/2026-08-18-visual-mode.md): renders one frame with
// the node's default pose and writes it to hello_frame.png so a human can
// eyeball cube + grid + horizon. Run with no $DISPLAY set to prove the
// headless-EGL path.
#include "visual_renderer/api.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cstdio>
#include <vector>

int main() {
    const uint32_t width = 1280;
    const uint32_t height = 720;

    mpviz::RenderConfig config{};
    config.width = width;
    config.height = height;
    config.quality = 2;

    mpviz::VisualRenderer* renderer = mpviz::create_renderer(config);
    if (renderer == nullptr) {
        std::fprintf(stderr, "hello_frame: create_renderer() failed (no GPU/EGL?)\n");
        return 1;
    }

    mpviz::CameraPose pose{};
    pose.eye[0] = -4.0;
    pose.eye[1] = 0.0;
    pose.eye[2] = 3.5;
    pose.target[0] = 2.0;
    pose.target[1] = 0.0;
    pose.target[2] = -0.5;
    pose.vfov_deg = 80.0;  // see tests/test_hello_frame.cpp for why not 50

    std::vector<uint8_t> rgb(static_cast<size_t>(width) * height * 3);
    mpviz::FrameView view{rgb.data(), width, height};

    if (!mpviz::render_frame(renderer, pose, view)) {
        std::fprintf(stderr, "hello_frame: render_frame() failed\n");
        mpviz::destroy_renderer(renderer);
        return 1;
    }

    const int ok = stbi_write_png("hello_frame.png", static_cast<int>(width),
                                   static_cast<int>(height), 3, rgb.data(),
                                   static_cast<int>(width) * 3);
    mpviz::destroy_renderer(renderer);

    if (!ok) {
        std::fprintf(stderr, "hello_frame: failed to write hello_frame.png\n");
        return 1;
    }
    std::printf("hello_frame: wrote hello_frame.png (%ux%u)\n", width, height);
    return 0;
}
