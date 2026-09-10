// camera_textures.cpp — VM-090 (unified-engine migration Task 1, ADR-0005):
// the POD-boundary camera-texture mechanism, now also (VM-091, Task 2) the
// call site that hands a freshly-(re)allocated camera-texture set off to
// bowl.cpp's build_bowl() for the mesh/material half of set_bowl_config()'s
// documented contract. set_bowl_visible()/set_camera_motion_delta() here
// only store state bowl.cpp's update_bowl() reads every render_frame() call.
#include "bowl.hpp"
#include "camera_textures.hpp"
#include "camera_textures_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/Engine.h>
#include <filament/Texture.h>

#include <backend/PixelBufferDescriptor.h>

#include <cstring>
#include <vector>

namespace mpviz {

namespace {

filament::Texture* build_camera_texture(filament::Engine& engine, uint32_t w, uint32_t h) {
    // RGB8, three channels -- the camera pixels this task's Interfaces block
    // documents (Task 2 Step 4 is where the isTextureFormatSupported/RGBA8
    // fallback check lands, alongside the first real sampler binding).
    return filament::Texture::Builder()
        .width(w)
        .height(h)
        .levels(1)
        .format(filament::Texture::InternalFormat::RGB8)
        .sampler(filament::Texture::Sampler::SAMPLER_2D)
        .build(engine);
}

// Uploads `rgb` (w*h*3 bytes) into `tex`. `release` non-null (ADR-0005:
// release-callback shipped) wraps the caller's buffer BY REFERENCE -- no
// library-side heap copy -- and hands it back to `release(rgb, size, user)`
// once Filament has consumed it. `release` null keeps the copy-on-call
// contract: a heap copy freed synchronously, same shape as
// ground_grid.cpp's own upload_occupancy_texture().
void upload_camera_frame(filament::Engine& engine, filament::Texture* tex, const uint8_t* rgb,
                          uint32_t width, uint32_t height,
                          void (*release)(void*, size_t, void*), void* user) {
    const size_t byteCount = static_cast<size_t>(width) * height * 3;
    if (release != nullptr) {
        filament::backend::PixelBufferDescriptor pbd(
            const_cast<uint8_t*>(rgb), byteCount,
            filament::backend::PixelBufferDescriptor::PixelDataFormat::RGB,
            filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE, release, user);
        tex->setImage(engine, 0, std::move(pbd));
    } else {
        auto* heap = new std::vector<uint8_t>(rgb, rgb + byteCount);
        filament::backend::PixelBufferDescriptor pbd(
            heap->data(), heap->size(),
            filament::backend::PixelBufferDescriptor::PixelDataFormat::RGB,
            filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
            [](void*, size_t, void* u) { delete static_cast<std::vector<uint8_t>*>(u); }, heap);
        tex->setImage(engine, 0, std::move(pbd));
    }
}

}  // namespace

bool set_bowl_config(VisualRenderer* r, const BowlConfig& cfg) {
    if (r == nullptr) return false;
    if (cfg.camera_count == 0 || cfg.camera_count > kMaxBowlCameras) return false;

    // Full re-bake: tear down whatever texture set is currently live (a
    // first call has nothing to tear down -- every slot's texture is
    // already null) and rebuild fresh from `cfg`. This task never reuses a
    // still-valid texture across a reconfigure; Task 2's bake is the
    // expensive path this call is meant to gate, not this allocation.
    for (auto& slot : r->cameraSlots) {
        if (slot.texture != nullptr) r->engine->destroy(slot.texture);
        slot = CameraTextureSlot{};
    }
    for (uint32_t i = 0; i < cfg.camera_count; ++i) {
        CameraTextureSlot& slot = r->cameraSlots[i];
        slot.width = cfg.cam_width[i];
        slot.height = cfg.cam_height[i];
        slot.extrinsics = cfg.extrinsics[i];
        slot.intrinsics = cfg.intrinsics[i];
        slot.texture = build_camera_texture(*r->engine, slot.width, slot.height);
    }
    r->cameraCount = cfg.camera_count;
    // Task 2's half of this function's documented contract: bake the bowl
    // mesh + per-vertex weight/index attributes and (re)build the bowl.mat
    // material instance, now that this camera set's textures exist for it
    // to sample. A bake failure (e.g. degenerate mesh params) leaves
    // r->bowl null -- the bowl renders nothing, same "missing config
    // renders nothing" convention set_bowl_config's own null/invalid-input
    // checks above already follow -- but the camera textures this function
    // just (re)built stay valid regardless, so this function still returns
    // true: the textures/dirty-tracking half of its contract succeeded.
    build_bowl(*r, cfg);
    return true;
}

bool set_bowl_visible(VisualRenderer* r, bool visible) {
    if (r == nullptr || r->cameraCount == 0) return false;
    r->bowlVisible = visible;
    return true;
}

bool set_camera_motion_delta(VisualRenderer* r, uint32_t cam_idx,
                              const double delta_4x4_row_major[16]) {
    if (r == nullptr || cam_idx >= r->cameraCount) return false;
    std::memcpy(r->cameraSlots[cam_idx].motionDelta, delta_4x4_row_major, sizeof(double) * 16);
    return true;
}

bool set_camera_frame(VisualRenderer* r, uint32_t cam_idx, const uint8_t* rgb, uint32_t width,
                       uint32_t height, uint64_t frame_id,
                       void (*release)(void*, size_t, void*), void* user) {
    // Ownership-transfer contract (ADR-0005): when `release` is non-null the
    // library ALWAYS takes ownership of `rgb` -- release() fires exactly
    // once per call, whether the upload happened, was skipped by the dirty
    // gate, or was rejected outright. The caller must never free `rgb`
    // itself. hand_back() is the one-line "reject/skip without uploading"
    // exit; the actual-upload path hands the buffer to Filament instead.
    const size_t byteCount = static_cast<size_t>(width) * height * 3;
    auto hand_back = [&] {
        if (release) release(const_cast<uint8_t*>(rgb), byteCount, user);
    };

    if (r == nullptr || cam_idx >= r->cameraCount) {
        hand_back();
        return false;
    }
    CameraTextureSlot& slot = r->cameraSlots[cam_idx];
    if (width != slot.width || height != slot.height) {
        hand_back();
        return false;
    }

    // Dirty-tracking gate (Task 1 Step 2): O(1) integer compare, deliberately
    // NOT a content memcmp -- see scene.h's set_camera_frame() comment /
    // ADR-0005 for why that would reintroduce Option A's rejected cost.
    if (!slot.hasUploaded || frame_id != slot.lastFrameId) {
        upload_camera_frame(*r->engine, slot.texture, rgb, width, height, release, user);
        slot.lastFrameId = frame_id;
        slot.hasUploaded = true;
        ++slot.uploadCount;
    } else {
        hand_back();
    }
    return true;
}

}  // namespace mpviz

// Filament-free test introspection hook; see camera_textures_test_hooks.hpp.
namespace mpviz::testing {

uint64_t camera_frame_upload_count(mpviz::VisualRenderer* r, uint32_t cam_idx) {
    if (r == nullptr || cam_idx >= r->cameraCount) return 0;
    return r->cameraSlots[cam_idx].uploadCount;
}

}  // namespace mpviz::testing
