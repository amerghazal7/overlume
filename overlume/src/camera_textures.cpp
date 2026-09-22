// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "bowl.hpp"
#include "camera_textures.hpp"
#include "camera_textures_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include <filament/Engine.h>
#include <filament/Texture.h>

#include <backend/PixelBufferDescriptor.h>

#include <cstring>
#include <vector>

namespace overlume {

namespace {

filament::Texture::InternalFormat choose_camera_format(filament::Engine& engine) {
    return filament::Texture::isTextureFormatSupported(engine,
                                                       filament::Texture::InternalFormat::SRGB8)
               ? filament::Texture::InternalFormat::SRGB8
               : filament::Texture::InternalFormat::SRGB8_A8;
}

filament::Texture* build_camera_texture(filament::Engine& engine, uint32_t w, uint32_t h,
                                        filament::Texture::InternalFormat format) {
    return filament::Texture::Builder()
        .width(w)
        .height(h)
        .levels(1)
        .format(format)
        .sampler(filament::Texture::Sampler::SAMPLER_2D)
        .build(engine);
}

void upload_camera_frame(filament::Engine& engine, filament::Texture* tex, const uint8_t* rgb,
                         uint32_t width, uint32_t height, void (*release)(void*, size_t, void*),
                         void* user) {
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

}

bool set_bowl_config(VisualRenderer* r, const BowlConfig& cfg) {
    if (r == nullptr) return false;
    if (cfg.camera_count == 0 || cfg.camera_count > kMaxBowlCameras) return false;

    for (auto& slot : r->cameraSlots) {
        if (slot.texture != nullptr) r->engine->destroy(slot.texture);
        slot = CameraTextureSlot{};
    }
    const filament::Texture::InternalFormat format = choose_camera_format(*r->engine);
    for (uint32_t i = 0; i < cfg.camera_count; ++i) {
        CameraTextureSlot& slot = r->cameraSlots[i];
        slot.width = cfg.cam_width[i];
        slot.height = cfg.cam_height[i];
        slot.extrinsics = cfg.extrinsics[i];
        slot.intrinsics = cfg.intrinsics[i];
        slot.format = format;
        slot.texture = build_camera_texture(*r->engine, slot.width, slot.height, format);
    }
    r->cameraCount = cfg.camera_count;
    build_bowl(*r, cfg);
    return true;
}

bool set_bowl_visible(VisualRenderer* r, bool visible) {
    if (r == nullptr || r->cameraCount == 0) return false;
    r->bowlVisible = visible;
    return true;
}

bool set_self_view_masks(VisualRenderer* r, bool enabled) {
    if (r == nullptr) return false;
    r->selfViewMasksEnabled = enabled;
    return true;
}

bool set_camera_motion_delta(VisualRenderer* r, uint32_t cam_idx,
                             const double delta_4x4_row_major[16]) {
    if (r == nullptr || cam_idx >= r->cameraCount) return false;
    std::memcpy(r->cameraSlots[cam_idx].motionDelta, delta_4x4_row_major, sizeof(double) * 16);
    return true;
}

bool set_camera_frame(VisualRenderer* r, uint32_t cam_idx, const uint8_t* rgb, uint32_t width,
                      uint32_t height, uint64_t frame_id, void (*release)(void*, size_t, void*),
                      void* user) {
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

}

namespace overlume::testing {

uint64_t camera_frame_upload_count(overlume::VisualRenderer* r, uint32_t cam_idx) {
    if (r == nullptr || cam_idx >= r->cameraCount) return 0;
    return r->cameraSlots[cam_idx].uploadCount;
}

void camera_motion_delta(overlume::VisualRenderer* r, uint32_t cam_idx, double out[16]) {
    if (r == nullptr || cam_idx >= r->cameraCount) return;
    std::memcpy(out, r->cameraSlots[cam_idx].motionDelta, sizeof(double) * 16);
}

}
