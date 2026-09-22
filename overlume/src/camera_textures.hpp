// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "overlume/scene.h"

#include <filament/Texture.h>

#include <cstdint>

namespace overlume {

struct CameraTextureSlot {
    filament::Texture* texture = nullptr;
    filament::Texture::InternalFormat format = filament::Texture::InternalFormat::SRGB8;
    uint32_t width = 0, height = 0;
    bool hasUploaded = false;
    uint64_t lastFrameId = 0;
    uint32_t uploadCount = 0;
    CameraExtrinsics extrinsics{};
    CameraIntrinsics intrinsics{};
    double motionDelta[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

}  // namespace overlume
