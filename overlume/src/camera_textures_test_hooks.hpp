// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>

#include "overlume/api.h"

namespace overlume::testing {

uint64_t camera_frame_upload_count(overlume::VisualRenderer* r, uint32_t cam_idx);

void camera_motion_delta(overlume::VisualRenderer* r, uint32_t cam_idx, double out[16]);

}  // namespace overlume::testing
