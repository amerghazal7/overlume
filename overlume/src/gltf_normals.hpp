// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

namespace overlume {

std::vector<uint8_t> ensure_flat_normals(std::vector<uint8_t> glb_bytes);

}
