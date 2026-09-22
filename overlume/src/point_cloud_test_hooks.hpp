// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>

#include "overlume/api.h"

namespace overlume::testing {

size_t point_cloud_mesh_count(overlume::VisualRenderer* r, size_t slot);

size_t point_cloud_vertex_count(overlume::VisualRenderer* r, size_t slot);

float point_cloud_material_alpha(overlume::VisualRenderer* r);

}
