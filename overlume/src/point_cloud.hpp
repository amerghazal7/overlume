// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

// Shared by the point-cloud rows and the hybrid splat layer.
struct PointVertex {
    filament::math::float3 position;
    uint32_t rgba;
};

filament::VertexBuffer* make_point_vertex_buffer(filament::Engine& engine,
                                                 std::vector<PointVertex> verts);

void update_point_clouds(VisualRenderer& r, const SceneGraph& scene);

}
