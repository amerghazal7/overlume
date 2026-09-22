// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

void update_objects(VisualRenderer& r, const SceneGraph& scene);

void release_object_entity(VisualRenderer& r, ObjectEntity& entity);

}  // namespace overlume
