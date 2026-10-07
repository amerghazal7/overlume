// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "overlume/api.h"

namespace overlume::testing {

// *_ne: the compare function last applied to that backdrop instance is NOT_EQUAL (stencil
// rejects splat pixels); false means ALWAYS. Filament has no getter, so the renderer records it.
struct HybridStencilState {
    bool view_stencil;
    bool bowl_ne, ground_ne, grid_ne;
};

HybridStencilState hybrid_stencil_state_for_test(overlume::VisualRenderer* r);

}
