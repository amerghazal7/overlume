// renderer_internal.hpp — internal-only, `-I src` visibility, not installed,
// not POD (Epic 1 Task 2 Step 7e). The `VisualRenderer` class definition,
// plus the `Mesh`/`Vertex`/`add_mesh` helper types/functions it needs by
// value or by call, extracted out of `renderer.cpp`'s former anonymous
// namespace so `src/ego.cpp` (Task 4, a separate translation unit compiled
// as part of the same `visual_renderer` library target, and so sharing that
// target's PRIVATE Filament include access) can see `class VisualRenderer`
// and reuse `add_mesh` for the ego's clay-box fallback.
//
// Pulls in <filament/...> headers, so it is deliberately NOT included by any
// tests/*.cpp — see Task 4's ego_test_hooks.hpp (Filament-free) for what
// tests use instead. Test binaries are only given `-I src` for headers like
// scene_buffer.hpp that don't reach into Filament; they are never granted
// visual_renderer's own PRIVATE Filament include dir, so a test TU
// including this header would fail to compile.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/SwapChain.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/Entity.h>

#include "scene_buffer.hpp"
#include "theme.hpp"
#include "theme_transition.hpp"

namespace mpviz {

// Full class body stays defined in renderer.cpp (no longer inside
// `namespace { ... }`, but still plain `namespace mpviz` scope there) —
// forward-declared here so VisualRenderer::platform (a bare pointer member)
// can name the type without this header needing the EGL-touching definition
// itself.
class HeadlessEglPlatform;

// One interleaved vertex: world-space position + the Filament "TANGENTS"
// quaternion that encodes the surface normal (see VertexBuffer::Builder::
// attribute()'s warning: TANGENTS is how normals are specified).
struct Vertex {
    filament::math::float3 position;
    filament::math::float4 tangentFrame;
};

filament::VertexBuffer* make_vertex_buffer(filament::Engine& engine, std::vector<Vertex> verts);
filament::IndexBuffer* make_index_buffer(filament::Engine& engine, std::vector<uint16_t> indices);

struct Mesh {
    filament::VertexBuffer* vb = nullptr;
    filament::IndexBuffer* ib = nullptr;
    utils::Entity entity;
};

// destroy_mesh stays a renderer.cpp-local (anonymous-namespace) helper — it
// is not named in the Step 7e finding and no other translation unit needs
// it, unlike add_mesh below.

class VisualRenderer {
public:
    HeadlessEglPlatform* platform = nullptr;
    filament::Engine* engine = nullptr;
    filament::SwapChain* swapChain = nullptr;
    filament::Renderer* renderer = nullptr;
    filament::Scene* scene = nullptr;
    filament::View* view = nullptr;
    filament::Camera* camera = nullptr;
    utils::Entity cameraEntity;
    utils::Entity sunEntity;
    filament::IndirectLight* ambient = nullptr;
    filament::ColorGrading* colorGrading = nullptr;
    // clay.mat (shared: ground here, ego clay-box fallback + glTF remap in
    // Task 4) and clay_faded.mat (grid-only, per-vertex alpha) — replace
    // Epic 0's unlit simple_color.mat (Task 2 / VM-011).
    filament::Material* clayMaterial = nullptr;
    filament::Material* clayFadedMaterial = nullptr;
    filament::MaterialInstance* groundMaterial = nullptr;
    filament::MaterialInstance* gridMaterial = nullptr;
    Mesh ground;
    Mesh grid;
    uint32_t width = 0;
    uint32_t height = 0;

    // Theme system (Task 2 Step 7/7b). theme_dir is the retained copy of
    // RenderConfig::theme_assets_dir (borrow-then-copy rule, see api.h) —
    // kept around for Task 3's set_theme() to re-load from later, even
    // though nothing in this task re-reads it yet. active_theme is
    // whatever create_renderer() resolved at startup (loaded file or
    // kFallbackTheme()); Task 3 is what makes it change after creation.
    std::string theme_dir;
    detail::Theme active_theme;

    // Epic 1 Task 3 (VM-014): set() when set_theme() is called, cleared
    // (nullopt) once the eased blend reaches t>=1.0 -- render_frame()'s
    // apply_current_theme() is then a no-op every subsequent call (steady
    // state), not a recomputation of an already-finished blend every frame.
    std::optional<detail::ThemeTransition> theme_transition;

    // Task 1's double-buffered scene ingest (VM-010). set_scene() (Task 2
    // Step 7) is exactly `r->scene_buffer.publish(scene);` — no Filament
    // call happens there; render_frame()/future ego-transform code reads
    // scene_buffer.active() instead.
    detail::SceneBuffer scene_buffer;
};

// Namespace-scope free function (Step 7e) — was a lambda local to
// create_renderer() capturing `&engine`/`&em`/`&r`, which meant only
// renderer.cpp's own create_renderer() could call it. A lambda can't be
// called from ego.cpp, a different translation unit, which is exactly what
// Task 4 Step 5 needs to do for the ego's clay-box fallback. `r.engine` and
// utils::EntityManager::get() replace the old `&engine`/`&em` captures (`em`
// was always just that singleton accessor, nothing stateful worth threading
// through).
void add_mesh(VisualRenderer& r, Mesh& mesh, std::vector<Vertex> verts,
              std::vector<uint16_t> indices,
              filament::RenderableManager::PrimitiveType primitive,
              filament::MaterialInstance* material, bool cast_shadows, bool receive_shadows);

}  // namespace mpviz
