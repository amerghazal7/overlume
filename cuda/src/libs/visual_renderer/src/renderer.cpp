// renderer.cpp — Epic 1 Task 2 (VM-011): real lit clay pipeline + theme
// system, replacing Epic 0's throwaway unlit hello-frame scene (docs/
// superpowers/plans/2026-08-18-visual-mode-epic1.md, "Known Epic 0 deviation
// this epic must fix").
//
// Scene: one directional sun + a themed clay ground plane + a distance-faded
// reference grid, lit for real (clay.mat/clay_faded.mat, both `shadingModel:
// lit`) with a theme-driven sun + analytic 2-band IBL + distance fog.
// Geometry is baked directly in world space (no TransformManager use) —
// Epic 2 (SceneGraph population) and Task 4 (ego from TF) are what start
// needing per-frame transforms.
//
// This file is internal to visual_renderer's clang/libc++ build, so (unlike
// api.h/scene.h) ordinary std:: usage is fine here — nothing here crosses
// the ABI boundary with the gcc/libstdc++ ROS node.
//
// Epic 1 Task 2 Step 7e: `VisualRenderer` (plus the `Mesh`/`HeadlessEglPlatform`/
// `Vertex`/`add_mesh` helper types it needs) is extracted into
// renderer_internal.hpp so a future separate translation unit (Task 4's
// src/ego.cpp) compiled into the same library target can see the class and
// reuse add_mesh. HeadlessEglPlatform's full body stays defined in THIS .cpp
// (just no longer inside an anonymous namespace) — only forward-declared in
// the header.
#include "visual_renderer/api.h"
#include "visual_renderer/scene.h"
#include "alert_polygons.hpp"
#include "alert_polygons_test_hooks.hpp"
#include "ego.hpp"
#include "ego_test_hooks.hpp"
#include "generic_markers.hpp"
#include "generic_markers_test_hooks.hpp"
#include "ground_grid.hpp"
#include "ground_grid_test_hooks.hpp"
#include "map_elements.hpp"
#include "map_elements_test_hooks.hpp"
#include "objects.hpp"
#include "objects_test_hooks.hpp"
#include "ribbon.hpp"
#include "ribbon_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "theme.hpp"
#include "theme_transition.hpp"

#include <cmath>

#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/MaterialEnums.h>
#include <filament/MaterialInstance.h>
#include <filament/Options.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/SwapChain.h>
#include <filament/TransformManager.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <filament/Viewport.h>

#include <backend/DriverEnums.h>
#include <backend/PixelBufferDescriptor.h>
#include <backend/platforms/OpenGLPlatform.h>

#include <geometry/SurfaceOrientation.h>

#include <math/mat4.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>

#include <utils/Entity.h>
#include <utils/EntityManager.h>

#include "clay_filamat.h"        // matc-generated (CMakeLists.txt); see assets/materials/clay.mat
#include "clay_faded_filamat.h"  // matc-generated; see assets/materials/clay_faded.mat
#include "clay_translucent_filamat.h"  // matc-generated; see assets/materials/clay_translucent.mat
#include "ribbon_emissive_filamat.h"   // matc-generated; see assets/materials/ribbon_emissive.mat
#include "ground_grid_filamat.h"       // matc-generated; see assets/materials/ground_grid.mat

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// bluegl::bind()/unbind() (libbluegl.a, already pulled in by
// cmake/GetFilament.cmake's archive glob): Filament's compiled OpenGLDriver
// calls GL functions through bluegl's function-pointer table, which starts
// out null until something calls bluegl::bind() on a thread with a current
// GL context — confirmed the hard way (SIGSEGV in
// OpenGLContext::queryOpenGLVersion() calling a null glGetString before this
// was added). The real bluegl/BlueGL.h is ~thousands of lines of generated
// `#define glFoo bluegl_glFoo` macros for Filament's own .cpp files to call
// GL through; none of that applies here since this file never calls a GL
// function directly, so only the two real entry points are hand-declared —
// at global scope (NOT inside namespace mpviz) so they name-match the real
// ::bluegl::bind()/unbind() the linker resolves against.
namespace bluegl {
int bind();
void unbind();
}  // namespace bluegl

// Compiled-in default theme-assets dir (CMakeLists.txt target_compile_
// definitions on the visual_renderer target, Task 2 Step 5a) — used
// whenever RenderConfig::theme_assets_dir is null.
#ifndef DEFAULT_THEME_ASSETS_DIR
#error "DEFAULT_THEME_ASSETS_DIR must be defined by CMakeLists.txt"
#endif

namespace mpviz {

using filament::math::float3;
using filament::math::float4;
using filament::math::quatf;

namespace {
float3 to_filament(const detail::Float3& c) { return float3{c.r, c.g, c.b}; }

// See the comment at the setFogOptions() call site for what this is and how
// it was measured (epic1 Task 2 review round 6). Two real anchors, not a
// one-point fit extrapolated by an unvalidated inverse-linear guess:
// light_clay (ibl.intensity 8750) needs scale 50 to be live without
// overshooting; dark_adas (ibl.intensity 256000) needs scale ~1 -- i.e.
// palette.fog rendered at the SAME radiance as the identical palette.sky
// value fed to the clear color (spec §4.3's "one sky/fog token" intent) --
// to stop manufacturing its own fog/sky mismatch. Solving
// scale = kFogScaleReferenceValue * (kFogScaleReferenceIntensity /
// ibl.intensity)^kFogScaleExponent for both anchors simultaneously gives
// exponent ~= 1.159 (not 1 -- round 5's formula was inverse-LINEAR,
// unvalidated past the single light_clay point it reproduced exactly).
constexpr float kFogScaleExponent = 1.159f;
constexpr float kFogScaleReferenceIntensity = 8750.0f;  // light_clay's ibl.intensity
constexpr float kFogScaleReferenceValue = 50.0f;        // light_clay's proven-good flat scale
}  // namespace

// HeadlessEglPlatform — a minimal from-scratch filament::backend::
// OpenGLPlatform, standing in for Filament's own PlatformEGLHeadless.
//
// Deviation from the plan (Task 2 Step 3 says "headless EGL" as if
// Filament::backend::PlatformEGLHeadless were usable directly): it isn't.
// `nm` on the pinned 1.56.5 prebuilt Linux SDK's libbackend.a shows only
// PlatformGLX (needs a real X11 $DISPLAY — fails Step 4's "run with no
// $DISPLAY set" requirement) and PlatformNoop compiled in; PlatformEGL /
// PlatformEGLHeadless are declared in the public header but their .cpp was
// never compiled into this release's binary. Rebuilding Google's actual
// PlatformEGL.cpp from the tagged source cascades into vendoring the
// generated bluegl macro header and other internal headers the prebuilt SDK
// doesn't ship — that's "build Filament from source", the documented
// last-resort fallback in GetFilament.cmake, not a small fix.
// OpenGLPlatform's public virtual interface IS shipped, though, and the two
// bluegl entry points above are enough to make the already-compiled
// OpenGLDriver (via createDefaultDriver(), also already compiled) fully
// functional — so a small custom Platform using only raw EGL calls plus
// those two bluegl calls is sufficient, and stays genuinely headless
// (verified below with $DISPLAY unset).
//
// Epic 1 Task 2 Step 7e: moved out of the anonymous namespace to plain
// `namespace mpviz` scope (still defined here in the .cpp, not in the
// header) — renderer_internal.hpp forward-declares this same type so
// VisualRenderer::platform can name it.
class HeadlessEglPlatform : public filament::backend::OpenGLPlatform {
public:
    struct EglSwapChain : public filament::backend::Platform::SwapChain {
        EGLSurface surface = EGL_NO_SURFACE;
    };

    int getOSVersion() const noexcept override { return 0; }

    filament::backend::Driver* createDriver(void* /*sharedContext*/,
                                             const DriverConfig& driverConfig) noexcept override {
        display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display_ == EGL_NO_DISPLAY) return nullptr;
        if (eglInitialize(display_, nullptr, nullptr) != EGL_TRUE) return nullptr;
        if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) return nullptr;

        const EGLint configAttribs[] = {
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 8,
            EGL_DEPTH_SIZE, 24,
            EGL_STENCIL_SIZE, 8,
            EGL_NONE,
        };
        EGLint numConfigs = 0;
        if (eglChooseConfig(display_, configAttribs, &config_, 1, &numConfigs) != EGL_TRUE ||
            numConfigs == 0) {
            return nullptr;
        }

        const EGLint ctxAttribs[] = {
            EGL_CONTEXT_MAJOR_VERSION, 4,
            EGL_CONTEXT_MINOR_VERSION, 5,
            EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE,
        };
        context_ = eglCreateContext(display_, config_, EGL_NO_CONTEXT, ctxAttribs);
        if (context_ == EGL_NO_CONTEXT) return nullptr;

        // A throwaway 1x1 pbuffer so a surface is current while the driver
        // queries GL state during construction; real render targets are the
        // per-createSwapChain pbuffers below.
        const EGLint bootstrapAttribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        bootstrapSurface_ = eglCreatePbufferSurface(display_, config_, bootstrapAttribs);
        if (bootstrapSurface_ == EGL_NO_SURFACE) return nullptr;
        if (eglMakeCurrent(display_, bootstrapSurface_, bootstrapSurface_, context_) !=
            EGL_TRUE) {
            return nullptr;
        }
        if (bluegl::bind() != 0) return nullptr;
        blueglBound_ = true;

        return createDefaultDriver(this, nullptr, driverConfig);
    }

    filament::backend::Platform::SwapChain* createSwapChain(void* /*nativeWindow*/,
                                                              uint64_t /*flags*/) noexcept override {
        return nullptr;  // never requested: this Platform is headless-only.
    }

    filament::backend::Platform::SwapChain* createSwapChain(
        uint32_t width, uint32_t height, uint64_t /*flags*/) noexcept override {
        const EGLint pbufferAttribs[] = {
            EGL_WIDTH, static_cast<EGLint>(width),
            EGL_HEIGHT, static_cast<EGLint>(height),
            EGL_NONE,
        };
        EGLSurface surface = eglCreatePbufferSurface(display_, config_, pbufferAttribs);
        if (surface == EGL_NO_SURFACE) return nullptr;
        auto* swapChain = new EglSwapChain();
        swapChain->surface = surface;
        return swapChain;
    }

    void destroySwapChain(filament::backend::Platform::SwapChain* swapChain) noexcept override {
        auto* sc = static_cast<EglSwapChain*>(swapChain);
        if (sc->surface != EGL_NO_SURFACE) eglDestroySurface(display_, sc->surface);
        delete sc;
    }

    bool makeCurrent(ContextType /*type*/,
                      filament::backend::Platform::SwapChain* drawSwapChain,
                      filament::backend::Platform::SwapChain* readSwapChain) noexcept override {
        auto* draw = static_cast<EglSwapChain*>(drawSwapChain);
        auto* read = static_cast<EglSwapChain*>(readSwapChain);
        return eglMakeCurrent(display_, draw->surface, read->surface, context_) == EGL_TRUE;
    }

    void commit(filament::backend::Platform::SwapChain* /*swapChain*/) noexcept override {
        // ponytail: pbuffers have no compositor/consumer to present to;
        // readPixels() (not commit()) is how frames leave this Platform.
    }

    void terminate() noexcept override {
        if (display_ == EGL_NO_DISPLAY) return;
        if (blueglBound_) {
            bluegl::unbind();
            blueglBound_ = false;
        }
        eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (bootstrapSurface_ != EGL_NO_SURFACE) eglDestroySurface(display_, bootstrapSurface_);
        if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
        eglTerminate(display_);
        display_ = EGL_NO_DISPLAY;
    }

private:
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLConfig config_ = nullptr;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface bootstrapSurface_ = EGL_NO_SURFACE;
    bool blueglBound_ = false;
};

namespace {

float4 quat_to_float4(const quatf& q) { return float4{q.x, q.y, q.z, q.w}; }

// Large ground quad in the XY plane at Z=0, facing +Z (up).
// 60 m: must cover the HD-map rolling window (~50 m around the ego, live
// feedback 2026-08-20: at 20 m the last ~10 m of lanes rendered floating
// over void at the horizon). The grid's baked fade still ends at
// theme.grid.fade_end_m (40 m), so widening the patch extends GROUND under
// the far lanes without densifying the visible grid.
constexpr float kGroundHalfExtent = 60.0f;

void build_ground_plane(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    const float h = kGroundHalfExtent;
    verts = {
        {{-h, -h, 0.0f}, {}},
        {{h, -h, 0.0f}, {}},
        {{h, h, 0.0f}, {}},
        {{-h, h, 0.0f}, {}},
    };
    indices = {0, 1, 2, 0, 2, 3};
    fill_tangent_frames(verts, std::vector<float3>(4, float3{0, 0, 1}));
}

// One grid-line vertex: same position+tangentFrame layout as the shared
// `Vertex` type, plus a COLOR attribute carrying the per-vertex distance
// fade alpha (Task 2 Step 2/7). Extending JUST this dedicated vertex buffer
// — not the shared `Vertex` type in renderer_internal.hpp — keeps the
// ground/ego/every future opaque clay surface out of a COLOR-attribute
// requirement they don't need.
struct GridVertex {
    float3 position;
    float4 tangentFrame;
    float4 color;  // rgb unused by clay_faded.mat's fragment shader; only .a read
};

// Linear falloff from 1.0 (at/inside fade_start_m) to 0.0 (at/beyond
// fade_end_m) — the "grid fades with distance" AC (spec §7). Deliberately
// linear, not smoothstep: this is a per-vertex bake done once at
// grid-build time on static geometry, not a per-frame shader term, and a
// straight line has one less thing to get subtly wrong for a purely
// cosmetic distance cue.
float grid_fade_alpha(float dist_m, float fade_start_m, float fade_end_m) {
    if (fade_end_m <= fade_start_m) return dist_m <= fade_start_m ? 1.0f : 0.0f;
    const float t = (dist_m - fade_start_m) / (fade_end_m - fade_start_m);
    return 1.0f - std::clamp(t, 0.0f, 1.0f);
}

// A sparse reference grid drawn as line segments resting just above the
// ground plane (avoids z-fighting), spaced 2 units apart across the ground.
// Distance-faded per vertex from theme.grid.fade_start_m/fade_end_m
// (radial distance from the world origin — the grid is static geometry, so
// this is baked once here rather than recomputed per frame).
void build_grid_lines(std::vector<GridVertex>& verts, std::vector<uint16_t>& indices,
                       float fade_start_m, float fade_end_m) {
    const float h = kGroundHalfExtent;
    const float step = kGridPitchM;  // Task 2 Step 2: the ego-following
    // patch snaps to this SAME symbol -- see renderer_internal.hpp's
    // comment for why they must never drift apart.
    const float z = 0.001f;
    std::vector<float3> normals;
    std::vector<Vertex> plainVerts;  // reused only to drive fill_tangent_frames
    auto push = [&](float x, float y) {
        verts.push_back(GridVertex{{x, y, z}, {}, {}});
        plainVerts.push_back(Vertex{{x, y, z}, {}});
        normals.push_back(float3{0, 0, 1});
    };
    for (float x = -h; x <= h + 1e-3f; x += step) {
        push(x, -h);
        push(x, h);
    }
    for (float y = -h; y <= h + 1e-3f; y += step) {
        push(-h, y);
        push(h, y);
    }
    indices.resize(verts.size());
    for (uint16_t i = 0; i < verts.size(); ++i) indices[i] = i;

    fill_tangent_frames(plainVerts, std::vector<float3>(verts.size(), float3{0, 0, 1}));
    for (size_t i = 0; i < verts.size(); ++i) {
        verts[i].tangentFrame = plainVerts[i].tangentFrame;
        const float dist = std::sqrt(verts[i].position.x * verts[i].position.x +
                                      verts[i].position.y * verts[i].position.y);
        const float alpha = grid_fade_alpha(dist, fade_start_m, fade_end_m);
        verts[i].color = float4{1.0f, 1.0f, 1.0f, alpha};
    }
}

// Grid-only vertex buffer builder (POSITION+TANGENTS+COLOR) — parallels
// make_vertex_buffer()/make_index_buffer() in renderer_internal.hpp, but
// local to this .cpp since only the grid needs a COLOR attribute.
filament::VertexBuffer* make_grid_vertex_buffer(filament::Engine& engine,
                                                 std::vector<GridVertex> verts) {
    auto* heapVerts = new std::vector<GridVertex>(std::move(verts));
    filament::VertexBuffer* vb =
        filament::VertexBuffer::Builder()
            .vertexCount(static_cast<uint32_t>(heapVerts->size()))
            .bufferCount(1)
            .attribute(filament::VertexAttribute::POSITION, 0,
                       filament::VertexBuffer::AttributeType::FLOAT3,
                       offsetof(GridVertex, position), sizeof(GridVertex))
            .attribute(filament::VertexAttribute::TANGENTS, 0,
                       filament::VertexBuffer::AttributeType::FLOAT4,
                       offsetof(GridVertex, tangentFrame), sizeof(GridVertex))
            .attribute(filament::VertexAttribute::COLOR, 0,
                       filament::VertexBuffer::AttributeType::FLOAT4,
                       offsetof(GridVertex, color), sizeof(GridVertex))
            .build(engine);
    vb->setBufferAt(
        engine, 0,
        filament::VertexBuffer::BufferDescriptor(
            heapVerts->data(), heapVerts->size() * sizeof(GridVertex),
            [](void*, size_t, void* user) { delete static_cast<std::vector<GridVertex>*>(user); },
            heapVerts));
    return vb;
}

// Builds the grid's renderable directly (not through the shared add_mesh()
// free function in renderer_internal.hpp, which is typed for the plain
// position+tangent `Vertex` layout that Task 4's ego reuse also needs —
// the grid's COLOR-attribute vertex layout is unique to this one mesh, so
// it gets its own small builder instead of a generic-vertex-type add_mesh).
void add_grid_mesh(VisualRenderer& r, Mesh& mesh, std::vector<GridVertex> verts,
                    std::vector<uint16_t> indices, filament::MaterialInstance* material) {
    mesh.vb = make_grid_vertex_buffer(*r.engine, std::move(verts));
    mesh.ib = make_index_buffer(*r.engine, std::move(indices));
    mesh.entity = utils::EntityManager::get().create();
    filament::RenderableManager::Builder(1)
        .boundingBox({{0, 0, 0}, {kGroundHalfExtent, kGroundHalfExtent, 1.0f}})
        .geometry(0, filament::RenderableManager::PrimitiveType::LINES, mesh.vb, mesh.ib)
        .material(0, material)
        .culling(false)
        .castShadows(false)
        .receiveShadows(false)  // alpha-blended: doesn't meaningfully receive shadows (Step 7)
        .build(*r.engine, mesh.entity);
    r.scene->addEntity(mesh.entity);
}

// Analytic 2-band (L0 + L1, 4 coefficients) irradiance SH for a two-color
// hemisphere gradient environment (`sky` above, `ground` below the world
// XY plane) — the "Stupid Spherical Harmonics Tricks" hemisphere-light
// closed form (Task 2 Step 7):
//
//   c_00 = Y00 * 2*pi * (sky + ground)              [[whole-sphere Y00 projection]]
//   c_10 = Y1  * pi   * (sky - ground)               [[z-axis Y10 projection; no
//                                                        horizontal (x/y) component
//                                                        since the field only varies
//                                                        with world Z]]
//   L_lm (irradiance) = A_l * c_lm, with A0 = pi, A1 = 2*pi/3 (Ramamoorthi &
//   Hanrahan's clamped-cosine convolution constants) — folded into the
//   closed-form constants below. Verified against the exact case sky==ground
//   (must reduce to the uniform-environment irradiance E = pi*C for every
//   normal) and the cardinal case (a normal facing straight along +Z must
//   receive irradiance == pi*sky, matching an unoccluded upper hemisphere).
//
// ponytail: clay materials in both reference images are matte/non-
// reflective — a full cmgen-baked prefiltered specular cubemap buys nothing
// here (see the plan's own note); this 4-coefficient analytic field is the
// whole IBL. Upgrade path if a future epic needs glossy reflections: swap
// this for cmgen-prefiltered per-theme cubemaps behind the same
// IndirectLight::Builder call site.
void sh_from_hemisphere(const float3& sky, const float3& ground, float3 sh[4]) {
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kY0 = 0.282095f;   // sqrt(1/(4*pi))
    constexpr float kY1 = 0.488603f;   // sqrt(3/(4*pi))
    const float kTwoPiSq = 2.0f * kPi * kPi;
    sh[0] = kTwoPiSq * kY0 * (sky + ground);         // L0,0
    sh[1] = float3{0.0f, 0.0f, 0.0f};                // L1,-1 (y) — no horizontal gradient
    sh[2] = (kTwoPiSq / 3.0f) * kY1 * (sky - ground);  // L1,0  (z, world "up")
    sh[3] = float3{0.0f, 0.0f, 0.0f};                // L1,1 (x) — no horizontal gradient
}

// Pushes every theme-driven Filament token (ground/grid material params, sun
// direction/color/intensity, IBL, fog, clear color) into the live scene.
// Epic 1 Task 3 (VM-014): the single place that decides what a `Theme`
// actually looks like on screen, called from TWO sites --
// create_renderer() once at startup, and apply_current_theme() (below,
// mpviz-namespace scope) every render_frame() call WHILE a set_theme()
// transition is animating (never in steady state -- see that function). All
// of `r`'s scene/view/renderer/sunEntity(component)/groundMaterial/
// gridMaterial must already exist before this is called.
//
// ponytail ceiling, stated once here (ambient rebuild): every field pushed
// below has a real runtime setter (MaterialInstance::setParameter,
// LightManager::setDirection/setColor/setIntensity, View::setFogOptions,
// Renderer::setClearOptions) EXCEPT IndirectLight -- the pinned Filament
// 1.56.5 SDK's IndirectLight only exposes setIntensity()/setRotation() at
// runtime (confirmed against its public header), not a way to feed new SH
// coefficients into an already-built instance. Animating theme.ibl.sky_
// color/ground_color therefore means destroying and rebuilding the small
// (4-coefficient, no cubemap) IndirectLight object on every call this
// function makes DURING an active transition -- bounded to the ~24-30
// frames of a default 0.8s transition, never in steady state (this
// function isn't called at all once a transition finishes). Upgrade path if
// this ever shows up in a profile: only rebuild when ibl.sky_color/
// ground_color actually changed since the last call (they're the only
// inputs to sh_from_hemisphere), skip it otherwise.
void push_theme_to_scene(VisualRenderer& r, const detail::Theme& theme) {
    r.groundMaterial->setParameter("baseColor", to_filament(theme.palette.ground));
    r.groundMaterial->setParameter("roughness", theme.material.roughness);
    r.groundMaterial->setParameter("metallic", theme.material.metallic);

    r.gridMaterial->setParameter("baseColor", to_filament(theme.grid.line_color));
    r.gridMaterial->setParameter("roughness", theme.material.roughness);
    r.gridMaterial->setParameter("metallic", theme.material.metallic);

    // Lane paint (Epic 2 Task 2 / VM-024 Step 8a): laneMaterial is created
    // EAGERLY in create_renderer(), BEFORE this function's first call, so
    // this call (the SAME one invoked at create_renderer() time and on
    // every render_frame() while a set_theme() transition animates) is what
    // themes it on the very first frame that has any map data at all -- no
    // separate "first data" code path, and nothing waits for a set_theme()
    // call to happen first. laneMaterialBaseColor mirrors what
    // MaterialInstance itself has no getter for (map_elements_test_hooks.hpp's
    // regression guard reads this, not live GPU state).
    r.laneMaterial->setParameter("baseColor", to_filament(theme.palette.lane_paint));
    r.laneMaterial->setParameter("roughness", theme.material.roughness);
    r.laneMaterial->setParameter("metallic", theme.material.metallic);
    r.laneMaterialBaseColor = theme.palette.lane_paint;

    // Ego contrast color (user directive 2026-08-20): egoMaterial is created
    // EAGERLY in create_renderer() (same reasoning as laneMaterial just
    // above), so this call themes it from the very first frame, whether or
    // not set_ego_model() has been called yet — no separate "first ego"
    // code path needed. egoMaterialBaseColor mirrors laneMaterialBaseColor's
    // own pattern (ego_test_hooks.hpp's regression test reads this).
    r.egoMaterial->setParameter("baseColor", to_filament(theme.palette.ego));
    r.egoMaterial->setParameter("roughness", theme.material.roughness);
    r.egoMaterial->setParameter("metallic", theme.material.metallic);
    r.egoMaterialBaseColor = theme.palette.ego;

    // Object class tints (Epic 2 Task 4 / VM-022): six clay.mat instances,
    // created EAGERLY in create_renderer() (same reasoning as laneMaterial/
    // egoMaterial above), themed here on every call -- the very first frame
    // any TrackedObject arrives, whether or not set_theme() has ever run.
    // Indexed by static_cast<uint8_t>(ObjectClass) (CAR..UNKNOWN).
    const detail::Float3 objectTints[VisualRenderer::kObjectClassCount] = {
        theme.palette.object_tints.car,        theme.palette.object_tints.truck_van,
        theme.palette.object_tints.bus,        theme.palette.object_tints.pedestrian,
        theme.palette.object_tints.cyclist,    theme.palette.object_tints.unknown,
    };
    for (size_t i = 0; i < VisualRenderer::kObjectClassCount; ++i) {
        r.objectClassMaterial[i]->setParameter("baseColor", to_filament(objectTints[i]));
        r.objectClassMaterial[i]->setParameter("roughness", theme.material.roughness);
        r.objectClassMaterial[i]->setParameter("metallic", theme.material.metallic);
        r.objectClassTint[i] = objectTints[i];
    }
    // Re-push any LIVE per-entity staleness-fade instance's tint too -- a
    // theme switch mid-fade must animate color without resetting the fade
    // (see the plan's "…and the material that can actually do it").
    for (auto& [id, entity] : r.objectEntities) {
        if (entity.fadeInstance == nullptr) continue;
        const detail::Float3& tint = objectTints[static_cast<uint8_t>(entity.cls)];
        entity.fadeInstance->setParameter(
            "baseColor", float4{tint.r, tint.g, tint.b, entity.fadeAlpha});
        entity.fadeInstance->setParameter("roughness", theme.material.roughness);
        entity.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

    // Path ribbon roles (Epic 2 Task 5 / VM-023): all THREE role instances,
    // created EAGERLY in create_renderer() (same reasoning as laneMaterial/
    // objectClassMaterial above) and themed here on every call -- the very
    // first frame any PathRibbon arrives, whether or not set_theme() has
    // ever run (RibbonMaterialIsThemedOnFirstDataWithNoTransition, this
    // task's copy of Step 8a's exemplar).
    //
    // BEHAVIOR is the emissive bloom hero on ribbon_emissive.mat:
    // palette.ribbon_core is the base tint, palette.ribbon_glow +
    // emissive.ribbon_strength drive the bloom-triggering emissive channel.
    // Alpha stays 1.0 here -- update_ribbons() (ribbon.cpp) is the ONLY
    // place that overwrites baseColor.a, every render_frame() call, from
    // staleness_alpha(); a theme push mid-fade must not reset it back to
    // opaque (update_ribbons() runs immediately after apply_current_theme()
    // in render_frame(), so the correct alpha always wins by frame's end
    // even when a push and a fade land the same tick).
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::BEHAVIOR)]->setParameter(
        "baseColor", float4{theme.palette.ribbon_core.r, theme.palette.ribbon_core.g,
                            theme.palette.ribbon_core.b, 1.0f});
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::BEHAVIOR)]->setParameter(
        "roughness", theme.material.roughness);
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::BEHAVIOR)]->setParameter(
        "metallic", theme.material.metallic);
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::BEHAVIOR)]->setParameter(
        "emissiveColor", to_filament(theme.palette.ribbon_glow));
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::BEHAVIOR)]->setParameter(
        "emissiveStrength", theme.emissive.ribbon_strength);
    r.ribbonTint[static_cast<uint8_t>(PathRole::BEHAVIOR)] = theme.palette.ribbon_core;

    // GLOBAL/LOCAL are plain clay.mat instances, each in its own theme role
    // tint. User directive 2026-08-20 (ITEM 1) gave them their own dedicated
    // palette tokens (ribbon_global/ribbon_local, theme.hpp) instead of
    // reusing the BEHAVIOR/hero ribbon's ribbon_core/ribbon_glow -- the old
    // reuse was an authoring gap (both shipped themes happened to author
    // ribbon_core == ribbon_glow, so GLOBAL/LOCAL rendered the same flat
    // tint until a theme author gave them distinct values); soft-defaulted
    // in theme.cpp's parse() to exactly the old reused values, so a theme
    // file that predates these tokens still renders identically.
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)]->setParameter(
        "baseColor", to_filament(theme.palette.ribbon_global));
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)]->setParameter(
        "roughness", theme.material.roughness);
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)]->setParameter(
        "metallic", theme.material.metallic);
    r.ribbonTint[static_cast<uint8_t>(PathRole::GLOBAL)] = theme.palette.ribbon_global;

    r.ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)]->setParameter(
        "baseColor", to_filament(theme.palette.ribbon_local));
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)]->setParameter(
        "roughness", theme.material.roughness);
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)]->setParameter(
        "metallic", theme.material.metallic);
    r.ribbonTint[static_cast<uint8_t>(PathRole::LOCAL)] = theme.palette.ribbon_local;

    // Re-push any LIVE GLOBAL/LOCAL translucent fade instance's tint too --
    // same "animate color without resetting the fade" reasoning as the
    // object staleness loop just above. BEHAVIOR never has a fadeInstance
    // (it fades on its own material, set unconditionally above).
    for (auto& slot : r.ribbonSlots) {
        if (slot.fadeInstance == nullptr) continue;
        const detail::Float3& tint = r.ribbonTint[static_cast<uint8_t>(slot.role)];
        slot.fadeInstance->setParameter("baseColor",
                                        float4{tint.r, tint.g, tint.b, slot.fadeAlpha});
        slot.fadeInstance->setParameter("roughness", theme.material.roughness);
        slot.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

    // Ground grids (Epic 2 Task 6 / VM-025): two eager per-kind
    // ground_grid.mat instances, created EAGERLY in create_renderer() (same
    // "lazy creation is a known trap" reasoning as laneMaterial/
    // objectClassMaterial/ribbonMaterial above) and themed here on every
    // call -- the very first frame any GroundGridLayer arrives, whether or
    // not set_theme() has ever run. theme.hpp has no dedicated OGM ramp
    // token (the epic's "zero theme fields added" rule) -- STATED DECISION,
    // same shape as ribbon.cpp's GLOBAL/LOCAL reuse: freeColor reuses
    // palette.ground (the 0%-occupied ramp endpoint -- unoccupied ground
    // then reads as literally the ground it's shading, per the plan's own
    // "ground shading, not a floating poster" AC), occupiedColor reuses
    // palette.alert.warning (the 100%-occupied endpoint -- an obstacle-ish
    // hazard tone already authored in both shipped themes). An authoring
    // gap, not a code gap: a future theme author could give OGM its own
    // dedicated tokens: this just avoids adding new theme.hpp fields this
    // epic doesn't strictly need. `alpha` is NOT set here -- it's the
    // staleness knob, driven every frame by update_ground_grids()
    // (ground_grid.cpp) from whichever GroundGridLayer is live, exactly the
    // same "never reset a live fade" split apply_current_theme()/
    // update_ribbons() already follow for ribbons.
    for (auto* inst : r.groundGridMaterialInstance) {
        inst->setParameter("freeColor", to_filament(theme.palette.ground));
        inst->setParameter("occupiedColor", to_filament(theme.palette.alert.warning));
        inst->setParameter("roughness", theme.material.roughness);
        inst->setParameter("metallic", theme.material.metallic);
    }
    r.groundGridFreeColor = theme.palette.ground;
    r.groundGridOccupiedColor = theme.palette.alert.warning;

    // Alert polygons (Epic 2 Task 7 / VM-026): three eager
    // clay_translucent.mat instances, created EAGERLY in create_renderer()
    // (same "lazy creation is a known trap" reasoning as laneMaterial/
    // objectClassMaterial/ribbonMaterial/groundGridMaterialInstance above)
    // and themed here on every call -- the very first frame any
    // AlertPolygon arrives, whether or not set_theme() has ever run
    // (Alerts.MaterialIsThemedOnFirstDataWithNoTransition, this task's own
    // copy of Step 8a's exemplar). rgb from palette.alert.{info,warning,
    // critical}; alpha is the FIXED kAlertSeverityAlpha constant
    // (renderer_internal.hpp) -- NOT a theme field (the epic's "zero new
    // theme fields" rule) and reset to that same constant on every push,
    // exactly like ribbonMaterial[BEHAVIOR]'s alpha=1.0 reset just above:
    // update_alert_polygons() (alert_polygons.cpp) runs immediately after
    // apply_current_theme() in render_frame(), so a live per-slot
    // fadeInstance's correct (possibly-faded) alpha always wins by frame's
    // end even when a push and a fade land the same tick.
    const detail::Float3 alertTints[VisualRenderer::kAlertSeverityCount] = {
        theme.palette.alert.info,
        theme.palette.alert.warning,
        theme.palette.alert.critical,
    };
    for (size_t i = 0; i < VisualRenderer::kAlertSeverityCount; ++i) {
        r.alertMaterial[i]->setParameter(
            "baseColor", float4{alertTints[i].r, alertTints[i].g, alertTints[i].b,
                                kAlertSeverityAlpha[i]});
        r.alertMaterial[i]->setParameter("roughness", theme.material.roughness);
        r.alertMaterial[i]->setParameter("metallic", theme.material.metallic);
        r.alertTint[i] = alertTints[i];
    }
    // Re-push any LIVE per-slot translucent fade instance's tint too --
    // same "animate color without resetting the fade" reasoning as the
    // object/ribbon staleness loops above.
    for (auto& slot : r.alertSlots) {
        if (slot.fadeInstance == nullptr) continue;
        const detail::Float3& tint = r.alertTint[slot.severity];
        slot.fadeInstance->setParameter("baseColor",
                                        float4{tint.r, tint.g, tint.b, slot.fadeAlpha});
        slot.fadeInstance->setParameter("roughness", theme.material.roughness);
        slot.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

    // Generic markers (Epic 2 Task 8 / VM-027): genericMarkerMaterial is the
    // theme-neutral default (GenericMarker::color alpha==0, "no colour
    // supplied") -- reuses palette.object_tints.unknown (the existing
    // "unclassified" tint token, the epic's "zero new theme fields" rule),
    // created EAGERLY in create_renderer() and themed here on every call --
    // the very first frame any GenericMarker arrives, whether or not
    // set_theme() has ever run, same reasoning as every other per-category
    // template above. Every LIVE per-marker supplied-color instance
    // (genericMarkerColorInstances) also gets its roughness/metallic
    // re-pushed here (never its baseColor -- that's the marker's OWN
    // supplied color, not a theme token) so a theme switch keeps every
    // marker's material response consistent with the rest of the scene.
    r.genericMarkerMaterial->setParameter("baseColor",
                                           to_filament(theme.palette.object_tints.unknown));
    r.genericMarkerMaterial->setParameter("roughness", theme.material.roughness);
    r.genericMarkerMaterial->setParameter("metallic", theme.material.metallic);
    r.genericMarkerNeutralTint = theme.palette.object_tints.unknown;
    for (auto& [key, inst] : r.genericMarkerColorInstances) {
        inst->setParameter("roughness", theme.material.roughness);
        inst->setParameter("metallic", theme.material.metallic);
    }
    // Re-push any LIVE per-slot translucent fade instance's tint too --
    // same "animate color without resetting the fade" reasoning as the
    // object/ribbon/alert staleness loops above.
    for (auto& slot : r.genericMarkerSlots) {
        if (slot.fadeInstance == nullptr) continue;
        slot.fadeInstance->setParameter(
            "baseColor", float4{slot.tint.r, slot.tint.g, slot.tint.b, slot.fadeAlpha});
        slot.fadeInstance->setParameter("roughness", theme.material.roughness);
        slot.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

    filament::LightManager& lm = r.engine->getLightManager();
    const filament::LightManager::Instance sunInst = lm.getInstance(r.sunEntity);
    lm.setDirection(sunInst, to_filament(theme.sun.direction));
    lm.setColor(sunInst, to_filament(theme.sun.color));
    lm.setIntensity(sunInst, theme.sun.intensity);

    // Analytic 2-band hemisphere IBL from the theme's ibl.sky_color/
    // ibl.ground_color (see sh_from_hemisphere()'s own comment) — see this
    // function's header comment for why this is a rebuild, not a setter.
    float3 sh[4];
    sh_from_hemisphere(to_filament(theme.ibl.sky_color), to_filament(theme.ibl.ground_color), sh);
    filament::IndirectLight* newAmbient = filament::IndirectLight::Builder()
                                               .irradiance(2, sh)
                                               .intensity(theme.ibl.intensity)
                                               .build(*r.engine);
    r.scene->setIndirectLight(newAmbient);
    if (r.ambient) r.engine->destroy(r.ambient);
    r.ambient = newAmbient;

    // Fog (Filament's built-in distance fog — native feature, no custom
    // skybox mesh). `enabled` defaults to false (FogOptions' last member);
    // omitting it here would silently render neither theme's `fog:` token.
    filament::FogOptions fogOptions{};
    // FogOptions::color is in-scattering RADIANCE (Options.h: "a good value
    // is to use the average of the ambient light"), evaluated in the same
    // pre-exposure HDR domain as the sun/IBL -- not a 0-1 display color like
    // palette.ground/palette.sky. palette.fog, though, IS authored as a 0-1
    // display-ish hue (dark_adas ~0.02-0.05, light_clay ~0.8-0.92), same
    // convention as every other palette.* token, and it needs to STAY in
    // that domain relative to itself (dark_adas near-black, light_clay
    // near-white) -- only its overall magnitude was wrong. Feeding it in
    // completely unscaled made it ~5-6 orders of magnitude dimmer than this
    // scene's actual sun/IBL and thus effectively inert: measured, forcing
    // light_clay's fog to pure red moved a golden's far-field row by
    // <=2/255 (epic1 Task 2 review).
    //
    // ponytail: two more-"principled" scalings were tried and rejected
    // empirically (rendered + inspected, not just hand-derived) before
    // this one:
    //  - dividing by camera exposure (so setExposure()'s multiply cancels
    //    back out to the authored hue, the way palette.sky's clear color
    //    already reads at roughly its authored value): wrong, because fog
    //    color isn't a display color like the clear color -- it's inserted
    //    at the same pipeline stage as the sun/IBL-lit surface radiance.
    //    1/exposure (~153600x here) overshot every surface in the scene
    //    and clipped both themes to solid white, even erasing the sun's
    //    directionality (ClayMaterial.RespondsToLightDirection started
    //    failing).
    //  - theme.ibl.intensity/pi (the literal "average of the ambient
    //    light" reading): wrong scale to use PER THEME, because dark_adas/
    //    light_clay's ibl.intensity are ~29x apart *on purpose* (the
    //    lux-rebalance comments below) to land their very differently-
    //    albedo'd surfaces at similar screen brightness -- multiplying
    //    fog by ibl.intensity directly reproduces that 29x gap instead,
    //    so no single divisor made light_clay's haze visible without
    //    clipping dark_adas to white.
    // What actually works: the authored hue already encodes each theme's
    // intended relative fog brightness correctly (that's not the bug) --
    // it just needs to be loud enough, in absolute terms, to compete with
    // the surface radiance it's blending against at long range. A flat
    // multiplier preserves the authored ratio *between themes* exactly --
    // which is exactly the problem: dark_adas/light_clay's ibl.intensity
    // (the "average ambient light" Options.h itself points at) are ~29x
    // apart *on purpose* (the lux-rebalance comment below), so how loud
    // "loud enough to compete with the surface radiance" needs to be is
    // *itself* wildly different between the two themes. A flat multiplier
    // (kFogRadianceScale = 50, tuned only against light_clay) can only get
    // this right for the one theme it was tuned on -- fed dark_adas's much
    // higher ibl.intensity, the same 50x turns its correct-magnitude
    // near-black fog hue into a bright lavender wall at the horizon
    // (epic1 Task 2 review, round 4): measured on a fresh build, far-field
    // ground rows 48-59 landed ~139 luminance levels away from the sky
    // backdrop they're authored to match (palette.fog == palette.sky, spec
    // §4.3) instead of within a few.
    //
    // (rounds 5/6/7 of this history live in git log / this same comment as
    // it stood before Task 3 -- unchanged reasoning, just relocated here
    // alongside the code it explains, since Task 3 made this a
    // callable-more-than-once function instead of create_renderer()'s own
    // inline setup.)
    const float fogScale = kFogScaleReferenceValue *
                            std::pow(kFogScaleReferenceIntensity / theme.ibl.intensity,
                                     kFogScaleExponent);
    fogOptions.color = to_filament(theme.palette.fog) * fogScale;
    fogOptions.density = theme.fog.density;
    // heightFalloff defaults to 1.0/m (Filament models fog as a height-
    // stratified layer, densest at `height`, which itself defaults to 0 —
    // i.e. our own ground plane). Our theme schema only exposes ONE fog
    // knob (fog.density, a flat extinction coefficient — see theme.hpp/the
    // YAML files) with no height concept at all, so leaving Filament's
    // real-world height-fog default active silently multiplies density
    // near ground level far beyond the authored value: confirmed
    // empirically (a grazing camera pose at exactly this density erased
    // the entire grid to nothing; forcing heightFalloff to 0 — uniform,
    // non-height-stratified exponential distance fog, the model our single-
    // scalar theme.fog.density actually represents — restored the expected
    // gentle distance fade with the grid still clearly visible).
    fogOptions.heightFalloff = 0.0f;
    fogOptions.enabled = true;
    r.view->setFogOptions(fogOptions);

    filament::Renderer::ClearOptions clearOptions;
    clearOptions.clearColor = {theme.palette.sky.r, theme.palette.sky.g, theme.palette.sky.b, 1.0f};
    clearOptions.clear = true;
    r.renderer->setClearOptions(clearOptions);
}

// Payload handed to the readPixels callback: where to signal completion.
struct ReadbackState {
    std::atomic<bool> done{false};
};

void on_readback_complete(void* /*buffer*/, size_t /*size*/, void* user) {
    static_cast<ReadbackState*>(user)->done.store(true, std::memory_order_release);
}

}  // namespace

// Namespace-scope definition of fill_tangent_frames() (renderer_internal.hpp
// declares it — Epic 2 Task 2 / VM-024's promotion, see that header's
// comment): body unchanged from its former anonymous-namespace version
// (still calls quat_to_float4(), which stays anonymous-namespace-local —
// unqualified lookup from this enclosing mpviz scope still finds it, same
// TU). map_elements.cpp (Task 2) is the second caller this exists for.
void fill_tangent_frames(std::vector<Vertex>& verts, const std::vector<float3>& normals) {
    filament::geometry::SurfaceOrientation::Builder builder;
    builder.vertexCount(normals.size());
    builder.normals(normals.data());
    filament::geometry::SurfaceOrientation* orientation = builder.build();
    std::vector<quatf> quats(normals.size());
    orientation->getQuats(quats.data(), quats.size());
    delete orientation;
    for (size_t i = 0; i < verts.size(); ++i) {
        verts[i].tangentFrame = quat_to_float4(quats[i]);
    }
}

// Namespace-scope definition of destroy_mesh() (renderer_internal.hpp
// declares it — Task 2's promotion): body unchanged from its former
// anonymous-namespace version. map_elements.cpp's diff-cache eviction is
// the second caller this exists for.
// The one definition of the shared unit arrow -- see renderer_internal.hpp's
// declaration comment (promoted from objects.cpp when generic_markers.cpp
// shipped a verbatim copy, review 2026-08-20). Flat shaft+head pointing +X,
// tail at x=0, tip at x=1, drawn in the XY plane.
void build_unit_arrow(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    constexpr float kShaftHalfW = 0.06f;
    constexpr float kHeadHalfW = 0.15f;
    constexpr float kShaftEndX = 0.7f;
    const float3 p[7] = {
        {0.0f, -kShaftHalfW, 0.0f}, {kShaftEndX, -kShaftHalfW, 0.0f},
        {kShaftEndX, kShaftHalfW, 0.0f}, {0.0f, kShaftHalfW, 0.0f},
        {kShaftEndX, -kHeadHalfW, 0.0f}, {1.0f, 0.0f, 0.0f}, {kShaftEndX, kHeadHalfW, 0.0f},
    };
    for (const float3& v : p) verts.push_back(Vertex{v, {}});
    indices = {0, 1, 2, 0, 2, 3, 4, 5, 6};
    fill_tangent_frames(verts, std::vector<float3>(verts.size(), float3{0, 0, 1}));
}

void destroy_mesh(filament::Engine& engine, filament::Scene& scene, Mesh& mesh) {
    if (mesh.entity) {
        scene.remove(mesh.entity);
        engine.destroy(mesh.entity);
        utils::EntityManager::get().destroy(mesh.entity);
    }
    if (mesh.vb) engine.destroy(mesh.vb);
    if (mesh.ib) engine.destroy(mesh.ib);
    mesh = {};
}

// Namespace-scope definitions of the two functions renderer_internal.hpp
// declares (Step 7e) — bodies unchanged from their former anonymous-
// namespace versions.
filament::VertexBuffer* make_vertex_buffer(filament::Engine& engine,
                                            std::vector<Vertex> verts) {
    // BufferDescriptor only *references* client memory; Filament's driver
    // thread consumes it asynchronously, so the backing storage must outlive
    // this call. Heap-allocate and free it from the descriptor's own
    // release callback rather than the (stack-local) caller's vector.
    auto* heapVerts = new std::vector<Vertex>(std::move(verts));
    filament::VertexBuffer* vb =
        filament::VertexBuffer::Builder()
            .vertexCount(static_cast<uint32_t>(heapVerts->size()))
            .bufferCount(1)
            .attribute(filament::VertexAttribute::POSITION, 0,
                       filament::VertexBuffer::AttributeType::FLOAT3, offsetof(Vertex, position),
                       sizeof(Vertex))
            .attribute(filament::VertexAttribute::TANGENTS, 0,
                       filament::VertexBuffer::AttributeType::FLOAT4,
                       offsetof(Vertex, tangentFrame), sizeof(Vertex))
            .build(engine);
    vb->setBufferAt(
        engine, 0,
        filament::VertexBuffer::BufferDescriptor(
            heapVerts->data(), heapVerts->size() * sizeof(Vertex),
            [](void*, size_t, void* user) { delete static_cast<std::vector<Vertex>*>(user); },
            heapVerts));
    return vb;
}

filament::IndexBuffer* make_index_buffer(filament::Engine& engine,
                                          std::vector<uint16_t> indices) {
    auto* heapIndices = new std::vector<uint16_t>(std::move(indices));
    filament::IndexBuffer* ib =
        filament::IndexBuffer::Builder()
            .indexCount(static_cast<uint32_t>(heapIndices->size()))
            .bufferType(filament::IndexBuffer::IndexType::USHORT)
            .build(engine);
    ib->setBuffer(
        engine, filament::IndexBuffer::BufferDescriptor(
                    heapIndices->data(), heapIndices->size() * sizeof(uint16_t),
                    [](void*, size_t, void* user) {
                        delete static_cast<std::vector<uint16_t>*>(user);
                    },
                    heapIndices));
    return ib;
}

// Namespace-scope free function (Step 7e) — was a lambda local to
// create_renderer() capturing `&engine`/`&em`/`&r`. A lambda can't be called
// from ego.cpp, a different translation unit, which is exactly what Task 4
// Step 5 needs to do for the ego's clay-box fallback. `r.engine`/
// `utils::EntityManager::get()` replace the old `&engine`/`&em` captures
// (`em` was always just that singleton accessor, nothing stateful worth
// threading through).
void add_mesh(VisualRenderer& r, Mesh& mesh, std::vector<Vertex> verts,
              std::vector<uint16_t> indices,
              filament::RenderableManager::PrimitiveType primitive,
              filament::MaterialInstance* material, bool cast_shadows, bool receive_shadows) {
    mesh.vb = make_vertex_buffer(*r.engine, std::move(verts));
    mesh.ib = make_index_buffer(*r.engine, std::move(indices));
    mesh.entity = utils::EntityManager::get().create();
    filament::RenderableManager::Builder(1)
        .boundingBox({{0, 0, 0}, {kGroundHalfExtent, kGroundHalfExtent, 1.0f}})
        .geometry(0, primitive, mesh.vb, mesh.ib)
        .material(0, material)
        .culling(false)
        .castShadows(cast_shadows)
        .receiveShadows(receive_shadows)
        .build(*r.engine, mesh.entity);
    r.scene->addEntity(mesh.entity);
}

VisualRenderer* create_renderer(const RenderConfig& config) {
    if (config.width == 0 || config.height == 0) return nullptr;

    // Theme resolution (Step 7/7b): both RenderConfig pointers are
    // caller-owned and borrowed only for this call (api.h) — copy into
    // owned std::string storage FIRST, before doing anything else with
    // them. load_theme() failure (missing dir, missing file, malformed
    // YAML) is non-fatal (spec §9): fall back to the compiled-in
    // kFallbackTheme() rather than letting create_renderer() fail.
    const std::string themeDir =
        config.theme_assets_dir ? std::string(config.theme_assets_dir) : std::string(DEFAULT_THEME_ASSETS_DIR);
    const std::string themeName = config.initial_theme ? std::string(config.initial_theme) : std::string("dark_adas");
    std::optional<detail::Theme> loaded = detail::load_theme(themeDir, themeName);
    const detail::Theme theme = loaded ? *loaded : detail::kFallbackTheme();

    auto* platform = new HeadlessEglPlatform();
    filament::Engine* engine = filament::Engine::Builder()
                                    .backend(filament::Engine::Backend::OPENGL)
                                    .platform(platform)
                                    .build();
    if (engine == nullptr) {
        delete platform;
        return nullptr;
    }

    auto* r = new VisualRenderer();
    r->platform = platform;
    r->engine = engine;
    r->width = config.width;
    r->height = config.height;
    r->theme_dir = themeDir;
    r->active_theme = theme;
    r->theme_assets_loaded = loaded.has_value();

    r->swapChain = engine->createSwapChain(config.width, config.height,
                                            filament::SwapChain::CONFIG_READABLE);
    r->renderer = engine->createRenderer();
    r->scene = engine->createScene();
    r->view = engine->createView();
    r->view->setScene(r->scene);
    r->view->setViewport({0, 0, config.width, config.height});

    // Step 7a: post-processing back on (Epic 0 turned it off, which made
    // setFogOptions below a silent no-op and left no tone mapping / gamma
    // encoding at all — the themes' photometric sun/IBL would otherwise
    // clip to flat white instead of rendering as lit).
    r->view->setPostProcessingEnabled(true);
    r->colorGrading = filament::ColorGrading::Builder()
                           .toneMapping(filament::ColorGrading::ToneMapping::ACES)
                           .build(*engine);
    r->view->setColorGrading(r->colorGrading);

    // Bloom: the theme YAMLs already ship emissive.ribbon_strength, which
    // presupposes bloom, even though Epic 1 has no emissive ribbon geometry
    // yet — enabling it now means Epic 2's ribbons don't need a renderer
    // change to glow. BloomOptions::strength is declared before `enabled`
    // in include/filament/Options.h; clang requires designated
    // initializers to follow declaration order.
    filament::BloomOptions bloom{};
    bloom.strength = 0.5f;  // tuned against goldens, not a spec number
    bloom.enabled = true;
    r->view->setBloomOptions(bloom);

    // SSAO + anti-aliasing, driven by config.quality (0=low, 1=med, 2=high;
    // spec §8 preset table) — decided now, not deferred, because both move
    // pixels in this task's own committed goldens (Step 7a rationale).
    filament::AmbientOcclusionOptions ao{};
    ao.enabled = config.quality >= 1;
    ao.resolution = config.quality >= 2 ? 1.0f : 0.5f;  // Options.h: must be 0.5 or 1.0
    r->view->setAmbientOcclusionOptions(ao);

    if (config.quality >= 2) {
        // high: TAA replaces FXAA (spec §8) — NONE here, TAA enabled separately.
        r->view->setAntiAliasing(filament::AntiAliasing::NONE);
        filament::TemporalAntiAliasingOptions taa{};
        taa.enabled = true;
        r->view->setTemporalAntiAliasingOptions(taa);
    } else {
        // low and medium both use FXAA (spec §8); this is also Filament's own
        // default, so this call is one line of explicitness, not new behavior.
        r->view->setAntiAliasing(filament::AntiAliasing::FXAA);
    }

    utils::EntityManager& em = utils::EntityManager::get();

    r->cameraEntity = em.create();
    r->camera = engine->createCamera(r->cameraEntity);
    r->view->setCamera(r->camera);
    // Physically-based lighting with Filament's default getExposure() (tuned
    // for a normal 1-10k lux daylight scene) would over/under-expose against
    // these themes' much higher sun/IBL numbers. Fixed exposure, calibrated
    // once against the committed goldens (Step 7a) by actually rendering
    // both and inspecting pixels (not just picking a textbook "sunny 16"
    // triple and assuming it works) — see the finding below.
    //
    // ponytail: an EARLIER version of dark_adas/light_clay's authored lux
    // put an ~80x (~6.3-stop) gap in reflected radiance between the two
    // themes -- no single fixed exposure could put both at a legible
    // reading simultaneously (confirmed empirically at the time: darkening
    // enough to pull light_clay off its clip crushed dark_adas to
    // indistinguishable near-black first). Root-caused and fixed at the
    // theme-data layer instead of here (review finding, epic1 Task 2):
    // dark_adas' sun/ibl intensity raised ~5 stops and light_clay's grid
    // line_color darkened for contrast + its sun/ibl lowered ~2 stops (see
    // assets/themes/*.yaml) -- this narrowed the gap enough that the SAME
    // fixed exposure below (unchanged) now renders both themes legibly:
    // dark_adas ~43/255 mean with the sunlit ground clearly brighter than
    // the sky backdrop, light_clay ~151/255 mean with the grid visibly
    // fading with distance, neither clipped nor crushed. Regression-guarded
    // by tests/test_theme.cpp's FrameStats checks (mean band, distinct
    // luminance levels, ground-vs-sky ordering), not just SSIM-against-
    // golden, so a future exposure/lux change that re-breaks legibility
    // fails loudly instead of only "looking wrong" in a diff nobody opens.
    r->camera->setExposure(16.0f, 1.0f / 500.0f, 100.0f);

    // Sun: the LightManager COMPONENT is created here (angular radius/
    // shadow-casting are creation-time-only properties this renderer never
    // changes at runtime); its direction/color/intensity — and the IBL's
    // SH/intensity, and the fog/clear-color tokens — are all theme-DRIVEN
    // values, pushed by the shared push_theme_to_scene() helper (Epic 1
    // Task 3 / VM-014, defined above create_renderer()) immediately below,
    // and re-pushed every render_frame() call while a set_theme()
    // transition is animating (apply_current_theme(), this same .cpp,
    // below create_renderer()). Builder() below is seeded with Filament's
    // own defaults for direction/color/intensity — push_theme_to_scene()
    // overwrites them immediately after, so there is exactly one place
    // that ever decides what a theme's sun/ibl/fog/clear-color actually is.
    r->sunEntity = em.create();
    filament::LightManager::Builder(filament::LightManager::Type::SUN)
        .sunAngularRadius(1.9f)
        .castShadows(true)
        .build(*engine, r->sunEntity);
    r->scene->addEntity(r->sunEntity);

    // clay.mat (shared, opaque — ground here, ego clay-box fallback + glTF
    // remap in Task 4) and clay_faded.mat (grid-only, per-vertex alpha)
    // replace Epic 0's unlit simple_color.mat — see those .mat files' own
    // comments for why two materials, not one.
    r->clayMaterial = filament::Material::Builder()
                          .package(mpviz::materials::kclayFilamat, mpviz::materials::kclayFilamatSize)
                          .build(*engine);
    r->clayFadedMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kclay_fadedFilamat, mpviz::materials::kclay_fadedFilamatSize)
            .build(*engine);

    r->groundMaterial = r->clayMaterial->createInstance();
    r->gridMaterial = r->clayFadedMaterial->createInstance();
    // Lane paint (Epic 2 Task 2 / VM-024): a third clay.mat instance, tinted
    // separately from the ground — created EAGERLY here (not lazily on
    // first map data) so push_theme_to_scene() below themes it on the very
    // first frame, transition-free (Step 8a's regression guard). Same
    // material file as ground/grid; see map_elements.cpp for why no new
    // .mat is needed at all.
    r->laneMaterial = r->clayMaterial->createInstance();
    // Ego contrast color (user directive 2026-08-20): a dedicated clay.mat
    // instance for the ego, created EAGERLY here for the exact same reason
    // laneMaterial just above is — see renderer_internal.hpp's field
    // comment and push_theme_to_scene()'s own comment for the "lazy
    // creation is a known trap" reasoning.
    r->egoMaterial = r->clayMaterial->createInstance();

    // Object class tints (Epic 2 Task 4 / VM-022): six clay.mat instances,
    // created EAGERLY here (not lazily on first TrackedObject) for the
    // exact same "lazy creation is a known trap" reasoning as laneMaterial/
    // egoMaterial above -- themed below by push_theme_to_scene().
    r->clayTranslucentMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kclay_translucentFilamat,
                     mpviz::materials::kclay_translucentFilamatSize)
            .build(*engine);
    for (size_t i = 0; i < VisualRenderer::kObjectClassCount; ++i) {
        r->objectClassMaterial[i] = r->clayMaterial->createInstance();
    }

    // Path ribbon roles (Epic 2 Task 5 / VM-023): ribbon_emissive.mat is a
    // FOURTH Material (its own float4 baseColor + blending: fade + an
    // emissive channel -- clay.mat can't carry either, see that .mat's
    // header comment), built once here. Three role instances, created
    // EAGERLY (not lazily on first PathRibbon) for the exact same reason
    // laneMaterial/objectClassMaterial above are: BEHAVIOR on
    // ribbonEmissiveMaterial (the bloom hero), GLOBAL/LOCAL on the SAME
    // clayMaterial every other opaque clay surface shares -- themed below
    // by push_theme_to_scene().
    r->ribbonEmissiveMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kribbon_emissiveFilamat,
                     mpviz::materials::kribbon_emissiveFilamatSize)
            .build(*engine);
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::BEHAVIOR)] =
        r->ribbonEmissiveMaterial->createInstance();
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)] = r->clayMaterial->createInstance();
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)] = r->clayMaterial->createInstance();

    // Ground grids (Epic 2 Task 6 / VM-025): ground_grid.mat is a FIFTH
    // Material (textured quad, its own float4-less float3 ramp endpoints +
    // a settable float alpha -- see that .mat's header comment), built once
    // here. TWO per-kind instances, created EAGERLY (not lazily on first
    // GroundGridLayer) for the exact same reason laneMaterial/
    // objectClassMaterial/ribbonMaterial above are -- themed below by
    // push_theme_to_scene().
    r->groundGridMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kground_gridFilamat,
                     mpviz::materials::kground_gridFilamatSize)
            .build(*engine);
    for (auto*& inst : r->groundGridMaterialInstance) {
        inst = r->groundGridMaterial->createInstance();
    }

    // Alert polygons (Epic 2 Task 7 / VM-026): THREE eager
    // clay_translucent.mat instances (0 info/1 warning/2 critical) on the
    // SAME clayTranslucentMaterial objects.cpp/ribbon.cpp already built
    // above -- no fourth Material, per the plan ("no fourth material and
    // no per-topic branch"). Created EAGERLY for the exact same reason
    // laneMaterial/objectClassMaterial/ribbonMaterial/
    // groundGridMaterialInstance above are -- themed (rgb + the fixed
    // per-severity alpha constant) below by push_theme_to_scene().
    for (size_t i = 0; i < VisualRenderer::kAlertSeverityCount; ++i) {
        r->alertMaterial[i] = r->clayTranslucentMaterial->createInstance();
    }

    // Generic markers (Epic 2 Task 8 / VM-027): ONE eager clay.mat instance,
    // the theme-neutral default -- created EAGERLY for the exact same
    // reason laneMaterial/objectClassMaterial/ribbonMaterial/
    // groundGridMaterialInstance/alertMaterial above are -- themed below by
    // push_theme_to_scene(). Per-supplied-color instances
    // (genericMarkerColorInstances) are created lazily instead (see that
    // map's own comment) -- the set of colors isn't known until data
    // arrives, unlike this one shared default.
    r->genericMarkerMaterial = r->clayMaterial->createInstance();

    // ponytail: don't chase hand-derived winding correctness for a large
    // flat quad / line list — CullingMode::NONE sidesteps backface culling
    // entirely so a winding mistake shows as visible-from-both-sides, not a
    // silently invisible surface. Lane geometry (polyline ribbons, fan-
    // triangulated crosswalk polygons + hatch) gets the same treatment for
    // the same reason — map_elements.cpp derives triangle winding from
    // recorded marker point order, which this renderer has no control over.
    // Object class materials get it too (procedural-box fallback winding,
    // same reasoning).
    r->groundMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->gridMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->laneMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->egoMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    for (size_t i = 0; i < VisualRenderer::kObjectClassCount; ++i) {
        r->objectClassMaterial[i]->setCullingMode(filament::backend::CullingMode::NONE);
    }
    for (auto* m : r->ribbonMaterial) {
        m->setCullingMode(filament::backend::CullingMode::NONE);
    }
    for (auto* m : r->groundGridMaterialInstance) {
        m->setCullingMode(filament::backend::CullingMode::NONE);
    }
    for (auto* m : r->alertMaterial) {
        m->setCullingMode(filament::backend::CullingMode::NONE);
    }
    r->genericMarkerMaterial->setCullingMode(filament::backend::CullingMode::NONE);

    // Epic 1 Task 3 (VM-014): pushes theme.{palette,material,sun,ibl,fog}
    // into everything created above — the single call site create_renderer()
    // and apply_current_theme() (below) both use; see push_theme_to_scene()'s
    // own header comment (above, before create_renderer()) for the full
    // fog-color-scale history that used to live inline here.
    push_theme_to_scene(*r, theme);

    std::vector<Vertex> groundVerts;
    std::vector<uint16_t> groundIdx;
    build_ground_plane(groundVerts, groundIdx);
    add_mesh(*r, r->ground, groundVerts, groundIdx,
             filament::RenderableManager::PrimitiveType::TRIANGLES, r->groundMaterial,
             /*cast_shadows=*/false, /*receive_shadows=*/true);

    std::vector<GridVertex> gridVerts;
    std::vector<uint16_t> gridIdx;
    build_grid_lines(gridVerts, gridIdx, theme.grid.fade_start_m, theme.grid.fade_end_m);
    add_grid_mesh(*r, r->grid, gridVerts, gridIdx, r->gridMaterial);

    // Epic 2 Task 2 (VM-024) Step 2: the REAL missing piece for the
    // ego-following patch is this TransformManager component — add_mesh()/
    // add_grid_mesh() never create one (ground/grid never moved before this
    // task), so without this render_frame()'s ego-following
    // setTransform() call below would resolve to a null Instance and
    // silently do nothing (tm.getInstance() returns an invalid Instance;
    // TransformManager::setTransform() on an invalid Instance is a no-op in
    // release, an assert in debug). The entity handles themselves
    // (r->ground.entity/r->grid.entity) already existed since Epic 1 —
    // this component is the only thing that was ever missing.
    r->engine->getTransformManager().create(r->ground.entity);
    r->engine->getTransformManager().create(r->grid.entity);

    return r;
}

void destroy_renderer(VisualRenderer* r) {
    if (r == nullptr) return;
    // Epic 2 Task 4 (VM-022): every live per-track object entity -- recycles
    // gltfio instances to their class free list / destroys procedural boxes
    // + arrow entities + path-ribbon meshes + any live fade instance. Must
    // run BEFORE the class pools' destroyAsset() calls below (destroyAsset
    // destroys every instance of that asset outright, live-or-free-listed,
    // gltfio's own documented behavior -- release_object_entity()'s
    // "recycle to the free list" here is otherwise immediately moot, which
    // is fine, it's still the correct per-entity teardown either way).
    for (auto& [id, entity] : r->objectEntities) {
        release_object_entity(*r, entity);
    }
    r->objectEntities.clear();
    for (auto& [cls, pool] : r->objectClassPools) {
        if (pool.asset) r->sharedAssetLoader->destroyAsset(pool.asset);
    }
    r->objectClassPools.clear();
    if (r->sharedArrowMesh.vb) r->engine->destroy(r->sharedArrowMesh.vb);
    if (r->sharedArrowMesh.ib) r->engine->destroy(r->sharedArrowMesh.ib);
    for (auto* m : r->objectClassMaterial) {
        if (m) r->engine->destroy(m);
    }
    // Epic 2 Task 5 (VM-023): every live ribbon slot -- meshes (possibly
    // several per slot, past the uint16 chunk-split ceiling) and any live
    // GLOBAL/LOCAL translucent fade instance. MUST run before
    // clayTranslucentMaterial is destroyed just below: a live fadeInstance
    // is an INSTANCE of that Material, and Filament requires an instance
    // torn down before its parent Material (same ordering
    // release_object_entity()'s objectEntities loop above already follows).
    // ribbonMaterial[role]/ribbonEmissiveMaterial are destroyed further
    // below, alongside laneMaterial/clayMaterial.
    for (auto& slot : r->ribbonSlots) {
        for (auto& mesh : slot.meshes) destroy_mesh(*r->engine, *r->scene, mesh);
        if (slot.fadeInstance) r->engine->destroy(slot.fadeInstance);
    }
    r->ribbonSlots.clear();

    // Epic 2 Task 7 (VM-026): every live alert slot -- mesh + any live
    // per-slot fadeInstance -- MUST run before clayTranslucentMaterial is
    // destroyed just below (same "instance before its Material" ordering
    // as ribbonSlots/objectEntities above): every alertMaterial[severity]
    // TEMPLATE and every live fadeInstance are both instances of it.
    for (auto& slot : r->alertSlots) {
        if (slot.mesh.vb) destroy_mesh(*r->engine, *r->scene, slot.mesh);
        if (slot.fadeInstance) r->engine->destroy(slot.fadeInstance);
    }
    r->alertSlots.clear();
    for (auto* m : r->alertMaterial) {
        if (m) r->engine->destroy(m);
    }

    // Epic 2 Task 8 (VM-027): every live generic-marker slot -- a
    // shared-geometry entity, an own mesh, OR a glTF asset, any live
    // per-slot fadeInstance, plus every per-supplied-color instance (both
    // are instances of clayMaterial/clayTranslucentMaterial -- MUST run
    // before either is destroyed further below, same "instance before its
    // Material" ordering as ribbonSlots/alertSlots/objectEntities above).
    // Also MUST run before sharedAssetLoader is destroyed further below
    // (alongside egoAsset) -- any live MESH slot's asset is one of its
    // instances.
    for (auto& slot : r->genericMarkerSlots) {
        if (slot.fadeInstance) r->engine->destroy(slot.fadeInstance);
        if (slot.sharedGeomEntity) {
            r->scene->remove(slot.sharedGeomEntity);
            r->engine->destroy(slot.sharedGeomEntity);
            utils::EntityManager::get().destroy(slot.sharedGeomEntity);
        }
        if (slot.ownMesh.vb) destroy_mesh(*r->engine, *r->scene, slot.ownMesh);
        if (slot.meshAsset) {
            r->scene->removeEntities(slot.meshAsset->getEntities(), slot.meshAsset->getEntityCount());
            r->sharedAssetLoader->destroyAsset(slot.meshAsset);
        }
    }
    r->genericMarkerSlots.clear();
    for (auto& [key, inst] : r->genericMarkerColorInstances) {
        r->engine->destroy(inst);
    }
    r->genericMarkerColorInstances.clear();
    if (r->genericMarkerMaterial) r->engine->destroy(r->genericMarkerMaterial);
    if (r->genericCubeMesh.vb) r->engine->destroy(r->genericCubeMesh.vb);
    if (r->genericCubeMesh.ib) r->engine->destroy(r->genericCubeMesh.ib);
    if (r->genericSphereMesh.vb) r->engine->destroy(r->genericSphereMesh.vb);
    if (r->genericSphereMesh.ib) r->engine->destroy(r->genericSphereMesh.ib);
    if (r->genericCylinderMesh.vb) r->engine->destroy(r->genericCylinderMesh.vb);
    if (r->genericCylinderMesh.ib) r->engine->destroy(r->genericCylinderMesh.ib);
    if (r->genericTextMesh.vb) r->engine->destroy(r->genericTextMesh.vb);
    if (r->genericTextMesh.ib) r->engine->destroy(r->genericTextMesh.ib);

    if (r->clayTranslucentMaterial) r->engine->destroy(r->clayTranslucentMaterial);

    // Ego (Epic 1 Task 4 / VM-012): tear down whichever path set_ego_model()
    // actually populated. Order matters — destroyAsset() before destroying
    // the shared loader/materials it (and every object class pool above)
    // was created through (mirrors AssetLoader.h's own documented teardown
    // order).
    if (r->egoAsset) r->sharedAssetLoader->destroyAsset(r->egoAsset);
    if (r->sharedResourceLoader) delete r->sharedResourceLoader;
    if (r->sharedAssetLoader) filament::gltfio::AssetLoader::destroy(&r->sharedAssetLoader);
    if (r->sharedMaterialProvider) {
        r->sharedMaterialProvider->destroyMaterials();
        delete r->sharedMaterialProvider;
    }
    destroy_mesh(*r->engine, *r->scene, r->egoFallback);
    destroy_mesh(*r->engine, *r->scene, r->ground);
    destroy_mesh(*r->engine, *r->scene, r->grid);
    // Epic 2 Task 2 (VM-024): every mesh update_map_elements() ever built
    // and never subsequently evicted (Step 9's diff cache).
    for (auto& [key, mesh] : r->mapElementMeshes) {
        destroy_mesh(*r->engine, *r->scene, mesh);
    }
    r->mapElementMeshes.clear();
    if (r->laneMaterial) r->engine->destroy(r->laneMaterial);
    if (r->egoMaterial) r->engine->destroy(r->egoMaterial);
    // Epic 2 Task 5 (VM-023): all three role instances, before either
    // Material they're instances of (ribbonEmissiveMaterial/clayMaterial,
    // just below) is destroyed.
    for (auto* m : r->ribbonMaterial) {
        if (m) r->engine->destroy(m);
    }
    if (r->ribbonEmissiveMaterial) r->engine->destroy(r->ribbonEmissiveMaterial);
    // Epic 2 Task 6 (VM-025): every live ground-grid slot -- quad mesh AND
    // its texture (the most expensive leak this epic can produce) -- MUST
    // run before groundGridMaterial is destroyed just below (its two
    // instances are destroyed here too, same "instance before its Material"
    // ordering as ribbonMaterial/ribbonEmissiveMaterial just above).
    for (auto& slot : r->groundGridSlots) {
        destroy_mesh(*r->engine, *r->scene, slot.quad);
        if (slot.texture) r->engine->destroy(slot.texture);
    }
    r->groundGridSlots.clear();
    for (auto* m : r->groundGridMaterialInstance) {
        if (m) r->engine->destroy(m);
    }
    if (r->groundGridMaterial) r->engine->destroy(r->groundGridMaterial);
    if (r->groundMaterial) r->engine->destroy(r->groundMaterial);
    if (r->gridMaterial) r->engine->destroy(r->gridMaterial);
    if (r->clayMaterial) r->engine->destroy(r->clayMaterial);
    if (r->clayFadedMaterial) r->engine->destroy(r->clayFadedMaterial);
    if (r->ambient) r->engine->destroy(r->ambient);
    if (r->colorGrading) r->engine->destroy(r->colorGrading);
    if (r->sunEntity) {
        r->scene->remove(r->sunEntity);
        r->engine->destroy(r->sunEntity);
        utils::EntityManager::get().destroy(r->sunEntity);
    }
    if (r->cameraEntity) {
        r->engine->destroy(r->cameraEntity);  // destroys the Camera component
        utils::EntityManager::get().destroy(r->cameraEntity);
    }
    if (r->view) r->engine->destroy(r->view);
    if (r->scene) r->engine->destroy(r->scene);
    if (r->renderer) r->engine->destroy(r->renderer);
    if (r->swapChain) r->engine->destroy(r->swapChain);
    filament::Engine::destroy(&r->engine);
    delete r->platform;
    delete r;
}

// Epic 1 Task 3 (VM-014): recomputes the blended Theme from `sim_time_sec`
// (the active SceneGraph's own clock -- never wall-clock, per scene.h's
// frozen SceneGraph::sim_time_sec contract) on every render_frame() call.
// No-op (touches no Filament state at all) whenever no set_theme()
// transition is in flight -- once a transition's `t` reaches 1.0 this
// clears r->theme_transition, so every call after that (until the next
// set_theme()) is this early return, not a from==to blend recomputed
// forever (Step 6's "idempotent no-op blend" AC).
void apply_current_theme(VisualRenderer& r, double sim_time_sec) {
    if (!r.theme_transition) return;
    const detail::ThemeTransition& tr = *r.theme_transition;
    const double t = tr.duration_sec > 0.0
                          ? std::clamp((sim_time_sec - tr.start_sec) / tr.duration_sec, 0.0, 1.0)
                          : 1.0;
    const detail::Theme blended = detail::blend(tr.from, tr.to, static_cast<float>(t));
    push_theme_to_scene(r, blended);
    // Kept up to date every call a transition is in flight, so a mid-flight
    // set_theme() retarget (see that function, below) snapshots the CURRENT
    // blend as its new `from`, not either endpoint -- no visible snap.
    r.active_theme = blended;
    if (t >= 1.0) {
        r.theme_transition.reset();
    }
}

namespace {

// Epic 2 Task 2 (VM-024) Step 2: moves the ego-following ground/grid patch
// to `ego.position` XY, QUANTIZED to kGridPitchM (2 m — the same symbol
// build_grid_lines() draws lines at, renderer_internal.hpp), Z always 0.
// Quantizing to anything other than the real line pitch would shift the
// grid by a fraction of a cell as the ego crosses non-multiple-of-pitch
// coordinates — exactly the crawl this exists to prevent (see the plan's
// "Grid-fade interaction" note). `ego.valid == 0` (no TF yet) snaps the
// patch back to the world origin, identical to Epic 1's static placement,
// so every Epic 1 golden (shot with no ego or ego at the origin) stays
// valid byte-for-byte.
//
// Deliberately does NOT touch build_grid_lines()'s baked per-vertex fade
// alpha or rebuild either mesh: the fade was always computed in PATCH-LOCAL
// coordinates (distance from the patch's own centre, at grid-build time),
// so translating the whole patch by a TransformManager transform silently
// and correctly changes its meaning from "fades with distance from the map
// origin" to "fades with distance from the ego" — which is what spec §7
// asks for and a rebuild-per-frame would only re-derive identically at the
// cost of doing it every frame. ponytail: 40 m follow-patch; a real
// streamed ground is Epic 4's EnvironmentLayer.
void update_ground_grid_transform(VisualRenderer& r, const EgoState& ego) {
    filament::TransformManager& tm = r.engine->getTransformManager();
    float3 t{0.0f, 0.0f, 0.0f};
    if (ego.valid) {
        t.x = static_cast<float>(std::round(ego.position.x / kGridPitchM) * kGridPitchM);
        t.y = static_cast<float>(std::round(ego.position.y / kGridPitchM) * kGridPitchM);
    }
    const filament::math::mat4f xf = filament::math::mat4f::translation(t);
    const auto groundInst = tm.getInstance(r.ground.entity);
    const auto gridInst = tm.getInstance(r.grid.entity);
    if (groundInst.isValid()) tm.setTransform(groundInst, xf);
    if (gridInst.isValid()) tm.setTransform(gridInst, xf);
}

}  // namespace

bool render_frame(VisualRenderer* r, const CameraPose& pose, FrameView out) {
    if (r == nullptr || out.rgb == nullptr || out.width == 0 || out.height == 0) return false;

    apply_current_theme(*r, r->scene_buffer.active().sim_time_sec);
    // Epic 1 Task 4 (VM-012): the ego's TransformManager transform is
    // re-derived from the last-published active() scene every call, same
    // split as set_scene()/render_frame() (Task 1) and set_theme()/
    // apply_current_theme() (Task 3) — set_ego_model() builds/loads the
    // entity once and never touches its transform itself.
    update_ego_transform(*r, r->scene_buffer.active().ego);
    // Epic 2 Task 2 (VM-024) Step 2: same re-derive-every-call split as the
    // ego transform and theme blend above — set_scene() never touches
    // Filament state itself.
    update_ground_grid_transform(*r, r->scene_buffer.active().ego);
    // Epic 2 Task 2 (VM-024) Step 9: diffs map_elements against the cached
    // meshes and rebuilds only what changed — see map_elements.cpp.
    update_map_elements(*r, r->scene_buffer.active());
    // Epic 2 Task 4 (VM-022) Step 5: diffs objects against the live entity
    // map (acquire/update/release) — see objects.cpp.
    update_objects(*r, r->scene_buffer.active());
    // Epic 2 Task 5 (VM-023): diffs paths against the live per-slot ribbon
    // cache (keyed by slot index, not role -- see ribbon.cpp) — see that
    // file's own comment.
    update_ribbons(*r, r->scene_buffer.active());
    // Epic 2 Task 6 (VM-025): diffs OGM ground grids against the live
    // per-slot quad+texture cache (keyed by slot index -- see that file's
    // own comment) — see ground_grid.cpp.
    update_ground_grids(*r, r->scene_buffer.active());
    // Epic 2 Task 7 (VM-026): diffs alert polygons against the live
    // per-slot mesh+material cache (keyed by slot index -- see that file's
    // own comment) — see alert_polygons.cpp. Overlays every category
    // above it (z-lift 0.06, the topmost layer of the epic's z-stack).
    update_alert_polygons(*r, r->scene_buffer.active());
    // Epic 2 Task 8 (VM-027): the §7 parity-guarantee fallback -- diffs
    // generic markers against the live per-slot pool (keyed by marker
    // index -- see that file's own comment) — see generic_markers.cpp.
    // Runs LAST: a debug/parity layer, not meant to hide under anything
    // else this epic draws.
    update_generic_markers(*r, r->scene_buffer.active());

    r->camera->lookAt({pose.eye[0], pose.eye[1], pose.eye[2]},
                       {pose.target[0], pose.target[1], pose.target[2]},
                       {0.0, 0.0, 1.0});
    const double aspect = static_cast<double>(out.width) / static_cast<double>(out.height);
    r->camera->setProjection(pose.vfov_deg, aspect, 0.1, 500.0,
                              filament::Camera::Fov::VERTICAL);

    // Viewport/render target sizing tracks the renderer's own fixed
    // swapchain size (Task 2 assumes out matches the RenderConfig used at
    // create_renderer() time; later epics can resize the swapchain instead).
    if (out.width != r->width || out.height != r->height) return false;

    // Filament's SwapChain readPixels() (unlike Renderer::readPixels() on a
    // Texture-backed RenderTarget, per that overload's own doc note) hands
    // back rows already top-down — confirmed empirically (a manual bottom-up
    // flip here produced an upside-down horizon) — so out.rgb can be the
    // readback target directly with no intermediate buffer or flip.
    const size_t byteCount = static_cast<size_t>(out.width) * out.height * 3;
    ReadbackState state;
    filament::backend::PixelBufferDescriptor buffer(
        out.rgb, byteCount, filament::backend::PixelBufferDescriptor::PixelDataFormat::RGB,
        filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE, on_readback_complete,
        &state);

    if (r->renderer->beginFrame(r->swapChain)) {
        r->renderer->render(r->view);
        r->renderer->readPixels(0, 0, out.width, out.height, std::move(buffer));
        r->renderer->endFrame();
    } else {
        return false;
    }

    r->engine->flushAndWait();
    // ponytail: readPixels' completion callback is dispatched by
    // pumpMessageQueues() on the calling thread, not by flushAndWait()
    // itself; poll with a small bound instead of assuming one pump
    // suffices. Upgrade to Fence-based waiting if this ever proves flaky.
    for (int i = 0; i < 200 && !state.done.load(std::memory_order_acquire); ++i) {
        r->engine->pumpMessageQueues();
        if (!state.done.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    if (!state.done.load(std::memory_order_acquire)) return false;

    return true;
}

// Task 2 Step 7: exactly one line, per scene.h's frozen contract comment —
// no Filament::Engine/Scene/TransformManager call happens here. render_frame
// (this task's sun/IBL/fog/theme-driven material params) and the future
// ego-transform hook (Task 4) both read scene_buffer.active() back out
// instead — set_scene() itself only ever touches the staging buffer.
void set_scene(VisualRenderer* r, const SceneGraph& scene) {
    r->scene_buffer.publish(scene);
}

// Epic 1 Task 3 (VM-014): see scene.h's frozen contract comment. Snapshots
// the CURRENTLY-blended theme (r->active_theme -- kept live by
// apply_current_theme() above on every render_frame() call while a
// transition is in flight, and left holding the final settled theme once
// one finishes) as the new transition's `from`, so retargeting mid-flight
// (calling this again before the previous transition finishes) starts the
// new ease from that blend, not from either endpoint. Does NOT touch
// Filament state itself -- render_frame()'s apply_current_theme() is what
// actually pushes anything, on whichever thread owns the Engine, same
// split as set_scene()/render_frame() (Task 1).
bool set_theme(VisualRenderer* r, const char* theme_name, double at_sec, double transition_sec) {
    if (r == nullptr || theme_name == nullptr) return false;
    const std::optional<detail::Theme> target = detail::load_theme(r->theme_dir, theme_name);
    if (!target) return false;  // unknown theme_name -- active theme unchanged
    r->theme_transition = detail::ThemeTransition{
        r->active_theme,
        *target,
        at_sec,
        transition_sec > 0.0 ? transition_sec : 0.8,
    };
    return true;
}

// Gate-review addition (2026-08-20, spec §9 minor) — see scene.h's comment.
bool theme_assets_loaded(VisualRenderer* r) {
    return r != nullptr && r->theme_assets_loaded;
}

}  // namespace mpviz

// Epic 2 Task 2 (VM-024): Filament-free test introspection hooks (see
// map_elements_test_hooks.hpp's own comment for why these live here rather
// than map_elements.cpp — both the ground/grid patch and the lane
// MaterialInstance's theming are wired up in THIS file).
namespace mpviz::testing {

mpviz::Vec3 ground_patch_centre(mpviz::VisualRenderer* r) {
    if (r == nullptr || !r->ground.entity) return {0.0, 0.0, 0.0};
    filament::TransformManager& tm = r->engine->getTransformManager();
    const auto inst = tm.getInstance(r->ground.entity);
    if (!inst.isValid()) return {0.0, 0.0, 0.0};
    const filament::math::mat4f xf = tm.getTransform(inst);
    const filament::math::float3 t = xf[3].xyz;  // translation column
    return {static_cast<double>(t.x), static_cast<double>(t.y), static_cast<double>(t.z)};
}

mpviz::detail::Float3 lane_material_base_color(mpviz::VisualRenderer* r) {
    if (r == nullptr) return {};
    return r->laneMaterialBaseColor;
}

mpviz::detail::Float3 ego_material_base_color(mpviz::VisualRenderer* r) {
    if (r == nullptr) return {};
    return r->egoMaterialBaseColor;
}

}  // namespace mpviz::testing
