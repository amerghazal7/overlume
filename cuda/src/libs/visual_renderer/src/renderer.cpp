// renderer.cpp — real lit clay pipeline + theme system (see docs/
// superpowers/plans/2026-08-18-visual-mode-epic1.md for the design history).
//
// Scene: one directional sun + a themed clay ground plane + a distance-faded
// reference grid, lit for real (clay.mat/clay_faded.mat, both `shadingModel:
// lit`) with a theme-driven sun + analytic 2-band IBL + distance fog.
// Geometry is baked directly in world space; later scene content (map
// elements, objects, ego) is what needs per-frame transforms.
//
// This file is internal to visual_renderer's clang/libc++ build, so (unlike
// api.h/scene.h) ordinary std:: usage is fine here — nothing here crosses
// the ABI boundary with the gcc/libstdc++ ROS node.
//
// `VisualRenderer` (plus the `Mesh`/`HeadlessEglPlatform`/`Vertex`/
// `add_mesh` helper types it needs) is extracted into renderer_internal.hpp
// so other translation units in the same library target (ego.cpp, etc) can
// see the class and reuse add_mesh. HeadlessEglPlatform's full body stays
// defined in this .cpp — only forward-declared in the header.
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
#include "point_cloud.hpp"
#include "point_cloud_test_hooks.hpp"
#include "ribbon.hpp"
#include "ribbon_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "renderer_quality_test_hooks.hpp"
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
#include "point_cloud_filamat.h"       // matc-generated; see assets/materials/point_cloud.mat

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// bluegl::bind()/unbind() (libbluegl.a, pulled in by
// cmake/GetFilament.cmake's archive glob): Filament's compiled OpenGLDriver
// calls GL through bluegl's function-pointer table, which is null until
// something calls bluegl::bind() on a thread with a current GL context
// (confirmed the hard way: SIGSEGV in queryOpenGLVersion() calling a null
// glGetString before this was added). Only the real entry points this file
// actually needs are hand-declared — at global scope (not inside namespace
// mpviz) so they name-match the real symbols the linker resolves against —
// rather than vendoring the generated BlueGL.h, which would pull in
// thousands of macro-renamed GL declarations this file never needs.
namespace bluegl {
int bind();
void unbind();
}  // namespace bluegl

// bluegl_glGetString(): the vendored BlueGL.h would `#define glGetString
// bluegl_glGetString` and declare it as a plain GL entry point (`nm` on
// libbluegl.a confirms this exact exported symbol name, no C++ mangling —
// it isn't inside namespace bluegl like bind()/unbind() above). Used once,
// by create_renderer() below (Step (h), VM-037), to log the real GL
// implementation. GL_VENDOR/GL_RENDERER/GL_VERSION are the standard GLenum
// values (stable across every GL version) — hand-declared as plain ints for
// the same header-avoidance reason as bind()/unbind() above.
extern "C" const unsigned char* bluegl_glGetString(unsigned int name);
constexpr unsigned int kGlVendor = 0x1F00;
constexpr unsigned int kGlRenderer = 0x1F01;
constexpr unsigned int kGlVersion = 0x1F02;

// Compiled-in default theme-assets dir (CMakeLists.txt target_compile_
// definitions on the visual_renderer target) — used whenever
// RenderConfig::theme_assets_dir is null.
#ifndef DEFAULT_THEME_ASSETS_DIR
#error "DEFAULT_THEME_ASSETS_DIR must be defined by CMakeLists.txt"
#endif

namespace mpviz {

using filament::math::float3;
using filament::math::float4;
using filament::math::quatf;

namespace {
float3 to_filament(const detail::Float3& c) { return float3{c.r, c.g, c.b}; }

// Fog radiance scale: two measured anchors, not a one-point extrapolation.
// light_clay (ibl.intensity 8750) needs scale 50; dark_adas (ibl.intensity
// 256000) needs scale ~1 -- i.e. palette.fog rendered at the same radiance
// as the identical palette.sky value fed to the clear color. Solving
// scale = kFogScaleReferenceValue * (kFogScaleReferenceIntensity /
// ibl.intensity)^kFogScaleExponent for both anchors gives exponent ~= 1.159
// (not 1: an inverse-linear fit only reproduces the single light_clay
// point). See push_theme_to_scene()'s fogOptions.color line for how this is
// applied, and docs/superpowers/plans/2026-08-18-visual-mode-epic1.md for
// the rejected alternatives.
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

        // Log the real GL implementation once (Step (h), VM-037): a recorded
        // render_ms budget number can't be attributed to real hardware vs. a
        // software rasterizer (Mesa llvmpipe also passes HasGpuEglDevice())
        // without this. MUST happen HERE, not in create_renderer() after
        // Engine::Builder()...build() returns: FEngine runs its OpenGL driver
        // on its own thread ("threading is enabled", confirmed in this
        // build's own log line), and the EGL context/bluegl binding above are
        // current only on THIS thread (the one createDriver() itself runs
        // on) -- calling bluegl_glGetString() from create_renderer()'s thread
        // after build() returns measured as always NULL (no current context
        // there), confirmed empirically before settling on this location.
        // NULL-guard (review 2026-09-09): glGetString can return NULL and
        // %s on NULL is UB -- print a literal "(null)" instead (keeps the
        // logging test's no-context assertion string).
        const auto gl_str = [](unsigned int n) {
            const unsigned char* s = bluegl_glGetString(n);
            return s != nullptr ? reinterpret_cast<const char*>(s) : "(null)";
        };
        std::fprintf(stderr, "[visual_renderer] GL_VENDOR: %s\n", gl_str(kGlVendor));
        std::fprintf(stderr, "[visual_renderer] GL_RENDERER: %s\n", gl_str(kGlRenderer));
        std::fprintf(stderr, "[visual_renderer] GL_VERSION: %s\n", gl_str(kGlVersion));

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
    const float step = kGridPitchM;  // the ego-following patch snaps to
    // this SAME symbol (renderer_internal.hpp) -- shared on purpose so the
    // grids can never drift apart.
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
// closed form:
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
// reflective, so this 4-coefficient analytic field is the whole IBL --
// upgrade to cmgen-prefiltered per-theme cubemaps behind the same
// IndirectLight::Builder call site if a future epic needs glossy
// reflections.
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
// direction/color/intensity, IBL, fog, clear color) into the live scene --
// the single place that decides what a `Theme` looks like on screen, called
// from create_renderer() once at startup, and apply_current_theme() every
// render_frame() call while a set_theme() transition is animating (never
// in steady state). All of `r`'s scene/view/renderer/sunEntity(component)/
// groundMaterial/gridMaterial must already exist before this is called.
//
// ponytail: every field pushed below has a real runtime setter except
// IndirectLight -- the pinned Filament 1.56.5 SDK only exposes
// setIntensity()/setRotation() at runtime, not a way to feed new SH
// coefficients into an already-built instance. Animating
// theme.ibl.sky_color/ground_color means destroying and rebuilding the
// small IndirectLight object on every call during an active transition
// (bounded to ~24-30 frames of a default 0.8s transition, never in steady
// state). Upgrade path if this ever shows up in a profile: only rebuild
// when sky_color/ground_color actually changed since the last call.
void push_theme_to_scene(VisualRenderer& r, const detail::Theme& theme) {
    r.groundMaterial->setParameter("baseColor", to_filament(theme.palette.ground));
    r.groundMaterial->setParameter("roughness", theme.material.roughness);
    r.groundMaterial->setParameter("metallic", theme.material.metallic);

    r.gridMaterial->setParameter("baseColor", to_filament(theme.grid.line_color));
    r.gridMaterial->setParameter("roughness", theme.material.roughness);
    r.gridMaterial->setParameter("metallic", theme.material.metallic);

    // laneMaterial is created eagerly (renderer_internal.hpp), so this call
    // themes it on the very first frame with map data, no separate "first
    // data" path. laneMaterialBaseColor mirrors it for
    // map_elements_test_hooks.hpp's regression guard.
    r.laneMaterial->setParameter("baseColor", to_filament(theme.palette.lane_paint));
    r.laneMaterial->setParameter("roughness", theme.material.roughness);
    r.laneMaterial->setParameter("metallic", theme.material.metallic);
    r.laneMaterialBaseColor = theme.palette.lane_paint;

    // Per-kind tints: four more clay.mat instances, same eager-creation
    // reasoning as laneMaterial above.
    r.laneCenterlineMaterial->setParameter("baseColor", to_filament(theme.palette.lane_centerline));
    r.laneCenterlineMaterial->setParameter("roughness", theme.material.roughness);
    r.laneCenterlineMaterial->setParameter("metallic", theme.material.metallic);
    r.laneCenterlineMaterialBaseColor = theme.palette.lane_centerline;

    r.laneBoundaryMaterial->setParameter("baseColor", to_filament(theme.palette.lane_boundary));
    r.laneBoundaryMaterial->setParameter("roughness", theme.material.roughness);
    r.laneBoundaryMaterial->setParameter("metallic", theme.material.metallic);
    r.laneBoundaryMaterialBaseColor = theme.palette.lane_boundary;

    r.crosswalkMaterial->setParameter("baseColor", to_filament(theme.palette.crosswalk));
    r.crosswalkMaterial->setParameter("roughness", theme.material.roughness);
    r.crosswalkMaterial->setParameter("metallic", theme.material.metallic);
    r.crosswalkMaterialBaseColor = theme.palette.crosswalk;

    r.roadMaterial->setParameter("baseColor", to_filament(theme.palette.road));
    r.roadMaterial->setParameter("roughness", theme.material.roughness);
    r.roadMaterial->setParameter("metallic", theme.material.metallic);
    r.roadMaterialBaseColor = theme.palette.road;

    // ROAD_EDGE: same eager-creation reasoning; added after the others.
    r.roadEdgeMaterial->setParameter("baseColor", to_filament(theme.palette.road_edge));
    r.roadEdgeMaterial->setParameter("roughness", theme.material.roughness);
    r.roadEdgeMaterial->setParameter("metallic", theme.material.metallic);
    r.roadEdgeMaterialBaseColor = theme.palette.road_edge;

    // egoMaterial: same eager-creation reasoning as laneMaterial, themed
    // whether or not set_ego_model() has run yet. egoMaterialBaseColor
    // mirrors it for ego_test_hooks.hpp's regression test.
    r.egoMaterial->setParameter("baseColor", to_filament(theme.palette.ego));
    r.egoMaterial->setParameter("roughness", theme.material.roughness);
    r.egoMaterial->setParameter("metallic", theme.material.metallic);
    r.egoMaterialBaseColor = theme.palette.ego;

    // Object class tints: six clay.mat instances, same eager-creation
    // reasoning as laneMaterial above. Indexed by
    // static_cast<uint8_t>(ObjectClass) (CAR..UNKNOWN).
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
    // Re-push any live per-entity staleness-fade instance's tint too -- a
    // theme switch mid-fade must animate color without resetting the fade.
    for (auto& [id, entity] : r.objectEntities) {
        if (entity.fadeInstance == nullptr) continue;
        const detail::Float3& tint = objectTints[static_cast<uint8_t>(entity.cls)];
        entity.fadeInstance->setParameter(
            "baseColor", float4{tint.r, tint.g, tint.b, entity.fadeAlpha});
        entity.fadeInstance->setParameter("roughness", theme.material.roughness);
        entity.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

    // Path ribbon roles: three role instances, same eager-creation
    // reasoning as laneMaterial above.
    //
    // BEHAVIOR is the emissive bloom hero on ribbon_emissive.mat:
    // palette.ribbon_core is the base tint, palette.ribbon_glow +
    // emissive.ribbon_strength drive the bloom-triggering emissive channel.
    // Alpha stays 1.0 here -- update_ribbons() (ribbon.cpp) is the only
    // place that overwrites baseColor.a, every render_frame() call, from
    // staleness_alpha(); update_ribbons() runs immediately after
    // apply_current_theme() in render_frame(), so the correct alpha always
    // wins by frame's end even when a push and a fade land the same tick.
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

    // GLOBAL/LOCAL are plain clay.mat instances with their own dedicated
    // palette tokens (ribbon_global/ribbon_local, theme.hpp), soft-defaulted
    // in theme.cpp's parse() to the old ribbon_core/ribbon_glow values so a
    // theme file that predates these tokens still renders identically.
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

    // Re-push any live GLOBAL/LOCAL fade instance's tint too, same
    // reasoning as the object staleness loop above. BEHAVIOR never has a
    // fadeInstance (it fades on its own material, set unconditionally
    // above).
    for (auto& slot : r.ribbonSlots) {
        if (slot.fadeInstance == nullptr) continue;
        const detail::Float3& tint = r.ribbonTint[static_cast<uint8_t>(slot.role)];
        slot.fadeInstance->setParameter("baseColor",
                                        float4{tint.r, tint.g, tint.b, slot.fadeAlpha});
        slot.fadeInstance->setParameter("roughness", theme.material.roughness);
        slot.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

    // Ground grids: two eager per-kind ground_grid.mat instances, same
    // eager-creation reasoning as laneMaterial above. theme.hpp has no
    // dedicated OGM ramp token (deliberately -- zero new theme fields):
    // freeColor reuses palette.ground (0%-occupied ground reads as the
    // ground it's shading), occupiedColor reuses palette.alert.warning
    // (100%-occupied, an obstacle-ish hazard tone already authored). `alpha`
    // is NOT set here -- it's the staleness knob update_ground_grids()
    // (ground_grid.cpp) drives every frame, same "never reset a live fade"
    // split as ribbons.
    for (auto* inst : r.groundGridMaterialInstance) {
        inst->setParameter("freeColor", to_filament(theme.palette.ground));
        inst->setParameter("occupiedColor", to_filament(theme.palette.alert.warning));
        inst->setParameter("roughness", theme.material.roughness);
        inst->setParameter("metallic", theme.material.metallic);
    }
    r.groundGridFreeColor = theme.palette.ground;
    r.groundGridOccupiedColor = theme.palette.alert.warning;

    // Alert polygons: three eager clay_translucent.mat instances, same
    // eager-creation reasoning as laneMaterial above. rgb from
    // palette.alert.{info,warning,critical}; alpha is the fixed
    // kAlertSeverityAlpha constant (renderer_internal.hpp), not a theme
    // field, reset to that constant on every push -- same "runs immediately
    // after apply_current_theme()" ordering as update_ribbons() ensures the
    // correct (possibly-faded) alpha wins by frame's end.
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
    // Re-push any live per-slot fade instance's tint too, same reasoning as
    // the object/ribbon staleness loops above.
    for (auto& slot : r.alertSlots) {
        if (slot.fadeInstance == nullptr) continue;
        const detail::Float3& tint = r.alertTint[slot.severity];
        slot.fadeInstance->setParameter("baseColor",
                                        float4{tint.r, tint.g, tint.b, slot.fadeAlpha});
        slot.fadeInstance->setParameter("roughness", theme.material.roughness);
        slot.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

    // genericMarkerMaterial is the theme-neutral default (GenericMarker::
    // color alpha==0), reusing palette.object_tints.unknown -- zero new
    // theme fields, same eager-creation reasoning as laneMaterial above.
    // Every live per-marker supplied-color instance
    // (genericMarkerColorInstances) gets its roughness/metallic re-pushed
    // here too (never its baseColor -- the marker's own supplied color) so
    // a theme switch keeps its material response consistent.
    r.genericMarkerMaterial->setParameter("baseColor",
                                           to_filament(theme.palette.object_tints.unknown));
    r.genericMarkerMaterial->setParameter("roughness", theme.material.roughness);
    r.genericMarkerMaterial->setParameter("metallic", theme.material.metallic);
    r.genericMarkerNeutralTint = theme.palette.object_tints.unknown;
    for (auto& [key, inst] : r.genericMarkerColorInstances) {
        inst->setParameter("roughness", theme.material.roughness);
        inst->setParameter("metallic", theme.material.metallic);
    }
    // Re-push any live per-slot fade instance's tint too, same reasoning as
    // the object/ribbon/alert staleness loops above.
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

    // Analytic 2-band hemisphere IBL from theme.ibl.sky_color/ground_color
    // (see sh_from_hemisphere()) -- see this function's header comment for
    // why this is a rebuild, not a setter.
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
    // FogOptions::color is in-scattering radiance (Options.h: "a good value
    // is to use the average of the ambient light"), evaluated in the same
    // pre-exposure HDR domain as the sun/IBL -- not a 0-1 display color like
    // palette.ground/palette.sky. palette.fog is authored as a 0-1
    // display-ish hue and needs to stay in that domain relative to itself;
    // only its overall magnitude needs scaling up to compete with the
    // scene's actual sun/IBL radiance -- see kFogScaleExponent/
    // kFogScaleReferenceIntensity/kFogScaleReferenceValue's own comment
    // above for the two-anchor fit.
    //
    // ponytail: dividing by camera exposure and scaling by
    // theme.ibl.intensity/pi were both tried and rejected empirically
    // before the two-anchor fit above -- exposure-division clipped both
    // themes to white (fog isn't a display color, it's inserted at the same
    // pipeline stage as the lit surface radiance), and ibl.intensity/pi
    // reproduces the ~29x lux gap between themes directly instead of
    // compensating for it. See docs/superpowers/plans/
    // 2026-08-18-visual-mode-epic1.md for the full history.
    const float fogScale = kFogScaleReferenceValue *
                            std::pow(kFogScaleReferenceIntensity / theme.ibl.intensity,
                                     kFogScaleExponent);
    fogOptions.color = to_filament(theme.palette.fog) * fogScale;
    fogOptions.density = theme.fog.density;
    // heightFalloff defaults to 1.0/m (Filament's height-stratified fog,
    // densest at `height`, default 0 -- our ground plane). Our theme schema
    // only exposes one fog knob (fog.density, a flat extinction
    // coefficient), with no height concept, so Filament's real-world
    // default silently multiplies density near ground level -- confirmed
    // empirically (a grazing camera pose erased the grid entirely). Forced
    // to 0: uniform, non-height-stratified exponential fog, the model
    // theme.fog.density actually represents.
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

// Definition of fill_tangent_frames() (renderer_internal.hpp declares it).
// Still calls quat_to_float4(), which stays anonymous-namespace-local --
// unqualified lookup from this enclosing mpviz scope still finds it, same
// TU.
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

// The one definition of the shared unit arrow (renderer_internal.hpp
// declares it). Flat shaft+head pointing +X, tail at x=0, tip at x=1,
// drawn in the XY plane.
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

// Definition of destroy_mesh() (renderer_internal.hpp declares it).
// map_elements.cpp's diff-cache eviction is the second caller this exists
// for.
void destroy_mesh(filament::Engine& engine, filament::Scene& scene, Mesh& mesh) {
    if (mesh.entity) {
        scene.remove(mesh.entity);
        engine.destroy(mesh.entity);
        utils::EntityManager::get().destroy(mesh.entity);
    }
    if (mesh.vb) engine.destroy(mesh.vb);
    if (mesh.ib) engine.destroy(mesh.ib);
    // A mesh torn down mid-fade owns a per-entity clay_translucent.mat
    // instance (Mesh::fadeInstance) that nothing else references -- destroy
    // it here too, same "every createInstance() has a matching destroy()"
    // rule objects.cpp/alert_polygons.cpp follow.
    if (mesh.fadeInstance) engine.destroy(mesh.fadeInstance);
    mesh = {};
}

// Definitions of the two functions renderer_internal.hpp declares.
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

// Namespace-scope free function so a different translation unit (ego.cpp,
// objects.cpp) can call it -- a lambda local to create_renderer() couldn't.
void add_mesh(VisualRenderer& r, Mesh& mesh, std::vector<Vertex> verts,
              std::vector<uint16_t> indices,
              filament::RenderableManager::PrimitiveType primitive,
              filament::MaterialInstance* material, bool cast_shadows, bool receive_shadows) {
    // Captured before the moves below empty `verts` -- see Mesh::
    // vertexCount's own comment (renderer_internal.hpp).
    mesh.vertexCount = static_cast<uint32_t>(verts.size());
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

    // Both RenderConfig pointers are caller-owned and borrowed only for
    // this call (api.h) -- copy into owned std::string storage first.
    // load_theme() failure (missing dir/file, malformed YAML) is non-fatal:
    // fall back to the compiled-in kFallbackTheme().
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
    // GL_VENDOR/GL_RENDERER/GL_VERSION are logged once inside
    // HeadlessEglPlatform::createDriver() above (Step (h), VM-037) -- see
    // that call site's comment for why it can't be done here.

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

    // Post-processing on: without it, setFogOptions below is a silent
    // no-op and there's no tone mapping/gamma encoding -- the themes'
    // photometric sun/IBL would clip to flat white instead of rendering lit.
    r->view->setPostProcessingEnabled(true);
    r->colorGrading = filament::ColorGrading::Builder()
                           .toneMapping(filament::ColorGrading::ToneMapping::ACES)
                           .build(*engine);
    r->view->setColorGrading(r->colorGrading);

    // Bloom: the theme YAMLs already ship emissive.ribbon_strength, which
    // presupposes bloom, so enabling it now means later ribbon geometry
    // doesn't need a renderer change to glow. BloomOptions::strength is
    // declared before `enabled` in Options.h; clang requires designated
    // initializers to follow declaration order.
    filament::BloomOptions bloom{};
    bloom.strength = 0.5f;  // tuned against goldens, not a spec number
    bloom.enabled = true;
    r->view->setBloomOptions(bloom);

    // SSAO + anti-aliasing, driven by config.quality (0=low, 1=med,
    // 2=high) -- both move pixels in committed goldens, so decided here,
    // not deferred.
    filament::AmbientOcclusionOptions ao{};
    ao.enabled = config.quality >= 1;
    ao.resolution = config.quality >= 2 ? 1.0f : 0.5f;  // Options.h: must be 0.5 or 1.0
    r->view->setAmbientOcclusionOptions(ao);

    if (config.quality >= 2) {
        // high: TAA replaces FXAA -- NONE here, TAA enabled separately.
        r->view->setAntiAliasing(filament::AntiAliasing::NONE);
        filament::TemporalAntiAliasingOptions taa{};
        taa.enabled = true;
        r->view->setTemporalAntiAliasingOptions(taa);
    } else {
        // low and medium both use FXAA; this is also Filament's own default,
        // so this call is one line of explicitness, not new behavior.
        r->view->setAntiAliasing(filament::AntiAliasing::FXAA);
    }

    // Epic 3 Task 5 (VM-032) Step 3: low-preset internal render scale --
    // spec §8 pins low to a fixed 960x540 internal target, upscaled to
    // whatever output size was requested. Filament's dynamic-resolution
    // path does this for free: pinning minScale == maxScale forces a
    // constant scale factor instead of the frame-time-driven scaling this
    // option exists for. LOW quality = bilinear blit (cheapest upscale,
    // matching the "low" preset's own budget). Medium/high leave dynamic
    // resolution off (Filament default) -- they render at the requested
    // output size directly.
    if (config.quality == 0 && config.width > 0 && config.height > 0) {
        filament::View::DynamicResolutionOptions dynRes{};
        dynRes.enabled = true;
        dynRes.homogeneousScaling = true;
        dynRes.quality = filament::QualityLevel::LOW;
        // ONE homogeneous scale for both axes (review 2026-09-09):
        // homogeneousScaling=true makes Filament force a single factor, so
        // per-axis values would silently disagree with the hook off-16:9;
        // min() keeps the internal target within 960x540 at any aspect.
        const float scale = std::min(960.0f / static_cast<float>(config.width),
                                     540.0f / static_cast<float>(config.height));
        dynRes.minScale = {scale, scale};
        dynRes.maxScale = {scale, scale};
        r->view->setDynamicResolutionOptions(dynRes);
    }

    utils::EntityManager& em = utils::EntityManager::get();

    r->cameraEntity = em.create();
    r->camera = engine->createCamera(r->cameraEntity);
    r->view->setCamera(r->camera);
    // Physically-based lighting with Filament's default getExposure() would
    // over/under-expose against these themes' much higher sun/IBL numbers.
    // Fixed exposure below, calibrated once against goldens by rendering
    // and inspecting pixels.
    //
    // ponytail: an earlier version of dark_adas/light_clay's authored lux
    // had too wide a gap for any single fixed exposure to read both
    // legibly -- fixed at the theme-data layer instead (assets/themes/
    // *.yaml sun/ibl intensity + grid line_color), not here. Regression-
    // guarded by tests/test_theme.cpp's FrameStats checks (mean band,
    // distinct luminance levels, ground-vs-sky ordering), not just
    // SSIM-against-golden, so a future exposure/lux change that re-breaks
    // legibility fails loudly.
    r->camera->setExposure(16.0f, 1.0f / 500.0f, 100.0f);

    // Sun: the LightManager component is created here (angular radius/
    // shadow-casting are creation-time-only properties this renderer never
    // changes at runtime); direction/color/intensity -- and the IBL's
    // SH/intensity, and fog/clear-color -- are all theme-driven, pushed by
    // push_theme_to_scene() immediately below and re-pushed every
    // render_frame() call while a set_theme() transition is animating
    // (apply_current_theme()). Builder() below is seeded with Filament's
    // own defaults; push_theme_to_scene() overwrites them immediately
    // after, so there is exactly one place that decides what a theme's
    // sun/ibl/fog/clear-color actually is.
    // Epic 3 Task 5 (VM-032) Step 3: shadows are the other two §8 preset
    // knobs -- disabled entirely at low (quality == 0), a 1024 shadow map
    // at medium, 2048 at high. Both are Builder-time-only LightManager
    // properties (see the comment above), so this is the same
    // config.quality dispatch as the SSAO/AA block above, just for a
    // different Filament option.
    filament::LightManager::ShadowOptions shadowOptions{};
    shadowOptions.mapSize = config.quality >= 2 ? 2048 : 1024;
    r->sunEntity = em.create();
    filament::LightManager::Builder(filament::LightManager::Type::SUN)
        .sunAngularRadius(1.9f)
        .castShadows(config.quality >= 1)
        .shadowOptions(shadowOptions)
        .build(*engine, r->sunEntity);
    r->scene->addEntity(r->sunEntity);

    // clay.mat (shared, opaque -- ground here, ego clay-box fallback + glTF
    // remap) and clay_faded.mat (grid-only, per-vertex alpha) -- see those
    // .mat files for why two materials, not one.
    r->clayMaterial = filament::Material::Builder()
                          .package(mpviz::materials::kclayFilamat, mpviz::materials::kclayFilamatSize)
                          .build(*engine);
    r->clayFadedMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kclay_fadedFilamat, mpviz::materials::kclay_fadedFilamatSize)
            .build(*engine);

    r->groundMaterial = r->clayMaterial->createInstance();
    r->gridMaterial = r->clayFadedMaterial->createInstance();
    // laneMaterial: a third clay.mat instance, tinted separately from the
    // ground -- created eagerly (not lazily on first map data) so
    // push_theme_to_scene() below themes it on the very first frame,
    // transition-free. Same material file as ground/grid; see
    // map_elements.cpp for why no new .mat is needed.
    r->laneMaterial = r->clayMaterial->createInstance();
    // Per-kind tints: four more clay.mat instances, same eager-creation
    // reasoning as laneMaterial above.
    r->laneCenterlineMaterial = r->clayMaterial->createInstance();
    r->laneBoundaryMaterial = r->clayMaterial->createInstance();
    r->crosswalkMaterial = r->clayMaterial->createInstance();
    r->roadMaterial = r->clayMaterial->createInstance();
    // ROAD_EDGE: same eager-creation reasoning.
    r->roadEdgeMaterial = r->clayMaterial->createInstance();
    // egoMaterial: a dedicated clay.mat instance for the ego, same
    // eager-creation reasoning as laneMaterial above.
    r->egoMaterial = r->clayMaterial->createInstance();

    // Object class tints: six clay.mat instances, same eager-creation
    // reasoning as laneMaterial above -- themed below by
    // push_theme_to_scene().
    r->clayTranslucentMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kclay_translucentFilamat,
                     mpviz::materials::kclay_translucentFilamatSize)
            .build(*engine);
    for (size_t i = 0; i < VisualRenderer::kObjectClassCount; ++i) {
        r->objectClassMaterial[i] = r->clayMaterial->createInstance();
    }

    // Path ribbon roles: ribbon_emissive.mat is a fourth Material (its own
    // float4 baseColor + blending: fade + an emissive channel -- clay.mat
    // can't carry either, see that .mat's header comment), built once
    // here. Three role instances, same eager-creation reasoning as
    // laneMaterial above: BEHAVIOR on ribbonEmissiveMaterial (the bloom
    // hero), GLOBAL/LOCAL on the same clayMaterial every other opaque clay
    // surface shares.
    r->ribbonEmissiveMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kribbon_emissiveFilamat,
                     mpviz::materials::kribbon_emissiveFilamatSize)
            .build(*engine);
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::BEHAVIOR)] =
        r->ribbonEmissiveMaterial->createInstance();
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)] = r->clayMaterial->createInstance();
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)] = r->clayMaterial->createInstance();

    // Ground grids: ground_grid.mat is a fifth Material (textured quad,
    // float3 ramp endpoints + a settable float alpha -- see that .mat's
    // header comment), built once here. Two per-kind instances, same
    // eager-creation reasoning as laneMaterial above.
    r->groundGridMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kground_gridFilamat,
                     mpviz::materials::kground_gridFilamatSize)
            .build(*engine);
    for (auto*& inst : r->groundGridMaterialInstance) {
        inst = r->groundGridMaterial->createInstance();
    }

    // Point clouds: point_cloud.mat is a sixth Material (UNLIT, packed
    // rgba8 vertex color, one settable float alpha -- see that .mat's
    // header comment), built once here. ONE instance for the whole layer
    // (Step 2's decision) -- no per-kind/per-role fan-out like
    // groundGridMaterialInstance/objectClassMaterial above, since a point
    // cloud's color comes entirely from its own per-point vertex data, not
    // a per-category tint.
    r->pointCloudMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::kpoint_cloudFilamat, mpviz::materials::kpoint_cloudFilamatSize)
            .build(*engine);
    r->pointCloudMaterialInstance = r->pointCloudMaterial->createInstance();
    r->pointCloudMaterialInstance->setCullingMode(filament::backend::CullingMode::NONE);

    // Alert polygons: three eager clay_translucent.mat instances (0 info/1
    // warning/2 critical) on the same clayTranslucentMaterial
    // objects.cpp/ribbon.cpp already use -- no fourth Material. Same
    // eager-creation reasoning as laneMaterial above.
    for (size_t i = 0; i < VisualRenderer::kAlertSeverityCount; ++i) {
        r->alertMaterial[i] = r->clayTranslucentMaterial->createInstance();
    }

    // Generic markers: one eager clay.mat instance, the theme-neutral
    // default -- same eager-creation reasoning as laneMaterial above.
    // Per-supplied-color instances (genericMarkerColorInstances) are
    // created lazily instead -- the set of colors isn't known until data
    // arrives.
    r->genericMarkerMaterial = r->clayMaterial->createInstance();

    // ponytail: don't chase hand-derived winding correctness for a large
    // flat quad / line list -- CullingMode::NONE sidesteps backface culling
    // so a winding mistake shows as visible-from-both-sides, not a silently
    // invisible surface. Lane geometry and object-class fallback boxes get
    // the same treatment for the same reason.
    r->groundMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->gridMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->laneMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->laneCenterlineMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->laneBoundaryMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->crosswalkMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->roadMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->roadEdgeMaterial->setCullingMode(filament::backend::CullingMode::NONE);
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

    // Pushes theme.{palette,material,sun,ibl,fog} into everything created
    // above; see push_theme_to_scene()'s own header comment for details.
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

    // The ego-following ground/grid patch needs this TransformManager
    // component -- add_mesh()/add_grid_mesh() never create one. Without it,
    // update_ground_grid_transform()'s setTransform() call resolves to a
    // null Instance and silently does nothing in release (an assert in
    // debug).
    r->engine->getTransformManager().create(r->ground.entity);
    r->engine->getTransformManager().create(r->grid.entity);

    return r;
}

void destroy_renderer(VisualRenderer* r) {
    if (r == nullptr) return;
    // Every live per-track object entity -- recycles gltfio instances to
    // their class free list / destroys procedural boxes + arrow entities +
    // path-ribbon meshes + any live fade instance. Must run before the
    // class pools' destroyAsset() calls below: destroyAsset() destroys
    // every instance of that asset outright, live-or-free-listed (gltfio's
    // documented behavior).
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
    // Every live ribbon slot -- meshes (possibly several per slot, past the
    // uint16 chunk-split ceiling) and any live GLOBAL/LOCAL fade instance.
    // Must run before clayTranslucentMaterial is destroyed below: a live
    // fadeInstance is an instance of that Material, and Filament requires
    // an instance torn down before its parent Material.
    // ribbonMaterial[role]/ribbonEmissiveMaterial are destroyed further
    // below, alongside laneMaterial/clayMaterial.
    for (auto& slot : r->ribbonSlots) {
        for (auto& mesh : slot.meshes) destroy_mesh(*r->engine, *r->scene, mesh);
        if (slot.fadeInstance) r->engine->destroy(slot.fadeInstance);
    }
    r->ribbonSlots.clear();

    // Every live alert slot -- mesh + any live per-slot fadeInstance --
    // must run before clayTranslucentMaterial is destroyed below (same
    // "instance before its Material" ordering): every alertMaterial
    // template and every live fadeInstance are both instances of it.
    for (auto& slot : r->alertSlots) {
        if (slot.mesh.vb) destroy_mesh(*r->engine, *r->scene, slot.mesh);
        if (slot.fadeInstance) r->engine->destroy(slot.fadeInstance);
    }
    r->alertSlots.clear();
    for (auto* m : r->alertMaterial) {
        if (m) r->engine->destroy(m);
    }

    // Every live generic-marker slot -- a shared-geometry entity, an own
    // mesh, or a glTF asset, any live per-slot fadeInstance, plus every
    // per-supplied-color instance (both are instances of
    // clayMaterial/clayTranslucentMaterial) -- must run before either is
    // destroyed further below (same "instance before its Material"
    // ordering). Also must run before sharedAssetLoader is destroyed
    // further below (alongside egoAsset) -- any live MESH slot's asset is
    // one of its instances.
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

    // Every live map-element fadeInstance is an instance of
    // clayTranslucentMaterial -- must run before it is destroyed below
    // (same "instance before its Material" ordering). The Mesh itself
    // (vb/ib/entity) is torn down later, alongside every other map-element
    // mesh; only the fadeInstance needs to move earlier -- destroy_mesh()
    // there sees it already null and skips it, no double-destroy.
    for (auto& [key, mesh] : r->mapElementMeshes) {
        (void)key;
        if (mesh.fadeInstance) {
            r->engine->destroy(mesh.fadeInstance);
            mesh.fadeInstance = nullptr;
        }
    }

    if (r->clayTranslucentMaterial) r->engine->destroy(r->clayTranslucentMaterial);

    // Tear down whichever path set_ego_model() actually populated. Order
    // matters -- destroyAsset() before destroying the shared loader it (and
    // every object class pool above) was created through, mirroring
    // AssetLoader.h's own documented teardown order.
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
    // Every mesh update_map_elements() ever built and never subsequently
    // evicted (the diff cache).
    for (auto& [key, mesh] : r->mapElementMeshes) {
        destroy_mesh(*r->engine, *r->scene, mesh);
    }
    r->mapElementMeshes.clear();
    if (r->laneMaterial) r->engine->destroy(r->laneMaterial);
    if (r->laneCenterlineMaterial) r->engine->destroy(r->laneCenterlineMaterial);
    if (r->laneBoundaryMaterial) r->engine->destroy(r->laneBoundaryMaterial);
    if (r->crosswalkMaterial) r->engine->destroy(r->crosswalkMaterial);
    if (r->roadMaterial) r->engine->destroy(r->roadMaterial);
    if (r->roadEdgeMaterial) r->engine->destroy(r->roadEdgeMaterial);
    if (r->egoMaterial) r->engine->destroy(r->egoMaterial);
    // All three role instances, before either Material they're instances
    // of (ribbonEmissiveMaterial/clayMaterial, just below) is destroyed.
    for (auto* m : r->ribbonMaterial) {
        if (m) r->engine->destroy(m);
    }
    if (r->ribbonEmissiveMaterial) r->engine->destroy(r->ribbonEmissiveMaterial);
    // Every live ground-grid slot -- quad mesh and its texture -- must run
    // before groundGridMaterial is destroyed below (its two instances are
    // destroyed here too, same "instance before its Material" ordering).
    for (auto& slot : r->groundGridSlots) {
        destroy_mesh(*r->engine, *r->scene, slot.quad);
        if (slot.texture) r->engine->destroy(slot.texture);
    }
    r->groundGridSlots.clear();
    for (auto* m : r->groundGridMaterialInstance) {
        if (m) r->engine->destroy(m);
    }
    if (r->groundGridMaterial) r->engine->destroy(r->groundGridMaterial);

    // Every live point-cloud slot -- meshes (possibly several per slot,
    // past the uint16 chunk-split ceiling) -- must run before
    // pointCloudMaterial is destroyed below (same "instance before its
    // Material" ordering; pointCloudMaterialInstance is destroyed there
    // too, there being only the one).
    for (auto& slot : r->pointCloudSlots) {
        for (auto& mesh : slot.meshes) destroy_mesh(*r->engine, *r->scene, mesh);
    }
    r->pointCloudSlots.clear();
    if (r->pointCloudMaterialInstance) r->engine->destroy(r->pointCloudMaterialInstance);
    if (r->pointCloudMaterial) r->engine->destroy(r->pointCloudMaterial);
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

// Recomputes the blended Theme from `sim_time_sec` (the active SceneGraph's
// own clock -- never wall-clock, per scene.h's frozen contract) on every
// render_frame() call. No-op whenever no set_theme() transition is in
// flight -- once a transition's `t` reaches 1.0 this clears
// r->theme_transition, so every call after that is this early return, not
// a from==to blend recomputed forever.
void apply_current_theme(VisualRenderer& r, double sim_time_sec) {
    if (!r.theme_transition) return;
    const detail::ThemeTransition& tr = *r.theme_transition;
    const double t = tr.duration_sec > 0.0
                          ? std::clamp((sim_time_sec - tr.start_sec) / tr.duration_sec, 0.0, 1.0)
                          : 1.0;
    const detail::Theme blended = detail::blend(tr.from, tr.to, static_cast<float>(t));
    push_theme_to_scene(r, blended);
    // Kept up to date every call a transition is in flight, so a mid-flight
    // set_theme() retarget snapshots the current blend as its new `from`,
    // not either endpoint -- no visible snap.
    r.active_theme = blended;
    if (t >= 1.0) {
        r.theme_transition.reset();
    }
}

namespace {

// Moves the ego-following ground/grid patch to `ego.position` XY,
// quantized to kGridPitchM (the same symbol build_grid_lines() draws lines
// at) -- quantizing to anything else would shift the grid by a fraction of
// a cell as the ego crosses non-multiple-of-pitch coordinates. `ego.valid
// == 0` (no TF yet) snaps the patch back to the world origin.
//
// Deliberately does NOT touch build_grid_lines()'s baked per-vertex fade
// alpha or rebuild either mesh: the fade was computed in patch-local
// coordinates (distance from the patch's own centre, at grid-build time),
// so translating the whole patch via TransformManager correctly changes
// its meaning from "fades with distance from the map origin" to "fades
// with distance from the ego" -- a rebuild-per-frame would only re-derive
// this identically, every frame.
// ponytail: 40 m follow-patch; a real streamed ground is Epic 4's EnvironmentLayer.
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
    // The ego's TransformManager transform is re-derived from the
    // last-published active() scene every call -- set_ego_model()
    // builds/loads the entity once and never touches its transform itself.
    update_ego_transform(*r, r->scene_buffer.active().ego);
    // Same re-derive-every-call split as the ego transform and theme blend
    // above -- set_scene() never touches Filament state itself.
    update_ground_grid_transform(*r, r->scene_buffer.active().ego);
    // Diffs map_elements against the cached meshes and rebuilds only what
    // changed — see map_elements.cpp.
    update_map_elements(*r, r->scene_buffer.active());
    // Diffs objects against the live entity map (acquire/update/release) —
    // see objects.cpp.
    update_objects(*r, r->scene_buffer.active());
    // Diffs paths against the live per-slot ribbon cache (keyed by slot
    // index, not role) — see ribbon.cpp.
    update_ribbons(*r, r->scene_buffer.active());
    // Diffs OGM ground grids against the live per-slot quad+texture cache
    // (keyed by slot index) — see ground_grid.cpp.
    update_ground_grids(*r, r->scene_buffer.active());
    // Diffs alert polygons against the live per-slot mesh+material cache
    // (keyed by slot index) — see alert_polygons.cpp. Overlays every
    // category above it (z-lift 0.06, the topmost layer of the z-stack).
    update_alert_polygons(*r, r->scene_buffer.active());
    // The §7 parity-guarantee fallback -- diffs generic markers against
    // the live per-slot pool (keyed by marker index) — see
    // generic_markers.cpp. Runs last: a debug/parity layer, not meant to
    // hide under anything else drawn.
    update_generic_markers(*r, r->scene_buffer.active());
    // Point clouds (Epic 3 Task 6 / VM-035): diffs point_clouds against the
    // live per-slot mesh cache (keyed by slot index) — see point_cloud.cpp.
    // Order doesn't matter for z-fighting the way map/ribbon/alert do
    // (unlit points, no shared plane to contend with), but runs last-ish
    // alongside generic markers as the other "large synthetic/sensor data"
    // category.
    update_point_clouds(*r, r->scene_buffer.active());

    r->camera->lookAt({pose.eye[0], pose.eye[1], pose.eye[2]},
                       {pose.target[0], pose.target[1], pose.target[2]},
                       {0.0, 0.0, 1.0});
    const double aspect = static_cast<double>(out.width) / static_cast<double>(out.height);
    r->camera->setProjection(pose.vfov_deg, aspect, 0.1, 500.0,
                              filament::Camera::Fov::VERTICAL);

    // Viewport/render target sizing tracks the renderer's own fixed
    // swapchain size (assumes out matches the RenderConfig used at
    // create_renderer() time; a resizable swapchain is future work).
    if (out.width != r->width || out.height != r->height) return false;

    // Filament's SwapChain readPixels() (unlike Renderer::readPixels() on a
    // Texture-backed RenderTarget) hands back rows already top-down --
    // confirmed empirically (a manual bottom-up flip produced an
    // upside-down horizon) -- so out.rgb can be the readback target
    // directly with no intermediate buffer or flip.
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

// Exactly one line, per scene.h's frozen contract comment -- no
// Filament::Engine/Scene/TransformManager call happens here. render_frame()
// reads scene_buffer.active() back out instead; set_scene() itself only
// ever touches the staging buffer.
void set_scene(VisualRenderer* r, const SceneGraph& scene) {
    r->scene_buffer.publish(scene);
}

// See scene.h's frozen contract comment. Snapshots the currently-blended
// theme (r->active_theme -- kept live by apply_current_theme() above) as
// the new transition's `from`, so retargeting mid-flight starts the new
// ease from that blend, not from either endpoint. Does not touch Filament
// state itself -- render_frame()'s apply_current_theme() is what actually
// pushes anything.
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

// See scene.h's comment.
bool theme_assets_loaded(VisualRenderer* r) {
    return r != nullptr && r->theme_assets_loaded;
}

// See scene.h's comment. r->active_theme is exactly the theme
// apply_current_theme() keeps live every render_frame() call (blended
// mid-transition, or the settled target once a transition completes) --
// read verbatim, no new state.
HudColors get_hud_colors(VisualRenderer* r) {
    if (r == nullptr) return HudColors{};
    const detail::Theme::Hud& hud = r->active_theme.hud;
    HudColors out{};
    out.text_color[0] = hud.text_color.r;
    out.text_color[1] = hud.text_color.g;
    out.text_color[2] = hud.text_color.b;
    out.accent_color[0] = hud.accent_color.r;
    out.accent_color[1] = hud.accent_color.g;
    out.accent_color[2] = hud.accent_color.b;
    out.scale = hud.scale;
    return out;
}

// See scene.h's comment. r->camera's view/projection are exactly what the
// most recent render_frame() call's lookAt()/setProjection() set (top of
// that function, above) -- read verbatim here, no separate camera state
// kept for this call.
bool project_to_screen(VisualRenderer* r, Vec3 world_point, float* out_x, float* out_y) {
    if (r == nullptr || out_x == nullptr || out_y == nullptr) return false;
    const filament::math::mat4 viewProj =
        r->camera->getProjectionMatrix() * r->camera->getViewMatrix();
    const filament::math::double4 clip =
        viewProj * filament::math::double4{world_point.x, world_point.y, world_point.z, 1.0};
    if (clip.w <= 1e-9) return false;  // behind the camera (or on the eye itself)
    const double ndcX = clip.x / clip.w;
    const double ndcY = clip.y / clip.w;
    if (ndcX < -1.0 || ndcX > 1.0 || ndcY < -1.0 || ndcY > 1.0) return false;  // outside frustum
    *out_x = static_cast<float>(ndcX * 0.5 + 0.5);
    // Flip: raw NDC +Y is up, FrameView's rows go top-to-bottom (scene.h's
    // own comment on this function states the convention).
    *out_y = static_cast<float>(1.0 - (ndcY * 0.5 + 0.5));
    return true;
}

}  // namespace mpviz

// Filament-free test introspection hooks; see map_elements_test_hooks.hpp
// for why these live here rather than map_elements.cpp -- both the
// ground/grid patch and the lane MaterialInstance's theming are wired up
// in this file.
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

// Mirrors map_elements.cpp's material_for_kind() switch field-for-field
// (that function is anonymous-namespace, not directly callable from here),
// against the *MaterialBaseColor mirrors rather than the MaterialInstance
// pointers themselves -- same "no Filament getter" reasoning as
// lane_material_base_color() above. Kept in sync with material_for_kind()
// by hand -- a kind added to one and not the other is caught the moment a
// test exercises the new kind.
mpviz::detail::Float3 map_kind_base_color(mpviz::VisualRenderer* r, mpviz::MapKind kind) {
    if (r == nullptr) return {};
    switch (kind) {
        case mpviz::MapKind::CENTERLINE:
            return r->laneCenterlineMaterialBaseColor;
        case mpviz::MapKind::LEFT_BOUNDARY:
        case mpviz::MapKind::RIGHT_BOUNDARY:
            return r->laneBoundaryMaterialBaseColor;
        case mpviz::MapKind::CROSSWALK:
            return r->crosswalkMaterialBaseColor;
        case mpviz::MapKind::ROAD_SURFACE:
            return r->roadMaterialBaseColor;
        case mpviz::MapKind::ROAD_EDGE:
            return r->roadEdgeMaterialBaseColor;
        default:
            return r->laneMaterialBaseColor;
    }
}

mpviz::detail::Float3 ego_material_base_color(mpviz::VisualRenderer* r) {
    if (r == nullptr) return {};
    return r->egoMaterialBaseColor;
}

// 0 if `r` is null.
uint64_t map_element_rebuild_count(mpviz::VisualRenderer* r) {
    return r == nullptr ? 0 : r->mapElementRebuildCount;
}

size_t map_element_mesh_count(mpviz::VisualRenderer* r) {
    return r == nullptr ? 0 : r->mapElementMeshes.size();
}

// Sums Mesh::vertexCount (add_mesh()'s own mirror of what it was called
// with -- Filament's VertexBuffer has no getter) across every mesh
// update_map_elements() currently holds. Used to distinguish a
// dot-disc-built CENTERLINE mesh (many small fan triangles) from a
// strip-built one (few) without a full-frame SSIM. 0 if `r` is null.
size_t map_element_total_vertex_count(mpviz::VisualRenderer* r) {
    if (r == nullptr) return 0;
    size_t total = 0;
    for (const auto& [key, mesh] : r->mapElementMeshes) {
        (void)key;
        total += mesh.vertexCount;
    }
    return total;
}

// Reads "the" live map-element mesh's fade state -- see
// map_elements_test_hooks.hpp for why this is only meaningful at
// map_element_mesh_count() == 1.
MapElementMaterialInfo map_element_material_info(mpviz::VisualRenderer* r) {
    MapElementMaterialInfo info;
    if (r == nullptr || r->mapElementMeshes.size() != 1) return info;
    const mpviz::Mesh& mesh = r->mapElementMeshes.begin()->second;
    info.alpha = mesh.fadeAlpha;
    if (!mesh.entity) return info;
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const auto ri = rm.getInstance(mesh.entity);
    if (!ri.isValid()) return info;
    filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    info.bound_to_translucent = mesh.fadeInstance != nullptr && bound == mesh.fadeInstance;
    return info;
}

// Epic 3 Task 5 (VM-032) Step 3 hooks -- read back the sun light's actual
// Filament-side state (both have real getters, no CPU mirror needed).
// false if `r` is null.
bool quality_shadows_enabled(mpviz::VisualRenderer* r) {
    if (r == nullptr) return false;
    filament::LightManager& lm = r->engine->getLightManager();
    return lm.isShadowCaster(lm.getInstance(r->sunEntity));
}

// 0 if `r` is null.
uint32_t quality_shadow_map_size(mpviz::VisualRenderer* r) {
    if (r == nullptr) return 0;
    filament::LightManager& lm = r->engine->getLightManager();
    return lm.getShadowOptions(lm.getInstance(r->sunEntity)).mapSize;
}

// The internal render target size setDynamicResolutionOptions() actually
// implies, derived from r->width/height (the requested output) and the
// fixed minScale==maxScale this task pins low-preset to -- View has no
// direct "current internal render size" getter, but the option struct it
// mirrors does, so this is arithmetic, not a CPU-side re-mirror of state
// Filament already owns. {0, 0} if `r` is null.
QualityRenderSize quality_internal_render_size(mpviz::VisualRenderer* r) {
    QualityRenderSize size;
    if (r == nullptr) return size;
    const filament::View::DynamicResolutionOptions opts = r->view->getDynamicResolutionOptions();
    if (!opts.enabled) {
        size.width = r->width;
        size.height = r->height;
        return size;
    }
    // Both dimensions derive from minScale.x alone (review 2026-09-09):
    // the low-preset block pins ONE homogeneous scale, and Filament's
    // homogeneousScaling forces a single factor regardless -- reading .y
    // separately would report a target the renderer never uses off-16:9.
    size.width = static_cast<uint32_t>(std::lround(r->width * opts.minScale.x));
    size.height = static_cast<uint32_t>(std::lround(r->height * opts.minScale.x));
    return size;
}

}  // namespace mpviz::testing
