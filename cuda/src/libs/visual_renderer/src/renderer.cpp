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
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <filament/Viewport.h>

#include <backend/DriverEnums.h>
#include <backend/PixelBufferDescriptor.h>
#include <backend/platforms/OpenGLPlatform.h>

#include <geometry/SurfaceOrientation.h>

#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>

#include <utils/Entity.h>
#include <utils/EntityManager.h>

#include "clay_filamat.h"        // matc-generated (CMakeLists.txt); see assets/materials/clay.mat
#include "clay_faded_filamat.h"  // matc-generated; see assets/materials/clay_faded.mat

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

// Builds per-vertex orientation quaternions from flat-shaded normals
// (SurfaceOrientation's "normals only" mode) and writes them into `verts`.
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

// Large ground quad in the XY plane at Z=0, facing +Z (up).
constexpr float kGroundHalfExtent = 20.0f;

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
    const float step = 2.0f;
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

void destroy_mesh(filament::Engine& engine, filament::Scene& scene, Mesh& mesh) {
    if (mesh.entity) {
        scene.remove(mesh.entity);
        engine.destroy(mesh.entity);
        utils::EntityManager::get().destroy(mesh.entity);
    }
    if (mesh.vb) engine.destroy(mesh.vb);
    if (mesh.ib) engine.destroy(mesh.ib);
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

    // ponytail: don't chase hand-derived winding correctness for a large
    // flat quad / line list — CullingMode::NONE sidesteps backface culling
    // entirely so a winding mistake shows as visible-from-both-sides, not a
    // silently invisible surface.
    r->groundMaterial->setCullingMode(filament::backend::CullingMode::NONE);
    r->gridMaterial->setCullingMode(filament::backend::CullingMode::NONE);

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

    return r;
}

void destroy_renderer(VisualRenderer* r) {
    if (r == nullptr) return;
    destroy_mesh(*r->engine, *r->scene, r->ground);
    destroy_mesh(*r->engine, *r->scene, r->grid);
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

bool render_frame(VisualRenderer* r, const CameraPose& pose, FrameView out) {
    if (r == nullptr || out.rgb == nullptr || out.width == 0 || out.height == 0) return false;

    apply_current_theme(*r, r->scene_buffer.active().sim_time_sec);

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

}  // namespace mpviz
