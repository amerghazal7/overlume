// renderer.cpp — Epic 0 Task 2: headless Filament hello-frame behind the POD
// api.h boundary (docs/superpowers/plans/2026-08-18-visual-mode.md).
//
// Scene: one directional sun + a gray ground plane + a sparse reference grid
// + a lit cube at the origin, per the plan's Task 2 Step 3. Geometry is
// baked directly in world space (no TransformManager use) — deliberately
// minimal for a spike; later epics (SceneGraph, Epic 1) replace this whole
// scene-construction path.
//
// This file is internal to visual_renderer's clang/libc++ build, so (unlike
// api.h) ordinary std:: usage is fine here — nothing here crosses the ABI
// boundary with the gcc/libstdc++ ROS node.
#include "visual_renderer/api.h"

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/MaterialEnums.h>
#include <filament/MaterialInstance.h>
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

#include "simple_color_filamat.h"  // matc-generated (CMakeLists.txt); see that file's mat source

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <atomic>
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

namespace mpviz {
namespace {

using filament::math::float3;
using filament::math::float4;
using filament::math::quatf;

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

// One interleaved vertex: world-space position + the Filament "TANGENTS"
// quaternion that encodes the surface normal (see VertexBuffer::Builder::
// attribute()'s warning: TANGENTS is how normals are specified).
struct Vertex {
    float3 position;
    float4 tangentFrame;
};

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

// A sparse reference grid drawn as line segments resting just above the
// ground plane (avoids z-fighting), spaced 2 units apart across the ground.
void build_grid_lines(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    const float h = kGroundHalfExtent;
    const float step = 2.0f;
    const float z = 0.001f;
    std::vector<float3> normals;
    for (float x = -h; x <= h + 1e-3f; x += step) {
        verts.push_back({{x, -h, z}, {}});
        verts.push_back({{x, h, z}, {}});
    }
    for (float y = -h; y <= h + 1e-3f; y += step) {
        verts.push_back({{-h, y, z}, {}});
        verts.push_back({{h, y, z}, {}});
    }
    indices.resize(verts.size());
    for (uint16_t i = 0; i < verts.size(); ++i) indices[i] = i;
    fill_tangent_frames(verts, std::vector<float3>(verts.size(), float3{0, 0, 1}));
}

// Unit cube resting on the ground, centered on the origin in X/Y (half-extent
// 0.5), spanning Z in [0, 1]. 24 vertices (4 per face, distinct normals) +
// 36 indices (2 triangles per face), wound CCW as seen from outside.
void build_cube(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    const float h = 0.5f;
    const float3 center{0.0f, 0.0f, 0.5f};
    struct Face {
        float3 normal;
        float3 corners[4];
    };
    const Face faces[6] = {
        {{1, 0, 0}, {{h, -h, -h}, {h, h, -h}, {h, h, h}, {h, -h, h}}},
        {{-1, 0, 0}, {{-h, h, -h}, {-h, -h, -h}, {-h, -h, h}, {-h, h, h}}},
        {{0, 1, 0}, {{h, h, -h}, {-h, h, -h}, {-h, h, h}, {h, h, h}}},
        {{0, -1, 0}, {{-h, -h, -h}, {h, -h, -h}, {h, -h, h}, {-h, -h, h}}},
        {{0, 0, 1}, {{-h, -h, h}, {h, -h, h}, {h, h, h}, {-h, h, h}}},
        {{0, 0, -1}, {{-h, h, -h}, {h, h, -h}, {h, -h, -h}, {-h, -h, -h}}},
    };

    std::vector<float3> normals;
    for (const Face& f : faces) {
        const uint16_t base = static_cast<uint16_t>(verts.size());
        for (const float3& c : f.corners) {
            verts.push_back({center + c, {}});
            normals.push_back(f.normal);
        }
        indices.push_back(base + 0);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
        indices.push_back(base + 0);
        indices.push_back(base + 2);
        indices.push_back(base + 3);
    }
    fill_tangent_frames(verts, normals);
}

struct Mesh {
    filament::VertexBuffer* vb = nullptr;
    filament::IndexBuffer* ib = nullptr;
    utils::Entity entity;
};

void destroy_mesh(filament::Engine& engine, filament::Scene& scene, Mesh& mesh) {
    if (mesh.entity) {
        scene.remove(mesh.entity);
        engine.destroy(mesh.entity);
        utils::EntityManager::get().destroy(mesh.entity);
    }
    if (mesh.vb) engine.destroy(mesh.vb);
    if (mesh.ib) engine.destroy(mesh.ib);
}

// Payload handed to the readPixels callback: where to signal completion.
struct ReadbackState {
    std::atomic<bool> done{false};
};

void on_readback_complete(void* /*buffer*/, size_t /*size*/, void* user) {
    static_cast<ReadbackState*>(user)->done.store(true, std::memory_order_release);
}

}  // namespace

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
    filament::Material* colorMaterial = nullptr;
    filament::MaterialInstance* groundMaterial = nullptr;
    filament::MaterialInstance* gridMaterial = nullptr;
    filament::MaterialInstance* cubeMaterial = nullptr;
    Mesh ground;
    Mesh grid;
    Mesh cube;
    uint32_t width = 0;
    uint32_t height = 0;
};

VisualRenderer* create_renderer(const RenderConfig& config) {
    if (config.width == 0 || config.height == 0) return nullptr;

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

    r->swapChain = engine->createSwapChain(config.width, config.height,
                                            filament::SwapChain::CONFIG_READABLE);
    r->renderer = engine->createRenderer();
    r->scene = engine->createScene();
    r->view = engine->createView();
    r->view->setScene(r->scene);
    r->view->setViewport({0, 0, config.width, config.height});
    r->view->setPostProcessingEnabled(false);

    filament::Renderer::ClearOptions clearOptions;
    clearOptions.clearColor = {0.45f, 0.65f, 0.9f, 1.0f};  // sky blue
    clearOptions.clear = true;
    r->renderer->setClearOptions(clearOptions);

    utils::EntityManager& em = utils::EntityManager::get();

    r->cameraEntity = em.create();
    r->camera = engine->createCamera(r->cameraEntity);
    r->view->setCamera(r->camera);

    r->sunEntity = em.create();
    filament::LightManager::Builder(filament::LightManager::Type::SUN)
        .direction({-0.5f, -0.3f, -1.0f})
        .color({1.0f, 1.0f, 0.98f})
        .intensity(110000.0f)
        .sunAngularRadius(1.9f)
        .castShadows(true)
        .build(*engine, r->sunEntity);
    r->scene->addEntity(r->sunEntity);

    // Flat ambient term so faces not directly facing the sun aren't pitch
    // black; a single spherical-harmonics band is a constant "sky color".
    const float3 ambientSh[1] = {float3{0.35f, 0.35f, 0.35f}};
    r->ambient = filament::IndirectLight::Builder()
                     .irradiance(1, ambientSh)
                     .intensity(30000.0f)
                     .build(*engine);
    r->scene->setIndirectLight(r->ambient);

    // Engine::getDefaultMaterial() was tried first and rejected: it renders
    // as a flat, fixed "80% white" regardless of scene lighting — confirmed
    // empirically (zeroing every light in the scene produced pixel-
    // identical output, and an enormous test cube filled the frame with the
    // *exact same* shade the ground already renders as). Useless for
    // telling ground/grid/cube apart. materials/simple_color.mat (matc-
    // compiled at configure time, see CMakeLists.txt) is a minimal unlit
    // material with one settable baseColor parameter instead — one
    // Material, three MaterialInstances (one solid color each).
    r->colorMaterial =
        filament::Material::Builder()
            .package(mpviz::materials::ksimple_colorFilamat, mpviz::materials::ksimple_colorFilamatSize)
            .build(*engine);
    r->groundMaterial = r->colorMaterial->createInstance();
    r->groundMaterial->setParameter("baseColor", float3{0.6f, 0.6f, 0.62f});
    r->gridMaterial = r->colorMaterial->createInstance();
    r->gridMaterial->setParameter("baseColor", float3{0.15f, 0.15f, 0.17f});
    r->cubeMaterial = r->colorMaterial->createInstance();
    r->cubeMaterial->setParameter("baseColor", float3{0.85f, 0.32f, 0.1f});
    // ponytail: don't chase hand-derived cube-face winding correctness for a
    // spike scene — CullingMode::NONE sidesteps backface culling entirely so
    // a winding mistake shows as a face colored the same regardless of which
    // side is "front", not a silently invisible one.
    for (auto* mat : {r->groundMaterial, r->gridMaterial, r->cubeMaterial}) {
        mat->setCullingMode(filament::backend::CullingMode::NONE);
    }

    auto add_mesh = [&](Mesh& mesh, std::vector<Vertex> verts, std::vector<uint16_t> indices,
                         filament::RenderableManager::PrimitiveType primitive,
                         filament::MaterialInstance* material) {
        mesh.vb = make_vertex_buffer(*engine, std::move(verts));
        mesh.ib = make_index_buffer(*engine, std::move(indices));
        mesh.entity = em.create();
        filament::RenderableManager::Builder(1)
            .boundingBox({{0, 0, 0}, {kGroundHalfExtent, kGroundHalfExtent, 1.0f}})
            .geometry(0, primitive, mesh.vb, mesh.ib)
            .material(0, material)
            .culling(false)
            .castShadows(false)
            .receiveShadows(false)
            .build(*engine, mesh.entity);
        r->scene->addEntity(mesh.entity);
    };

    std::vector<Vertex> groundVerts, gridVerts, cubeVerts;
    std::vector<uint16_t> groundIdx, gridIdx, cubeIdx;
    build_ground_plane(groundVerts, groundIdx);
    build_grid_lines(gridVerts, gridIdx);
    build_cube(cubeVerts, cubeIdx);

    add_mesh(r->ground, groundVerts, groundIdx,
             filament::RenderableManager::PrimitiveType::TRIANGLES, r->groundMaterial);
    add_mesh(r->grid, gridVerts, gridIdx, filament::RenderableManager::PrimitiveType::LINES,
             r->gridMaterial);
    add_mesh(r->cube, cubeVerts, cubeIdx, filament::RenderableManager::PrimitiveType::TRIANGLES,
             r->cubeMaterial);

    return r;
}

void destroy_renderer(VisualRenderer* r) {
    if (r == nullptr) return;
    destroy_mesh(*r->engine, *r->scene, r->ground);
    destroy_mesh(*r->engine, *r->scene, r->grid);
    destroy_mesh(*r->engine, *r->scene, r->cube);
    if (r->groundMaterial) r->engine->destroy(r->groundMaterial);
    if (r->gridMaterial) r->engine->destroy(r->gridMaterial);
    if (r->cubeMaterial) r->engine->destroy(r->cubeMaterial);
    if (r->colorMaterial) r->engine->destroy(r->colorMaterial);
    if (r->ambient) r->engine->destroy(r->ambient);
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

bool render_frame(VisualRenderer* r, const CameraPose& pose, FrameView out) {
    if (r == nullptr || out.rgb == nullptr || out.width == 0 || out.height == 0) return false;

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

}  // namespace mpviz
