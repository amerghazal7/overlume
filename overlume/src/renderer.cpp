// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"
#include "alert_polygons.hpp"
#include "alert_polygons_test_hooks.hpp"
#include "bowl.hpp"
#include "ego.hpp"
#include "ego_test_hooks.hpp"
#include "environment.hpp"
#include "environment_test_hooks.hpp"
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
#include "trajectory_carpet.hpp"
#include "trajectory_carpet_test_hooks.hpp"
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

#include "clay_filamat.h"
#include "clay_faded_filamat.h"
#include "clay_translucent_filamat.h"
#include "ribbon_emissive_filamat.h"
#include "ribbon_faded_filamat.h"
#include "ground_grid_filamat.h"
#include "point_cloud_filamat.h"
#include "trajectory_carpet_filamat.h"
#include "trajectory_carpet_faded_filamat.h"

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

namespace bluegl {
int bind();
void unbind();
}

extern "C" const unsigned char* bluegl_glGetString(unsigned int name);
constexpr unsigned int kGlVendor = 0x1F00;
constexpr unsigned int kGlRenderer = 0x1F01;
constexpr unsigned int kGlVersion = 0x1F02;

#ifndef DEFAULT_THEME_ASSETS_DIR
#error "DEFAULT_THEME_ASSETS_DIR must be defined by CMakeLists.txt"
#endif

namespace overlume {

using filament::math::float3;
using filament::math::float4;
using filament::math::quatf;

namespace {
float3 to_filament(const detail::Float3& c) { return float3{c.r, c.g, c.b}; }

constexpr float kFogScaleExponent = 1.159f;
constexpr float kFogScaleReferenceIntensity = 8750.0f;
constexpr float kFogScaleReferenceValue = 50.0f;
}

class HeadlessEglPlatform : public filament::backend::OpenGLPlatform {
public:
    struct EglSwapChain : public filament::backend::Platform::SwapChain {
        EGLSurface surface = EGL_NO_SURFACE;
    };

    int getOSVersion() const noexcept override { return 0; }

    filament::backend::Driver* createDriver(void*,
                                            const DriverConfig& driverConfig) noexcept override {
        display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display_ == EGL_NO_DISPLAY) return nullptr;
        if (eglInitialize(display_, nullptr, nullptr) != EGL_TRUE) return nullptr;
        if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) return nullptr;

        const EGLint configAttribs[] = {
            EGL_SURFACE_TYPE,
            EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE,
            EGL_OPENGL_BIT,
            EGL_RED_SIZE,
            8,
            EGL_GREEN_SIZE,
            8,
            EGL_BLUE_SIZE,
            8,
            EGL_ALPHA_SIZE,
            8,
            EGL_DEPTH_SIZE,
            24,
            EGL_STENCIL_SIZE,
            8,
            EGL_NONE,
        };
        EGLint numConfigs = 0;
        if (eglChooseConfig(display_, configAttribs, &config_, 1, &numConfigs) != EGL_TRUE ||
            numConfigs == 0) {
            return nullptr;
        }

        const EGLint ctxAttribs[] = {
            EGL_CONTEXT_MAJOR_VERSION,
            4,
            EGL_CONTEXT_MINOR_VERSION,
            5,
            EGL_CONTEXT_OPENGL_PROFILE_MASK,
            EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE,
        };
        context_ = eglCreateContext(display_, config_, EGL_NO_CONTEXT, ctxAttribs);
        if (context_ == EGL_NO_CONTEXT) return nullptr;

        const EGLint bootstrapAttribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        bootstrapSurface_ = eglCreatePbufferSurface(display_, config_, bootstrapAttribs);
        if (bootstrapSurface_ == EGL_NO_SURFACE) return nullptr;
        if (eglMakeCurrent(display_, bootstrapSurface_, bootstrapSurface_, context_) != EGL_TRUE) {
            return nullptr;
        }
        if (bluegl::bind() != 0) return nullptr;
        blueglBound_ = true;

        const auto gl_str = [](unsigned int n) {
            const unsigned char* s = bluegl_glGetString(n);
            return s != nullptr ? reinterpret_cast<const char*>(s) : "(null)";
        };
        std::fprintf(stderr, "[overlume] GL_VENDOR: %s\n", gl_str(kGlVendor));
        std::fprintf(stderr, "[overlume] GL_RENDERER: %s\n", gl_str(kGlRenderer));
        std::fprintf(stderr, "[overlume] GL_VERSION: %s\n", gl_str(kGlVersion));

        return createDefaultDriver(this, nullptr, driverConfig);
    }

    filament::backend::Platform::SwapChain* createSwapChain(void*, uint64_t) noexcept override {
        return nullptr;
    }

    filament::backend::Platform::SwapChain* createSwapChain(uint32_t width, uint32_t height,
                                                            uint64_t) noexcept override {
        const EGLint pbufferAttribs[] = {
            EGL_WIDTH, static_cast<EGLint>(width), EGL_HEIGHT, static_cast<EGLint>(height),
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

    bool makeCurrent(ContextType, filament::backend::Platform::SwapChain* drawSwapChain,
                     filament::backend::Platform::SwapChain* readSwapChain) noexcept override {
        auto* draw = static_cast<EglSwapChain*>(drawSwapChain);
        auto* read = static_cast<EglSwapChain*>(readSwapChain);
        return eglMakeCurrent(display_, draw->surface, read->surface, context_) == EGL_TRUE;
    }

    void commit(filament::backend::Platform::SwapChain*) noexcept override {}

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

struct GridVertex {
    float3 position;
    float4 tangentFrame;
    float4 color;
};

float grid_fade_alpha(float dist_m, float fade_start_m, float fade_end_m) {
    if (fade_end_m <= fade_start_m) return dist_m <= fade_start_m ? 1.0f : 0.0f;
    const float t = (dist_m - fade_start_m) / (fade_end_m - fade_start_m);
    return 1.0f - std::clamp(t, 0.0f, 1.0f);
}

void build_grid_lines(std::vector<GridVertex>& verts, std::vector<uint16_t>& indices,
                      float fade_start_m, float fade_end_m) {
    const float h = kGroundHalfExtent;
    const float step = kGridPitchM;
    const float z = 0.001f;
    std::vector<float3> normals;
    std::vector<Vertex> plainVerts;
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
                       filament::VertexBuffer::AttributeType::FLOAT4, offsetof(GridVertex, color),
                       sizeof(GridVertex))
            .build(engine);
    vb->setBufferAt(
        engine, 0,
        filament::VertexBuffer::BufferDescriptor(
            heapVerts->data(), heapVerts->size() * sizeof(GridVertex),
            [](void*, size_t, void* user) { delete static_cast<std::vector<GridVertex>*>(user); },
            heapVerts));
    return vb;
}

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
        .receiveShadows(false)
        .build(*r.engine, mesh.entity);
    r.scene->addEntity(mesh.entity);
}

void sh_from_hemisphere(const float3& sky, const float3& ground, float3 sh[4]) {
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kY0 = 0.282095f;
    constexpr float kY1 = 0.488603f;
    const float kTwoPiSq = 2.0f * kPi * kPi;
    sh[0] = kTwoPiSq * kY0 * (sky + ground);
    sh[1] = float3{0.0f, 0.0f, 0.0f};
    sh[2] = (kTwoPiSq / 3.0f) * kY1 * (sky - ground);
    sh[3] = float3{0.0f, 0.0f, 0.0f};
}

void push_theme_to_scene(VisualRenderer& r, const detail::Theme& theme) {
    r.groundMaterial->setParameter("baseColor", to_filament(theme.palette.ground));
    r.groundMaterial->setParameter("roughness", theme.material.roughness);
    r.groundMaterial->setParameter("metallic", theme.material.metallic);

    r.gridMaterial->setParameter("baseColor", to_filament(theme.grid.line_color));
    r.gridMaterial->setParameter("roughness", theme.material.roughness);
    r.gridMaterial->setParameter("metallic", theme.material.metallic);

    r.laneMaterial->setParameter("baseColor", to_filament(theme.palette.lane_paint));
    r.laneMaterial->setParameter("roughness", theme.material.roughness);
    r.laneMaterial->setParameter("metallic", theme.material.metallic);
    r.laneMaterialBaseColor = theme.palette.lane_paint;

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

    r.roadEdgeMaterial->setParameter("baseColor", to_filament(theme.palette.road_edge));
    r.roadEdgeMaterial->setParameter("roughness", theme.material.roughness);
    r.roadEdgeMaterial->setParameter("metallic", theme.material.metallic);
    r.roadEdgeMaterialBaseColor = theme.palette.road_edge;

    r.egoMaterial->setParameter("baseColor", to_filament(theme.palette.ego));
    r.egoMaterial->setParameter("roughness", theme.material.roughness);
    r.egoMaterial->setParameter("metallic", theme.material.metallic);
    r.egoMaterialBaseColor = theme.palette.ego;

    const detail::Float3 objectTints[VisualRenderer::kObjectClassCount] = {
        theme.palette.object_tints.car,     theme.palette.object_tints.truck_van,
        theme.palette.object_tints.bus,     theme.palette.object_tints.pedestrian,
        theme.palette.object_tints.cyclist, theme.palette.object_tints.unknown,
    };
    for (size_t i = 0; i < VisualRenderer::kObjectClassCount; ++i) {
        r.objectClassMaterial[i]->setParameter("baseColor", to_filament(objectTints[i]));
        r.objectClassMaterial[i]->setParameter("roughness", theme.material.roughness);
        r.objectClassMaterial[i]->setParameter("metallic", theme.material.metallic);
        r.objectClassTint[i] = objectTints[i];
    }
    for (auto& [id, entity] : r.objectEntities) {
        if (entity.fadeInstance == nullptr) continue;
        const detail::Float3& tint = objectTints[static_cast<uint8_t>(entity.cls)];
        entity.fadeInstance->setParameter("baseColor",
                                          float4{tint.r, tint.g, tint.b, entity.fadeAlpha});
        entity.fadeInstance->setParameter("roughness", theme.material.roughness);
        entity.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

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

    r.ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)]->setParameter(
        "baseColor", to_filament(theme.palette.ribbon_global));
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)]->setParameter(
        "roughness", theme.material.roughness);
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)]->setParameter("metallic",
                                                                           theme.material.metallic);
    r.ribbonTint[static_cast<uint8_t>(PathRole::GLOBAL)] = theme.palette.ribbon_global;

    r.ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)]->setParameter(
        "baseColor", to_filament(theme.palette.ribbon_local));
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)]->setParameter("roughness",
                                                                          theme.material.roughness);
    r.ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)]->setParameter("metallic",
                                                                          theme.material.metallic);
    r.ribbonTint[static_cast<uint8_t>(PathRole::LOCAL)] = theme.palette.ribbon_local;

    for (auto& slot : r.ribbonSlots) {
        if (slot.fadeInstance == nullptr) continue;
        const detail::Float3& tint = r.ribbonTint[static_cast<uint8_t>(slot.role)];
        slot.fadeInstance->setParameter("baseColor",
                                        float4{tint.r, tint.g, tint.b, slot.fadeAlpha});
        slot.fadeInstance->setParameter("roughness", theme.material.roughness);
        slot.fadeInstance->setParameter("metallic", theme.material.metallic);
        slot.fadeInstance->setParameter("emissiveColor", to_filament(theme.palette.ribbon_glow));
        slot.fadeInstance->setParameter("emissiveStrength", slot.role == PathRole::BEHAVIOR
                                                                ? theme.emissive.ribbon_strength
                                                                : 0.0f);
    }

    const detail::Theme::OgmRamp* ramps[VisualRenderer::kGroundGridKindCount] = {
        &theme.ogm.dynamic, &theme.ogm.geometric};
    for (size_t kind = 0; kind < VisualRenderer::kGroundGridKindCount; ++kind) {
        filament::MaterialInstance* inst = r.groundGridMaterialInstance[kind];
        inst->setParameter("roughness", theme.material.roughness);
        inst->setParameter("metallic", theme.material.metallic);
        r.groundGridRamp[kind] = *ramps[kind];
        auto* texels = new std::vector<float>(detail::Theme::OgmRamp::kEntries * 4);
        for (size_t v = 0; v < detail::Theme::OgmRamp::kEntries; ++v) {
            (*texels)[v * 4 + 0] = ramps[kind]->color[v].r;
            (*texels)[v * 4 + 1] = ramps[kind]->color[v].g;
            (*texels)[v * 4 + 2] = ramps[kind]->color[v].b;
            (*texels)[v * 4 + 3] = ramps[kind]->alpha[v];
        }
        r.groundGridRampTexture[kind]->setImage(
            *r.engine, 0,
            filament::Texture::PixelBufferDescriptor(
                texels->data(), texels->size() * sizeof(float), filament::Texture::Format::RGBA,
                filament::Texture::Type::FLOAT,
                [](void*, size_t, void* user) { delete static_cast<std::vector<float>*>(user); },
                texels));
    }

    const detail::Float3 alertTints[VisualRenderer::kAlertSeverityCount] = {
        theme.palette.alert.info,
        theme.palette.alert.warning,
        theme.palette.alert.critical,
    };
    for (size_t i = 0; i < VisualRenderer::kAlertSeverityCount; ++i) {
        r.alertMaterial[i]->setParameter(
            "baseColor",
            float4{alertTints[i].r, alertTints[i].g, alertTints[i].b, kAlertSeverityAlpha[i]});
        r.alertMaterial[i]->setParameter("roughness", theme.material.roughness);
        r.alertMaterial[i]->setParameter("metallic", theme.material.metallic);
        r.alertTint[i] = alertTints[i];
    }
    for (auto& slot : r.alertSlots) {
        if (slot.fadeInstance == nullptr) continue;
        const detail::Float3& tint = r.alertTint[slot.severity];
        slot.fadeInstance->setParameter("baseColor",
                                        float4{tint.r, tint.g, tint.b, slot.fadeAlpha});
        slot.fadeInstance->setParameter("roughness", theme.material.roughness);
        slot.fadeInstance->setParameter("metallic", theme.material.metallic);
    }

    r.genericMarkerMaterial->setParameter("baseColor",
                                          to_filament(theme.palette.object_tints.unknown));
    r.genericMarkerMaterial->setParameter("roughness", theme.material.roughness);
    r.genericMarkerMaterial->setParameter("metallic", theme.material.metallic);
    r.genericMarkerNeutralTint = theme.palette.object_tints.unknown;
    for (auto& [key, inst] : r.genericMarkerColorInstances) {
        inst->setParameter("roughness", theme.material.roughness);
        inst->setParameter("metallic", theme.material.metallic);
    }
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

    float3 sh[4];
    sh_from_hemisphere(to_filament(theme.ibl.sky_color), to_filament(theme.ibl.ground_color), sh);
    filament::IndirectLight* newAmbient = filament::IndirectLight::Builder()
                                              .irradiance(2, sh)
                                              .intensity(theme.ibl.intensity)
                                              .build(*r.engine);
    r.scene->setIndirectLight(newAmbient);
    if (r.ambient) r.engine->destroy(r.ambient);
    r.ambient = newAmbient;

    filament::FogOptions fogOptions{};
    const float fogScale =
        kFogScaleReferenceValue *
        std::pow(kFogScaleReferenceIntensity / theme.ibl.intensity, kFogScaleExponent);
    fogOptions.color = to_filament(theme.palette.fog) * fogScale;
    fogOptions.density = theme.fog.density;
    fogOptions.heightFalloff = 0.0f;
    fogOptions.enabled = true;
    r.view->setFogOptions(fogOptions);

    filament::Renderer::ClearOptions clearOptions;
    clearOptions.clearColor = {theme.palette.sky.r, theme.palette.sky.g, theme.palette.sky.b, 1.0f};
    clearOptions.clear = true;
    r.renderer->setClearOptions(clearOptions);

    r.buildingMaterial->setParameter("baseColor", to_filament(theme.palette.building));
    r.buildingMaterial->setParameter("roughness", theme.material.roughness);
    r.buildingMaterial->setParameter("metallic", theme.material.metallic);
}

struct ReadbackState {
    std::atomic<bool> done{false};
};

void on_readback_complete(void*, size_t, void* user) {
    static_cast<ReadbackState*>(user)->done.store(true, std::memory_order_release);
}

}

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

void build_unit_arrow(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    constexpr float kShaftHalfW = 0.06f;
    constexpr float kHeadHalfW = 0.15f;
    constexpr float kShaftEndX = 0.7f;
    const float3 p[7] = {
        {0.0f, -kShaftHalfW, 0.0f},      {kShaftEndX, -kShaftHalfW, 0.0f},
        {kShaftEndX, kShaftHalfW, 0.0f}, {0.0f, kShaftHalfW, 0.0f},
        {kShaftEndX, -kHeadHalfW, 0.0f}, {1.0f, 0.0f, 0.0f},
        {kShaftEndX, kHeadHalfW, 0.0f},
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
    if (mesh.fadeInstance) engine.destroy(mesh.fadeInstance);
    mesh = {};
}

filament::VertexBuffer* make_vertex_buffer(filament::Engine& engine, std::vector<Vertex> verts) {
    auto* heapVerts = new std::vector<Vertex>(std::move(verts));
    filament::VertexBuffer* vb = filament::VertexBuffer::Builder()
                                     .vertexCount(static_cast<uint32_t>(heapVerts->size()))
                                     .bufferCount(1)
                                     .attribute(filament::VertexAttribute::POSITION, 0,
                                                filament::VertexBuffer::AttributeType::FLOAT3,
                                                offsetof(Vertex, position), sizeof(Vertex))
                                     .attribute(filament::VertexAttribute::TANGENTS, 0,
                                                filament::VertexBuffer::AttributeType::FLOAT4,
                                                offsetof(Vertex, tangentFrame), sizeof(Vertex))
                                     .attribute(filament::VertexAttribute::COLOR, 0,
                                                filament::VertexBuffer::AttributeType::FLOAT4,
                                                offsetof(Vertex, color), sizeof(Vertex))
                                     .build(engine);
    vb->setBufferAt(
        engine, 0,
        filament::VertexBuffer::BufferDescriptor(
            heapVerts->data(), heapVerts->size() * sizeof(Vertex),
            [](void*, size_t, void* user) { delete static_cast<std::vector<Vertex>*>(user); },
            heapVerts));
    return vb;
}

void update_mesh_positions(filament::Engine& engine, Mesh& mesh, std::vector<Vertex> verts) {
    if (mesh.vb == nullptr) return;
    auto* heapVerts = new std::vector<Vertex>(std::move(verts));
    mesh.vb->setBufferAt(
        engine, 0,
        filament::VertexBuffer::BufferDescriptor(
            heapVerts->data(), heapVerts->size() * sizeof(Vertex),
            [](void*, size_t, void* user) { delete static_cast<std::vector<Vertex>*>(user); },
            heapVerts));
}

filament::IndexBuffer* make_index_buffer(filament::Engine& engine, std::vector<uint16_t> indices) {
    auto* heapIndices = new std::vector<uint16_t>(std::move(indices));
    filament::IndexBuffer* ib = filament::IndexBuffer::Builder()
                                    .indexCount(static_cast<uint32_t>(heapIndices->size()))
                                    .bufferType(filament::IndexBuffer::IndexType::USHORT)
                                    .build(engine);
    ib->setBuffer(engine, filament::IndexBuffer::BufferDescriptor(
                              heapIndices->data(), heapIndices->size() * sizeof(uint16_t),
                              [](void*, size_t, void* user) {
                                  delete static_cast<std::vector<uint16_t>*>(user);
                              },
                              heapIndices));
    return ib;
}

void add_mesh(VisualRenderer& r, Mesh& mesh, std::vector<Vertex> verts,
              std::vector<uint16_t> indices, filament::RenderableManager::PrimitiveType primitive,
              filament::MaterialInstance* material, bool cast_shadows, bool receive_shadows) {
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

namespace {

void ApplyQualityViewOptions(filament::View& view, uint32_t quality, uint32_t width,
                             uint32_t height) {
    filament::AmbientOcclusionOptions ao{};
    ao.enabled = quality >= 1;
    ao.resolution = quality >= 2 ? 1.0f : 0.5f;
    view.setAmbientOcclusionOptions(ao);

    filament::TemporalAntiAliasingOptions taa{};
    if (quality >= 2) {
        view.setAntiAliasing(filament::AntiAliasing::NONE);
        taa.enabled = true;
    } else {
        view.setAntiAliasing(filament::AntiAliasing::FXAA);
        taa.enabled = false;
    }
    view.setTemporalAntiAliasingOptions(taa);

    filament::View::DynamicResolutionOptions dynRes{};
    if (quality == 0 && width > 0 && height > 0) {
        dynRes.enabled = true;
        dynRes.homogeneousScaling = true;
        dynRes.quality = filament::QualityLevel::LOW;
        const float scale =
            std::min(960.0f / static_cast<float>(width), 540.0f / static_cast<float>(height));
        dynRes.minScale = {scale, scale};
        dynRes.maxScale = {scale, scale};
    }
    view.setDynamicResolutionOptions(dynRes);
}

filament::LightManager::ShadowOptions ShadowOptionsForQuality(uint32_t quality) {
    filament::LightManager::ShadowOptions opts{};
    opts.mapSize = quality >= 2 ? 2048 : 1024;
    return opts;
}

}

VisualRenderer* create_renderer(const RenderConfig& config) {
    if (config.width == 0 || config.height == 0) return nullptr;

    const std::string themeDir = config.theme_assets_dir ? std::string(config.theme_assets_dir)
                                                         : std::string(DEFAULT_THEME_ASSETS_DIR);
    const std::string themeName =
        config.initial_theme ? std::string(config.initial_theme) : std::string("dark_adas");
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

    r->swapChain =
        engine->createSwapChain(config.width, config.height, filament::SwapChain::CONFIG_READABLE);
    r->renderer = engine->createRenderer();
    r->scene = engine->createScene();
    r->view = engine->createView();
    r->view->setScene(r->scene);
    r->view->setViewport({0, 0, config.width, config.height});

    r->view->setPostProcessingEnabled(true);
    r->colorGrading = filament::ColorGrading::Builder()
                          .toneMapping(filament::ColorGrading::ToneMapping::ACES)
                          .build(*engine);
    r->view->setColorGrading(r->colorGrading);

    filament::BloomOptions bloom{};
    bloom.strength = 0.5f;
    bloom.enabled = true;
    r->view->setBloomOptions(bloom);

    ApplyQualityViewOptions(*r->view, config.quality, config.width, config.height);

    utils::EntityManager& em = utils::EntityManager::get();

    r->cameraEntity = em.create();
    r->camera = engine->createCamera(r->cameraEntity);
    r->view->setCamera(r->camera);
    r->camera->setExposure(16.0f, 1.0f / 500.0f, 100.0f);

    r->sunEntity = em.create();
    filament::LightManager::Builder(filament::LightManager::Type::SUN)
        .sunAngularRadius(1.9f)
        .castShadows(config.quality >= 1)
        .shadowOptions(ShadowOptionsForQuality(config.quality))
        .build(*engine, r->sunEntity);
    r->scene->addEntity(r->sunEntity);
    r->qualityPreset = config.quality;

    r->clayMaterial =
        filament::Material::Builder()
            .package(overlume::materials::kclayFilamat, overlume::materials::kclayFilamatSize)
            .build(*engine);
    r->clayFadedMaterial = filament::Material::Builder()
                               .package(overlume::materials::kclay_fadedFilamat,
                                        overlume::materials::kclay_fadedFilamatSize)
                               .build(*engine);

    r->groundMaterial = r->clayMaterial->createInstance();
    r->gridMaterial = r->clayFadedMaterial->createInstance();
    r->laneMaterial = r->clayMaterial->createInstance();
    r->laneCenterlineMaterial = r->clayMaterial->createInstance();
    r->laneBoundaryMaterial = r->clayMaterial->createInstance();
    r->crosswalkMaterial = r->clayMaterial->createInstance();
    r->roadMaterial = r->clayMaterial->createInstance();
    r->roadEdgeMaterial = r->clayMaterial->createInstance();
    r->egoMaterial = r->clayMaterial->createInstance();

    r->clayTranslucentMaterial = filament::Material::Builder()
                                     .package(overlume::materials::kclay_translucentFilamat,
                                              overlume::materials::kclay_translucentFilamatSize)
                                     .build(*engine);
    for (size_t i = 0; i < VisualRenderer::kObjectClassCount; ++i) {
        r->objectClassMaterial[i] = r->clayMaterial->createInstance();
    }

    r->ribbonEmissiveMaterial = filament::Material::Builder()
                                    .package(overlume::materials::kribbon_emissiveFilamat,
                                             overlume::materials::kribbon_emissiveFilamatSize)
                                    .build(*engine);
    r->ribbonFadedMaterial = filament::Material::Builder()
                                 .package(overlume::materials::kribbon_fadedFilamat,
                                          overlume::materials::kribbon_fadedFilamatSize)
                                 .build(*engine);
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::BEHAVIOR)] =
        r->ribbonEmissiveMaterial->createInstance();
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::GLOBAL)] = r->clayMaterial->createInstance();
    r->ribbonMaterial[static_cast<uint8_t>(PathRole::LOCAL)] = r->clayMaterial->createInstance();

    r->groundGridMaterial = filament::Material::Builder()
                                .package(overlume::materials::kground_gridFilamat,
                                         overlume::materials::kground_gridFilamatSize)
                                .build(*engine);
    for (size_t kind = 0; kind < VisualRenderer::kGroundGridKindCount; ++kind) {
        r->groundGridMaterialInstance[kind] = r->groundGridMaterial->createInstance();
        r->groundGridRampTexture[kind] =
            filament::Texture::Builder()
                .width(static_cast<uint32_t>(detail::Theme::OgmRamp::kEntries))
                .height(1)
                .levels(1)
                .format(filament::Texture::InternalFormat::RGBA32F)
                .sampler(filament::Texture::Sampler::SAMPLER_2D)
                .build(*engine);
        r->groundGridMaterialInstance[kind]->setParameter(
            "rampTexture", r->groundGridRampTexture[kind],
            filament::TextureSampler(filament::TextureSampler::MinFilter::NEAREST,
                                     filament::TextureSampler::MagFilter::NEAREST));
    }

    r->pointCloudMaterial = filament::Material::Builder()
                                .package(overlume::materials::kpoint_cloudFilamat,
                                         overlume::materials::kpoint_cloudFilamatSize)
                                .build(*engine);
    r->pointCloudMaterialInstance = r->pointCloudMaterial->createInstance();
    r->pointCloudMaterialInstance->setCullingMode(filament::backend::CullingMode::NONE);

    r->trajectoryCarpetMaterial = filament::Material::Builder()
                                      .package(overlume::materials::ktrajectory_carpetFilamat,
                                               overlume::materials::ktrajectory_carpetFilamatSize)
                                      .build(*engine);
    r->trajectoryCarpetMaterialInstance = r->trajectoryCarpetMaterial->createInstance();
    r->trajectoryCarpetMaterialInstance->setCullingMode(filament::backend::CullingMode::NONE);
    r->trajectoryCarpetFadedMaterial =
        filament::Material::Builder()
            .package(overlume::materials::ktrajectory_carpet_fadedFilamat,
                     overlume::materials::ktrajectory_carpet_fadedFilamatSize)
            .build(*engine);
    r->trajectoryCarpetFadedMaterialInstance = r->trajectoryCarpetFadedMaterial->createInstance();
    r->trajectoryCarpetFadedMaterialInstance->setCullingMode(filament::backend::CullingMode::NONE);

    for (size_t i = 0; i < VisualRenderer::kAlertSeverityCount; ++i) {
        r->alertMaterial[i] = r->clayTranslucentMaterial->createInstance();
    }

    r->genericMarkerMaterial = r->clayMaterial->createInstance();

    r->buildingMaterial = r->clayMaterial->createInstance();

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
    r->buildingMaterial->setCullingMode(filament::backend::CullingMode::NONE);

    push_theme_to_scene(*r, theme);

    std::vector<Vertex> groundVerts;
    std::vector<uint16_t> groundIdx;
    build_ground_plane(groundVerts, groundIdx);
    add_mesh(*r, r->ground, groundVerts, groundIdx,
             filament::RenderableManager::PrimitiveType::TRIANGLES, r->groundMaterial, false, true);

    std::vector<GridVertex> gridVerts;
    std::vector<uint16_t> gridIdx;
    build_grid_lines(gridVerts, gridIdx, theme.grid.fade_start_m, theme.grid.fade_end_m);
    add_grid_mesh(*r, r->grid, gridVerts, gridIdx, r->gridMaterial);

    r->engine->getTransformManager().create(r->ground.entity);
    r->engine->getTransformManager().create(r->grid.entity);

    return r;
}

void set_quality(VisualRenderer* r, uint32_t preset) {
    if (r == nullptr) return;
    const uint32_t clamped = preset > 2 ? 2 : preset;
    ApplyQualityViewOptions(*r->view, clamped, r->width, r->height);
    filament::LightManager& lm = r->engine->getLightManager();
    const auto sun = lm.getInstance(r->sunEntity);
    lm.setShadowCaster(sun, clamped >= 1);
    lm.setShadowOptions(sun, ShadowOptionsForQuality(clamped));
    r->qualityPreset = clamped;
}

uint32_t get_quality(VisualRenderer* r) { return r == nullptr ? 0 : r->qualityPreset; }

void destroy_renderer(VisualRenderer* r) {
    if (r == nullptr) return;
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
    for (auto& slot : r->ribbonSlots) {
        for (auto& mesh : slot.meshes) destroy_mesh(*r->engine, *r->scene, mesh);
        if (slot.fadeInstance) r->engine->destroy(slot.fadeInstance);
    }
    r->ribbonSlots.clear();

    for (auto& slot : r->alertSlots) {
        if (slot.mesh.vb) destroy_mesh(*r->engine, *r->scene, slot.mesh);
        if (slot.fadeInstance) r->engine->destroy(slot.fadeInstance);
    }
    r->alertSlots.clear();
    for (auto* m : r->alertMaterial) {
        if (m) r->engine->destroy(m);
    }

    for (auto& slot : r->genericMarkerSlots) {
        if (slot.fadeInstance) r->engine->destroy(slot.fadeInstance);
        if (slot.sharedGeomEntity) {
            r->scene->remove(slot.sharedGeomEntity);
            r->engine->destroy(slot.sharedGeomEntity);
            utils::EntityManager::get().destroy(slot.sharedGeomEntity);
        }
        if (slot.ownMesh.vb) destroy_mesh(*r->engine, *r->scene, slot.ownMesh);
        if (slot.meshAsset) {
            r->scene->removeEntities(slot.meshAsset->getEntities(),
                                     slot.meshAsset->getEntityCount());
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

    for (auto& [key, mesh] : r->mapElementMeshes) {
        (void)key;
        if (mesh.fadeInstance) {
            r->engine->destroy(mesh.fadeInstance);
            mesh.fadeInstance = nullptr;
        }
    }

    if (r->clayTranslucentMaterial) r->engine->destroy(r->clayTranslucentMaterial);

    if (r->environmentSource) {
        r->environmentSource->teardown(*r);
        r->environmentSource.reset();
    }

    if (r->egoAsset) r->sharedAssetLoader->destroyAsset(r->egoAsset);
    if (r->sharedResourceLoader) delete r->sharedResourceLoader;
    if (r->sharedTextureProvider) delete r->sharedTextureProvider;
    if (r->sharedAssetLoader) filament::gltfio::AssetLoader::destroy(&r->sharedAssetLoader);
    if (r->sharedMaterialProvider) {
        r->sharedMaterialProvider->destroyMaterials();
        delete r->sharedMaterialProvider;
    }
    destroy_mesh(*r->engine, *r->scene, r->egoFallback);
    destroy_mesh(*r->engine, *r->scene, r->ground);
    destroy_mesh(*r->engine, *r->scene, r->grid);
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
    if (r->buildingMaterial) r->engine->destroy(r->buildingMaterial);
    for (auto* m : r->ribbonMaterial) {
        if (m) r->engine->destroy(m);
    }
    if (r->ribbonEmissiveMaterial) r->engine->destroy(r->ribbonEmissiveMaterial);
    if (r->ribbonFadedMaterial) r->engine->destroy(r->ribbonFadedMaterial);
    for (auto& slot : r->groundGridSlots) {
        destroy_mesh(*r->engine, *r->scene, slot.quad);
        if (slot.texture) r->engine->destroy(slot.texture);
    }
    r->groundGridSlots.clear();
    for (auto* m : r->groundGridMaterialInstance) {
        if (m) r->engine->destroy(m);
    }
    for (auto* tex : r->groundGridRampTexture) {
        if (tex) r->engine->destroy(tex);
    }
    if (r->groundGridMaterial) r->engine->destroy(r->groundGridMaterial);

    if (r->bowl) {
        destroy_mesh(*r->engine, *r->scene, r->bowl->mesh);
        if (r->bowl->instance) r->engine->destroy(r->bowl->instance);
        if (r->bowl->material) r->engine->destroy(r->bowl->material);
        r->bowl.reset();
    }
    for (auto& slot : r->cameraSlots) {
        if (slot.texture) r->engine->destroy(slot.texture);
    }

    for (auto& slot : r->pointCloudSlots) {
        for (auto& mesh : slot.meshes) destroy_mesh(*r->engine, *r->scene, mesh);
    }
    r->pointCloudSlots.clear();
    if (r->pointCloudMaterialInstance) r->engine->destroy(r->pointCloudMaterialInstance);
    if (r->pointCloudMaterial) r->engine->destroy(r->pointCloudMaterial);

    for (auto& slot : r->trajectoryCarpetSlots) {
        for (auto& mesh : slot.meshes) destroy_mesh(*r->engine, *r->scene, mesh);
    }
    r->trajectoryCarpetSlots.clear();
    if (r->trajectoryCarpetMaterialInstance)
        r->engine->destroy(r->trajectoryCarpetMaterialInstance);
    if (r->trajectoryCarpetMaterial) r->engine->destroy(r->trajectoryCarpetMaterial);
    if (r->trajectoryCarpetFadedMaterialInstance)
        r->engine->destroy(r->trajectoryCarpetFadedMaterialInstance);
    if (r->trajectoryCarpetFadedMaterial) r->engine->destroy(r->trajectoryCarpetFadedMaterial);
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
        r->engine->destroy(r->cameraEntity);
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

void apply_current_theme(VisualRenderer& r, double sim_time_sec) {
    if (!r.theme_transition) return;
    const detail::ThemeTransition& tr = *r.theme_transition;
    const double t = tr.duration_sec > 0.0
                         ? std::clamp((sim_time_sec - tr.start_sec) / tr.duration_sec, 0.0, 1.0)
                         : 1.0;
    const detail::Theme blended = detail::blend(tr.from, tr.to, static_cast<float>(t));
    push_theme_to_scene(r, blended);
    r.active_theme = blended;
    if (t >= 1.0) {
        r.theme_transition.reset();
    }
}

namespace {

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

}

bool render_frame(VisualRenderer* r, const CameraPose& pose, FrameView out) {
    if (r == nullptr || out.rgb == nullptr || out.width == 0 || out.height == 0) return false;

    apply_current_theme(*r, r->scene_buffer.active().sim_time_sec);
    update_ego_transform(*r, r->scene_buffer.active().ego);
    update_bowl(*r, r->scene_buffer.active().ego);
    update_ground_grid_transform(*r, r->scene_buffer.active().ego);
    update_map_elements(*r, r->scene_buffer.active());
    update_objects(*r, r->scene_buffer.active());
    update_ribbons(*r, r->scene_buffer.active());
    update_ground_grids(*r, r->scene_buffer.active());
    update_alert_polygons(*r, r->scene_buffer.active());
    update_generic_markers(*r, r->scene_buffer.active());
    update_point_clouds(*r, r->scene_buffer.active());
    update_trajectory_carpets(*r, r->scene_buffer.active());
    if (r->environmentSource != nullptr && r->scene_buffer.active().ego.valid) {
        r->environmentSource->update(*r, r->scene_buffer.active().ego.position);
    }

    const bool hideGround = r->environmentSource != nullptr && r->environmentVisible &&
                            r->environmentSource->provides_ground();
    const bool groundInScene = r->scene->hasEntity(r->ground.entity);
    if (hideGround && groundInScene) {
        r->scene->remove(r->ground.entity);
    } else if (!hideGround && !groundInScene) {
        r->scene->addEntity(r->ground.entity);
    }

    r->camera->lookAt({pose.eye[0], pose.eye[1], pose.eye[2]},
                      {pose.target[0], pose.target[1], pose.target[2]}, {0.0, 0.0, 1.0});
    const double aspect = static_cast<double>(out.width) / static_cast<double>(out.height);
    r->camera->setProjection(pose.vfov_deg, aspect, 0.1, 500.0, filament::Camera::Fov::VERTICAL);

    if (out.width != r->width || out.height != r->height) return false;

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
    for (int i = 0; i < 200 && !state.done.load(std::memory_order_acquire); ++i) {
        r->engine->pumpMessageQueues();
        if (!state.done.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    if (!state.done.load(std::memory_order_acquire)) return false;

    return true;
}

void set_scene(VisualRenderer* r, const SceneGraph& scene) { r->scene_buffer.publish(scene); }

bool set_theme(VisualRenderer* r, const char* theme_name, double at_sec, double transition_sec) {
    if (r == nullptr || theme_name == nullptr) return false;
    const std::optional<detail::Theme> target = detail::load_theme(r->theme_dir, theme_name);
    if (!target) return false;
    r->theme_transition = detail::ThemeTransition{
        r->active_theme,
        *target,
        at_sec,
        transition_sec > 0.0 ? transition_sec : 0.8,
    };
    return true;
}

bool theme_assets_loaded(VisualRenderer* r) { return r != nullptr && r->theme_assets_loaded; }

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

bool project_to_screen(VisualRenderer* r, Vec3 world_point, float* out_x, float* out_y) {
    if (r == nullptr || out_x == nullptr || out_y == nullptr) return false;
    const filament::math::mat4 viewProj =
        r->camera->getProjectionMatrix() * r->camera->getViewMatrix();
    const filament::math::double4 clip =
        viewProj * filament::math::double4{world_point.x, world_point.y, world_point.z, 1.0};
    if (clip.w <= 1e-9) return false;
    const double ndcX = clip.x / clip.w;
    const double ndcY = clip.y / clip.w;
    if (ndcX < -1.0 || ndcX > 1.0 || ndcY < -1.0 || ndcY > 1.0) return false;
    *out_x = static_cast<float>(ndcX * 0.5 + 0.5);
    *out_y = static_cast<float>(1.0 - (ndcY * 0.5 + 0.5));
    return true;
}

}

namespace overlume::testing {

overlume::Vec3 ground_patch_centre(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->ground.entity) return {0.0, 0.0, 0.0};
    filament::TransformManager& tm = r->engine->getTransformManager();
    const auto inst = tm.getInstance(r->ground.entity);
    if (!inst.isValid()) return {0.0, 0.0, 0.0};
    const filament::math::mat4f xf = tm.getTransform(inst);
    const filament::math::float3 t = xf[3].xyz;
    return {static_cast<double>(t.x), static_cast<double>(t.y), static_cast<double>(t.z)};
}

overlume::detail::Float3 lane_material_base_color(overlume::VisualRenderer* r) {
    if (r == nullptr) return {};
    return r->laneMaterialBaseColor;
}

overlume::detail::Float3 map_kind_base_color(overlume::VisualRenderer* r, overlume::MapKind kind) {
    if (r == nullptr) return {};
    switch (kind) {
        case overlume::MapKind::CENTERLINE:
            return r->laneCenterlineMaterialBaseColor;
        case overlume::MapKind::LEFT_BOUNDARY:
        case overlume::MapKind::RIGHT_BOUNDARY:
            return r->laneBoundaryMaterialBaseColor;
        case overlume::MapKind::CROSSWALK:
            return r->crosswalkMaterialBaseColor;
        case overlume::MapKind::ROAD_SURFACE:
            return r->roadMaterialBaseColor;
        case overlume::MapKind::ROAD_EDGE:
            return r->roadEdgeMaterialBaseColor;
        default:
            return r->laneMaterialBaseColor;
    }
}

overlume::detail::Float3 ego_material_base_color(overlume::VisualRenderer* r) {
    if (r == nullptr) return {};
    return r->egoMaterialBaseColor;
}

uint64_t map_element_rebuild_count(overlume::VisualRenderer* r) {
    return r == nullptr ? 0 : r->mapElementRebuildCount;
}

size_t map_element_mesh_count(overlume::VisualRenderer* r) {
    return r == nullptr ? 0 : r->mapElementMeshes.size();
}

size_t map_element_total_vertex_count(overlume::VisualRenderer* r) {
    if (r == nullptr) return 0;
    size_t total = 0;
    for (const auto& [key, mesh] : r->mapElementMeshes) {
        (void)key;
        total += mesh.vertexCount;
    }
    return total;
}

MapElementMaterialInfo map_element_material_info(overlume::VisualRenderer* r) {
    MapElementMaterialInfo info;
    if (r == nullptr || r->mapElementMeshes.size() != 1) return info;
    const overlume::Mesh& mesh = r->mapElementMeshes.begin()->second;
    info.alpha = mesh.fadeAlpha;
    if (!mesh.entity) return info;
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const auto ri = rm.getInstance(mesh.entity);
    if (!ri.isValid()) return info;
    filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    info.bound_to_translucent = mesh.fadeInstance != nullptr && bound == mesh.fadeInstance;
    return info;
}

bool quality_shadows_enabled(overlume::VisualRenderer* r) {
    if (r == nullptr) return false;
    filament::LightManager& lm = r->engine->getLightManager();
    return lm.isShadowCaster(lm.getInstance(r->sunEntity));
}

uint32_t quality_shadow_map_size(overlume::VisualRenderer* r) {
    if (r == nullptr) return 0;
    filament::LightManager& lm = r->engine->getLightManager();
    return lm.getShadowOptions(lm.getInstance(r->sunEntity)).mapSize;
}

QualityRenderSize quality_internal_render_size(overlume::VisualRenderer* r) {
    QualityRenderSize size;
    if (r == nullptr) return size;
    const filament::View::DynamicResolutionOptions opts = r->view->getDynamicResolutionOptions();
    if (!opts.enabled) {
        size.width = r->width;
        size.height = r->height;
        return size;
    }
    size.width = static_cast<uint32_t>(std::lround(r->width * opts.minScale.x));
    size.height = static_cast<uint32_t>(std::lround(r->height * opts.minScale.x));
    return size;
}

QualitySsao quality_ssao(overlume::VisualRenderer* r) {
    QualitySsao out;
    if (r == nullptr) return out;
    const filament::View::AmbientOcclusionOptions& ao = r->view->getAmbientOcclusionOptions();
    out.enabled = ao.enabled;
    out.resolution = ao.resolution;
    return out;
}

bool quality_taa_enabled(overlume::VisualRenderer* r) {
    if (r == nullptr) return false;
    return r->view->getTemporalAntiAliasingOptions().enabled;
}

QualityAntiAliasing quality_antialiasing(overlume::VisualRenderer* r) {
    if (r == nullptr) return QualityAntiAliasing::NONE;
    return r->view->getAntiAliasing() == filament::AntiAliasing::FXAA ? QualityAntiAliasing::FXAA
                                                                      : QualityAntiAliasing::NONE;
}

}
