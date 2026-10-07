// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
#include <filament/Texture.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/Entity.h>

#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/TextureProvider.h>

#include "camera_textures.hpp"
#include "platform.hpp"
#include "scene_buffer.hpp"
#include "theme.hpp"
#include "theme_transition.hpp"

namespace overlume {

class EnvironmentSource;
struct BowlState;

struct Vertex {
    filament::math::float3 position;
    filament::math::float4 tangentFrame;
    filament::math::float4 color = {1.0f, 1.0f, 1.0f, 1.0f};
};

filament::VertexBuffer* make_vertex_buffer(filament::Engine& engine, std::vector<Vertex> verts);
filament::IndexBuffer* make_index_buffer(filament::Engine& engine, std::vector<uint16_t> indices);

struct Mesh {
    filament::VertexBuffer* vb = nullptr;
    filament::IndexBuffer* ib = nullptr;
    utils::Entity entity;
    uint32_t vertexCount = 0;
    filament::MaterialInstance* fadeInstance = nullptr;
    float fadeAlpha = 1.0f;
};

void update_mesh_positions(filament::Engine& engine, Mesh& mesh, std::vector<Vertex> verts);

inline constexpr float kGridPitchM = 2.0f;

inline constexpr double kStaleFadeStartSec = 0.5;
inline constexpr double kStaleFadeTimeoutSec = 1.0;

inline constexpr float kRibbonMinHalfWidthM = 0.12f;

inline constexpr float kAlertSeverityAlpha[3] = {0.18f, 0.35f, 0.45f};

void fill_tangent_frames(std::vector<Vertex>& verts,
                         const std::vector<filament::math::float3>& normals);
void build_unit_arrow(std::vector<Vertex>& verts, std::vector<uint16_t>& indices);

void destroy_mesh(filament::Engine& engine, filament::Scene& scene, Mesh& mesh);

bool ensure_gltf_loader(VisualRenderer& r);

struct ObjectClassPool {
    filament::gltfio::FilamentAsset* asset = nullptr;
    std::vector<filament::gltfio::FilamentInstance*> pool;
    std::vector<filament::gltfio::FilamentInstance*> freeList;
    Vec3 unitFootprint{1.0, 1.0, 1.0};
    bool growthLogged = false;
    bool capWarned = false;
};
inline constexpr size_t kInitialInstancesPerClass = 16;
inline constexpr size_t kMaxInstancesPerClass = 256;

struct ObjectEntity {
    ObjectClass cls = ObjectClass::UNKNOWN;
    filament::gltfio::FilamentInstance* glInstance = nullptr;
    Mesh proceduralBox;
    utils::Entity transformRoot;
    utils::Entity arrowEntity;
    Mesh pathRibbon;
    uint64_t pathSignature = 0;
    Vec3 appliedScale{0.0, 0.0, 0.0};
    filament::MaterialInstance* fadeInstance = nullptr;
    float fadeAlpha = 1.0f;
};

class VisualRenderer {
public:
    detail::HeadlessPlatform platform;
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
    filament::Material* clayMaterial = nullptr;
    filament::Material* clayFadedMaterial = nullptr;
    filament::MaterialInstance* groundMaterial = nullptr;
    filament::MaterialInstance* gridMaterial = nullptr;
    filament::Material* groundClayMaterial = nullptr;
    filament::Material* groundLinesMaterial = nullptr;
    // Footprint of height_grids[0] in world xy; the ground and grid instances discard inside it.
    struct GroundHole {
        bool enabled = false;
        float center[2] = {0.0f, 0.0f};
        float axis_x[2] = {1.0f, 0.0f};
        float half_extent[2] = {0.0f, 0.0f};
    } groundHole;
    filament::MaterialInstance* egoMaterial = nullptr;
    detail::Float3 egoMaterialBaseColor{};
    Mesh ground;
    Mesh grid;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t qualityPreset = 0;

    filament::MaterialInstance* laneMaterial = nullptr;
    detail::Float3 laneMaterialBaseColor{};
    filament::MaterialInstance* laneCenterlineMaterial = nullptr;
    detail::Float3 laneCenterlineMaterialBaseColor{};
    filament::MaterialInstance* laneBoundaryMaterial = nullptr;
    detail::Float3 laneBoundaryMaterialBaseColor{};
    filament::MaterialInstance* crosswalkMaterial = nullptr;
    detail::Float3 crosswalkMaterialBaseColor{};
    filament::MaterialInstance* roadMaterial = nullptr;
    detail::Float3 roadMaterialBaseColor{};
    filament::MaterialInstance* roadEdgeMaterial = nullptr;
    detail::Float3 roadEdgeMaterialBaseColor{};
    std::unordered_map<uint64_t, Mesh> mapElementMeshes;
    uint64_t mapElementRebuildCount = 0;

    std::string theme_dir;
    detail::Theme active_theme;
    bool theme_assets_loaded = false;

    std::optional<detail::ThemeTransition> theme_transition;

    detail::SceneBuffer scene_buffer;

    filament::gltfio::MaterialProvider* sharedMaterialProvider = nullptr;
    filament::gltfio::AssetLoader* sharedAssetLoader = nullptr;
    filament::gltfio::ResourceLoader* sharedResourceLoader = nullptr;
    filament::gltfio::TextureProvider* sharedTextureProvider = nullptr;
    filament::gltfio::FilamentAsset* egoAsset = nullptr;
    Mesh egoFallback;
    utils::Entity egoTransformEntity;
    Vec3 egoFallbackDims{0.0, 0.0, 0.0};

    static constexpr size_t kObjectClassCount = 6;
    filament::Material* clayTranslucentMaterial = nullptr;
    filament::MaterialInstance* objectClassMaterial[kObjectClassCount] = {};
    detail::Float3 objectClassTint[kObjectClassCount] = {};
    bool objectClassMissingWarned[kObjectClassCount] = {};

    std::unordered_map<uint8_t, ObjectClassPool> objectClassPools;

    Mesh sharedArrowMesh;

    std::unordered_map<uint32_t, ObjectEntity> objectEntities;

    static constexpr size_t kPathRoleCount = 3;
    filament::Material* ribbonEmissiveMaterial = nullptr;
    filament::Material* ribbonFadedMaterial = nullptr;
    filament::MaterialInstance* ribbonMaterial[kPathRoleCount] = {};
    detail::Float3 ribbonTint[kPathRoleCount] = {};

    struct RibbonSlot {
        PathRole role = PathRole::LOCAL;
        uint64_t signature = 0;
        bool has_signature = false;
        std::vector<Mesh> meshes;
        uint32_t totalVertexCount = 0;
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 1.0f;
        float halfWidthM = 0.0f;
        std::optional<double> fadeOriginStationM;
        float minVertexAlpha = 1.0f;
        Vec3 firstPointM{};
        std::vector<std::vector<Vec3>> baseStripPositions;
        std::vector<std::vector<double>> pointStations;
        bool has_applied_clip = false;
        bool appliedClipActive = false;
        int64_t appliedClipUnits = 0;
    };
    std::vector<RibbonSlot> ribbonSlots;
    uint64_t ribbonRebuildCount = 0;

    static constexpr size_t kGroundGridKindCount = 2;
    filament::Material* groundGridMaterial = nullptr;
    filament::MaterialInstance* groundGridMaterialInstance[kGroundGridKindCount] = {};
    filament::Texture* groundGridRampTexture[kGroundGridKindCount] = {};
    detail::Theme::OgmRamp groundGridRamp[kGroundGridKindCount];
    float groundGridAlpha[kGroundGridKindCount] = {1.0f, 1.0f};

    struct GroundGridSlot {
        uint8_t kind = 0;
        Mesh quad;
        bool has_geometry = false;
        Vec3 origin{};
        double yaw_rad = 0.0;
        Vec3 corners[4]{};
        double resolution_m = 0.0;
        uint32_t width_cells = 0, height_cells = 0;
        filament::Texture* texture = nullptr;
        uint32_t texWidth = 0, texHeight = 0;
        uint32_t textureGeneration = 0;
        double last_upload_sec = -1.0;
        uint32_t uploadCount = 0;
    };
    std::vector<GroundGridSlot> groundGridSlots;
    filament::Material* heightGridMaterial = nullptr;
    filament::Material* heightGridFadedMaterial = nullptr;
    filament::MaterialInstance* heightGridInstance = nullptr;
    filament::Texture* heightGridRampTexture = nullptr;
    detail::HeightRampTable heightGridRamp;

    struct HeightGridSlot {
        Mesh mesh;
        filament::MaterialInstance* fadeInstance = nullptr;
        uint32_t width = 0, height = 0;
        double resolution = 0.0;
        double yaw = 0.0;
        Vec3 origin{};  // bookkeeping only; tests read the TransformManager
        double lastUpdateSec = -1.0;
        uint32_t uploadCount = 0;
        int materialState = 0;  // bookkeeping only; tests read the bound material instance
        std::vector<float> cpuPositions;
        std::vector<float> cpuCustom;
        bool inScene = false;
        float alpha = 0.0f;
        float groundBias = 0.0f;
    };
    std::vector<HeightGridSlot> heightGridSlots;

    static constexpr size_t kAlertSeverityCount = 3;
    filament::MaterialInstance* alertMaterial[kAlertSeverityCount] = {};
    detail::Float3 alertTint[kAlertSeverityCount] = {};

    struct AlertSlot {
        uint8_t severity = 0;
        uint64_t signature = 0;
        bool has_signature = false;
        Mesh mesh;
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 0.0f;
    };
    std::vector<AlertSlot> alertSlots;

    filament::MaterialInstance* genericMarkerMaterial = nullptr;
    detail::Float3 genericMarkerNeutralTint{};

    std::unordered_map<uint32_t, filament::MaterialInstance*> genericMarkerColorInstances;

    Mesh genericCubeMesh, genericSphereMesh, genericCylinderMesh, genericTextMesh;

    struct GenericMarkerSlot {
        bool active = false;
        MarkerPrimitive primitive = MarkerPrimitive::CUBE;
        utils::Entity sharedGeomEntity;
        Mesh ownMesh;
        uint64_t geomSignature = 0;
        bool hasGeomSignature = false;
        filament::gltfio::FilamentAsset* meshAsset = nullptr;
        std::string meshPathLoaded;
        bool meshIsFallback = false;
        detail::Float3 tint{};
        filament::MaterialInstance* fadeInstance = nullptr;
        float fadeAlpha = 1.0f;
    };
    std::vector<GenericMarkerSlot> genericMarkerSlots;

    uint32_t genericMarkerAllocCount = 0;
    uint32_t genericMarkerUnknownCount = 0;
    std::unordered_set<std::string> genericMarkerMeshWarned;

    filament::Material* pointCloudMaterial = nullptr;
    filament::MaterialInstance* pointCloudMaterialInstance = nullptr;
    float pointCloudAlpha = 1.0f;

    struct PointCloudSlot {
        uint64_t signature = 0;
        bool has_signature = false;
        std::vector<Mesh> meshes;
        uint32_t totalVertexCount = 0;
    };
    std::vector<PointCloudSlot> pointCloudSlots;

    filament::Material* trajectoryCarpetMaterial = nullptr;
    filament::MaterialInstance* trajectoryCarpetMaterialInstance = nullptr;
    filament::Material* trajectoryCarpetFadedMaterial = nullptr;
    filament::MaterialInstance* trajectoryCarpetFadedMaterialInstance = nullptr;
    float trajectoryCarpetAlpha = 1.0f;

    struct TrajectoryCarpetSlot {
        uint64_t signature = 0;
        bool has_signature = false;
        std::vector<Mesh> meshes;
        uint32_t totalVertexCount = 0;
        float halfWidthM = 0.0f;
        std::vector<uint32_t> firstMeshRgba;
        std::vector<float> firstMeshZ;
        std::vector<std::vector<Vec3>> baseStripPositions;
        std::vector<std::vector<uint32_t>> baseStripRgba;
        std::vector<std::vector<double>> pointStations;
        bool has_applied_clip = false;
        bool appliedClipActive = false;
        int64_t appliedClipUnits = 0;
        bool boundFaded = false;
        Vec3 firstPointM{};
        std::optional<double> fadeOriginStationM;
    };
    std::vector<TrajectoryCarpetSlot> trajectoryCarpetSlots;
    uint64_t trajectoryCarpetRebuildCount = 0;

    filament::Material* hybridSplatMaterial = nullptr;
    filament::MaterialInstance* hybridSplatInstance = nullptr;
    std::vector<Mesh> hybridSplatMeshes;
    uint32_t hybridSplatCount = 0;
    float hybridSplatSizePx = 0.0f;
    float bowlExposure = 1.56f;
    bool hybridStencilOn = false;
    bool hybridGroundNe = false;
    bool hybridGridNe = false;

    uint32_t cameraCount = 0;
    CameraTextureSlot cameraSlots[kMaxBowlCameras];
    bool bowlVisible = false;
    bool selfViewMasksEnabled = false;
    std::unique_ptr<BowlState> bowl;
    filament::MaterialInstance* buildingMaterial = nullptr;
    std::unique_ptr<EnvironmentSource> environmentSource;
    bool environmentVisible = true;
};

void add_mesh(VisualRenderer& r, Mesh& mesh, std::vector<Vertex> verts,
              std::vector<uint16_t> indices, filament::RenderableManager::PrimitiveType primitive,
              filament::MaterialInstance* material, bool cast_shadows, bool receive_shadows);

}
