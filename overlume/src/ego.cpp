// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "ego.hpp"
#include "ego_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "overlume/api.h"
#include "overlume/scene.h"

#include <filament/Box.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

#include <geometry/SurfaceOrientation.h>

#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>

#include <math/mat4.h>
#include <math/quat.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/Entity.h>
#include <utils/EntityManager.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <vector>

namespace overlume {

namespace {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;
using filament::math::quatf;

void build_ego_box(std::vector<Vertex>& verts, std::vector<uint16_t>& indices, const Vec3& dims) {
    const float hx = static_cast<float>(dims.x) * 0.5f;
    const float hy = static_cast<float>(dims.y) * 0.5f;
    const float hz = static_cast<float>(dims.z);
    const float3 p[8] = {
        {-hx, -hy, 0.0f}, {hx, -hy, 0.0f}, {hx, hy, 0.0f}, {-hx, hy, 0.0f},
        {-hx, -hy, hz},   {hx, -hy, hz},   {hx, hy, hz},   {-hx, hy, hz},
    };
    struct Face {
        float3 n;
        int i[4];
    };
    const Face faces[6] = {
        {{0, 0, -1}, {0, 3, 2, 1}}, {{0, 0, 1}, {4, 5, 6, 7}},  {{0, -1, 0}, {0, 1, 5, 4}},
        {{0, 1, 0}, {3, 7, 6, 2}},  {{-1, 0, 0}, {0, 4, 7, 3}}, {{1, 0, 0}, {1, 2, 6, 5}},
    };
    std::vector<float3> normals;
    for (const Face& f : faces) {
        const auto base = static_cast<uint16_t>(verts.size());
        for (int idx : f.i) {
            verts.push_back(Vertex{p[idx], {}});
            normals.push_back(f.n);
        }
        indices.insert(indices.end(),
                       {base, static_cast<uint16_t>(base + 1), static_cast<uint16_t>(base + 2),
                        base, static_cast<uint16_t>(base + 2), static_cast<uint16_t>(base + 3)});
    }
    fill_tangent_frames(verts, normals);
}

void build_ego_fallback(VisualRenderer& r, const Vec3& dims) {
    std::vector<Vertex> verts;
    std::vector<uint16_t> indices;
    build_ego_box(verts, indices, dims);
    add_mesh(r, r.egoFallback, std::move(verts), std::move(indices),
             filament::RenderableManager::PrimitiveType::TRIANGLES, r.egoMaterial, true, false);
    r.egoFallbackDims = dims;
    r.engine->getTransformManager().create(r.egoFallback.entity);
    r.egoTransformEntity = r.egoFallback.entity;
}

}  // namespace

bool set_ego_model(VisualRenderer* r, const char* gltf_path, Vec3 fallback_dims) {
    if (r == nullptr || gltf_path == nullptr) return false;

    std::ifstream file(gltf_path, std::ios::binary | std::ios::ate);
    if (!file) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }

    namespace gltfio = filament::gltfio;

    if (!ensure_gltf_loader(*r)) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }
    gltfio::FilamentAsset* asset =
        r->sharedAssetLoader->createAsset(bytes.data(), static_cast<uint32_t>(bytes.size()));
    if (asset == nullptr) {
        build_ego_fallback(*r, fallback_dims);
        return false;
    }

    if (!r->sharedResourceLoader->loadResources(asset)) {
        r->sharedAssetLoader->destroyAsset(asset);
        build_ego_fallback(*r, fallback_dims);
        return false;
    }
    asset->releaseSourceData();

    filament::RenderableManager& rm = r->engine->getRenderableManager();
    const utils::Entity* renderables = asset->getRenderableEntities();
    const size_t renderableCount = asset->getRenderableEntityCount();
    for (size_t i = 0; i < renderableCount; ++i) {
        const auto inst = rm.getInstance(renderables[i]);
        if (!inst.isValid()) continue;
        const size_t primCount = rm.getPrimitiveCount(inst);
        for (size_t p = 0; p < primCount; ++p) {
            rm.setMaterialInstanceAt(inst, p, r->egoMaterial);
        }
        rm.setCastShadows(inst, true);
        rm.setReceiveShadows(inst, false);
    }

    r->scene->addEntities(asset->getEntities(), asset->getEntityCount());

    r->egoAsset = asset;
    r->egoTransformEntity = asset->getRoot();
    return true;
}

void update_ego_transform(VisualRenderer& r, const EgoState& ego) {
    if (!r.egoTransformEntity) return;
    filament::TransformManager& tm = r.engine->getTransformManager();
    const auto inst = tm.getInstance(r.egoTransformEntity);
    if (!inst.isValid()) return;
    const float3 pos{static_cast<float>(ego.position.x), static_cast<float>(ego.position.y),
                     static_cast<float>(ego.position.z)};
    const quatf rot = quatf::fromAxisAngle(float3{0, 0, 1}, static_cast<float>(ego.heading_rad));
    const float scale = ego.valid ? 1.0f : 0.0f;
    tm.setTransform(inst, mat4f::translation(pos) * mat4f(rot) * mat4f::scaling(scale));
}

}  // namespace overlume

namespace overlume::testing {

double rendered_bounding_box_diagonal(overlume::VisualRenderer* r) {
    if (r == nullptr) return 0.0;
    if (r->egoAsset != nullptr) {
        const filament::Aabb box = r->egoAsset->getBoundingBox();
        const filament::math::float3 d = box.max - box.min;
        return std::sqrt(static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y +
                         static_cast<double>(d.z) * d.z);
    }
    if (r->egoFallback.entity) {
        const Vec3& d = r->egoFallbackDims;
        return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    }
    return 0.0;
}

}  // namespace overlume::testing
