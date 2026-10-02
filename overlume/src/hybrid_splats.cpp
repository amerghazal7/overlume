// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "hybrid_splats.hpp"
#include "hybrid_splats_test_hooks.hpp"

#include "bowl.hpp"
#include "point_cloud.hpp"
#include "polyline.hpp"

#include "hybrid_splat_filamat.h"

#include <filament/Box.h>
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

#include <utils/EntityManager.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace overlume {

namespace {

constexpr float kBoundsM = 200.0f;

using filament::backend::SamplerCompareFunc;

void clear_meshes(VisualRenderer& r) {
    for (auto& m : r.hybridSplatMeshes) destroy_mesh(*r.engine, *r.scene, m);
    r.hybridSplatMeshes.clear();
}

}

void apply_backdrop_stencil(VisualRenderer& r, bool active) {
    r.view->setStencilBufferEnabled(active);
    r.hybridStencilOn = active;
    const SamplerCompareFunc f = active ? SamplerCompareFunc::NE : SamplerCompareFunc::A;
    // Each flag is recorded in the call that applies it, so the test hook
    // reports what was applied, not what was intended.
    auto apply = [&](filament::MaterialInstance* mi, bool& rec) {
        mi->setStencilCompareFunction(f);
        mi->setStencilReferenceValue(1);
        rec = active;
    };
    if (r.bowl) apply(r.bowl->instance, r.bowl->stencilNe);
    apply(r.groundMaterial, r.hybridGroundNe);
    apply(r.gridMaterial, r.hybridGridNe);
}

void create_hybrid_splat_material(VisualRenderer& r) {
    r.hybridSplatMaterial = filament::Material::Builder()
                                .package(overlume::materials::khybrid_splatFilamat,
                                         overlume::materials::khybrid_splatFilamatSize)
                                .build(*r.engine);
    filament::MaterialInstance* mi = r.hybridSplatMaterial->createInstance();
    mi->setCullingMode(filament::backend::CullingMode::NONE);
    mi->setStencilWrite(true);
    mi->setStencilCompareFunction(SamplerCompareFunc::A);
    mi->setStencilOpDepthStencilPass(filament::backend::StencilOperation::REPLACE);
    mi->setStencilReferenceValue(1);
    r.hybridSplatInstance = mi;
}

void destroy_hybrid_splats(VisualRenderer& r) {
    clear_meshes(r);
    if (r.hybridSplatInstance) r.engine->destroy(r.hybridSplatInstance);
    if (r.hybridSplatMaterial) r.engine->destroy(r.hybridSplatMaterial);
    r.hybridSplatInstance = nullptr;
    r.hybridSplatMaterial = nullptr;
}

bool set_hybrid_splats(VisualRenderer* r, const PointCloudPoint* pts, uint32_t count,
                       float size_px) {
    if (r == nullptr) return false;
    clear_meshes(*r);
    r->hybridSplatCount = 0;
    r->hybridSplatSizePx = size_px;
    if (pts == nullptr || count == 0) return true;

    filament::TransformManager& tm = r->engine->getTransformManager();
    for (auto [a, b] : detail::polyline_chunks(count)) {
        const uint32_t n = b - a;
        std::vector<PointVertex> verts(n);
        std::vector<uint16_t> indices(n);
        for (uint32_t i = 0; i < n; ++i) {
            const PointCloudPoint& p = pts[a + i];
            verts[i].position = {static_cast<float>(p.position.x), static_cast<float>(p.position.y),
                                 static_cast<float>(p.position.z)};
            verts[i].rgba = p.rgba;
            indices[i] = static_cast<uint16_t>(i);
        }
        Mesh mesh;
        mesh.vertexCount = n;
        mesh.vb = make_point_vertex_buffer(*r->engine, std::move(verts));
        mesh.ib = make_index_buffer(*r->engine, std::move(indices));
        mesh.entity = utils::EntityManager::get().create();
        filament::RenderableManager::Builder(1)
            .boundingBox({{0, 0, 0}, {kBoundsM, kBoundsM, kBoundsM}})
            .geometry(0, filament::RenderableManager::PrimitiveType::POINTS, mesh.vb, mesh.ib)
            .material(0, r->hybridSplatInstance)
            .priority(0)
            .culling(false)
            .castShadows(false)
            .receiveShadows(false)
            .build(*r->engine, mesh.entity);
        tm.create(mesh.entity);
        r->hybridSplatMeshes.push_back(std::move(mesh));
        r->hybridSplatCount += n;
    }
    return true;
}

void update_hybrid_splats(VisualRenderer& r, const EgoState& ego) {
    const bool active = r.bowlVisible && r.bowl && r.hybridSplatCount > 0;
    if (active != r.hybridStencilOn) apply_backdrop_stencil(r, active);

    filament::TransformManager& tm = r.engine->getTransformManager();
    const filament::math::mat4f xf = ego_anchor_transform(ego);
    for (auto& m : r.hybridSplatMeshes) {
        const bool in = r.scene->hasEntity(m.entity);
        if (active && !in) r.scene->addEntity(m.entity);
        if (!active && in) r.scene->remove(m.entity);
        const auto inst = tm.getInstance(m.entity);
        if (inst.isValid()) tm.setTransform(inst, xf);
    }
    r.hybridSplatInstance->setParameter("pointSizePx", r.hybridSplatSizePx > 0.0f
                                                           ? r.hybridSplatSizePx
                                                           : r.active_theme.hybrid_splat.size_px);
    r.hybridSplatInstance->setParameter("exposureCompensation", r.bowlExposure);
}

namespace testing {

HybridStencilState hybrid_stencil_state_for_test(VisualRenderer* r) {
    if (r == nullptr) return {};
    return {r->view->isStencilBufferEnabled(), r->bowl && r->bowl->stencilNe, r->hybridGroundNe,
            r->hybridGridNe};
}

}

}
