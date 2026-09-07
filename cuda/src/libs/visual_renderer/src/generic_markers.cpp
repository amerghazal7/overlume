// generic_markers.cpp — Epic 2 Task 8 (VM-027): the generic-marker fallback
// renderer, i.e. the spec §7 parity guarantee ("ANY MarkerArray topic
// renders via one YAML row"). Unit meshes for the primitives that don't
// carry their own point data (cube/sphere/cylinder/arrow/text) are built
// ONCE and shared; every marker of that primitive gets its own entity bound
// to the shared geometry, posed per marker via TransformManager.
// LINE_STRIP/LINE_LIST/POINTS/TRIANGLE_LIST bake their own geometry
// straight from the marker's already-world-space `points` (the
// map_elements.cpp/alert_polygons.cpp convention — the adapter has already
// composed marker.pose + FrameTransformer before these ever reach the
// library). MESH goes through the SHARED gltfio loader (Task 4's hoisted
// ensure_gltf_loader()) via the simple non-instanced createAsset() path
// (ego.cpp's precedent — MESH markers aren't grouped into a fixed class
// set the way TrackedObjects are, so there's no natural instancing pool
// key here).
//
// FIXTURE GAP 5 (epic2 plan, Task 8): 7 of the 12 ROS marker types never
// appear in the recorded bag (SPHERE, CYLINDER, CUBE_LIST, SPHERE_LIST,
// POINTS, MESH_RESOURCE, TRIANGLE_LIST) — GenericMarkersGolden.
// EveryPrimitiveType_DarkAdas's scene is entirely synthetic BY DESIGN (the
// backlog AC itself asks for "one of every primitive type", which the real
// traffic never exercises).
#include "generic_markers.hpp"
#include "generic_markers_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "visual_renderer/scene.h"

#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>

#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/ResourceLoader.h>

#include <math/mat4.h>
#include <math/quat.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/Entity.h>
#include <utils/EntityManager.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

namespace mpviz {

namespace {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;
using filament::math::quatf;

constexpr float kPi = 3.14159265358979323846f;

float3 to_f3(const Vec3& v) {
    return float3{static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

uint64_t hash_combine(uint64_t seed, uint64_t v) {
    return seed ^ (v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

uint64_t hash_vec3(const Vec3& v) {
    uint64_t h = std::hash<double>{}(v.x);
    h = hash_combine(h, std::hash<double>{}(v.y));
    h = hash_combine(h, std::hash<double>{}(v.z));
    return h;
}

// Content signature for a LINE_*/POINTS/TRIANGLE_LIST slot's OWN geometry
// (map_elements.cpp's chunk_signature()/alert_polygons.cpp's
// alert_signature() shape: point count + first/last point). The primitive
// itself is already fixed for the slot by the time this is called (a
// primitive change tears the slot down and rebuilds fresh, see
// update_generic_markers() below), so it isn't part of the hash.
uint64_t points_signature(const Vec3* pts, uint32_t n) {
    uint64_t h = hash_combine(0, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    return h;
}

// 8-bit-per-channel quantization for GenericMarker::color (a supplied,
// non-zero-alpha rgb) — the key into VisualRenderer::
// genericMarkerColorInstances. 256^3 possible keys is not a real ceiling in
// practice (see that map's own ponytail comment on eviction, not
// quantization granularity).
uint32_t quantize_color(float r, float g, float b) {
    auto q = [](float c) -> uint32_t {
        return static_cast<uint32_t>(std::clamp(c, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    return (q(r) << 16) | (q(g) << 8) | q(b);
}

detail::Float3 dequantize_color(uint32_t key) {
    return detail::Float3{
        static_cast<float>((key >> 16) & 0xFFu) / 255.0f,
        static_cast<float>((key >> 8) & 0xFFu) / 255.0f,
        static_cast<float>(key & 0xFFu) / 255.0f,
    };
}

// ── Shared unit geometry (built once, lazily, per primitive) ───────────────
// All centered on the marker's own position (ROS Marker convention for
// CUBE/SPHERE/CYLINDER/ARROW: the pose is the geometric center, extending
// +-scale/2 in every axis) — deliberately NOT objects.cpp's
// ground-contact-origin unit box, a different convention for a different
// struct (TrackedObject's measured bbox vs. a marker's own pose+scale).

void build_unit_cube(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    constexpr float h = 0.5f;
    const float3 p[8] = {
        {-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
        {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h},
    };
    struct Face {
        float3 n;
        int i[4];
    };
    const Face faces[6] = {
        {{0, 0, -1}, {0, 3, 2, 1}}, {{0, 0, 1}, {4, 5, 6, 7}}, {{0, -1, 0}, {0, 1, 5, 4}},
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

// Low-poly UV sphere, radius 0.5, Z-up (ROS convention) — enough
// resolution to read as "a sphere" in a 320x240 golden, not a
// production-quality asset.
void build_unit_sphere(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    constexpr int kStacks = 8, kSlices = 12;
    constexpr float kRadius = 0.5f;
    std::vector<float3> normals;
    for (int i = 0; i <= kStacks; ++i) {
        const float phi = kPi * static_cast<float>(i) / static_cast<float>(kStacks);  // 0..pi from +Z
        const float z = std::cos(phi);
        const float ring = std::sin(phi);
        for (int j = 0; j <= kSlices; ++j) {
            const float theta = 2.0f * kPi * static_cast<float>(j) / static_cast<float>(kSlices);
            const float3 n{ring * std::cos(theta), ring * std::sin(theta), z};
            verts.push_back(Vertex{n * kRadius, {}});
            normals.push_back(n);
        }
    }
    const int stride = kSlices + 1;
    for (int i = 0; i < kStacks; ++i) {
        for (int j = 0; j < kSlices; ++j) {
            const auto a = static_cast<uint16_t>(i * stride + j);
            const auto b = static_cast<uint16_t>(a + stride);
            const auto c = static_cast<uint16_t>(a + 1);
            const auto d = static_cast<uint16_t>(b + 1);
            indices.insert(indices.end(), {a, b, c, c, b, d});
        }
    }
    fill_tangent_frames(verts, normals);
}

// Cylinder, radius 0.5 (X/Y), height 1 (Z in [-0.5, 0.5], centered — ROS
// convention), capped top/bottom.
void build_unit_cylinder(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    constexpr int kSegs = 16;
    constexpr float kR = 0.5f, kHz = 0.5f;
    std::vector<float3> normals;
    for (int i = 0; i <= kSegs; ++i) {
        const float theta = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(kSegs);
        const float3 n{std::cos(theta), std::sin(theta), 0.0f};
        verts.push_back(Vertex{{n.x * kR, n.y * kR, -kHz}, {}});
        normals.push_back(n);
        verts.push_back(Vertex{{n.x * kR, n.y * kR, kHz}, {}});
        normals.push_back(n);
    }
    for (int i = 0; i < kSegs; ++i) {
        const auto a = static_cast<uint16_t>(i * 2);
        const auto b = static_cast<uint16_t>(a + 1);
        const auto c = static_cast<uint16_t>(a + 2);
        const auto d = static_cast<uint16_t>(a + 3);
        indices.insert(indices.end(), {a, b, c, c, b, d});
    }
    const auto topCenter = static_cast<uint16_t>(verts.size());
    verts.push_back(Vertex{{0, 0, kHz}, {}});
    normals.push_back({0, 0, 1});
    const auto botCenter = static_cast<uint16_t>(verts.size());
    verts.push_back(Vertex{{0, 0, -kHz}, {}});
    normals.push_back({0, 0, -1});
    const auto topRingStart = static_cast<uint16_t>(verts.size());
    for (int i = 0; i <= kSegs; ++i) {
        const float theta = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(kSegs);
        verts.push_back(Vertex{{std::cos(theta) * kR, std::sin(theta) * kR, kHz}, {}});
        normals.push_back({0, 0, 1});
    }
    const auto botRingStart = static_cast<uint16_t>(verts.size());
    for (int i = 0; i <= kSegs; ++i) {
        const float theta = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(kSegs);
        verts.push_back(Vertex{{std::cos(theta) * kR, std::sin(theta) * kR, -kHz}, {}});
        normals.push_back({0, 0, -1});
    }
    for (int i = 0; i < kSegs; ++i) {
        indices.insert(indices.end(), {topCenter, static_cast<uint16_t>(topRingStart + i),
                                        static_cast<uint16_t>(topRingStart + i + 1)});
        indices.insert(indices.end(), {botCenter, static_cast<uint16_t>(botRingStart + i + 1),
                                        static_cast<uint16_t>(botRingStart + i)});
    }
    fill_tangent_frames(verts, normals);
}

// TEXT placeholder billboard (Step 3: there is no text rendering in this
// library at all — real SDF glyphs are VM-030/Epic 3). A small flat quad,
// fixed size, facing -Y (legible from every committed golden's camera,
// which looks toward the scene from -Y-ish) — translation-only per marker
// (see update_slot_transform()): a placeholder needs to be visible and
// positioned, not scaled/rotated to a real label's metrics.
void build_text_billboard(std::vector<Vertex>& verts, std::vector<uint16_t>& indices) {
    constexpr float h = 0.25f;
    const float3 p[4] = {{-h, 0.0f, -h}, {h, 0.0f, -h}, {h, 0.0f, h}, {-h, 0.0f, h}};
    for (const float3& v : p) verts.push_back(Vertex{v, {}});
    indices = {0, 1, 2, 0, 2, 3};
    fill_tangent_frames(verts, std::vector<float3>(4, float3{0, -1, 0}));
}

void ensure_cube_mesh(VisualRenderer& r) {
    if (r.genericCubeMesh.vb != nullptr) return;
    std::vector<Vertex> v;
    std::vector<uint16_t> idx;
    build_unit_cube(v, idx);
    r.genericCubeMesh.vb = make_vertex_buffer(*r.engine, std::move(v));
    r.genericCubeMesh.ib = make_index_buffer(*r.engine, std::move(idx));
}
void ensure_sphere_mesh(VisualRenderer& r) {
    if (r.genericSphereMesh.vb != nullptr) return;
    std::vector<Vertex> v;
    std::vector<uint16_t> idx;
    build_unit_sphere(v, idx);
    r.genericSphereMesh.vb = make_vertex_buffer(*r.engine, std::move(v));
    r.genericSphereMesh.ib = make_index_buffer(*r.engine, std::move(idx));
}
void ensure_cylinder_mesh(VisualRenderer& r) {
    if (r.genericCylinderMesh.vb != nullptr) return;
    std::vector<Vertex> v;
    std::vector<uint16_t> idx;
    build_unit_cylinder(v, idx);
    r.genericCylinderMesh.vb = make_vertex_buffer(*r.engine, std::move(v));
    r.genericCylinderMesh.ib = make_index_buffer(*r.engine, std::move(idx));
}
// ARROW reuses r.sharedArrowMesh -- the SAME GPU mesh objects.cpp's velocity
// arrows draw, filled from the promoted shared build_unit_arrow()
// (review 2026-08-20: this TU shipped a verbatim copy + a second GPU
// allocation of identical geometry).
void ensure_arrow_mesh(VisualRenderer& r) {
    if (r.sharedArrowMesh.vb != nullptr) return;
    std::vector<Vertex> v;
    std::vector<uint16_t> idx;
    build_unit_arrow(v, idx);
    r.sharedArrowMesh.vb = make_vertex_buffer(*r.engine, std::move(v));
    r.sharedArrowMesh.ib = make_index_buffer(*r.engine, std::move(idx));
}
void ensure_text_mesh(VisualRenderer& r) {
    if (r.genericTextMesh.vb != nullptr) return;
    std::vector<Vertex> v;
    std::vector<uint16_t> idx;
    build_text_billboard(v, idx);
    r.genericTextMesh.vb = make_vertex_buffer(*r.engine, std::move(v));
    r.genericTextMesh.ib = make_index_buffer(*r.engine, std::move(idx));
}

filament::RenderableManager::PrimitiveType to_filament_primitive(MarkerPrimitive p) {
    switch (p) {
        case MarkerPrimitive::LINE_STRIP:
            return filament::RenderableManager::PrimitiveType::LINE_STRIP;
        case MarkerPrimitive::LINE_LIST:
            return filament::RenderableManager::PrimitiveType::LINES;
        case MarkerPrimitive::POINTS:
            return filament::RenderableManager::PrimitiveType::POINTS;
        default:
            return filament::RenderableManager::PrimitiveType::TRIANGLES;  // TRIANGLE_LIST
    }
}

// Creates (once) the entity a shared-geometry slot (CUBE/SPHERE/CYLINDER/
// ARROW/TEXT) needs, bound to `shared`'s vb/ib. Its own TransformManager
// component is created here too (update_slot_transform() drives it every
// frame) — this IS a new GPU/ECS allocation (a fresh entity), counted.
void ensure_slot_shared_entity(VisualRenderer& r, VisualRenderer::GenericMarkerSlot& slot,
                                const Mesh& shared, filament::MaterialInstance* initialMaterial) {
    if (slot.sharedGeomEntity) return;
    slot.sharedGeomEntity = utils::EntityManager::get().create();
    filament::RenderableManager::Builder(1)
        .boundingBox({{0, 0, 0}, {50.0f, 50.0f, 50.0f}})  // culling(false) below -- exact box irrelevant
        .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, shared.vb, shared.ib)
        .material(0, initialMaterial)
        .culling(false)
        .castShadows(false)
        .receiveShadows(true)
        .build(*r.engine, slot.sharedGeomEntity);
    r.scene->addEntity(slot.sharedGeomEntity);
    r.engine->getTransformManager().create(slot.sharedGeomEntity);
    ++r.genericMarkerAllocCount;
}

// LINE_STRIP/LINE_LIST/POINTS/TRIANGLE_LIST: this slot's OWN geometry,
// baked directly from the marker's already-world-space `points` (no
// TransformManager transform — see this file's header comment). Rebuilt
// only when the content signature changes.
void update_own_mesh_geometry(VisualRenderer& r, const GenericMarker& m,
                               VisualRenderer::GenericMarkerSlot& slot) {
    const uint64_t sig = points_signature(m.points, m.point_count);
    if (slot.hasGeomSignature && slot.geomSignature == sig) return;
    if (slot.ownMesh.vb != nullptr) destroy_mesh(*r.engine, *r.scene, slot.ownMesh);
    slot.hasGeomSignature = true;
    slot.geomSignature = sig;
    if (m.points == nullptr || m.point_count == 0) return;  // malformed -- nothing to render this frame

    std::vector<Vertex> verts(m.point_count);
    for (uint32_t i = 0; i < m.point_count; ++i) verts[i].position = to_f3(m.points[i]);
    fill_tangent_frames(verts, std::vector<float3>(m.point_count, float3{0, 0, 1}));
    // Flat sequential indexing + uint16 guard -- map_elements.cpp/
    // alert_polygons.cpp's precedent (a generic marker is never
    // ribbon.cpp's 32000-point scale).
    if (verts.size() > 65535) {
        std::fprintf(stderr,
                     "[visual_renderer] generic marker mesh (%zu verts) exceeds the uint16 index "
                     "ceiling; marker dropped\n",
                     verts.size());
        return;
    }
    std::vector<uint16_t> indices(verts.size());
    for (size_t i = 0; i < indices.size(); ++i) indices[i] = static_cast<uint16_t>(i);
    add_mesh(r, slot.ownMesh, std::move(verts), std::move(indices), to_filament_primitive(m.primitive),
             r.genericMarkerMaterial, /*cast_shadows=*/false, /*receive_shadows=*/true);
    ++r.genericMarkerAllocCount;
}

// MESH: loads (or re-loads, on a path change) `m.mesh_path` through the
// shared gltfio loader (Task 4's ensure_gltf_loader()); any failure (null/
// missing/unparseable path) falls back to a clay box built into
// slot.ownMesh, WARNed once per distinct failing path (spec §9).
void update_mesh_geometry(VisualRenderer& r, const GenericMarker& m,
                          VisualRenderer::GenericMarkerSlot& slot) {
    const std::string path = m.mesh_path != nullptr ? std::string(m.mesh_path) : std::string();
    // Unchanged since the last publish (same path, same success/fallback
    // outcome) -- nothing to (re)build.
    if (slot.meshPathLoaded == path && (slot.meshAsset != nullptr || slot.meshIsFallback)) return;

    if (slot.meshAsset != nullptr) {
        r.scene->removeEntities(slot.meshAsset->getEntities(), slot.meshAsset->getEntityCount());
        r.sharedAssetLoader->destroyAsset(slot.meshAsset);
        slot.meshAsset = nullptr;
    }
    if (slot.ownMesh.vb != nullptr) destroy_mesh(*r.engine, *r.scene, slot.ownMesh);
    slot.meshIsFallback = false;
    slot.meshPathLoaded = path;

    bool loaded = false;
    if (!path.empty()) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (file) {
            const std::streamsize size = file.tellg();
            if (size > 0) {
                std::vector<uint8_t> bytes(static_cast<size_t>(size));
                file.seekg(0);
                if (file.read(reinterpret_cast<char*>(bytes.data()), size) && ensure_gltf_loader(r)) {
                    filament::gltfio::FilamentAsset* asset = r.sharedAssetLoader->createAsset(
                        bytes.data(), static_cast<uint32_t>(bytes.size()));
                    if (asset != nullptr) {
                        if (r.sharedResourceLoader->loadResources(asset)) {
                            asset->releaseSourceData();
                            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
                            slot.meshAsset = asset;
                            loaded = true;
                        } else {
                            r.sharedAssetLoader->destroyAsset(asset);
                        }
                    }
                }
            }
        }
    }

    if (!loaded) {
        // WARN once per distinct failing path (spec §9's "asset load
        // failure -> clay-box fallback, WARN once") -- objectClassMissing
        // Warned's per-CLASS shape, keyed by path here since MESH markers
        // aren't grouped into a fixed class set.
        if (r.genericMarkerMeshWarned.insert(path).second) {
            std::fprintf(stderr,
                         "[visual_renderer] generic marker mesh_path '%s' failed to load; "
                         "falling back to a clay box\n",
                         path.c_str());
        }
        std::vector<Vertex> verts;
        std::vector<uint16_t> indices;
        build_unit_cube(verts, indices);
        add_mesh(r, slot.ownMesh, std::move(verts), std::move(indices),
                 filament::RenderableManager::PrimitiveType::TRIANGLES, r.genericMarkerMaterial,
                 /*cast_shadows=*/true, /*receive_shadows=*/false);
        r.engine->getTransformManager().create(slot.ownMesh.entity);
        slot.meshIsFallback = true;
    }
    ++r.genericMarkerAllocCount;
}

void update_slot_geometry(VisualRenderer& r, const GenericMarker& m,
                          VisualRenderer::GenericMarkerSlot& slot) {
    switch (m.primitive) {
        case MarkerPrimitive::CUBE:
            ensure_cube_mesh(r);
            ensure_slot_shared_entity(r, slot, r.genericCubeMesh, r.genericMarkerMaterial);
            break;
        case MarkerPrimitive::SPHERE:
            ensure_sphere_mesh(r);
            ensure_slot_shared_entity(r, slot, r.genericSphereMesh, r.genericMarkerMaterial);
            break;
        case MarkerPrimitive::CYLINDER:
            ensure_cylinder_mesh(r);
            ensure_slot_shared_entity(r, slot, r.genericCylinderMesh, r.genericMarkerMaterial);
            break;
        case MarkerPrimitive::ARROW:
            ensure_arrow_mesh(r);
            ensure_slot_shared_entity(r, slot, r.sharedArrowMesh, r.genericMarkerMaterial);
            break;
        case MarkerPrimitive::TEXT:
            ensure_text_mesh(r);
            ensure_slot_shared_entity(r, slot, r.genericTextMesh, r.genericMarkerMaterial);
            break;
        case MarkerPrimitive::LINE_STRIP:
        case MarkerPrimitive::LINE_LIST:
        case MarkerPrimitive::POINTS:
        case MarkerPrimitive::TRIANGLE_LIST:
            update_own_mesh_geometry(r, m, slot);
            break;
        case MarkerPrimitive::MESH:
            update_mesh_geometry(r, m, slot);
            break;
    }
}

// TransformManager pose for the primitives that carry position/heading_rad/
// scale (everything except LINE_*/POINTS/TRIANGLE_LIST, whose geometry is
// already baked in world space -- see update_own_mesh_geometry()). TEXT is
// translation-only (this file's build_text_billboard() comment).
void update_slot_transform(VisualRenderer& r, const GenericMarker& m,
                          VisualRenderer::GenericMarkerSlot& slot) {
    utils::Entity xformEntity;
    switch (slot.primitive) {
        case MarkerPrimitive::CUBE:
        case MarkerPrimitive::SPHERE:
        case MarkerPrimitive::CYLINDER:
        case MarkerPrimitive::ARROW:
        case MarkerPrimitive::TEXT:
            xformEntity = slot.sharedGeomEntity;
            break;
        case MarkerPrimitive::MESH:
            xformEntity = slot.meshIsFallback
                              ? slot.ownMesh.entity
                              : (slot.meshAsset != nullptr ? slot.meshAsset->getRoot() : utils::Entity{});
            break;
        default:
            return;  // LINE_*/POINTS/TRIANGLE_LIST -- no transform at all
    }
    if (!xformEntity) return;
    filament::TransformManager& tm = r.engine->getTransformManager();
    const auto inst = tm.getInstance(xformEntity);
    if (!inst.isValid()) return;
    const float3 pos = to_f3(m.position);
    if (slot.primitive == MarkerPrimitive::TEXT) {
        tm.setTransform(inst, mat4f::translation(pos));
        return;
    }
    const quatf rot = quatf::fromAxisAngle(float3{0, 0, 1}, static_cast<float>(m.heading_rad));
    const float3 scale = to_f3(m.scale);
    tm.setTransform(inst, mat4f::translation(pos) * mat4f(rot) * mat4f::scaling(scale));
}

// GenericMarker::color alpha==0 -> theme-neutral default (this file's
// header comment / renderer_internal.hpp's genericMarkerMaterial); else a
// per-quantized-color clay.mat instance, created lazily and pooled by
// VisualRenderer::genericMarkerColorInstances (that map's own ponytail
// comment covers the no-eviction ceiling).
filament::MaterialInstance* resolve_marker_material(VisualRenderer& r, const GenericMarker& m,
                                                     detail::Float3& outTint) {
    if (m.color[3] == 0.0f) {
        outTint = r.genericMarkerNeutralTint;
        return r.genericMarkerMaterial;
    }
    const uint32_t key = quantize_color(m.color[0], m.color[1], m.color[2]);
    auto it = r.genericMarkerColorInstances.find(key);
    if (it != r.genericMarkerColorInstances.end()) {
        outTint = dequantize_color(key);
        return it->second;
    }
    filament::MaterialInstance* inst = r.clayMaterial->createInstance();
    inst->setCullingMode(filament::backend::CullingMode::NONE);
    const detail::Float3 tint = dequantize_color(key);
    inst->setParameter("baseColor", float3{tint.r, tint.g, tint.b});
    inst->setParameter("roughness", r.active_theme.material.roughness);
    inst->setParameter("metallic", r.active_theme.material.metallic);
    r.genericMarkerColorInstances.emplace(key, inst);
    outTint = tint;
    return inst;
}

// Rebinds every one of slot `slot`'s renderable entities to `mat` -- the
// fresh<->stale swap needs this every frame, mirroring objects.cpp's
// remap_to_material()/alert_polygons.cpp's rebind_slot_material(),
// generalized over this file's three geometry shapes (single
// shared-geometry entity, single own-mesh entity, or a whole glTF asset's
// N renderable entities).
void bind_marker_material(filament::RenderableManager& rm, VisualRenderer::GenericMarkerSlot& slot,
                          filament::MaterialInstance* mat) {
    if (slot.sharedGeomEntity) {
        const auto ri = rm.getInstance(slot.sharedGeomEntity);
        if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
        return;
    }
    if (slot.ownMesh.entity) {
        const auto ri = rm.getInstance(slot.ownMesh.entity);
        if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
        return;
    }
    if (slot.meshAsset != nullptr) {
        const utils::Entity* ents = slot.meshAsset->getRenderableEntities();
        const size_t n = slot.meshAsset->getRenderableEntityCount();
        for (size_t i = 0; i < n; ++i) {
            const auto ri = rm.getInstance(ents[i]);
            if (!ri.isValid()) continue;
            const size_t primCount = rm.getPrimitiveCount(ri);
            for (size_t p = 0; p < primCount; ++p) rm.setMaterialInstanceAt(ri, p, mat);
        }
    }
}

// Staleness fade -- the exact clay_translucent.mat per-entity swap
// mechanism objects.cpp/ribbon.cpp/alert_polygons.cpp already established
// (see the plan's "…and the material that can actually do it"). Runs every
// frame regardless of whether geometry rebuilt this tick, since a marker's
// resolved material (fresh template identity, or the staleness ramp) can
// change even when its geometry/pose didn't.
void update_slot_material(VisualRenderer& r, const GenericMarker& m,
                          VisualRenderer::GenericMarkerSlot& slot, double sim_time_sec) {
    detail::Float3 tint{};
    filament::MaterialInstance* freshMat = resolve_marker_material(r, m, tint);
    const auto staleness = static_cast<float>(detail::SceneBuffer::staleness_alpha(
        sim_time_sec, m.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
    filament::RenderableManager& rm = r.engine->getRenderableManager();

    if (staleness >= 1.0f) {
        if (slot.fadeInstance != nullptr) {
            r.engine->destroy(slot.fadeInstance);
            slot.fadeInstance = nullptr;
        }
        bind_marker_material(rm, slot, freshMat);
        slot.fadeAlpha = 1.0f;
    } else {
        if (slot.fadeInstance == nullptr) {
            slot.fadeInstance = r.clayTranslucentMaterial->createInstance();
            slot.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
        }
        slot.fadeInstance->setParameter("baseColor", float4{tint.r, tint.g, tint.b, staleness});
        slot.fadeInstance->setParameter("roughness", r.active_theme.material.roughness);
        slot.fadeInstance->setParameter("metallic", r.active_theme.material.metallic);
        bind_marker_material(rm, slot, slot.fadeInstance);
        slot.fadeAlpha = staleness;
    }
    slot.tint = tint;
}

void release_generic_marker_slot(VisualRenderer& r, VisualRenderer::GenericMarkerSlot& slot) {
    if (slot.fadeInstance != nullptr) {
        r.engine->destroy(slot.fadeInstance);
        slot.fadeInstance = nullptr;
    }
    if (slot.sharedGeomEntity) {
        r.scene->remove(slot.sharedGeomEntity);
        r.engine->destroy(slot.sharedGeomEntity);
        utils::EntityManager::get().destroy(slot.sharedGeomEntity);
        slot.sharedGeomEntity = {};
    }
    if (slot.ownMesh.vb != nullptr) destroy_mesh(*r.engine, *r.scene, slot.ownMesh);
    if (slot.meshAsset != nullptr) {
        r.scene->removeEntities(slot.meshAsset->getEntities(), slot.meshAsset->getEntityCount());
        r.sharedAssetLoader->destroyAsset(slot.meshAsset);
        slot.meshAsset = nullptr;
    }
    slot.meshPathLoaded.clear();
    slot.meshIsFallback = false;
    slot.hasGeomSignature = false;
    slot.geomSignature = 0;
    slot.tint = {};
    slot.fadeAlpha = 1.0f;
    slot.active = false;
}

}  // namespace

void update_generic_markers(VisualRenderer& r, const SceneGraph& s) {
    while (r.genericMarkerSlots.size() > s.marker_count) {
        release_generic_marker_slot(r, r.genericMarkerSlots.back());
        r.genericMarkerSlots.pop_back();
    }
    if (r.genericMarkerSlots.size() < s.marker_count) r.genericMarkerSlots.resize(s.marker_count);

    for (uint32_t i = 0; i < s.marker_count; ++i) {
        const GenericMarker& m = s.markers[i];
        VisualRenderer::GenericMarkerSlot& slot = r.genericMarkerSlots[i];

        // The frozen scene.h enum has exactly 10 values (0..9). A raw
        // out-of-range uint8_t here means a caller (never this epic's own
        // adapter, see generic_marker.cpp's ROS-type table) constructed a
        // GenericMarker the library doesn't know how to draw --
        // UnknownOrUnsupportedPrimitiveIsSkippedAndCounted, the library's
        // own defensive floor (collision.cpp's severity_for_role() is the
        // same "assert-don't-default" shape for a different category).
        if (static_cast<uint8_t>(m.primitive) > 9) {
            if (slot.active) release_generic_marker_slot(r, slot);
            ++r.genericMarkerUnknownCount;
            continue;
        }

        if (!slot.active || slot.primitive != m.primitive) {
            if (slot.active) release_generic_marker_slot(r, slot);
            slot.primitive = m.primitive;
            slot.active = true;
        }

        update_slot_geometry(r, m, slot);
        update_slot_transform(r, m, slot);
        update_slot_material(r, m, slot, s.sim_time_sec);
    }
}

}  // namespace mpviz

// Epic 2 Task 8 (VM-027): Filament-free test introspection hooks (see
// generic_markers_test_hooks.hpp's own comment for why these live here,
// mirroring alert_polygons.cpp/objects.cpp's own hook definitions).
namespace mpviz::testing {

size_t generic_marker_slot_count(mpviz::VisualRenderer* r) {
    return r == nullptr ? 0 : r->genericMarkerSlots.size();
}

uint32_t generic_marker_alloc_count(mpviz::VisualRenderer* r) {
    return r == nullptr ? 0 : r->genericMarkerAllocCount;
}

uint32_t generic_marker_unknown_count(mpviz::VisualRenderer* r) {
    return r == nullptr ? 0 : r->genericMarkerUnknownCount;
}

bool generic_marker_mesh_is_fallback(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->genericMarkerSlots.size()) return false;
    const auto& s = r->genericMarkerSlots[slot];
    return s.primitive == mpviz::MarkerPrimitive::MESH && s.meshIsFallback;
}

mpviz::detail::Float3 generic_marker_tint(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->genericMarkerSlots.size()) return {};
    return r->genericMarkerSlots[slot].tint;
}

GenericMarkerMaterialInfo generic_marker_material_info(mpviz::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->genericMarkerSlots.size()) return {};
    const auto& s = r->genericMarkerSlots[slot];
    GenericMarkerMaterialInfo info;
    info.alpha = s.fadeAlpha;
    filament::RenderableManager& rm = r->engine->getRenderableManager();
    utils::Entity ent;
    if (s.sharedGeomEntity) {
        ent = s.sharedGeomEntity;
    } else if (s.ownMesh.entity) {
        ent = s.ownMesh.entity;
    } else if (s.meshAsset != nullptr && s.meshAsset->getRenderableEntityCount() > 0) {
        ent = s.meshAsset->getRenderableEntities()[0];
    }
    if (!ent) return info;
    const auto ri = rm.getInstance(ent);
    if (!ri.isValid()) return info;
    filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    info.bound_to_translucent = bound != nullptr && bound->getMaterial() == r->clayTranslucentMaterial;
    return info;
}

}  // namespace mpviz::testing
