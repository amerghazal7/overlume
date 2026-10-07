// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "map_elements.hpp"
#include "polyline.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include <filament/RenderableManager.h>

#include <math/vec3.h>
#include <math/vec4.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace overlume {

namespace {

using filament::math::float3;
using filament::math::float4;

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

uint64_t chunk_signature(bool is_polygon, const Vec3* pts, uint32_t n) {
    uint64_t h = is_polygon ? 0x1ULL : 0x0ULL;
    h = hash_combine(h, static_cast<uint64_t>(n));
    if (n > 0) {
        h = hash_combine(h, hash_vec3(pts[0]));
        h = hash_combine(h, hash_vec3(pts[n - 1]));
    }
    return h;
}

std::vector<Vertex> to_verts(const std::vector<Vec3>& positions) {
    std::vector<Vertex> verts(positions.size());
    std::vector<float3> normals(positions.size(), float3{0.0f, 0.0f, 1.0f});
    for (size_t i = 0; i < positions.size(); ++i) verts[i].position = to_f3(positions[i]);
    fill_tangent_frames(verts, normals);
    return verts;
}

}

constexpr double kCrosswalkStripePitchM = 1.2;
constexpr int kCrosswalkStripesMin = 3;
constexpr int kCrosswalkStripesMax = 24;

std::vector<Vec3> detail::build_crosswalk_hatch(const Vec3* pts, uint32_t n, float z_lift) {
    std::vector<Vec3> tris;
    if (n != 4) return tris;
    auto edge_len = [&](int a, int b) {
        const double dx = pts[a].x - pts[b].x;
        const double dy = pts[a].y - pts[b].y;
        return std::sqrt(dx * dx + dy * dy);
    };
    const double lenA = edge_len(0, 1) + edge_len(2, 3);
    const double lenB = edge_len(1, 2) + edge_len(3, 0);
    Vec3 r0a, r0b, r1a, r1b;
    double longAxisLen;
    if (lenA >= lenB) {
        r0a = pts[0];
        r0b = pts[1];
        r1a = pts[3];
        r1b = pts[2];
        longAxisLen = lenA / 2.0;
    } else {
        r0a = pts[1];
        r0b = pts[2];
        r1a = pts[0];
        r1b = pts[3];
        longAxisLen = lenB / 2.0;
    }
    const int kStripes =
        std::clamp(static_cast<int>(std::lround(longAxisLen / kCrosswalkStripePitchM)),
                   kCrosswalkStripesMin, kCrosswalkStripesMax);
    auto lerp = [](const Vec3& a, const Vec3& b, double t) {
        return Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    };
    auto lift = [&](Vec3 v) {
        v.z += z_lift;
        return v;
    };
    for (int i = 0; i < kStripes; ++i) {
        const double t0 = (2.0 * i) / (2.0 * kStripes);
        const double t1 = (2.0 * i + 1.0) / (2.0 * kStripes);
        const Vec3 a0 = lift(lerp(r0a, r0b, t0));
        const Vec3 a1 = lift(lerp(r0a, r0b, t1));
        const Vec3 b0 = lift(lerp(r1a, r1b, t0));
        const Vec3 b1 = lift(lerp(r1a, r1b, t1));
        tris.push_back(a0);
        tris.push_back(a1);
        tris.push_back(b1);
        tris.push_back(a0);
        tris.push_back(b1);
        tris.push_back(b0);
    }
    return tris;
}

namespace {

constexpr float kLaneHalfWidthM = 0.05f;
constexpr float kRoadZLiftM = 0.005f;

constexpr float kOtherZLiftM = 0.016f;
constexpr float kCenterlineZLiftM = 0.017f;
constexpr float kLeftBoundaryZLiftM = 0.018f;
constexpr float kRightBoundaryZLiftM = 0.019f;
constexpr float kRoadEdgeZLiftM = 0.020f;
constexpr float kJunctionZLiftM = 0.021f;
constexpr float kStoplineZLiftM = 0.022f;
constexpr float kCrosswalkZLiftM = 0.023f;

bool IsBoundaryKind(MapKind kind) {
    return kind == MapKind::LEFT_BOUNDARY || kind == MapKind::RIGHT_BOUNDARY;
}

float z_lift_for_kind(MapKind kind) {
    switch (kind) {
        case MapKind::ROAD_SURFACE:
            return kRoadZLiftM;
        case MapKind::CENTERLINE:
            return kCenterlineZLiftM;
        case MapKind::LEFT_BOUNDARY:
            return kLeftBoundaryZLiftM;
        case MapKind::RIGHT_BOUNDARY:
            return kRightBoundaryZLiftM;
        case MapKind::ROAD_EDGE:
            return kRoadEdgeZLiftM;
        case MapKind::JUNCTION:
            return kJunctionZLiftM;
        case MapKind::STOPLINE:
            return kStoplineZLiftM;
        case MapKind::CROSSWALK:
            return kCrosswalkZLiftM;
        default:
            return kOtherZLiftM;
    }
}

filament::MaterialInstance* material_for_kind(VisualRenderer& r, MapKind kind) {
    switch (kind) {
        case MapKind::CENTERLINE:
            return r.laneCenterlineMaterial;
        case MapKind::LEFT_BOUNDARY:
        case MapKind::RIGHT_BOUNDARY:
            return r.laneBoundaryMaterial;
        case MapKind::CROSSWALK:
            return r.crosswalkMaterial;
        case MapKind::ROAD_SURFACE:
            return r.roadMaterial;
        case MapKind::ROAD_EDGE:
            return r.roadEdgeMaterial;
        default:
            return r.laneMaterial;
    }
}

detail::Float3 tint_for_kind(const VisualRenderer& r, MapKind kind) {
    switch (kind) {
        case MapKind::CENTERLINE:
            return r.laneCenterlineMaterialBaseColor;
        case MapKind::LEFT_BOUNDARY:
        case MapKind::RIGHT_BOUNDARY:
            return r.laneBoundaryMaterialBaseColor;
        case MapKind::CROSSWALK:
            return r.crosswalkMaterialBaseColor;
        case MapKind::ROAD_SURFACE:
            return r.roadMaterialBaseColor;
        case MapKind::ROAD_EDGE:
            return r.roadEdgeMaterialBaseColor;
        default:
            return r.laneMaterialBaseColor;
    }
}

constexpr double kDashLenM = 1.5;
constexpr double kGapLenM = 1.5;
constexpr double kMinDashLenM = 0.25;

double dash_dist(const Vec3& a, const Vec3& b) {
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Vec3 point_at_arc_length(const Vec3* pts, uint32_t n, const std::vector<double>& cum, double s) {
    const double total_len = cum.back();
    s = std::clamp(s, 0.0, total_len);
    size_t i = static_cast<size_t>(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
    if (i == 0) i = 1;
    if (i >= n) i = n - 1;
    const double seg_len = cum[i] - cum[i - 1];
    const double t = seg_len > 0.0 ? (s - cum[i - 1]) / seg_len : 0.0;
    const Vec3& a = pts[i - 1];
    const Vec3& b = pts[i];
    return Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

std::vector<std::vector<Vec3>> chop_into_dashes(const Vec3* pts, uint32_t n) {
    std::vector<std::vector<Vec3>> out;
    if (n < 2) return out;

    std::vector<double> cum(n, 0.0);
    for (uint32_t i = 1; i < n; ++i) cum[i] = cum[i - 1] + dash_dist(pts[i - 1], pts[i]);
    const double total_len = cum.back();
    if (total_len <= 0.0) return out;

    constexpr double kPeriod = kDashLenM + kGapLenM;
    for (double s0 = 0.0; s0 < total_len; s0 += kPeriod) {
        const double s1 = std::min(s0 + kDashLenM, total_len);
        if (s1 - s0 < kMinDashLenM) continue;
        std::vector<Vec3> dash;
        dash.push_back(point_at_arc_length(pts, n, cum, s0));
        for (uint32_t i = 0; i < n; ++i) {
            if (cum[i] > s0 && cum[i] < s1) dash.push_back(pts[i]);
        }
        dash.push_back(point_at_arc_length(pts, n, cum, s1));
        out.push_back(std::move(dash));
    }
    return out;
}

constexpr float kCenterlineDotRadiusM = 0.15f;
constexpr float kCenterlineDotSpacingM = 2.0f;
constexpr int kCenterlineDotSegments = 10;
constexpr float kTwoPi = 6.28318530717958647692f;

std::vector<Vec3> build_centerline_dots(const Vec3* pts, uint32_t n, float z_lift) {
    std::vector<Vec3> tris;
    if (n < 2) return tris;

    std::vector<double> cum(n, 0.0);
    for (uint32_t i = 1; i < n; ++i) cum[i] = cum[i - 1] + dash_dist(pts[i - 1], pts[i]);
    const double total_len = cum.back();
    if (total_len <= 0.0) return tris;

    for (double s = 0.0; s <= total_len; s += kCenterlineDotSpacingM) {
        const Vec3 c = point_at_arc_length(pts, n, cum, s);
        const Vec3 centre{c.x, c.y, c.z + z_lift};
        for (int k = 0; k < kCenterlineDotSegments; ++k) {
            const float a0 = kTwoPi * static_cast<float>(k) / kCenterlineDotSegments;
            const float a1 = kTwoPi * static_cast<float>(k + 1) / kCenterlineDotSegments;
            tris.push_back(centre);
            tris.push_back(Vec3{c.x + kCenterlineDotRadiusM * std::cos(a0),
                                c.y + kCenterlineDotRadiusM * std::sin(a0), centre.z});
            tris.push_back(Vec3{c.x + kCenterlineDotRadiusM * std::cos(a1),
                                c.y + kCenterlineDotRadiusM * std::sin(a1), centre.z});
        }
    }
    return tris;
}

std::vector<Vec3> build_ribbon_flat(const Vec3* pts, uint32_t n, float half_width, float z_lift) {
    std::vector<Vec3> ribbon = detail::extrude_polyline(pts, n, half_width, z_lift);
    if (ribbon.empty()) return ribbon;
    const auto stripIdx =
        detail::extrude_polyline_indices(static_cast<uint32_t>(ribbon.size() / 2));
    std::vector<Vec3> flat(stripIdx.size());
    for (size_t k = 0; k < stripIdx.size(); ++k) flat[k] = ribbon[stripIdx[k]];
    return flat;
}

std::vector<Vec3> build_road_strip(const Vec3* pts, uint32_t point_count, float z_lift) {
    std::vector<Vec3> tris;
    if (point_count < 4 || point_count % 2 != 0) return tris;
    const uint32_t n = point_count / 2;
    const Vec3* left = pts;
    const Vec3* right = pts + n;
    const auto lift = [&](Vec3 v) {
        v.z += z_lift;
        return v;
    };
    tris.reserve(static_cast<size_t>(n - 1) * 6);
    for (uint32_t i = 0; i + 1 < n; ++i) {
        const Vec3 l0 = lift(left[i]), l1 = lift(left[i + 1]);
        const Vec3 r0 = lift(right[i]), r1 = lift(right[i + 1]);
        tris.push_back(l0);
        tris.push_back(l1);
        tris.push_back(r1);
        tris.push_back(l0);
        tris.push_back(r1);
        tris.push_back(r0);
    }
    return tris;
}

void apply_map_element_staleness(VisualRenderer& r, Mesh& mesh, MapKind kind,
                                 double last_update_sec, double sim_time_sec, bool ego_valid) {
    if (!mesh.entity) return;
    const auto staleness = static_cast<float>(detail::SceneBuffer::staleness_alpha(
        sim_time_sec, last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec));
    const float alpha = ego_valid ? staleness : 0.0f;

    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto ri = rm.getInstance(mesh.entity);

    if (alpha >= 1.0f) {
        if (mesh.fadeInstance != nullptr) {
            if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, material_for_kind(r, kind));
            r.engine->destroy(mesh.fadeInstance);
            mesh.fadeInstance = nullptr;
        }
        mesh.fadeAlpha = 1.0f;
        return;
    }
    if (mesh.fadeInstance == nullptr) {
        mesh.fadeInstance = r.clayTranslucentMaterial->createInstance();
        mesh.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
        if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mesh.fadeInstance);
    }
    const detail::Float3 tint = tint_for_kind(r, kind);
    mesh.fadeInstance->setParameter("baseColor", float4{tint.r, tint.g, tint.b, alpha});
    mesh.fadeInstance->setParameter("roughness", r.active_theme.material.roughness);
    mesh.fadeInstance->setParameter("metallic", r.active_theme.material.metallic);
    mesh.fadeAlpha = alpha;
}

}

void update_map_elements(VisualRenderer& r, const SceneGraph& s) {
    std::unordered_map<uint64_t, Mesh> next;
    next.reserve(r.mapElementMeshes.size());

    auto adopt_or_build = [&](uint64_t key, filament::MaterialInstance* material, MapKind kind,
                              double last_update_sec, auto build_fn) {
        if (next.count(key) != 0) return;
        auto it = r.mapElementMeshes.find(key);
        if (it != r.mapElementMeshes.end()) {
            next.emplace(key, std::move(it->second));
            r.mapElementMeshes.erase(it);
        } else {
            ++r.mapElementRebuildCount;
            std::vector<Vec3> positions = build_fn();
            if (positions.empty()) return;
            std::vector<Vertex> verts = to_verts(positions);
            if (verts.size() > 65535) {
                std::fprintf(stderr,
                             "[overlume] map element mesh (%zu verts) exceeds the uint16 "
                             "index ceiling; element dropped\n",
                             verts.size());
                return;
            }
            std::vector<uint16_t> indices(verts.size());
            for (size_t i = 0; i < verts.size(); ++i) indices[i] = static_cast<uint16_t>(i);
            Mesh mesh;
            add_mesh(r, mesh, std::move(verts), std::move(indices),
                     filament::RenderableManager::PrimitiveType::TRIANGLES, material, false, true);
            next.emplace(key, std::move(mesh));
        }
        apply_map_element_staleness(r, next.at(key), kind, last_update_sec, s.sim_time_sec,
                                    s.ego.valid != 0);
    };

    for (uint32_t i = 0; i < s.map_element_count; ++i) {
        const MapElement& e = s.map_elements[i];
        if (e.points == nullptr || e.point_count < 2) continue;

        filament::MaterialInstance* const material = material_for_kind(r, e.kind);
        const float z_lift = z_lift_for_kind(e.kind);

        if (e.kind == MapKind::ROAD_SURFACE) {
            // is_polygon=1: closed outline -> fan; is_polygon=0: paired rails [left;right] ->
            // strip. ponytail: fan assumes a convex outline; concave road outlines need ear
            // clipping.
            const bool poly = e.is_polygon != 0;
            const uint64_t key = chunk_signature(poly, e.points, e.point_count);
            adopt_or_build(key, material, e.kind, e.last_update_sec, [&]() {
                return poly ? detail::triangulate_convex_polygon(e.points, e.point_count, z_lift)
                            : build_road_strip(e.points, e.point_count, z_lift);
            });
        } else if (e.is_polygon) {
            const uint64_t key = chunk_signature(true, e.points, e.point_count);
            adopt_or_build(key, material, e.kind, e.last_update_sec, [&]() {
                std::vector<Vec3> hatch;
                if (e.kind == MapKind::CROSSWALK) {
                    hatch = detail::build_crosswalk_hatch(e.points, e.point_count, z_lift);
                }
                if (!hatch.empty()) return hatch;
                return detail::triangulate_convex_polygon(e.points, e.point_count, z_lift);
            });
        } else if (IsBoundaryKind(e.kind)) {
            for (auto& dash : chop_into_dashes(e.points, e.point_count)) {
                for (auto [a, b] : detail::polyline_chunks(static_cast<uint32_t>(dash.size()))) {
                    const uint32_t chunkStart = a;
                    const uint32_t n = b - a;
                    const uint64_t key = chunk_signature(false, dash.data() + chunkStart, n);
                    adopt_or_build(key, material, e.kind, e.last_update_sec,
                                   [dash, chunkStart, n, z_lift]() {
                                       return build_ribbon_flat(dash.data() + chunkStart, n,
                                                                kLaneHalfWidthM, z_lift);
                                   });
                }
            }
        } else if (e.kind == MapKind::CENTERLINE) {
            for (auto [a, b] : detail::polyline_chunks(e.point_count)) {
                const uint32_t n = b - a;
                const Vec3* chunkPts = e.points + a;
                const uint64_t key = chunk_signature(false, chunkPts, n);
                adopt_or_build(key, material, e.kind, e.last_update_sec, [chunkPts, n, z_lift]() {
                    return build_centerline_dots(chunkPts, n, z_lift);
                });
            }
        } else {
            for (auto [a, b] : detail::polyline_chunks(e.point_count)) {
                const uint32_t n = b - a;
                const Vec3* chunkPts = e.points + a;
                const uint64_t key = chunk_signature(false, chunkPts, n);
                adopt_or_build(key, material, e.kind, e.last_update_sec, [chunkPts, n, z_lift]() {
                    return build_ribbon_flat(chunkPts, n, kLaneHalfWidthM, z_lift);
                });
            }
        }
    }

    for (auto& [key, mesh] : r.mapElementMeshes) {
        destroy_mesh(*r.engine, *r.scene, mesh);
    }
    r.mapElementMeshes = std::move(next);
}

}
