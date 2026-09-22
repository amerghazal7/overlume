// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "golden.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace overlume::testing {
namespace {

constexpr uint32_t kWidth = 320;
constexpr uint32_t kHeight = 240;

double luminance(uint8_t r, uint8_t g, uint8_t b) { return 0.2126 * r + 0.7152 * g + 0.0722 * b; }

double block_ssim(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, uint32_t width,
                  uint32_t height) {
    constexpr int kBlock = 8;
    constexpr double kC1 = (0.01 * 255) * (0.01 * 255);
    constexpr double kC2 = (0.03 * 255) * (0.03 * 255);
    double total = 0.0;
    int blockCount = 0;
    for (uint32_t by = 0; by + kBlock <= height; by += kBlock) {
        for (uint32_t bx = 0; bx + kBlock <= width; bx += kBlock) {
            double sumA = 0, sumB = 0, sumAA = 0, sumBB = 0, sumAB = 0;
            const int n = kBlock * kBlock;
            for (int y = 0; y < kBlock; ++y) {
                for (int x = 0; x < kBlock; ++x) {
                    const uint32_t px = bx + x, py = by + y;
                    const size_t idx = (static_cast<size_t>(py) * width + px) * 3;
                    const double la = luminance(a[idx], a[idx + 1], a[idx + 2]);
                    const double lb = luminance(b[idx], b[idx + 1], b[idx + 2]);
                    sumA += la;
                    sumB += lb;
                    sumAA += la * la;
                    sumBB += lb * lb;
                    sumAB += la * lb;
                }
            }
            const double meanA = sumA / n, meanB = sumB / n;
            const double varA = sumAA / n - meanA * meanA;
            const double varB = sumBB / n - meanB * meanB;
            const double covAB = sumAB / n - meanA * meanB;
            const double ssim = ((2 * meanA * meanB + kC1) * (2 * covAB + kC2)) /
                                ((meanA * meanA + meanB * meanB + kC1) * (varA + varB + kC2));
            total += ssim;
            ++blockCount;
        }
    }
    return blockCount > 0 ? total / blockCount : 0.0;
}

}

double render_and_compare(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
                          const char* golden_png_path, const char* out_png_path) {
    if (r == nullptr) return -1.0;

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3);
    overlume::FrameView view{rgb.data(), kWidth, kHeight};
    if (!overlume::render_frame(r, pose, view)) return 0.0;

    stbi_write_png(out_png_path, static_cast<int>(kWidth), static_cast<int>(kHeight), 3, rgb.data(),
                   static_cast<int>(kWidth) * 3);

    int goldenWidth = 0, goldenHeight = 0, goldenChannels = 0;
    uint8_t* golden = stbi_load(golden_png_path, &goldenWidth, &goldenHeight, &goldenChannels, 3);
    if (golden == nullptr) {
        return 0.0;
    }
    if (static_cast<uint32_t>(goldenWidth) != kWidth ||
        static_cast<uint32_t>(goldenHeight) != kHeight) {
        stbi_image_free(golden);
        return 0.0;
    }
    const std::vector<uint8_t> goldenPixels(
        golden, golden + static_cast<size_t>(goldenWidth) * goldenHeight * 3);
    stbi_image_free(golden);

    return block_ssim(rgb, goldenPixels, kWidth, kHeight);
}

FrameStats analyze_png(const char* png_path) {
    int width = 0, height = 0, channels = 0;
    uint8_t* img = stbi_load(png_path, &width, &height, &channels, 3);
    if (img == nullptr) return {};

    FrameStats stats{};
    const size_t n = static_cast<size_t>(width) * height;
    bool seenLevel[256] = {};
    double topSum = 0.0, bottomSum = 0.0;
    size_t topN = 0, bottomN = 0;
    const int topEnd = height / 3;
    const int bottomStart = (2 * height) / 3;
    constexpr int kSkyRowStart = 10, kSkyRowEnd = 40;
    constexpr int kHorizonRowStart = 50, kHorizonRowEnd = 60;
    double skySum = 0.0, horizonSum = 0.0;
    size_t skyN = 0, horizonN = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t idx = (static_cast<size_t>(y) * width + x) * 3;
            const double l = luminance(img[idx], img[idx + 1], img[idx + 2]);
            stats.mean += l;
            const int level = static_cast<int>(l + 0.5);
            if (level >= 0 && level < 256) seenLevel[level] = true;
            if (y < topEnd) {
                topSum += l;
                ++topN;
            } else if (y >= bottomStart) {
                bottomSum += l;
                ++bottomN;
            }
            if (y >= kSkyRowStart && y < kSkyRowEnd) {
                skySum += l;
                ++skyN;
            } else if (y >= kHorizonRowStart && y < kHorizonRowEnd) {
                horizonSum += l;
                ++horizonN;
            }
        }
    }
    stbi_image_free(img);

    stats.mean /= static_cast<double>(n);
    for (bool seen : seenLevel) stats.distinct_levels += seen ? 1 : 0;
    stats.top_third_mean = topN > 0 ? topSum / static_cast<double>(topN) : 0.0;
    stats.bottom_third_mean = bottomN > 0 ? bottomSum / static_cast<double>(bottomN) : 0.0;
    stats.sky_row_mean = skyN > 0 ? skySum / static_cast<double>(skyN) : 0.0;
    stats.horizon_row_mean = horizonN > 0 ? horizonSum / static_cast<double>(horizonN) : 0.0;
    return stats;
}

MapGeom load_map_geom(const char* path) {
    MapGeom g;
    std::ifstream in(path);
    if (!in) return g;

    struct Meta {
        uint8_t is_polygon;
        overlume::MapKind kind;
        uint32_t lane_id;
        uint32_t offset;
        uint32_t count;
    };
    std::vector<Meta> meta;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        int isPolygon = 0;
        int kindRaw = 0;
        uint32_t laneId = 0;
        uint32_t n = 0;
        if (!(iss >> isPolygon >> kindRaw >> laneId >> n)) continue;
        const auto offset = static_cast<uint32_t>(g.points.size());
        bool ok = true;
        for (uint32_t i = 0; i < n && ok; ++i) {
            double x = 0, y = 0, z = 0;
            if (!(iss >> x >> y >> z)) {
                ok = false;
                break;
            }
            g.points.push_back({x, y, z});
        }
        if (!ok) {
            g.points.resize(offset);
            continue;
        }
        meta.push_back(Meta{static_cast<uint8_t>(isPolygon != 0),
                            static_cast<overlume::MapKind>(kindRaw), laneId, offset, n});
    }

    g.elements.reserve(meta.size());
    for (const auto& m : meta) {
        overlume::MapElement e{};
        e.points = m.count > 0 ? g.points.data() + m.offset : nullptr;
        e.point_count = m.count;
        e.is_polygon = m.is_polygon;
        e.kind = m.kind;
        e.lane_id = m.lane_id;
        g.elements.push_back(e);
    }
    return g;
}

ObjectScene make_mixed_class_objects(double now) {
    struct Spec {
        overlume::ObjectClass cls;
        overlume::Vec3 position;
        double heading;
        overlume::Vec3 dims;
        overlume::Vec3 velocity;
        std::vector<overlume::Vec3> path;
        double age_sec;
    };
    const std::vector<Spec> specs = {
        {overlume::ObjectClass::CAR,
         {-6.0, -3.0, 0.0},
         0.3,
         {4.5, 1.8, 1.5},
         {3.0, 1.0, 0.0},
         {},
         0.0},
        {overlume::ObjectClass::TRUCK_VAN,
         {-3.0, 4.0, 0.0},
         -0.5,
         {5.5, 2.0, 2.2},
         {0.0, 0.0, 0.0},
         {{-3.0, 4.0, 0.0}, {-1.0, 3.0, 0.0}, {1.0, 2.5, 0.0}, {3.0, 2.0, 0.0}},
         0.0},
        {overlume::ObjectClass::BUS,
         {5.0, 5.0, 0.0},
         2.1,
         {12.0, 2.5, 3.2},
         {0.0, 0.0, 0.0},
         {},
         0.0},
        {overlume::ObjectClass::PEDESTRIAN,
         {2.0, -5.0, 0.0},
         1.0,
         {0.6, 0.6, 1.8},
         {0.0, 0.0, 0.0},
         {},
         0.0},
        {overlume::ObjectClass::CYCLIST,
         {-2.0, -6.0, 0.0},
         0.6,
         {1.9, 0.7, 1.7},
         {0.0, 0.0, 0.0},
         {},
         0.0},
        {overlume::ObjectClass::UNKNOWN,
         {6.0, -6.0, 0.0},
         -1.2,
         {2.0, 2.0, 2.0},
         {0.0, 0.0, 0.0},
         {},
         0.9},
    };

    ObjectScene s;
    size_t totalPathPoints = 0;
    for (const auto& sp : specs) totalPathPoints += sp.path.size();
    s.path_points.reserve(totalPathPoints);
    s.objects.reserve(specs.size());

    uint32_t nextId = 1;
    for (const auto& sp : specs) {
        overlume::TrackedObject obj{};
        obj.id = nextId++;
        obj.cls = sp.cls;
        obj.position = sp.position;
        obj.heading_rad = sp.heading;
        obj.dimensions = sp.dims;
        obj.velocity = sp.velocity;
        obj.label = nullptr;
        obj.last_update_sec = now - sp.age_sec;
        if (sp.path.empty()) {
            obj.predicted_path = nullptr;
            obj.predicted_path_count = 0;
        } else {
            const size_t offset = s.path_points.size();
            for (const auto& p : sp.path) s.path_points.push_back(p);
            obj.predicted_path = s.path_points.data() + offset;
            obj.predicted_path_count = static_cast<uint32_t>(sp.path.size());
        }
        s.objects.push_back(obj);
    }
    return s;
}

RibbonScene make_three_role_ribbons(double now) {
    struct Spec {
        overlume::PathRole role;
        std::vector<overlume::Vec3> points;
        double age_sec;
    };
    const std::vector<Spec> specs = {
        {overlume::PathRole::BEHAVIOR, {{-4.0, -1.2, 0.0}, {0.0, 0.0, 0.0}, {4.0, 1.2, 0.0}}, 0.0},
        {overlume::PathRole::GLOBAL,
         {{-10.0, -3.0, 0.0},
          {-5.0, -1.5, 0.0},
          {0.0, 0.0, 0.0},
          {5.0, 1.5, 0.0},
          {10.0, 3.0, 0.0}},
         0.0},
        {overlume::PathRole::LOCAL,
         {{-6.0, -1.8, 0.0}, {-3.0, -0.9, 0.0}, {0.0, 0.0, 0.0}, {3.0, 0.9, 0.0}, {6.0, 1.8, 0.0}},
         0.0},
    };

    RibbonScene s;
    size_t totalPoints = 0;
    for (const auto& sp : specs) totalPoints += sp.points.size();
    s.point_storage.reserve(totalPoints);
    s.ribbons.reserve(specs.size());

    for (const auto& sp : specs) {
        const size_t offset = s.point_storage.size();
        for (const auto& p : sp.points) s.point_storage.push_back(p);
        overlume::PathRibbon r{};
        r.role = sp.role;
        r.points = s.point_storage.data() + offset;
        r.point_count = static_cast<uint32_t>(sp.points.size());
        r.last_update_sec = now - sp.age_sec;
        s.ribbons.push_back(r);
    }
    s.ego = overlume::EgoState{{0.0, 0.0, 0.0}, 0.0, 0.0, 1};
    return s;
}

AlertScene make_sweep_and_predicted_alerts(double now) {
    struct Spec {
        uint8_t severity;
        std::vector<overlume::Vec3> points;
        double age_sec;
    };
    const std::vector<Spec> specs = {
        {0, {{-3.0, -2.0, 0.0}, {3.0, -2.0, 0.0}, {3.0, 2.0, 0.0}, {-3.0, 2.0, 0.0}}, 0.75},
        {1, {{5.0, 4.0, 0.0}, {8.0, 4.0, 0.0}, {8.0, 7.0, 0.0}, {5.0, 7.0, 0.0}}, 0.0},
    };

    AlertScene s;
    size_t totalPoints = 0;
    for (const auto& sp : specs) totalPoints += sp.points.size();
    s.point_storage.reserve(totalPoints);
    s.alerts.reserve(specs.size());

    for (const auto& sp : specs) {
        const size_t offset = s.point_storage.size();
        for (const auto& p : sp.points) s.point_storage.push_back(p);
        overlume::AlertPolygon a{};
        a.points = s.point_storage.data() + offset;
        a.point_count = static_cast<uint32_t>(sp.points.size());
        a.severity = sp.severity;
        a.last_update_sec = now - sp.age_sec;
        s.alerts.push_back(a);
    }
    return s;
}

GenericMarkerScene make_all_primitive_markers(double now, const char* mesh_glb_path) {
    GenericMarkerScene s;
    s.point_storage.reserve(4 + 4 + 5 + 3);
    s.markers.reserve(16);

    auto push_posed = [&](overlume::MarkerPrimitive prim, double x, overlume::Vec3 scale) {
        overlume::GenericMarker m{};
        m.primitive = prim;
        m.position = {x, 0.0, 0.5};
        m.heading_rad = 0.3;
        m.scale = scale;
        m.last_update_sec = now;
        s.markers.push_back(m);
    };
    auto push_points = [&](overlume::MarkerPrimitive prim, const overlume::Vec3* pts, uint32_t n,
                           bool colored) {
        const size_t offset = s.point_storage.size();
        for (uint32_t i = 0; i < n; ++i) s.point_storage.push_back(pts[i]);
        overlume::GenericMarker m{};
        m.primitive = prim;
        m.points = s.point_storage.data() + offset;
        m.point_count = n;
        if (colored) {
            m.color[0] = 1.0f;
            m.color[1] = 1.0f;
            m.color[2] = 1.0f;
            m.color[3] = 1.0f;
        }
        m.last_update_sec = now;
        s.markers.push_back(m);
    };

    push_posed(overlume::MarkerPrimitive::CUBE, 0.0, {1.0, 1.0, 1.0});
    push_posed(overlume::MarkerPrimitive::SPHERE, 2.0, {1.0, 1.0, 1.0});
    push_posed(overlume::MarkerPrimitive::CYLINDER, 4.0, {1.0, 1.0, 1.0});
    push_posed(overlume::MarkerPrimitive::ARROW, 6.0, {1.5, 1.0, 1.0});

    const overlume::Vec3 lineStrip[] = {
        {7.5, -0.5, 0.2}, {8.0, 0.5, 0.2}, {8.5, -0.5, 0.2}, {9.0, 0.5, 0.2}};
    push_points(overlume::MarkerPrimitive::LINE_STRIP, lineStrip, 4, true);

    const overlume::Vec3 lineList[] = {
        {9.5, -0.5, 0.2}, {10.5, 0.5, 0.2}, {9.5, 0.5, 0.2}, {10.5, -0.5, 0.2}};
    push_points(overlume::MarkerPrimitive::LINE_LIST, lineList, 4, false);

    const overlume::Vec3 points[] = {
        {11.0, 0.0, 0.3}, {11.5, 0.3, 0.3}, {12.0, -0.3, 0.3}, {12.5, 0.2, 0.3}, {13.0, -0.2, 0.3}};
    push_points(overlume::MarkerPrimitive::POINTS, points, 5, false);

    {
        overlume::GenericMarker m{};
        m.primitive = overlume::MarkerPrimitive::TEXT;
        m.position = {14.0, 0.0, 1.0};
        m.text = "marker";
        m.last_update_sec = now;
        s.markers.push_back(m);
    }

    const overlume::Vec3 triangle[] = {{15.5, -0.5, 0.0}, {16.5, -0.5, 0.0}, {16.0, 0.5, 0.0}};
    push_points(overlume::MarkerPrimitive::TRIANGLE_LIST, triangle, 3, true);

    {
        overlume::GenericMarker m{};
        m.primitive = overlume::MarkerPrimitive::MESH;
        m.position = {18.0, 0.0, 0.5};
        m.scale = {1.0, 1.0, 1.0};
        m.mesh_path = mesh_glb_path;
        m.last_update_sec = now;
        s.markers.push_back(m);
    }

    for (int i = 0; i < 3; ++i) {
        push_posed(overlume::MarkerPrimitive::CUBE, 20.0 + i * 1.2, {0.6, 0.6, 0.6});
    }
    for (int i = 0; i < 3; ++i) {
        push_posed(overlume::MarkerPrimitive::SPHERE, 24.0 + i * 1.2, {0.6, 0.6, 0.6});
    }

    return s;
}

GridScene make_two_layer_grids(double now) {
    constexpr uint32_t kW = 16, kH = 16;
    constexpr double kRes = 0.5;

    GridScene s;
    s.cell_storage.reserve(2);
    s.grids.reserve(2);

    {
        std::vector<uint8_t> cells(static_cast<size_t>(kW) * kH);
        const double cx = (kW - 1) / 2.0, cy = (kH - 1) / 2.0;
        const double maxDist = std::sqrt(cx * cx + cy * cy);
        for (uint32_t y = 0; y < kH; ++y) {
            for (uint32_t x = 0; x < kW; ++x) {
                const double dx = x - cx, dy = y - cy;
                const double dist = std::sqrt(dx * dx + dy * dy);
                const double occ = 100.0 * (1.0 - std::min(1.0, dist / maxDist));
                cells[y * kW + x] = static_cast<uint8_t>(occ + 0.5);
            }
        }
        cells[0] = 255;
        cells[kW - 1] = 255;
        cells[(kH - 1) * kW] = 255;
        cells[(kH - 1) * kW + (kW - 1)] = 255;
        s.cell_storage.push_back(std::move(cells));

        overlume::GroundGridLayer layer{};
        layer.kind = 0;
        layer.origin = {-4.0, -4.0, 0.0};
        layer.resolution_m = kRes;
        layer.width_cells = kW;
        layer.height_cells = kH;
        layer.cells = s.cell_storage.back().data();
        layer.last_update_sec = now;
        s.grids.push_back(layer);
    }

    {
        std::vector<uint8_t> cells(static_cast<size_t>(kW) * kH);
        for (uint32_t y = 0; y < kH; ++y) {
            for (uint32_t x = 0; x < kW; ++x) {
                const double occ = 100.0 * (static_cast<double>(x) / (kW - 1));
                cells[y * kW + x] = static_cast<uint8_t>(occ + 0.5);
            }
        }
        s.cell_storage.push_back(std::move(cells));

        overlume::GroundGridLayer layer{};
        layer.kind = 1;
        layer.origin = {-1.0, -4.0, 0.0};
        layer.resolution_m = kRes;
        layer.width_cells = kW;
        layer.height_cells = kH;
        layer.cells = s.cell_storage.back().data();
        layer.last_update_sec = now;
        s.grids.push_back(layer);
    }

    return s;
}

overlume::Vec3 centroid(const std::vector<overlume::MapElement>& elems) {
    double sx = 0.0, sy = 0.0, sz = 0.0;
    uint64_t n = 0;
    for (const auto& e : elems) {
        for (uint32_t i = 0; i < e.point_count; ++i) {
            sx += e.points[i].x;
            sy += e.points[i].y;
            sz += e.points[i].z;
            ++n;
        }
    }
    if (n == 0) return {0.0, 0.0, 0.0};
    return {sx / static_cast<double>(n), sy / static_cast<double>(n), sz / static_cast<double>(n)};
}

}
