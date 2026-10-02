// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// Renders the README hero animation through the public API only: one rich
// scene (ego, tracked objects, HD map, path ribbons, trajectory carpet,
// occupancy grids, point cloud, generic markers, alerts, baked environment)
// with a level-horizon chase camera that swings out to the verge and back,
// and a selectable theme (--theme). Scene content
// is lifted from the golden test fixtures (tests/golden.cpp,
// tests/test_theme_showcase.cpp).
//
// Loop: the ego waits at the crosswalk and the traffic flows past it. Camera,
// theme, LiDAR sweep, pulses and every actor are exact periodic functions of
// t / (frames / fps), so frame `frames` would equal frame 0 and the sequence
// wraps with no cross-fade. Traffic laps wrap around a stretch of road wider
// than the camera sees.

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "overlume/api.h"
#include "overlume/scene.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>
#include <vector>

namespace {

using overlume::Vec3;

constexpr double kPi = 3.14159265358979323846;

// ponytail: default asset root is derived from this file's path at compile time.
std::string default_root() {
    std::string p = __FILE__;
    for (int i = 0; i < 2; ++i) {
        const size_t s = p.find_last_of('/');
        if (s == std::string::npos) return ".";
        p.resize(s);
    }
    return p;
}

// Road centre line sits at local y = -14 of the fixture town so it threads
// between the baked buildings instead of through them.
const Vec3 kOrigin{-129.2, -37.1, 0.0};
constexpr overlume::GeoAnchor kAnchor{25.0803, 55.3910, 0.0};

constexpr double kHalfWidth = 3.5;
constexpr double kEdgeOffset = kHalfWidth + 0.6;
constexpr double kRoadX0 = -30.0;
constexpr double kRoadX1 = 90.0;
constexpr double kEgoX = 12.0;  // ego waits here; the world moves past it
constexpr double kLaneY = -1.75;
constexpr double kCrosswalkX = 24.0;

Vec3 W(double lx, double ly, double z = 0.0) { return {kOrigin.x + lx, kOrigin.y + ly, z}; }

uint32_t rgba(int r, int g, int b, int a) {
    return static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(a) << 24);
}

double smooth01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

struct Args {
    std::string out = ".";
    std::string root = default_root();
    uint32_t width = 1280, height = 720;
    int frames = 96;
    int fps = 12;
    uint32_t quality = 2;
    std::string theme = "dark_adas";
    std::set<int> only;
};

bool parse(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto val = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        const char* v = nullptr;
        if (k == "--help" || k == "-h") return false;
        if (!(v = val())) return false;
        if (k == "--out")
            a.out = v;
        else if (k == "--root")
            a.root = v;
        else if (k == "--width")
            a.width = static_cast<uint32_t>(std::atoi(v));
        else if (k == "--height")
            a.height = static_cast<uint32_t>(std::atoi(v));
        else if (k == "--frames")
            a.frames = std::atoi(v);
        else if (k == "--fps")
            a.fps = std::atoi(v);
        else if (k == "--theme")
            a.theme = v;
        else if (k == "--quality")
            a.quality = static_cast<uint32_t>(std::atoi(v));
        else if (k == "--only") {
            std::string s = v;
            size_t p = 0;
            while (p < s.size()) {
                size_t c = s.find(',', p);
                if (c == std::string::npos) c = s.size();
                a.only.insert(std::atoi(s.substr(p, c - p).c_str()));
                p = c + 1;
            }
        } else
            return false;
    }
    return a.frames > 0 && a.fps > 0 && a.width > 0 && a.height > 0;
}

// Everything the SceneGraph points into lives here so the pointers stay valid
// until the next build().
struct SceneData {
    std::vector<Vec3> pts;  // reserved up front, never reallocated
    std::vector<overlume::MapElement> map;
    std::vector<overlume::TrackedObject> objects;
    std::vector<overlume::PathRibbon> ribbons;
    std::vector<overlume::GroundGridLayer> grids;
    std::vector<std::vector<uint8_t>> cells;
    std::vector<overlume::AlertPolygon> alerts;
    std::vector<overlume::GenericMarker> markers;
    std::vector<overlume::PointCloudPoint> cloud;
    std::vector<overlume::PointCloudPoint> carpet;
    std::vector<overlume::AlertChip> chips;
    std::vector<std::string> strings;
    overlume::PointCloud pc{};
    overlume::TrajectoryCarpet tc{};
    overlume::SceneGraph graph{};

    const Vec3* poly(const std::vector<Vec3>& v) {
        const size_t o = pts.size();
        pts.insert(pts.end(), v.begin(), v.end());
        return pts.data() + o;
    }
};

// ponytail: scene content is rebuilt every frame; fine for an offline tool.
void build(SceneData& d, double t, double periodSec, const std::string& meshPath) {
    d = SceneData{};
    d.pts.reserve(4096);
    d.strings.reserve(16);

    const double egoX = kEgoX;
    const Vec3 ego = W(egoX, kLaneY);
    const double ph = 2 * kPi * t / periodSec;  // one full turn per loop

    // ---- HD map -------------------------------------------------------
    auto add_map = [&](std::vector<Vec3> v, uint8_t polygon, overlume::MapKind kind) {
        overlume::MapElement e{};
        e.point_count = static_cast<uint32_t>(v.size());
        e.points = d.poly(v);
        e.is_polygon = polygon;
        e.kind = kind;
        e.last_update_sec = t;
        d.map.push_back(e);
    };
    add_map({W(kRoadX0, -kHalfWidth), W(kRoadX1, -kHalfWidth), W(kRoadX1, kHalfWidth),
             W(kRoadX0, kHalfWidth)},
            1, overlume::MapKind::ROAD_SURFACE);
    add_map({W(kRoadX0, -kHalfWidth), W(kRoadX1, -kHalfWidth)}, 0,
            overlume::MapKind::LEFT_BOUNDARY);
    add_map({W(kRoadX0, kHalfWidth), W(kRoadX1, kHalfWidth)}, 0, overlume::MapKind::RIGHT_BOUNDARY);
    {
        std::vector<Vec3> cl;
        for (double x = kRoadX0; x <= kRoadX1; x += 2.0) cl.push_back(W(x, 0.0));
        add_map(cl, 0, overlume::MapKind::CENTERLINE);
    }
    add_map({W(kRoadX0, -kEdgeOffset), W(kRoadX1, -kEdgeOffset)}, 0, overlume::MapKind::ROAD_EDGE);
    add_map({W(kRoadX0, kEdgeOffset), W(kRoadX1, kEdgeOffset)}, 0, overlume::MapKind::ROAD_EDGE);
    add_map({W(kCrosswalkX - 1.5, -kEdgeOffset - 0.3), W(kCrosswalkX + 1.5, -kEdgeOffset - 0.3),
             W(kCrosswalkX + 1.5, kEdgeOffset + 0.3), W(kCrosswalkX - 1.5, kEdgeOffset + 0.3)},
            1, overlume::MapKind::CROSSWALK);
    add_map({W(kCrosswalkX - 3.0, -kHalfWidth), W(kCrosswalkX - 3.0, 0.0)}, 0,
            overlume::MapKind::STOPLINE);
    // side street + junction patch at x = 52..60
    add_map({W(52, kHalfWidth), W(60, kHalfWidth), W(60, 26), W(52, 26)}, 1,
            overlume::MapKind::ROAD_SURFACE);
    add_map({W(52, -kHalfWidth), W(60, -kHalfWidth), W(60, kHalfWidth), W(52, kHalfWidth)}, 1,
            overlume::MapKind::JUNCTION);
    add_map({W(56, kHalfWidth), W(56, 26)}, 0, overlume::MapKind::CENTERLINE);

    // ---- tracked objects ------------------------------------------------
    std::vector<std::vector<Vec3>> predicted;
    auto add_obj = [&](overlume::ObjectClass cls, Vec3 pos, double hd, Vec3 dims, Vec3 vel,
                       std::vector<Vec3> pred = {}) {
        overlume::TrackedObject o{};
        o.id = static_cast<uint32_t>(d.objects.size() + 1);
        o.cls = cls;
        o.position = pos;
        o.heading_rad = hd;
        o.dimensions = dims;
        o.velocity = vel;
        if (!pred.empty()) {
            o.predicted_path_count = static_cast<uint32_t>(pred.size());
            o.predicted_path = d.poly(pred);
        }
        o.last_update_sec = t;
        d.objects.push_back(o);
    };
    // Traffic runs in a window [kWinX0, kWinX1] ahead of the ego, far from the
    // camera, and wraps once per loop. Each actor scales in/out at the window
    // ends (invisible at that range), so the loop needs no cross-fade.
    constexpr double kWinX0 = 14.0, kWinX1 = 50.0;
    const double v = (kWinX1 - kWinX0) / periodSec;
    struct Lap {
        double x, s;
    };
    auto lapw = [&](double off, bool oncoming) {
        const double u = std::fmod(off + t / periodSec, 1.0);
        const double s = smooth01(u / 0.12) * smooth01((1.0 - u) / 0.12);
        return Lap{
            egoX + (oncoming ? kWinX1 - u * (kWinX1 - kWinX0) : kWinX0 + u * (kWinX1 - kWinX0)),
            std::max(s, 0.02)};
    };
    auto sc = [](Vec3 d, double s) { return Vec3{d.x * s, d.y * s, d.z * s}; };
    {
        const Lap c = lapw(0.0, true);  // oncoming car in the left lane
        add_obj(overlume::ObjectClass::CAR, W(c.x, 1.75), kPi, sc({4.5, 1.8, 1.5}, c.s),
                {-v, 0, 0});
        const Lap c2 = lapw(0.5, true);
        add_obj(overlume::ObjectClass::CAR, W(c2.x, 1.75), kPi, sc({4.5, 1.8, 1.5}, c2.s),
                {-v, 0, 0});
    }
    std::vector<Vec3> truckPath;  // predicted lane change into the ego lane
    {
        const Lap tk = lapw(0.25, true);
        for (int i = 1; i <= 6; ++i)
            truckPath.push_back(W(tk.x - i * 3.0, 1.75 - 3.5 * smooth01((i - 2) / 4.0)));
        add_obj(overlume::ObjectClass::TRUCK_VAN, W(tk.x, 1.75), kPi, sc({5.5, 2.0, 2.2}, tk.s),
                {-v, 0, 0}, truckPath);
    }
    // BUS has no model: it renders as a clay box. Parked far down the verge.
    add_obj(overlume::ObjectClass::BUS, W(58.0, -kEdgeOffset - 5.0), 0.0, {12.0, 2.5, 3.2},
            {0, 0, 0});
    // Pedestrian crosses and comes back (triangle wave, eased).
    const double pedTri = 1.0 - std::fabs(2.0 * std::fmod(t / periodSec, 1.0) - 1.0);
    const double pedY = -7.0 + 14.0 * smooth01(pedTri);
    const double pedDir = std::fmod(t / periodSec, 1.0) < 0.5 ? 1.0 : -1.0;
    add_obj(overlume::ObjectClass::PEDESTRIAN, W(kCrosswalkX, pedY), pedDir * kPi / 2,
            {0.6, 0.6, 1.8}, {0, pedDir * 1.0, 0});
    const Lap cy = lapw(0.1, false);
    const double cycX = cy.x;
    add_obj(overlume::ObjectClass::CYCLIST, W(cycX, -3.2), 0.0, sc({1.9, 0.7, 1.7}, cy.s),
            {v, 0, 0});
    add_obj(overlume::ObjectClass::UNKNOWN, W(37.0, kEdgeOffset + 2.0), 0.6, {2.0, 2.0, 2.0},
            {0, 0, 0});
    // parked group on the right verge, well ahead of the camera
    add_obj(overlume::ObjectClass::CAR, W(29.0, -kEdgeOffset - 1.6), 0.08, {4.5, 1.8, 1.5},
            {0, 0, 0});
    add_obj(overlume::ObjectClass::CAR, W(35.0, -kEdgeOffset - 1.6), -0.06, {4.5, 1.8, 1.5},
            {0, 0, 0});
    add_obj(overlume::ObjectClass::TRUCK_VAN, W(42.0, -kEdgeOffset - 1.7), 0.04, {5.5, 2.0, 2.2},
            {0, 0, 0});

    // ---- path ribbons (all three roles, told apart by shape) ------------
    // global: long route that holds the lane then curves left into the side
    // street; behavior: lane-nudge around the cyclist; local: short stub.
    auto add_ribbon = [&](overlume::PathRole role, std::vector<Vec3> v) {
        overlume::PathRibbon r{};
        r.role = role;
        r.point_count = static_cast<uint32_t>(v.size());
        r.points = d.poly(v);
        r.last_update_sec = t;
        d.ribbons.push_back(r);
    };
    {
        std::vector<Vec3> g;
        for (double x = egoX; x <= 50.0; x += 1.5) g.push_back(W(x, kLaneY - 0.9));
        for (int i = 1; i <= 12; ++i) {  // quarter-circle left into the side street
            const double a = kPi / 2 * i / 12.0, r = 8.0;
            g.push_back(W(50.0 + r * std::sin(a), kLaneY - 0.9 + r * (1 - std::cos(a))));
        }
        for (double y = kLaneY - 0.9 + 8.0; y <= 24.0; y += 1.5) g.push_back(W(58.0, y));
        add_ribbon(overlume::PathRole::GLOBAL, g);
    }
    {
        std::vector<Vec3> b;
        for (double s2 = 0.0; s2 <= 24.0; s2 += 1.5)
            b.push_back(W(egoX + s2,
                          kLaneY + 1.0 * std::sin(kPi * std::clamp((s2 - 3.0) / 18.0, 0.0, 1.0))));
        add_ribbon(overlume::PathRole::BEHAVIOR, b);
    }
    {
        std::vector<Vec3> l;
        for (double s2 = 0.0; s2 <= 8.0; s2 += 1.0) l.push_back(W(egoX + s2, kLaneY));
        add_ribbon(overlume::PathRole::LOCAL, l);
    }

    // ---- trajectory carpet (swept footprint of the behavior path) --------
    for (int i = 0; i < 40; ++i) {
        const double k = i / 39.0, s2 = i * 0.5;
        overlume::PointCloudPoint p{};
        p.position =
            W(egoX + s2, kLaneY + 1.0 * std::sin(kPi * std::clamp((s2 - 3.0) / 18.0, 0.0, 1.0)));
        p.rgba = rgba(static_cast<int>(40 + 200 * k), static_cast<int>(220 - 120 * k),
                      static_cast<int>(255 - 120 * k), 200);
        d.carpet.push_back(p);
    }
    d.tc.points = d.carpet.data();
    d.tc.point_count = static_cast<uint32_t>(d.carpet.size());
    d.tc.last_update_sec = t;

    // ---- occupancy grids ------------------------------------------------
    {  // static gradient layer (kind 1): inflation ramp around the parked vehicles
        constexpr uint32_t kW = 56, kH = 14;
        constexpr double kRes = 0.5, kInflate = 1.0;
        const Vec3 org = W(22.0, -kEdgeOffset - 6.4);
        std::vector<uint8_t> c(static_cast<size_t>(kW) * kH, 0);
        for (uint32_t y = 0; y < kH; ++y)
            for (uint32_t x = 0; x < kW; ++x) {
                const double wx = org.x - kOrigin.x + (x + 0.5) * kRes;
                const double wy = org.y - kOrigin.y + (y + 0.5) * kRes;
                double dmin = 1e9;
                for (const auto& o : d.objects) {
                    if (o.velocity.x != 0.0 || o.velocity.y != 0.0 ||
                        o.cls == overlume::ObjectClass::UNKNOWN ||
                        o.cls == overlume::ObjectClass::BUS)
                        continue;
                    const double dx = wx - (o.position.x - kOrigin.x),
                                 dy = wy - (o.position.y - kOrigin.y);
                    const double cs = std::cos(o.heading_rad), sn = std::sin(o.heading_rad);
                    const double lx = std::fabs(cs * dx + sn * dy),
                                 ly = std::fabs(-sn * dx + cs * dy);
                    dmin = std::min(dmin, std::hypot(std::max(lx - o.dimensions.x / 2, 0.0),
                                                     std::max(ly - o.dimensions.y / 2, 0.0)));
                }
                const double cost = std::clamp(1.0 - dmin / kInflate, 0.0, 1.0);
                c[y * kW + x] = static_cast<uint8_t>(40.0 * cost);
            }
        d.cells.push_back(std::move(c));
        overlume::GroundGridLayer g{};
        g.kind = 1;
        g.origin = org;
        g.resolution_m = static_cast<float>(kRes);
        g.width_cells = kW;
        g.height_cells = kH;
        g.last_update_sec = t;
        d.grids.push_back(g);
    }
    {  // dynamic layer (kind 0, drawn over): blobs track every moving actor
        constexpr uint32_t kW = 130, kH = 32;
        std::vector<uint8_t> c(static_cast<size_t>(kW) * kH, 0);
        const double ox = egoX - 14.0, oy = -8.0;
        for (uint32_t y = 0; y < kH; ++y)
            for (uint32_t x = 0; x < kW; ++x) {
                const double wx = ox + (x + 0.5) * 0.5;
                const double wy = oy + (y + 0.5) * 0.5;
                double val = 0.0;
                for (const auto& o : d.objects) {
                    if (o.velocity.x == 0.0 && o.velocity.y == 0.0) continue;
                    const double r = std::max(0.9, 0.42 * std::max(o.dimensions.x, o.dimensions.y));
                    const double dd = std::hypot(wx - (o.position.x - kOrigin.x),
                                                 wy - (o.position.y - kOrigin.y));
                    val = std::max(val, 100.0 * std::exp(-dd * dd / (r * r)));
                }
                c[y * kW + x] = val < 8.0 ? 0 : static_cast<uint8_t>(val);
            }
        d.cells.push_back(std::move(c));
        overlume::GroundGridLayer g{};
        g.kind = 0;
        g.origin = W(ox, oy);
        g.resolution_m = 0.5;
        g.width_cells = kW;
        g.height_cells = kH;
        g.last_update_sec = t;
        d.grids.push_back(g);
    }
    for (size_t i = 0; i < d.grids.size(); ++i) d.grids[i].cells = d.cells[i].data();

    // ---- point cloud: sweeping lidar rings ----------------------------
    for (int ring = 0; ring < 16; ++ring) {
        const double r = 2.5 + ring * 1.7;
        const int n = 200 + ring * 26;
        for (int i = 0; i < n; ++i) {
            const double a = 2 * kPi * i / n;
            const double sweep = std::fmod(a - 3 * ph + 8 * kPi, 2 * kPi) / (2 * kPi);
            const int br = static_cast<int>(70 + 185 * std::pow(1.0 - sweep, 3.0));
            const double k = ring / 15.0;
            overlume::PointCloudPoint p{};
            p.position = {ego.x + r * std::cos(a), ego.y + r * std::sin(a), 0.12};
            p.rgba = rgba(static_cast<int>(br * (1.0 - 0.7 * k)),
                          static_cast<int>(br * (0.6 + 0.25 * k)),
                          static_cast<int>(br * (0.35 + 0.65 * k)), 255);
            d.cloud.push_back(p);
        }
    }
    for (const auto& o : d.objects) {  // dense returns on every actor's box surface
        const double ox = o.position.x - ego.x, oy = o.position.y - ego.y;
        if (std::hypot(ox, oy) > 38.0 || o.cls == overlume::ObjectClass::BUS) continue;
        const double hl = o.dimensions.x / 2, hw = o.dimensions.y / 2;
        const double c = std::cos(o.heading_rad), sn = std::sin(o.heading_rad);
        const double perim = 4 * (hl + hw);
        for (double sp = 0.0; sp < perim; sp += 0.16) {
            double lx, ly;  // walk the footprint rectangle
            if (sp < 2 * hl) {
                lx = -hl + sp;
                ly = -hw;
            } else if (sp < 2 * hl + 2 * hw) {
                lx = hl;
                ly = -hw + (sp - 2 * hl);
            } else if (sp < 4 * hl + 2 * hw) {
                lx = hl - (sp - 2 * hl - 2 * hw);
                ly = hw;
            } else {
                lx = -hl;
                ly = hw - (sp - 4 * hl - 2 * hw);
            }
            for (double z = 0.1; z < o.dimensions.z; z += 0.17) {
                const double k = z / o.dimensions.z;
                overlume::PointCloudPoint p{};
                p.position = {o.position.x + c * lx - sn * ly, o.position.y + sn * lx + c * ly, z};
                p.rgba = rgba(static_cast<int>(60 + 195 * k), static_cast<int>(210 - 20 * k),
                              static_cast<int>(255 - 195 * k), 255);
                d.cloud.push_back(p);
            }
        }
    }
    d.pc.points = d.cloud.data();
    d.pc.point_count = static_cast<uint32_t>(d.cloud.size());
    d.pc.last_update_sec = t;

    // ---- alerts ---------------------------------------------------------
    auto add_alert = [&](double cx, double cy, double hx, double hy, uint8_t sev) {
        overlume::AlertPolygon a{};
        a.severity = sev;
        a.last_update_sec = t;
        std::vector<Vec3> v = {W(cx - hx, cy - hy), W(cx + hx, cy - hy), W(cx + hx, cy + hy),
                               W(cx - hx, cy + hy)};
        a.point_count = 4;
        a.points = d.poly(v);
        d.alerts.push_back(a);
    };
    add_alert(kCrosswalkX, pedY, 1.6, 1.6, 2);
    add_alert(cycX, -3.2, 2.0, 1.2, 1);

    // ---- generic markers -------------------------------------------------
    auto base_marker = [&](overlume::MarkerPrimitive prim, Vec3 pos, Vec3 scale,
                           std::array<float, 4> col) {
        overlume::GenericMarker m{};
        m.primitive = prim;
        m.position = pos;
        m.scale = scale;
        for (int i = 0; i < 4; ++i) m.color[i] = col[static_cast<size_t>(i)];
        m.last_update_sec = t;
        return m;
    };
    for (int i = 0; i < 6; ++i) {  // traffic cones along the left verge
        d.markers.push_back(base_marker(overlume::MarkerPrimitive::CYLINDER,
                                        W(22.0 + i * 1.8, kEdgeOffset + 0.1, 0.35),
                                        {0.35, 0.35, 0.7}, {1.0f, 0.55f, 0.1f, 1.0f}));
    }
    for (int i = 0; i < 5; ++i) {  // pulsing waypoint spheres above the route
        const double pulse = 0.35 + 0.12 * std::sin(4 * ph + i);
        d.markers.push_back(base_marker(overlume::MarkerPrimitive::SPHERE,
                                        W(egoX + 5.0 + i * 4.0, kLaneY, 2.2), {pulse, pulse, pulse},
                                        {0.3f, 1.0f, 0.8f, 1.0f}));
    }
    {
        auto m = base_marker(overlume::MarkerPrimitive::ARROW, W(egoX + 1.0, kLaneY, 2.2),
                             {2.0, 0.5, 0.5}, {1.0f, 0.9f, 0.2f, 1.0f});
        d.markers.push_back(m);
    }
    {
        auto m = base_marker(overlume::MarkerPrimitive::CUBE, W(31.0, kEdgeOffset + 1.6, 0.5),
                             {1.0, 1.0, 1.0}, {0.9f, 0.3f, 0.6f, 1.0f});
        m.heading_rad = ph;
        d.markers.push_back(m);
    }
    {
        auto m = base_marker(overlume::MarkerPrimitive::MESH, W(34.0, kEdgeOffset + 1.6, 0.6),
                             {1.2, 1.2, 1.2}, {1, 1, 1, 1});
        m.mesh_path = meshPath.c_str();
        m.heading_rad = -ph;
        d.markers.push_back(m);
    }
    {  // zig-zag line strip, line list, points, triangle list
        std::vector<Vec3> zig;
        for (int i = 0; i < 9; ++i)
            zig.push_back(W(egoX + 6.0 + i * 1.0, kLaneY + ((i % 2) ? 0.7 : -0.7), 0.25));
        auto m = base_marker(overlume::MarkerPrimitive::LINE_STRIP, {0, 0, 0}, {1, 1, 1},
                             {1.0f, 0.3f, 0.5f, 1.0f});
        m.points = d.poly(zig);
        m.point_count = static_cast<uint32_t>(zig.size());
        d.markers.push_back(m);
        std::vector<Vec3> lines;
        for (int i = 0; i < 4; ++i) {
            lines.push_back(W(25.0 + i * 1.2, kEdgeOffset + 3.5, 0.25));
            lines.push_back(W(25.0 + i * 1.2 + 0.8, kEdgeOffset + 4.5, 1.2));
        }
        auto l = base_marker(overlume::MarkerPrimitive::LINE_LIST, {0, 0, 0}, {1, 1, 1},
                             {0.5f, 0.8f, 1.0f, 1.0f});
        l.points = d.poly(lines);
        l.point_count = static_cast<uint32_t>(lines.size());
        d.markers.push_back(l);
        std::vector<Vec3> tri = {W(18.0, kEdgeOffset + 1.0, 0.05), W(19.4, kEdgeOffset + 1.0, 0.05),
                                 W(18.7, kEdgeOffset + 2.2, 0.05)};
        auto tr = base_marker(overlume::MarkerPrimitive::TRIANGLE_LIST, {0, 0, 0}, {1, 1, 1},
                              {1.0f, 0.8f, 0.0f, 1.0f});
        tr.points = d.poly(tri);
        tr.point_count = 3;
        d.markers.push_back(tr);
    }
    d.strings.push_back("pedestrian");
    {
        auto m = base_marker(overlume::MarkerPrimitive::TEXT, W(kCrosswalkX, pedY, 2.6), {1, 1, 1},
                             {1, 1, 1, 1});
        m.text = d.strings.back().c_str();
        d.markers.push_back(m);
    }

    // ---- HUD ------------------------------------------------------------
    d.strings.push_back("CAUTION");
    overlume::AlertChip chip{d.strings.back().c_str(), W(kCrosswalkX, pedY, 3.2)};
    d.chips.push_back(chip);

    overlume::SceneGraph& s = d.graph;
    s.sim_time_sec = t;
    s.ego = overlume::EgoState{ego, 0.0, 0.0, 1};
    s.objects = d.objects.data();
    s.object_count = static_cast<uint32_t>(d.objects.size());
    s.paths = d.ribbons.data();
    s.path_count = static_cast<uint32_t>(d.ribbons.size());
    s.map_elements = d.map.data();
    s.map_element_count = static_cast<uint32_t>(d.map.size());
    s.grids = d.grids.data();
    s.grid_count = static_cast<uint32_t>(d.grids.size());
    s.alerts = d.alerts.data();
    s.alert_count = static_cast<uint32_t>(d.alerts.size());
    s.markers = d.markers.data();
    s.marker_count = static_cast<uint32_t>(d.markers.size());
    s.hud.speed_mps = 0.0f;
    s.hud.active_mode = 3;
    s.hud.chips = d.chips.data();
    s.hud.chip_count = static_cast<uint32_t>(d.chips.size());
    s.point_clouds = &d.pc;
    s.point_cloud_count = 1;
    s.trajectory_carpets = &d.tc;
    s.trajectory_carpet_count = 1;
}

// Periodic camera: level 3/4 chase view with a gentle swing toward the
// parked-car side (+y side is a baked building). Pitch stays under ~20 deg.
overlume::CameraPose camera(double u) {
    const double w = 0.5 - 0.5 * std::cos(2 * kPi * u);  // one swing per loop
    const double az = kPi + 0.22 * w;
    const double R = 15.0;
    const double H = 5.2 + 0.4 * w;
    const double tx = kEgoX + 7.0, ty = kLaneY;
    overlume::CameraPose p{};
    p.eye[0] = kOrigin.x + tx + R * std::cos(az);
    p.eye[1] = kOrigin.y + ty + R * std::sin(az);
    p.eye[2] = H;
    p.target[0] = kOrigin.x + tx;
    p.target[1] = kOrigin.y + ty;
    p.target[2] = 1.0;
    p.vfov_deg = 40.0;
    return p;
}

}

int main(int argc, char** argv) {
    Args a;
    if (!parse(argc, argv, a)) {
        std::fprintf(stderr,
                     "usage: render_hero --out DIR [--width W] [--height H] [--frames N] "
                     "[--fps F] [--quality Q] [--theme NAME] [--root overlume_dir] "
                     "[--only i,j,..]\n");
        return 2;
    }
    const int period = a.frames;
    const double periodSec = static_cast<double>(period) / a.fps;

    const std::string themes = a.root + "/assets/themes";
    const std::string models = a.root + "/assets/models";
    const std::string town = a.root + "/tests/fixtures/environment_test_town_0";
    const std::string mesh = a.root + "/tests/fixtures/test_cube.glb";

    overlume::RenderConfig cfg{a.width, a.height, static_cast<uint8_t>(a.quality), themes.c_str(),
                               a.theme.c_str()};
    overlume::VisualRenderer* r = overlume::create_renderer(cfg);
    if (!r) {
        std::fprintf(stderr, "render_hero: create_renderer failed (no GPU/EGL?)\n");
        return 1;
    }
    overlume::set_object_model_dir(r, models.c_str());
    overlume::set_ego_model(r, (models + "/car.glb").c_str(), {4.5, 1.9, 1.6});
    if (!overlume::set_environment_source(r, town.c_str(), kAnchor))
        std::fprintf(stderr, "render_hero: environment fixture failed to open (continuing)\n");

    const double dt = 1.0 / a.fps;
    std::vector<uint8_t> rgb(static_cast<size_t>(a.width) * a.height * 3);
    SceneData data;
    int written = 0;

    // Render a few warm-up frames at t=0 so lazily loaded assets settle.
    build(data, 0.0, periodSec, mesh);
    overlume::set_scene(r, data.graph);
    for (int i = 0; i < 3; ++i)
        overlume::render_frame(r, camera(0.0), {rgb.data(), a.width, a.height});

    for (int i = 0; i < period; ++i) {
        const double t = i * dt;
        build(data, t, periodSec, mesh);
        overlume::set_scene(r, data.graph);
        if (!overlume::render_frame(r, camera(static_cast<double>(i) / period),
                                    {rgb.data(), a.width, a.height})) {
            std::fprintf(stderr, "render_hero: render_frame failed at %d\n", i);
            overlume::destroy_renderer(r);
            return 1;
        }
        if (!a.only.empty() && !a.only.count(i)) continue;
        char name[1024];
        std::snprintf(name, sizeof name, "%s/frame_%04d.png", a.out.c_str(), i);
        if (!stbi_write_png(name, static_cast<int>(a.width), static_cast<int>(a.height), 3,
                            rgb.data(), static_cast<int>(a.width) * 3)) {
            std::fprintf(stderr, "render_hero: failed to write %s\n", name);
            return 1;
        }
        ++written;
    }
    std::printf("render_hero: wrote %d frame(s) to %s\n", written, a.out.c_str());
    overlume::destroy_renderer(r);
    return 0;
}
