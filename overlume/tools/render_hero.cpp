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
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <sstream>
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
constexpr double kRoadX1 = 60.0;
// The library's ground plane is a fixed +-60 m square centred on the ego (renderer.cpp
// kGroundHalfExtent, snapped to a 2 m grid). Every map / ribbon / actor / grid point stays
// kGroundMargin inside it so nothing is ever drawn past the ground's far edge; fog (hero theme
// copies) dissolves the last stretch into the sky.
constexpr double kGroundHalf = 60.0, kGroundMargin = 8.0;
constexpr double kEgoX = 12.0;  // ego waits here; the world moves past it
constexpr double kLaneY = -1.75;
constexpr double kCrosswalkX = 19.5;

Vec3 W(double lx, double ly, double z = 0.0) { return {kOrigin.x + lx, kOrigin.y + ly, z}; }

uint32_t rgba(int r, int g, int b, int a) {
    return static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(a) << 24);
}

double gPedY = 0.0;  // ponytail: file-scope so alerts can read it

double smooth01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}
// Traffic grows from a vanishing size over the last 20 m of the window (far end only; the
// near end leaves past the bottom edge at full size). Occupancy blobs/paths share this scale.
double farScale(double lx) { return std::max(smooth01((kEgoX + 48.0 - lx) / 20.0), 0.02); }

// ---- crosswalk choreography (shared by build() and --check) ---------------------------------
// The pedestrian waits at a kerb, crosses at walking pace, waits at the far kerb and crosses back,
// all inside one loop period. Every vehicle / cyclist whose lane he occupies (plus kClear) brakes
// to a stop at its stop spot, waits, and pulls away: a monotone periodic time-warp phi(t) of the
// actor's nominal lap phase (phi(t+T) = phi(t)+T, so travel per loop is unchanged and the loop
// stays exactly seamless).
constexpr double kWinX0 = kEgoX - 36.0, kWinX1 = kEgoX + 48.0, kLap = kWinX1 - kWinX0;
// Kerb |y| (just outside the 3.5 m road edge, clear of the bike lane) and walking speed. The two
// kerb waits split whatever the loop leaves after the two crossings, equally.
constexpr double kPedK = 3.95, kPedV = 1.3;
// fmod without the libm call (positive result in [0,b)); the verifier saw a rare, unexplained
// SIGSEGV inside glibc fmod on this path.
double pmod(double a, double b) {
    a -= b * std::floor(a / b);
    return a >= b ? a - b : a;
}
constexpr double kClear = 1.5 + 0.3 + 0.4;  // 1.5 m rule + pedestrian half width + margin
constexpr double kStopRamp = 2.4;           // s to brake from cruise to a stop (and the mirror)
constexpr double kLead = 9.0;               // m of predicted path ahead of a cruising actor

// Stop lines sit this far (m) before the crosswalk edge, so a stopped actor never touches the
// pedestrian on screen from the chase camera. Edge x = kCrosswalkX -/+ 1.5.
constexpr double kStopClear = 3.5;
constexpr double kStopLineOnc = kCrosswalkX + 1.5 + kStopClear;  // oncoming front stops here (x)
constexpr double kStopLineBike = kCrosswalkX - 1.5 - kStopClear + 0.5;  // bike front (3.0 m clear)
// Catch-up after a stop: rise 0 -> kCatch (x cruise), hold, relax to 1; never above kCatch.
constexpr double kCatch = 1.27, kCatchR1 = 1.6, kCatchR2 = 1.6;
constexpr double kCatchMax = 1.3;  // --check fails above this (peak speed / cruise)
// Theme timeline as fractions of the loop (frame 0 = dark hold): crossfade to light_clay over
// [kFa,kFb], light hold [kFb,kFc], crossfade back over [kFc,kFd], dark until the loop ends.
constexpr double kFa = 0.235, kFb = 0.33, kFc = 0.625, kFd = 0.72;
// Weight of light_clay at frame time sf (s since frame 0) in a loop of T s; matches the renderer's
// linear blend, so scene colours that follow the theme can lerp with it.
double themeLight(double sf, double T) {
    const double u = pmod(sf / T, 1.0);
    if (u < kFa || u >= kFd) return 0.0;
    if (u < kFb) return (u - kFa) / (kFb - kFa);
    if (u < kFc) return 1.0;
    return 1.0 - (u - kFc) / (kFd - kFc);
}

struct PedState {
    double y, dir;  // dir: +1 / -1 while walking, 0 waiting
    double face;    // heading sign (waiting pedestrian faces the way he will go)
};
PedState pedAt(double t, double T) {
    const double tw = 2 * kPedK / kPedV;
    const double wait = std::max(0.0, (T - 2 * tw) / 2);  // each kerb
    const double s = pmod(t - (0.25 * T - tw / 2), T);    // crossing 1 is mid-road at u = 0.25
    if (s < tw) return {-kPedK + kPedV * s, 1, 1};
    if (s < tw + wait) return {kPedK, 0, -1};
    if (s < 2 * tw + wait) return {kPedK - kPedV * (s - tw - wait), -1, -1};
    return {-kPedK, 0, 1};
}
// First time (in [0,T)) the pedestrian leaves the lane band [lo,hi] (centre-line y range).
double bandExit(double lo, double hi, double T) {
    auto in = [&](double t) {
        const double y = pedAt(t, T).y;
        return y > lo && y < hi;
    };
    for (double t = 0; t < T; t += 1e-3)
        if (in(t) && !in(t + 1e-3)) return t + 1e-3;
    return 0.0;
}
void herm(double a, double b, double fa, double fb, double ma, double mb, double s, double& f,
          double& g) {
    const double d = b - a, h = (s - a) / d, h2 = h * h, h3 = h2 * h;
    f = (2 * h3 - 3 * h2 + 1) * fa + (h3 - 2 * h2 + h) * d * ma + (-2 * h3 + 3 * h2) * fb +
        (h3 - h2) * d * mb;
    g = (6 * h2 - 6 * h) * fa / d + (3 * h2 - 4 * h + 1) * ma + (-6 * h2 + 6 * h) * fb / d +
        (3 * h2 - 2 * h) * mb;
}
struct Yielder {
    bool oncoming;
    double spotX;       // stop spot (actor centre)
    double lo, hi;      // lane band the pedestrian must stay out of
    double lag, shift;  // s behind nominal at pull-away; queue release delay
};
struct Pose {
    double x, g;  // road position, speed factor (0 = stopped, 1 = cruise)
    double w;     // 0 while its predicted path must end at the stop spot, ramps to 1 after release
};
Pose yieldPose(const Yielder& y, double t, double T, bool yield) {
    const double D = kStopRamp, r = bandExit(y.lo, y.hi, T) + y.shift, ts = r - y.lag;
    double phi = t, g = 1.0, w = 1.0;
    if (yield) {
        const double t0 = ts - D / 2, R = r - t0;  // R = lag + D/2 > D
        const double k = std::floor((t - t0) / T);
        const double s = t - t0 - k * T;
        double f = s;
        if (s < D)
            herm(0, D, 0, D / 2, 1, 0, s, f, g);
        else if (s < R)
            f = D / 2, g = 0.0;
        else {
            // Catch-up: g rises 0 -> p (smoothstep), holds, relaxes to 1. Hold length from
            // integral(g) over the ramp = ramp length + lag (it has to win back `lag` seconds).
            const double p = kCatch, r1 = kCatchR1, r2 = kCatchR2;
            const double Lp = (y.lag + r1 * (1 - p / 2) + r2 * (1 - p) / 2) / (p - 1);
            const double x = s - R;
            auto I = [](double h) { return h * h * h - 0.5 * h * h * h * h; };  // int smooth01
            if (x < r1) {
                const double h = x / r1;
                f = D / 2 + p * r1 * I(h), g = p * smooth01(h);
            } else if (x < r1 + Lp) {
                f = D / 2 + p * r1 * 0.5 + p * (x - r1), g = p;
            } else if (x < r1 + Lp + r2) {
                const double x2 = x - r1 - Lp, h = x2 / r2;
                f = D / 2 + p * r1 * 0.5 + p * Lp + x2 + (p - 1) * (x2 - r2 * I(h));
                g = 1 + (p - 1) * (1 - smooth01(h));
            } else {
                f = s;  // back on the nominal schedule (f = s there: continuous by construction)
            }
        }
        phi = t0 + k * T + f;
        // Planner view: from 2 s before braking until release the path stops at the spot.
        const double q = pmod(t - (t0 - 2.0), T);
        const double span = R + 2.0;
        w = q < span ? 0.0 : smooth01((q - span) / 1.5);
    }
    const double off = (y.oncoming ? (kWinX1 - y.spotX) : (y.spotX - kWinX0)) / kLap - ts / T;
    const double u = pmod(off + phi / T, 1.0);
    return {y.oncoming ? kWinX1 - u * kLap : kWinX0 + u * kLap, g, w};
}
// Queue (head first): truck, car A, car B at >= 2 m bumper gaps; the stop line is kStopLineOnc
// (front).
const Yielder kTruck{true, kStopLineOnc + 2.75, -1.45, 4.95, 2.0, 0.0};
const Yielder kCarA{true, kStopLineOnc + 5.5 + 2.1 + 2.25, -1.45, 4.95, 2.0, 0.5};
const Yielder kCarB{true, kStopLineOnc + 5.5 + 2.1 + 4.5 + 2.1 + 2.25, -1.45, 4.95, 2.0, 1.0};
const Yielder kBike{false, kStopLineBike - 0.95, -5.75, -0.65, 2.0, 0.0};

struct Args {
    std::string out = ".";
    std::string root = default_root();
    uint32_t width = 1280, height = 720;
    int frames = 136;
    int fps = 8;
    uint32_t quality = 2;
    std::string theme = "dark_adas";
    std::set<int> only;
    bool check = false, yield = true;
    double phase = 0.25;  // loop phase of frame 0: 0.25 = pedestrian mid-crosswalk (busiest)
};

bool parse(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto val = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        const char* v = nullptr;
        if (k == "--help" || k == "-h") return false;
        if (k == "--check") {
            a.check = true;
            continue;
        }
        if (k == "--no-yield") {
            a.yield = false;
            continue;
        }  // debug: proves --check bites
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
        else if (k == "--phase")
            a.phase = std::atof(v);
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
void build(SceneData& d, double t, double periodSec, const std::string& meshPath, bool yield) {
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
    // (no side street / junction slab: a pale rectangle at the far end read as floating)

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
    // Traffic runs in a window [kWinX0, kWinX1] ahead of the ego (from behind the camera to beyond
    // the far road end) and wraps once per loop; see the choreography block above.
    const double v = kLap / periodSec;
    auto sc = [](Vec3 d, double s) { return Vec3{d.x * s, d.y * s, d.z * s}; };
    auto vel = [&](double dir, double g) {
        return dir * std::max(v * g, 1e-3);
    };  // never exactly 0
    // Oncoming actor: rigid straight path along its own lane, ending at its stop spot while it
    // brakes / waits (it lengthens again as the actor pulls away).
    auto onc = [&](overlume::ObjectClass cls, const Yielder& y, Vec3 dims, double lead) {
        const Pose p = yieldPose(y, t, periodSec, yield);
        const double s = farScale(p.x);
        const double len = std::min(kLead, std::max(0.0, p.x - y.spotX) + kLead * p.w);
        std::vector<Vec3> path;
        if (len > 0.5)
            for (int i = 1; i <= 5; ++i) path.push_back(W(p.x - lead - s * len * i / 5.0, 1.75));
        add_obj(cls, W(p.x, 1.75), kPi, sc(dims, s), {vel(-1, p.g), 0, 0}, path);
    };
    onc(overlume::ObjectClass::CAR, kCarA, {4.5, 1.8, 1.5}, 2.5);
    onc(overlume::ObjectClass::CAR, kCarB, {4.5, 1.8, 1.5}, 2.5);
    onc(overlume::ObjectClass::TRUCK_VAN, kTruck, {5.5, 2.0, 2.2}, 3.0);
    // Pedestrian: waits at a kerb, crosses at kPedV, waits at the far kerb, crosses back.
    {
        const PedState ps = pedAt(t, periodSec);
        add_obj(overlume::ObjectClass::PEDESTRIAN, W(kCrosswalkX, ps.y), ps.face * kPi / 2,
                {0.6, 0.6, 1.8}, {0, ps.dir == 0.0 ? ps.face * 1e-3 : ps.dir * kPedV, 0});
        gPedY = ps.y;
    }
    const Pose cp = yieldPose(kBike, t, periodSec, yield);
    const double cycX = cp.x, cycS = farScale(cp.x);
    add_obj(overlume::ObjectClass::CYCLIST, W(cycX, -3.2), 0.0, sc({1.9, 0.7, 1.7}, cycS),
            {vel(1, cp.g), 0, 0});
    add_obj(overlume::ObjectClass::UNKNOWN, W(34.0, kEdgeOffset + 2.0), 0.6, {2.0, 2.0, 2.0},
            {0, 0, 0});
    // parked group on the right verge, well ahead of the camera
    add_obj(overlume::ObjectClass::CAR, W(24.0, -kEdgeOffset - 1.6), 0.08, {4.5, 1.8, 1.5},
            {0, 0, 0});
    add_obj(overlume::ObjectClass::CAR, W(29.5, -kEdgeOffset - 1.6), -0.06, {4.5, 1.8, 1.5},
            {0, 0, 0});
    add_obj(overlume::ObjectClass::CAR, W(19.0, -kEdgeOffset - 5.6), -0.12, {4.5, 1.8, 1.5},
            {0, 0, 0});
    add_obj(overlume::ObjectClass::TRUCK_VAN, W(27.0, -kEdgeOffset - 5.8), 0.1, {5.5, 2.0, 2.2},
            {0, 0, 0});
    add_obj(overlume::ObjectClass::CAR, W(33.5, -kEdgeOffset - 5.5), 0.05, {4.5, 1.8, 1.5},
            {0, 0, 0});
    add_obj(overlume::ObjectClass::TRUCK_VAN, W(35.5, -kEdgeOffset - 1.7), 0.04, {5.5, 2.0, 2.2},
            {0, 0, 0});

    // outer verge row (fills the right of the frame): parked vehicles and two bystanders
    add_obj(overlume::ObjectClass::CAR, W(21.0, -16.0), 0.05, {4.5, 1.8, 1.5}, {0, 0, 0});
    add_obj(overlume::ObjectClass::CAR, W(27.5, -16.6), -0.08, {4.5, 1.8, 1.5}, {0, 0, 0});
    add_obj(overlume::ObjectClass::TRUCK_VAN, W(34.5, -15.8), 0.04, {5.5, 2.0, 2.2}, {0, 0, 0});
    add_obj(overlume::ObjectClass::PEDESTRIAN, W(23.0, -11.4), 0.5, {0.6, 0.6, 1.8}, {0, 0, 0});
    add_obj(overlume::ObjectClass::PEDESTRIAN, W(23.9, -11.7), 2.6, {0.6, 0.6, 1.8}, {0, 0, 0});

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
        for (double x = egoX; x <= 42.0; x += 1.5) g.push_back(W(x, kLaneY));
        for (int i = 1; i <= 12; ++i) {  // quarter-circle left into the side street
            const double a = kPi / 2 * i / 12.0, r = 8.0;
            g.push_back(W(42.0 + r * std::sin(a), kLaneY + r * (1 - std::cos(a))));
        }
        for (double y = kLaneY + 8.0; y <= 24.0; y += 1.5) g.push_back(W(50.0, y));
        add_ribbon(overlume::PathRole::GLOBAL, g);
    }
    {
        std::vector<Vec3> b;
        for (double s2 = 0.0; s2 <= 24.0; s2 += 1.5) b.push_back(W(egoX + s2, kLaneY));
        add_ribbon(overlume::PathRole::BEHAVIOR, b);
        std::vector<Vec3> l;  // local: short stub on the same centreline
        for (double s2 = 0.0; s2 <= 7.0; s2 += 1.0) l.push_back(W(egoX + s2, kLaneY));
        add_ribbon(overlume::PathRole::LOCAL, l);
    }

    // ---- trajectory carpet (swept footprint of the behavior path) --------
    for (int i = 0; i < 20; ++i) {
        const double k = i / 19.0, s2 = i * 1.0;
        overlume::PointCloudPoint p{};
        p.position = W(egoX + s2, kLaneY);
        p.rgba = rgba(static_cast<int>(40 + 200 * k), static_cast<int>(220 - 120 * k),
                      static_cast<int>(255 - 120 * k), 200);
        d.carpet.push_back(p);
    }
    d.tc.points = d.carpet.data();
    d.tc.point_count = static_cast<uint32_t>(d.carpet.size());
    d.tc.last_update_sec = t;

    // ---- occupancy grids ------------------------------------------------
    if (false) {  // static gradient layer (kind 1): off in the hero, it smeared the parked group:
                  // inflation ramp around the parked vehicles
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
        constexpr uint32_t kW = 124, kH = 32;  // x: egoX-14 .. egoX+48
        std::vector<uint8_t> c(static_cast<size_t>(kW) * kH, 0);
        const double ox = egoX - 14.0, oy = -8.0;  // covers the whole traffic window
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
                    const double fs = farScale(o.position.x - kOrigin.x);
                    val = std::max(val, 100.0 * fs * std::exp(-dd * dd / (r * r)));
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
    for (int ring = 0; ring < 9; ++ring) {
        const double r = 3.0 + ring * 2.2;
        const int n = 120 + ring * 18;
        for (int i = 0; i < n; ++i) {
            const double a = 2 * kPi * i / n;
            const double sweep = pmod(a - 3 * ph + 8 * kPi, 2 * kPi) / (2 * kPi);
            const int br = static_cast<int>(70 + 185 * std::pow(1.0 - sweep, 3.0));
            const double k = ring / 8.0;
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
        if (std::hypot(ox, oy) > 45.0 || (o.velocity.x == 0.0 && o.velocity.y == 0.0))
            continue;  // returns on moving actors only
        const double hl = o.dimensions.x / 2, hw = o.dimensions.y / 2;
        const double c = std::cos(o.heading_rad), sn = std::sin(o.heading_rad);
        const double perim = 4 * (hl + hw);
        for (double sp = 0.0; sp < perim; sp += 0.3) {
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
            for (double z = 0.15; z < o.dimensions.z; z += 0.3) {
                const double k = z / o.dimensions.z;
                overlume::PointCloudPoint p{};
                p.position = {o.position.x + c * lx - sn * ly, o.position.y + sn * lx + c * ly, z};
                p.rgba =
                    rgba(255, static_cast<int>(240 - 40 * k), static_cast<int>(120 - 60 * k), 255);
                d.cloud.push_back(p);
            }
        }
    }
    // A few clearly visible returns: bright scan lines on the road-facing side of the
    // right-verge parked group (not the dense per-actor shells).
    for (double x0 : {24.0, 29.5, 35.5}) {
        const double hl = x0 > 35 ? 2.75 : 2.25;
        for (double x = x0 - hl; x <= x0 + hl; x += 0.1)
            for (double z : {0.3, 1.0}) {
                overlume::PointCloudPoint p{};
                p.position = W(x, -kEdgeOffset - 0.55, z);
                p.rgba = rgba(255, 190, 70, 255);
                d.cloud.push_back(p);
            }
    }
    // Near-end faces of the parked group, plus brighter 2x2 ring dots where the rings cross it.
    for (double x0 : {24.0, 29.5, 35.5}) {
        const double hl = x0 > 35 ? 2.75 : 2.25;
        for (double y = -kEdgeOffset - 0.55; y >= -kEdgeOffset - 2.5; y -= 0.1)
            for (double z : {0.3, 1.0}) {
                overlume::PointCloudPoint p{};
                p.position = W(x0 - hl - 0.05, y, z);
                p.rgba = rgba(255, 190, 70, 255);
                d.cloud.push_back(p);
            }
    }
    for (double r : {14.0, 16.2, 18.4, 20.6, 22.8}) {
        for (double deg = -26.0; deg <= -2.0; deg += 0.35 * 57.3 / r)
            for (double dz : {0.0, 0.08}) {
                const double a = deg / 57.29578;
                overlume::PointCloudPoint p{};
                p.position = {ego.x + r * std::cos(a), ego.y + r * std::sin(a), 0.14 + dz};
                p.rgba = rgba(255, 215, 120, 255);
                d.cloud.push_back(p);
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
    add_alert(kCrosswalkX, gPedY, 2.2, 2.2, 2);
    if (cycS > 0.3) add_alert(cycX, -3.2, 2.0 * cycS, 1.2 * cycS, 1);

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
                                        {0.3f, 1.0, 0.8, 1.0f}));
    }
    {
        auto m = base_marker(overlume::MarkerPrimitive::ARROW, W(10.0, -kEdgeOffset - 5.0, 0.4),
                             {2.4, 0.6, 0.6}, {1.0, 0.9, 0.2, 1.0f});
        d.markers.push_back(m);
    }
    {
        auto m = base_marker(overlume::MarkerPrimitive::CUBE, W(28.0, kEdgeOffset + 1.6, 0.5),
                             {1.0, 1.0, 1.0}, {0.9f, 0.3f, 0.6f, 1.0f});
        m.heading_rad = ph;
        d.markers.push_back(m);
    }
    {
        auto m = base_marker(overlume::MarkerPrimitive::MESH, W(17.5, kEdgeOffset + 3.0, 0.0),
                             {1.5, 1.5, 1.5}, {1.0f, 0.55f, 0.1f, 1.0f});
        m.mesh_path = meshPath.c_str();
        m.heading_rad = -ph;
        d.markers.push_back(m);
    }
    {  // left-verge marker group (clear of the ego path and the crosswalk): strip + triangle
        std::vector<Vec3> zig;
        for (int i = 0; i < 7; ++i)
            zig.push_back(W(14.0 + i * 1.0, -kEdgeOffset - 9.0 + ((i % 2) ? 0.5 : -0.5), 0.3));
        auto m = base_marker(overlume::MarkerPrimitive::LINE_STRIP, {0, 0, 0}, {1, 1, 1},
                             {1.0, 0.3, 0.5, 1.0f});
        m.points = d.poly(zig);
        m.point_count = static_cast<uint32_t>(zig.size());
        d.markers.push_back(m);
        std::vector<Vec3> tri = {W(8.0, -kEdgeOffset - 8.0, 0.05), W(9.6, -kEdgeOffset - 8.0, 0.05),
                                 W(8.8, -kEdgeOffset - 9.2, 0.05)};
        auto tr = base_marker(overlume::MarkerPrimitive::TRIANGLE_LIST, {0, 0, 0}, {1, 1, 1},
                              {1.0, 0.8, 0.0f, 1.0f});
        tr.points = d.poly(tri);
        tr.point_count = 3;
        d.markers.push_back(tr);
    }
    // ---- HUD ------------------------------------------------------------
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
    s.point_clouds = &d.pc;
    s.point_cloud_count = 1;
    s.trajectory_carpets = &d.tc;
    s.trajectory_carpet_count = 1;
}

// Periodic camera: level 3/4 chase view with a gentle swing toward the
// parked-car side (+y side is a baked building). Pitch stays under ~20 deg.
static double getenv_d(const char* k, double d) {
    const char* v = std::getenv(k);
    return v ? std::atof(v) : d;
}
overlume::CameraPose camera(double u) {
    const double w = 0.5 - 0.5 * std::cos(2 * kPi * u);  // one swing per loop
    // Eye sits behind the ego; the look-at is well ahead so the actor band
    // fills the middle of the frame and the ego anchors the bottom.
    const double back = getenv_d("HERO_BACK", 9.4), ahead = getenv_d("HERO_AHEAD", 5.0);
    const double H = getenv_d("HERO_H", 10.0) + 0.4 * w;
    overlume::CameraPose p{};
    p.eye[0] = kOrigin.x + kEgoX - back;
    p.eye[1] = kOrigin.y + kLaneY - 0.5 - 1.0 * w;
    p.eye[2] = H;
    p.target[0] = kOrigin.x + kEgoX + ahead;
    p.target[1] = kOrigin.y + kLaneY - 0.5 + 0.5 * w;
    p.target[2] = 0.5;
    p.vfov_deg = getenv_d("HERO_VFOV", 49.0);
    return p;
}

// ---- --check: debug self-check of the choreography and framing ---------------------------------
struct Rect {
    double cx, cy, c, s, hl, hw;
    std::array<std::array<double, 2>, 4> corners() const {
        std::array<std::array<double, 2>, 4> k{};
        int i = 0;
        for (double sx : {-1.0, 1.0})
            for (double sy : {-1.0, 1.0}) {
                const double lx = sx * hl, ly = (sx * sy) * hw;  // ccw order
                k[static_cast<size_t>(i++)] = {cx + c * lx - s * ly, cy + s * lx + c * ly};
            }
        return k;
    }
};
double segDist(const std::array<double, 2>& p, const std::array<double, 2>& a,
               const std::array<double, 2>& b) {
    const double dx = b[0] - a[0], dy = b[1] - a[1], L = dx * dx + dy * dy;
    const double u =
        L > 0 ? std::clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L, 0.0, 1.0) : 0;
    return std::hypot(p[0] - a[0] - u * dx, p[1] - a[1] - u * dy);
}
// Gap between two oriented rectangles (0 when they overlap).
double rectDist(const Rect& A, const Rect& B) {
    const auto ca = A.corners(), cb = B.corners();
    bool apart = false;
    for (const Rect* R : {&A, &B})
        for (int ax = 0; ax < 2 && !apart; ++ax) {
            const double nx = ax ? -R->s : R->c, ny = ax ? R->c : R->s;
            double a0 = 1e18, a1 = -1e18, b0 = 1e18, b1 = -1e18;
            for (int i = 0; i < 4; ++i) {
                const double pa = nx * ca[static_cast<size_t>(i)][0] +
                                  ny * ca[static_cast<size_t>(i)][1],
                             pb = nx * cb[static_cast<size_t>(i)][0] +
                                  ny * cb[static_cast<size_t>(i)][1];
                a0 = std::min(a0, pa), a1 = std::max(a1, pa), b0 = std::min(b0, pb),
                b1 = std::max(b1, pb);
            }
            apart = a1 < b0 || b1 < a0;
        }
    if (!apart) return 0.0;
    double d = 1e18;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            const auto ui = static_cast<size_t>(i), uj = static_cast<size_t>(j),
                       uk = static_cast<size_t>((j + 1) % 4);
            d = std::min({d, segDist(ca[ui], cb[uj], cb[uk]), segDist(cb[ui], ca[uj], ca[uk])});
        }
    return d;
}
// Pixel rows between the rendered ego silhouette's lowest edge and the bottom image edge. The
// silhouette's rear edge was measured at x = kEgoX - 0.4 (its ground contact, not the 4.5 m box).
double egoBottomMargin(const overlume::CameraPose& cam, uint32_t w, uint32_t h) {
    const double e[3] = {cam.eye[0], cam.eye[1], cam.eye[2]};
    double f[3] = {cam.target[0] - e[0], cam.target[1] - e[1], cam.target[2] - e[2]};
    const double fl = std::hypot(f[0], f[1], f[2]);
    for (double& c : f) c /= fl;
    double r[3] = {f[1], -f[0], 0.0};  // forward x up(0,0,1)
    const double rl = std::hypot(r[0], r[1]);
    r[0] /= rl, r[1] /= rl;
    const double u[3] = {r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2],
                         r[0] * f[1] - r[1] * f[0]};
    double worst = 1e18;
    for (double dx : {-0.4})
        for (double dy : {-0.95, 0.95}) {
            const double p[3] = {kOrigin.x + kEgoX + dx - e[0], kOrigin.y + kLaneY + dy - e[1],
                                 -e[2]};
            const double zc = p[0] * f[0] + p[1] * f[1] + p[2] * f[2],
                         yc = p[0] * u[0] + p[1] * u[1] + p[2] * u[2];
            const double ndc = yc / (zc * std::tan(cam.vfov_deg * kPi / 360.0));
            worst = std::min(worst, (1.0 + ndc) / 2.0 * h);  // pixels above the bottom edge
        }
    (void)w;
    return worst;
}
int selfCheck(const Args& a) {
    constexpr int kSub = 4;  // samples per frame
    const double T = static_cast<double>(a.frames) / a.fps, dt = 1.0 / a.fps;
    const Rect ego{kOrigin.x + kEgoX, kOrigin.y + kLaneY, 1, 0, 2.25, 0.95};
    double minPed = 1e9, minGap = 1e9, minMargin = 1e9, seam = 0, peak = 0;
    double catchV = 0, catchX = 0, stopClear = 1e9, groundWorst = 0;
    std::string groundWhat;
    const double cruise = kLap / T;
    std::vector<double> prevX;
    std::string err;
    SceneData d, d2;
    for (int i = 0; i < a.frames * kSub; ++i) {
        const double t = (static_cast<double>(i) / kSub + a.phase * a.frames) * dt;
        build(d, t, T, "", a.yield);
        build(d2, t + T, T, "", a.yield);
        char at[64];
        std::snprintf(at, sizeof at, " at t=%.3f s (frame %.2f)", t, i / double(kSub));
        std::vector<Rect> rects;
        for (size_t k = 0; k < d.objects.size(); ++k) {
            const auto& o = d.objects[k];
            rects.push_back({o.position.x, o.position.y, std::cos(o.heading_rad),
                             std::sin(o.heading_rad), o.dimensions.x / 2, o.dimensions.y / 2});
            seam = std::max(seam, std::hypot(o.position.x - d2.objects[k].position.x,
                                             o.position.y - d2.objects[k].position.y));
        }
        rects.push_back(ego);
        {  // ground extent: every scene point inside the +-kGroundHalf ground plane, minus a margin
            const double lim = kGroundHalf - kGroundMargin, cx = kOrigin.x + kEgoX,
                         cy = kOrigin.y + kLaneY;
            auto chk = [&](const Vec3& p, const char* what) {
                const double ex = std::max(std::fabs(p.x - cx), std::fabs(p.y - cy));
                if (ex > groundWorst) {
                    groundWorst = ex;
                    groundWhat = std::string(what) + " x=" + std::to_string(p.x - kOrigin.x) +
                                 " y=" + std::to_string(p.y - kOrigin.y) + at;
                }
                if (ex > lim && err.empty())
                    err = std::string(what) + " outside the ground extent (" + std::to_string(ex) +
                          " m > " + std::to_string(lim) + " m)" + at;
            };
            for (const auto& m : d.map)
                for (uint32_t q = 0; q < m.point_count; ++q) chk(m.points[q], "map element");
            for (const auto& rb : d.ribbons)
                for (uint32_t q = 0; q < rb.point_count; ++q) chk(rb.points[q], "ribbon");
            for (const auto& o : d.objects) {
                chk({o.position.x + o.dimensions.x / 2, o.position.y + o.dimensions.y / 2, 0},
                    "actor");
                chk({o.position.x - o.dimensions.x / 2, o.position.y - o.dimensions.y / 2, 0},
                    "actor");
                for (uint32_t q = 0; q < o.predicted_path_count; ++q)
                    chk(o.predicted_path[q], "predicted path");
            }
            for (const auto& g : d.grids)
                for (double fx : {0.0, 1.0})
                    for (double fy : {0.0, 1.0})
                        chk({g.origin.x + fx * g.width_cells * g.resolution_m,
                             g.origin.y + fy * g.height_cells * g.resolution_m, 0},
                            "occupancy grid");
            for (const auto& al : d.alerts)
                for (uint32_t q = 0; q < al.point_count; ++q) chk(al.points[q], "alert");
            for (const auto& mk : d.markers) {
                if (!mk.point_count) chk(mk.position, "marker");
                for (uint32_t q = 0; q < mk.point_count; ++q) chk(mk.points[q], "marker");
            }
            for (const auto& c : d.carpet) chk(c.position, "carpet");
        }
        {  // catch-up speed (reported velocity and, independently, d(position)/dt) + stop clearance
            const double ddt = dt / kSub;
            for (size_t k = 0; k < d.objects.size(); ++k) {
                const auto& o = d.objects[k];
                if (o.cls == overlume::ObjectClass::PEDESTRIAN ||
                    (o.velocity.x == 0 && o.velocity.y == 0))
                    continue;
                catchV = std::max(catchV, std::fabs(o.velocity.x) / cruise);
                if (prevX.size() <= k) prevX.resize(d.objects.size(), 1e9);
                const double step = std::fabs(o.position.x - prevX[k]);
                if (step < kLap / 2) catchX = std::max(catchX, step / ddt / cruise);
                prevX[k] = o.position.x;
                if (std::hypot(o.velocity.x, o.velocity.y) < 0.3) {
                    const double hl = o.dimensions.x / 2, lx = o.position.x - kOrigin.x;
                    const double c = std::cos(o.heading_rad) < 0 ? lx - hl - (kCrosswalkX + 1.5)
                                                                 : (kCrosswalkX - 1.5) - (lx + hl);
                    stopClear = std::min(stopClear, c);
                }
            }
        }
        auto dyn = [&](size_t k) {
            return k < d.objects.size() &&
                   (d.objects[k].velocity.x != 0 || d.objects[k].velocity.y != 0);
        };
        auto isVeh = [&](size_t k) {
            const auto c = d.objects[k].cls;
            return dyn(k) && c != overlume::ObjectClass::PEDESTRIAN;
        };
        for (size_t k = 0; k < d.objects.size(); ++k) {
            if (dyn(k))
                peak = std::max(peak, std::hypot(d.objects[k].velocity.x, d.objects[k].velocity.y));
            if (d.objects[k].cls == overlume::ObjectClass::PEDESTRIAN && dyn(k))
                for (size_t m = 0; m < d.objects.size(); ++m)
                    if (isVeh(m)) {
                        const double g = rectDist(rects[k], rects[m]);
                        minPed = std::min(minPed, g);
                        if (g < 1.5)
                            err = "vehicle/cyclist (object " + std::to_string(m + 1) +
                                  " x=" + std::to_string(d.objects[m].position.x - kOrigin.x) +
                                  " ped y=" + std::to_string(d.objects[k].position.y - kOrigin.y) +
                                  ") within 1.5 m of the pedestrian (" + std::to_string(g) + " m)" +
                                  at;
                        const auto& vo = d.objects[m];
                        for (uint32_t q = 0; q < vo.predicted_path_count; ++q)
                            if (std::hypot(vo.predicted_path[q].x - d.objects[k].position.x,
                                           vo.predicted_path[q].y - d.objects[k].position.y) < 1.5)
                                err = "predicted path runs through the pedestrian (object " +
                                      std::to_string(m + 1) + ")" + at;
                    }
            for (size_t m = 0; m < rects.size(); ++m) {
                if (m == k || !dyn(k) || (m < d.objects.size() && dyn(m) && m < k)) continue;
                const double g = rectDist(rects[k], rects[m]);
                if (g <= 0.0 && err.empty())
                    err = "actors overlap (ids " + std::to_string(k + 1) + ", " +
                          std::to_string(m + 1) + ")" + at;
                if (isVeh(k) && m < d.objects.size() && isVeh(m)) {
                    minGap = std::min(minGap, g);
                    if (g < 2.0 && err.empty())
                        err = "queued actors closer than 2 m (" + std::to_string(g) + " m)" + at;
                }
            }
        }
        const double mg = egoBottomMargin(camera(pmod(double(i) / kSub / a.frames + a.phase, 1.0)),
                                          a.width, a.height);
        minMargin = std::min(minMargin, mg);
        if (mg < 24.0 * a.height / 495.0 && err.empty())
            err = "ego bottom margin " + std::to_string(mg) + " px" + at;
    }
    std::printf(
        "render_hero --check: ped clearance min %.2f m, vehicle gap min %.2f m, peak speed %.1f "
        "m/s, "
        "ego margin min %.1f px of %u (= %.1f px at 495), loop seam %.2g m\n",
        minPed, minGap, peak, minMargin, a.height, minMargin * 495.0 / a.height, seam);
    if (seam > 1e-6) err = "loop is not seamless";
    // theme timeline: dark at frame 0 and at the loop end (so the seam is exact), light hold >= 3 s
    const double th0 = themeLight(0.0, T), thEnd = themeLight(T, T), thLast = themeLight(T - dt, T);
    const double hold = (kFc - kFb) * T;
    const double tw = 2 * kPedK / kPedV, kerbWait = (T - 2 * tw) / 2;
    std::printf(
        "render_hero --check: theme light weight t=0 %.3f, loop end %.3f, last frame %.3f, light "
        "hold %.1f s; "
        "catch-up peak %.3fx (positions %.3fx, cap %.2fx); stop clearance min %.2f m; kerb wait "
        "%.2f s; "
        "content extent max %.1f m (%s) of %.1f m allowed\n",
        th0, thEnd, thLast, hold, catchV, catchX, kCatchMax, stopClear, kerbWait, groundWorst,
        groundWhat.c_str(), kGroundHalf - kGroundMargin);
    if (th0 != 0.0 || thEnd != th0 || thLast != 0.0)
        err = "theme state at t=0 differs from loop end";
    if (themeLight((kFb + 0.5 * (kFc - kFb)) * T, T) != 1.0 || hold < 3.0)
        err = "light hold shorter than 3 s";
    if (std::max(catchV, catchX) > kCatchMax)
        err = "catch-up speed above " + std::to_string(kCatchMax) + "x cruise";
    if (stopClear < 2.5)
        err = "stopped actor front closer than 2.5 m to the crosswalk edge (" +
              std::to_string(stopClear) + " m)";
    if (kerbWait < 2.0) err = "pedestrian kerb wait under 2 s";
    if (!err.empty()) {
        std::fprintf(stderr, "render_hero --check FAIL: %s\n", err.c_str());
        return 1;
    }
    std::printf("render_hero --check PASS\n");
    return 0;
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
    if (a.check) return selfCheck(a);
    const int period = a.frames;
    const double periodSec = static_cast<double>(period) / a.fps;

    // The hero renders with its own theme dir: dark_adas is copied with hero-only
    // overrides (opaque, saturated object tints, thinner ribbon, distance fog);
    // light_clay is copied verbatim so the light hold shows the shipped palette.
    // The shipped YAMLs under assets/themes are never modified.
    namespace fs = std::filesystem;
    const fs::path heroThemes =
        fs::temp_directory_path() / ("overlume_hero_themes_" + std::to_string(::getpid()));
    fs::create_directories(heroThemes);
    for (const char* nm : {"dark_adas", "light_clay"}) {
        std::ifstream in(a.root + "/assets/themes/" + nm + ".yaml");
        std::stringstream ss;
        ss << in.rdbuf();
        std::string y = ss.str();
        auto sub = [&](const std::string& from, const std::string& to) {
            const size_t p = y.find(from);
            if (p != std::string::npos)
                y.replace(p, from.size(), to);
            else
                std::fprintf(stderr, "render_hero: theme override not found in %s: %s\n", nm,
                             from.c_str());
        };
        const bool light = std::string(nm) == "light_clay";
        if (light) {  // the light hold shows the shipped light_clay palette exactly as it ships
            std::ofstream(heroThemes / (std::string(nm) + ".yaml")) << y;
            continue;
        }
        sub("objects: { opacity: 0.25 }", "objects: { opacity: 1.0 }");
        // Distance fade: far road / ground dissolve into the sky instead of ending at the ground's
        // rim. Fog colour is calibrated (by probing a dense-fog render) so a fully fogged pixel
        // equals the clear/sky colour: the renderer scales the fog colour by the IBL intensity.
        sub("fog:         [0.028, 0.036, 0.085]", "fog:         [0.018, 0.022, 0.053]");
        sub("fog: { density: 0.010 }", "fog: { density: 0.035 }");
        sub("emissive: { ribbon_strength: 0.0 }", "emissive: { ribbon_strength: 0.6 }");
        sub("car: [0.180, 0.210, 0.320]", "car: [0.10, 0.45, 1.00]");
        sub("truck_van: [0.28, 0.32, 0.55]", "truck_van: [0.60, 0.25, 0.95]");
        sub("pedestrian: [0.85, 0.25, 0.25]", "pedestrian: [1.00, 0.20, 0.30]");
        sub("cyclist: [0.80, 0.50, 0.15]", "cyclist: [1.00, 0.75, 0.05]");
        sub("opacity: 0.75, fade_start_m", "opacity: 0.45, fade_start_m");
        std::ofstream(heroThemes / (std::string(nm) + ".yaml")) << y;
    }
    const std::string themes = heroThemes.string();
    const std::string models = a.root + "/assets/models";
    const std::string town = a.root + "/tests/fixtures/environment_test_town_0";
    const std::string mesh = a.root + "/assets/models/car.glb";

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
    bool toLight = false, toDark = false;

    // Warm-up: render the three frames preceding frame 0 (same scene/camera as the loop's tail)
    // so temporal history and lazily loaded assets are continuous at the wrap.
    for (int i = -3; i < 0; ++i) {
        build(data, (i + a.phase * period) * dt, periodSec, mesh, a.yield);
        overlume::set_scene(r, data.graph);
        overlume::render_frame(
            r, camera(pmod(static_cast<double>(i + period) / period + a.phase, 1.0)),
            {rgb.data(), a.width, a.height});
    }

    for (int i = 0; i < period; ++i) {
        const double t = (i + a.phase * period) * dt;
        // Theme beat: scene time of each crossfade start is a pure function of the loop phase.
        const double t0 = a.phase * periodSec;
        if (!toLight && i * dt >= kFa * periodSec) {
            toLight =
                overlume::set_theme(r, "light_clay", t0 + kFa * periodSec, (kFb - kFa) * periodSec);
        }
        if (toLight && !toDark && i * dt >= kFc * periodSec) {
            toDark =
                overlume::set_theme(r, "dark_adas", t0 + kFc * periodSec, (kFd - kFc) * periodSec);
        }
        build(data, t, periodSec, mesh, a.yield);
        overlume::set_scene(r, data.graph);
        if (!overlume::render_frame(r, camera(pmod(static_cast<double>(i) / period + a.phase, 1.0)),
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
    fs::remove_all(heroThemes);
    return 0;
}
