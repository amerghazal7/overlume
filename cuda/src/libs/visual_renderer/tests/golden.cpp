// golden.cpp — Epic 1 Task 2 Step 9. See golden.hpp. Not a gtest file
// (excluded from CMakeLists.txt's auto-glob-as-gtest-binary loop by name;
// compiled as a plain extra source into every other test binary instead).
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

namespace mpviz::testing {
namespace {

// This epic's goldens are all committed at a fixed 320x240 (see golden.hpp
// / spec: "purely for CI speed/determinism, not a statement about the
// shipped default").
constexpr uint32_t kWidth = 320;
constexpr uint32_t kHeight = 240;

double luminance(uint8_t r, uint8_t g, uint8_t b) {
    return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

// ponytail: block-wise (8x8, non-overlapping, luminance-only) mean/
// variance/covariance SSIM averaged over blocks -- a deliberately simpler
// approximation of the full windowed-Gaussian SSIM (unnecessary precision
// for a pass/fail regression gate at low-preset resolution).
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

}  // namespace

double render_and_compare(mpviz::VisualRenderer* r, const mpviz::CameraPose& pose,
                           const char* golden_png_path, const char* out_png_path) {
    if (r == nullptr) return -1.0;

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3);
    mpviz::FrameView view{rgb.data(), kWidth, kHeight};
    if (!mpviz::render_frame(r, pose, view)) return 0.0;

    stbi_write_png(out_png_path, static_cast<int>(kWidth), static_cast<int>(kHeight), 3,
                    rgb.data(), static_cast<int>(kWidth) * 3);

    int goldenWidth = 0, goldenHeight = 0, goldenChannels = 0;
    uint8_t* golden = stbi_load(golden_png_path, &goldenWidth, &goldenHeight, &goldenChannels, 3);
    if (golden == nullptr) {
        // Golden missing -- first run of a new golden always fails loudly,
        // never silently "passes" with nothing to compare against.
        return 0.0;
    }
    if (static_cast<uint32_t>(goldenWidth) != kWidth || static_cast<uint32_t>(goldenHeight) != kHeight) {
        stbi_image_free(golden);
        return 0.0;
    }
    const std::vector<uint8_t> goldenPixels(golden, golden + static_cast<size_t>(goldenWidth) *
                                                                 goldenHeight * 3);
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
    // sky_row_mean/horizon_row_mean row bands -- see golden.hpp's comment on
    // FrameStats for why these specific rows (fixed to this epic's fixed
    // 320x240 / CameraPose test setup).
    constexpr int kSkyRowStart = 10, kSkyRowEnd = 40;
    // Starts at 50, not 48: rows 48-49 are pure sky at this fixed pose (the
    // first ground row is 50) -- including them let ~2/12 of this band read
    // as sky, understating the true ground/sky gap by ~4 levels (epic1
    // Task 2 review round 6). See golden.hpp's FrameStats comment.
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

// Epic 2 Task 2 (VM-024) Step 8: see golden.hpp. `.geom` format, one
// element per line: `<is_polygon:0|1> <n> <x1> <y1> <z1> ... <xn> <yn> <zn>`.
// A malformed line (fewer than n points, non-numeric field) is skipped, not
// half-consumed into the next line's read.
MapGeom load_map_geom(const char* path) {
    MapGeom g;
    std::ifstream in(path);
    if (!in) return g;

    // Two-pass: first pass appends every point into g.points (so its final
    // buffer address is fixed before anything points into it), recording
    // each element's (is_polygon, offset, count); second pass builds
    // g.elements from that metadata. Doing it in one pass would mean
    // g.points might reallocate mid-way and invalidate offsets computed
    // against an earlier capacity -- offsets survive that fine (they're
    // integers, not pointers), but computing the final `MapElement::points`
    // pointers only after all growth is done is simpler to reason about
    // than re-deriving them from a moving target.
    std::vector<std::tuple<uint8_t, uint32_t, uint32_t>> meta;  // is_polygon, offset, count
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        int isPolygon = 0;
        uint32_t n = 0;
        if (!(iss >> isPolygon >> n)) continue;
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
            g.points.resize(offset);  // drop the partially-read element's points
            continue;
        }
        meta.emplace_back(static_cast<uint8_t>(isPolygon != 0), offset, n);
    }

    g.elements.reserve(meta.size());
    for (const auto& [isPolygon, offset, n] : meta) {
        mpviz::MapElement e{};
        e.points = n > 0 ? g.points.data() + offset : nullptr;
        e.point_count = n;
        e.is_polygon = isPolygon;
        g.elements.push_back(e);
    }
    return g;
}

// Epic 2 Task 4 (VM-022) Step 2: see golden.hpp. One TrackedObject per
// ObjectClass, positioned so a camera looking roughly at the world origin
// sees all six.
ObjectScene make_mixed_class_objects(double now) {
    struct Spec {
        mpviz::ObjectClass cls;
        mpviz::Vec3 position;
        double heading;
        mpviz::Vec3 dims;
        mpviz::Vec3 velocity;
        std::vector<mpviz::Vec3> path;  // empty -> no predicted path
        double age_sec;                 // last_update_sec = now - age_sec
    };
    const std::vector<Spec> specs = {
        // CAR: fresh, nonzero velocity -- exercises the velocity-arrow path.
        {mpviz::ObjectClass::CAR, {-6.0, -3.0, 0.0}, 0.3, {4.5, 1.8, 1.5}, {3.0, 1.0, 0.0}, {}, 0.0},
        // TRUCK_VAN: fresh, has a predicted path -- exercises extrude_polyline.
        {mpviz::ObjectClass::TRUCK_VAN,
         {-3.0, 4.0, 0.0},
         -0.5,
         {5.5, 2.0, 2.2},
         {0.0, 0.0, 0.0},
         {{-3.0, 4.0, 0.0}, {-1.0, 3.0, 0.0}, {1.0, 2.5, 0.0}, {3.0, 2.0, 0.0}},
         0.0},
        // BUS: fresh, plain.
        {mpviz::ObjectClass::BUS, {5.0, 5.0, 0.0}, 2.1, {12.0, 2.5, 3.2}, {0.0, 0.0, 0.0}, {}, 0.0},
        // PEDESTRIAN: fresh, plain.
        {mpviz::ObjectClass::PEDESTRIAN, {2.0, -5.0, 0.0}, 1.0, {0.6, 0.6, 1.8}, {0.0, 0.0, 0.0}, {}, 0.0},
        // CYCLIST: fresh, plain.
        {mpviz::ObjectClass::CYCLIST, {-2.0, -6.0, 0.0}, 0.6, {1.9, 0.7, 1.7}, {0.0, 0.0, 0.0}, {}, 0.0},
        // UNKNOWN: deliberately stale -- 0.9s behind `now`, inside the
        // 0.5s/1.0s fade window (alpha ~0.2).
        {mpviz::ObjectClass::UNKNOWN, {6.0, -6.0, 0.0}, -1.2, {2.0, 2.0, 2.0}, {0.0, 0.0, 0.0}, {}, 0.9},
    };

    ObjectScene s;
    size_t totalPathPoints = 0;
    for (const auto& sp : specs) totalPathPoints += sp.path.size();
    s.path_points.reserve(totalPathPoints);  // fixed capacity FIRST -- see golden.hpp/MapGeom's
                                              // own comment on why this must not reallocate mid-loop
    s.objects.reserve(specs.size());

    uint32_t nextId = 1;
    for (const auto& sp : specs) {
        mpviz::TrackedObject obj{};
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

// Epic 2 Task 5 (VM-023) Step 3: see golden.hpp. One ribbon per role,
// camera-visible from the same kind of origin-looking pose test_objects.cpp
// uses (kPose there / RibbonGolden.ThreeRoles_DarkAdas here).
RibbonScene make_three_role_ribbons(double now)
{
    struct Spec
    {
        mpviz::PathRole role;
        std::vector<mpviz::Vec3> points;
        double age_sec;  // last_update_sec = now - age_sec
    };
    const std::vector<Spec> specs = {
        // BEHAVIOR: the hero ribbon, fresh, sweeping toward the camera.
        {mpviz::PathRole::BEHAVIOR,
         {{-6.0, -2.0, 0.0}, {-3.0, -1.0, 0.0}, {0.0, 0.0, 0.0}, {3.0, 1.0, 0.0}, {6.0, 2.0, 0.0}},
         0.0},
        // GLOBAL: fresh, a distinct straight line off to one side.
        {mpviz::PathRole::GLOBAL, {{-8.0, 5.0, 0.0}, {0.0, 6.0, 0.0}, {8.0, 5.0, 0.0}}, 0.0},
        // LOCAL: fresh, a short curl off to the other side.
        {mpviz::PathRole::LOCAL, {{-4.0, -6.0, 0.0}, {0.0, -8.0, 0.0}, {4.0, -6.0, 0.0}}, 0.0},
    };

    RibbonScene s;
    size_t totalPoints = 0;
    for (const auto& sp : specs) totalPoints += sp.points.size();
    s.point_storage.reserve(totalPoints);  // fixed capacity FIRST -- see golden.hpp's own
                                            // comment on why this must not reallocate mid-loop
    s.ribbons.reserve(specs.size());

    for (const auto& sp : specs)
    {
        const size_t offset = s.point_storage.size();
        for (const auto& p : sp.points) s.point_storage.push_back(p);
        mpviz::PathRibbon r{};
        r.role = sp.role;
        r.points = s.point_storage.data() + offset;
        r.point_count = static_cast<uint32_t>(sp.points.size());
        r.last_update_sec = now - sp.age_sec;
        s.ribbons.push_back(r);
    }
    return s;
}

// Epic 2 Task 7 (VM-026) Step 3: see golden.hpp. Synthetic (FIXTURE GAP 4 --
// the five collision-checker topics were silent in the recorded bag).
AlertScene make_sweep_and_predicted_alerts(double now)
{
    struct Spec
    {
        uint8_t severity;
        std::vector<mpviz::Vec3> points;
        double age_sec;  // last_update_sec = now - age_sec
    };
    const std::vector<Spec> specs = {
        // Ego footprint sweep: severity 0 (info -- the ghost trail, see the
        // plan's "the ego sweep gets a ghost alpha" -- that's a property of
        // severity 0 itself, not a topic/role AlertPolygon has no field
        // for). Aged 0.75s stale (kStaleFadeStartSec=0.5/
        // kStaleFadeTimeoutSec=1.0 -- solidly mid-fade) so the golden shows
        // the fade actually applied, not just the constant's already-low
        // alpha.
        {0, {{-3.0, -2.0, 0.0}, {3.0, -2.0, 0.0}, {3.0, 2.0, 0.0}, {-3.0, 2.0, 0.0}}, 0.75},
        // Object predicted polygon: severity 1 (warning), fresh, off to one
        // side so a human sees both shapes distinctly.
        {1, {{5.0, 4.0, 0.0}, {8.0, 4.0, 0.0}, {8.0, 7.0, 0.0}, {5.0, 7.0, 0.0}}, 0.0},
    };

    AlertScene s;
    size_t totalPoints = 0;
    for (const auto& sp : specs) totalPoints += sp.points.size();
    s.point_storage.reserve(totalPoints);  // fixed capacity FIRST -- see golden.hpp's own
                                            // comment on why this must not reallocate mid-loop
    s.alerts.reserve(specs.size());

    for (const auto& sp : specs)
    {
        const size_t offset = s.point_storage.size();
        for (const auto& p : sp.points) s.point_storage.push_back(p);
        mpviz::AlertPolygon a{};
        a.points = s.point_storage.data() + offset;
        a.point_count = static_cast<uint32_t>(sp.points.size());
        a.severity = sp.severity;
        a.last_update_sec = now - sp.age_sec;
        s.alerts.push_back(a);
    }
    return s;
}

// Epic 2 Task 6 (VM-025) Step 3: see golden.hpp. Synthetic (FIXTURE GAP 3 --
// no OccupancyGrid topic exists in the recorded bag/stack).
GridScene make_two_layer_grids(double now) {
    constexpr uint32_t kW = 16, kH = 16;
    constexpr double kRes = 0.5;  // 16 * 0.5 = 8m footprint

    GridScene s;
    s.cell_storage.reserve(2);
    s.grids.reserve(2);

    // Layer 0: dynamic OGM (kind 0) -- concentric occupancy blob, centered
    // in its own footprint, corners forced to the 255 unknown sentinel.
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
        // Corners: the unknown sentinel, not a legal occupancy value --
        // proves the golden itself (not just the adapter unit test) renders
        // "ground shows through", never "very occupied".
        cells[0] = 255;
        cells[kW - 1] = 255;
        cells[(kH - 1) * kW] = 255;
        cells[(kH - 1) * kW + (kW - 1)] = 255;
        s.cell_storage.push_back(std::move(cells));

        mpviz::GroundGridLayer layer{};
        layer.kind = 0;
        layer.origin = {-4.0, -4.0, 0.0};
        layer.resolution_m = kRes;
        layer.width_cells = kW;
        layer.height_cells = kH;
        layer.cells = s.cell_storage.back().data();
        layer.last_update_sec = now;
        s.grids.push_back(layer);
    }

    // Layer 1: gradient OGM (kind 1) -- smooth left-to-right ramp, no
    // unknown cells, offset +3m in X so the two footprints only partially
    // overlap in the golden (two DISTINCT layers, not one fully occluding
    // the other).
    {
        std::vector<uint8_t> cells(static_cast<size_t>(kW) * kH);
        for (uint32_t y = 0; y < kH; ++y) {
            for (uint32_t x = 0; x < kW; ++x) {
                const double occ = 100.0 * (static_cast<double>(x) / (kW - 1));
                cells[y * kW + x] = static_cast<uint8_t>(occ + 0.5);
            }
        }
        s.cell_storage.push_back(std::move(cells));

        mpviz::GroundGridLayer layer{};
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

mpviz::Vec3 centroid(const std::vector<mpviz::MapElement>& elems) {
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

}  // namespace mpviz::testing
