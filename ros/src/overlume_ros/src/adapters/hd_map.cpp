// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/hd_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume::ros {
namespace {

constexpr int32_t kActionAdd = 0;
constexpr int32_t kActionModify = 1;
constexpr int32_t kActionDelete = 2;
constexpr int32_t kActionDeleteAll = 3;
constexpr int32_t kMarkerTypeLineStrip = 4;

bool HasNan(const overlume::Vec3& p) {
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}

bool MarkerPoseIsIdentity(const geometry_msgs::msg::Pose& p) {
    constexpr double kEps = 1e-12;
    return std::abs(p.position.x) < kEps && std::abs(p.position.y) < kEps &&
           std::abs(p.position.z) < kEps && std::abs(p.orientation.x) < kEps &&
           std::abs(p.orientation.y) < kEps && std::abs(p.orientation.z) < kEps &&
           std::abs(p.orientation.w - 1.0) < kEps;
}

bool MarkerPoseHasNan(const geometry_msgs::msg::Pose& p) {
    return std::isnan(p.position.x) || std::isnan(p.position.y) || std::isnan(p.position.z) ||
           std::isnan(p.orientation.x) || std::isnan(p.orientation.y) ||
           std::isnan(p.orientation.z) || std::isnan(p.orientation.w);
}

constexpr uint32_t kRoadFillSamples = 16;

constexpr double kMapFadeWindowSec = 1.0;

std::vector<double> CumulativeArcLength(const std::vector<overlume::Vec3>& pts) {
    std::vector<double> cum(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
        const double dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y,
                     dz = pts[i].z - pts[i - 1].z;
        cum[i] = cum[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return cum;
}

overlume::Vec3 PointAtArcLength(const std::vector<overlume::Vec3>& pts,
                                const std::vector<double>& cum, double s) {
    const double total_len = cum.back();
    s = std::clamp(s, 0.0, total_len);
    size_t i = static_cast<size_t>(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
    if (i == 0) i = 1;
    if (i >= pts.size()) i = pts.size() - 1;
    const double seg_len = cum[i] - cum[i - 1];
    const double t = seg_len > 0.0 ? (s - cum[i - 1]) / seg_len : 0.0;
    const auto& a = pts[i - 1];
    const auto& b = pts[i];
    return overlume::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

std::vector<overlume::Vec3> ResampleByArcLength(const std::vector<overlume::Vec3>& pts,
                                                uint32_t n_stations) {
    std::vector<overlume::Vec3> out;
    if (pts.size() < 2 || n_stations == 0) return out;

    const std::vector<double> cum = CumulativeArcLength(pts);
    const double total_len = cum.back();

    out.reserve(n_stations);
    for (uint32_t k = 0; k < n_stations; ++k) {
        const double s = (n_stations == 1) ? 0.0
                                           : total_len * static_cast<double>(k) /
                                                 static_cast<double>(n_stations - 1);
        out.push_back(PointAtArcLength(pts, cum, s));
    }
    return out;
}

bool KindCarriesLaneId(overlume::MapKind kind) {
    return kind == overlume::MapKind::CENTERLINE || kind == overlume::MapKind::LEFT_BOUNDARY ||
           kind == overlume::MapKind::RIGHT_BOUNDARY || kind == overlume::MapKind::ROAD_EDGE;
}

constexpr double kRoadEdgeCoincidenceThresholdM = 1.0;

double PointToPolylineDist2D(const overlume::Vec3& p, const std::vector<overlume::Vec3>& poly) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
        const auto& a = poly[i];
        const auto& b = poly[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = 0.0;
        if (len2 > 1e-12) {
            t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
            t = std::clamp(t, 0.0, 1.0);
        }
        const double cx = a.x + t * dx, cy = a.y + t * dy;
        const double d = std::hypot(p.x - cx, p.y - cy);
        if (d < best) best = d;
    }
    return best;
}

double NearestStationOnPolyline(const overlume::Vec3& p, const std::vector<overlume::Vec3>& poly,
                                const std::vector<double>& cum) {
    double best_d2 = std::numeric_limits<double>::infinity();
    double best_s = 0.0;
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
        const auto& a = poly[i];
        const auto& b = poly[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = 0.0;
        if (len2 > 1e-12) {
            t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
            t = std::clamp(t, 0.0, 1.0);
        }
        const double cx = a.x + t * dx, cy = a.y + t * dy;
        const double d2 = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
        if (d2 < best_d2) {
            best_d2 = d2;
            best_s = cum[i] + t * (cum[i + 1] - cum[i]);
        }
    }
    return best_s;
}

bool IsRoadEdge(
    uint32_t lane_id, const std::vector<overlume::Vec3>& pts,
    const std::unordered_map<uint32_t, const std::vector<overlume::Vec3>*>& opposite_by_lane) {
    if (pts.size() < 2) return false;
    const overlume::Vec3 samples[3] = {pts.front(), pts[pts.size() / 2], pts.back()};
    for (const auto& [other_lane, other_pts] : opposite_by_lane) {
        if (other_lane == lane_id || other_pts == nullptr) continue;
        for (const auto& s : samples) {
            if (PointToPolylineDist2D(s, *other_pts) < kRoadEdgeCoincidenceThresholdM) return false;
        }
    }
    return true;
}

constexpr double kJunctionCutBackoffM = 2.0;

constexpr double kJunctionGapMergeM = 6.6;

constexpr double kArcRadiusThresholdM = 20.0;
constexpr double kArcMinTotalTurnDeg = 15.0;
constexpr double kArcSearchMarginM = 6.0;

double CircumradiusXY(const overlume::Vec3& A, const overlume::Vec3& B, const overlume::Vec3& C) {
    const double abx = B.x - A.x, aby = B.y - A.y;
    const double acx = C.x - A.x, acy = C.y - A.y;
    const double cross2 = std::abs(abx * acy - aby * acx);
    if (cross2 < 1e-9) return std::numeric_limits<double>::infinity();
    const double a = std::hypot(C.x - B.x, C.y - B.y);
    const double b = std::hypot(C.x - A.x, C.y - A.y);
    const double c = std::hypot(B.x - A.x, B.y - A.y);
    return (a * b * c) / (2.0 * cross2);
}

double TurnAngleDeg(const overlume::Vec3& A, const overlume::Vec3& B, const overlume::Vec3& C) {
    const double d1x = B.x - A.x, d1y = B.y - A.y;
    const double d2x = C.x - B.x, d2y = C.y - B.y;
    return std::abs(std::atan2(d1x * d2y - d1y * d2x, d1x * d2x + d1y * d2y)) * 180.0 / M_PI;
}

bool FindArcSpanNear(const std::vector<overlume::Vec3>& pts, const std::vector<double>& cum,
                     double b, double& lo, double& hi) {
    if (pts.size() < 3) return false;
    const double lo_bound = b - kArcSearchMarginM;
    const double hi_bound = b + kArcSearchMarginM;

    bool best_found = false;
    double best_lo = 0.0, best_hi = 0.0;
    bool in_run = false;
    size_t run_start = 0;
    double run_turn_deg = 0.0;

    auto close_run = [&](size_t run_end_vertex) {
        if (in_run && run_turn_deg >= kArcMinTotalTurnDeg) {
            size_t ext_start = run_start;
            while (ext_start > 1 && CircumradiusXY(pts[ext_start - 2], pts[ext_start - 1],
                                                   pts[ext_start]) < kArcRadiusThresholdM) {
                --ext_start;
            }
            size_t ext_end = run_end_vertex;
            while (ext_end + 2 < pts.size() &&
                   CircumradiusXY(pts[ext_end], pts[ext_end + 1], pts[ext_end + 2]) <
                       kArcRadiusThresholdM) {
                ++ext_end;
            }
            const double r_lo = cum[ext_start], r_hi = cum[ext_end];
            if (!best_found || (r_hi - r_lo) > (best_hi - best_lo)) {
                best_lo = r_lo;
                best_hi = r_hi;
                best_found = true;
            }
        }
        in_run = false;
        run_turn_deg = 0.0;
    };

    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        const bool in_window = cum[i] >= lo_bound && cum[i] <= hi_bound;
        const bool is_arc_vertex =
            in_window && CircumradiusXY(pts[i - 1], pts[i], pts[i + 1]) < kArcRadiusThresholdM;
        if (is_arc_vertex) {
            if (!in_run) {
                in_run = true;
                run_start = i;
                run_turn_deg = 0.0;
            }
            run_turn_deg += TurnAngleDeg(pts[i - 1], pts[i], pts[i + 1]);
        } else {
            close_run(i - 1);
        }
    }
    close_run(pts.size() - 2);

    if (best_found) {
        lo = best_lo;
        hi = best_hi;
    }
    return best_found;
}

struct PendingRoadEdge {
    std::vector<overlume::Vec3> points;
    uint32_t lane_id;
    double last_update_sec;
    uint8_t is_polygon;
};

void SnapWindowsToArcs(const std::vector<overlume::Vec3>& pts, const std::vector<double>& cum,
                       std::vector<std::pair<double, double>>& windows) {
    for (auto& w : windows) {
        double lo = 0.0, hi = 0.0;
        if (FindArcSpanNear(pts, cum, w.first, lo, hi)) w.first = std::min(w.first, lo);
        if (FindArcSpanNear(pts, cum, w.second, lo, hi)) w.second = std::max(w.second, hi);
    }
}

constexpr double kSharedNodeEpsM = 0.10;

bool FindArcDepartureFromEnd(const std::vector<overlume::Vec3>& pts, bool from_back, size_t& idx) {
    const size_t n = pts.size();
    if (n < 3) return false;
    auto at = [&](size_t k) -> const overlume::Vec3& {
        return from_back ? pts[n - 1 - k] : pts[k];
    };

    bool in_run = false;
    size_t run_start_k = 0;
    double run_turn_deg = 0.0;
    for (size_t k = 1; k + 1 < n; ++k) {
        if (CircumradiusXY(at(k - 1), at(k), at(k + 1)) < kArcRadiusThresholdM) {
            if (!in_run) {
                in_run = true;
                run_start_k = k;
                run_turn_deg = 0.0;
            }
            run_turn_deg += TurnAngleDeg(at(k - 1), at(k), at(k + 1));
        } else if (in_run) {
            if (run_turn_deg >= kArcMinTotalTurnDeg) {
                idx = from_back ? (n - 1 - run_start_k) : run_start_k;
                return true;
            }
            in_run = false;
            run_turn_deg = 0.0;
        }
    }
    if (in_run && run_turn_deg >= kArcMinTotalTurnDeg) {
        idx = from_back ? (n - 1 - run_start_k) : run_start_k;
        return true;
    }
    return false;
}

bool FindNeighborArcDepartureStation(const std::vector<PendingRoadEdge>& pieces, size_t self,
                                     const std::vector<std::vector<double>>& cum,
                                     const overlume::Vec3& node, double& station_out) {
    const auto& pts_self = pieces[self].points;
    for (size_t j = 0; j < pieces.size(); ++j) {
        if (j == self || pieces[j].points.size() < 3) continue;
        const auto& pts_j = pieces[j].points;
        const double d_front = std::hypot(node.x - pts_j.front().x, node.y - pts_j.front().y);
        const double d_back = std::hypot(node.x - pts_j.back().x, node.y - pts_j.back().y);
        bool coincident_back = false;
        if (d_front < kSharedNodeEpsM) {
            coincident_back = false;
        } else if (d_back < kSharedNodeEpsM) {
            coincident_back = true;
        } else {
            continue;
        }
        size_t dep_idx = 0;
        if (!FindArcDepartureFromEnd(pts_j, coincident_back, dep_idx)) continue;
        station_out = NearestStationOnPolyline(pts_j[dep_idx], pts_self, cum[self]);
        return true;
    }
    return false;
}

void SnapWindowsToNeighborArcDepartures(const std::vector<PendingRoadEdge>& pieces, size_t self,
                                        const std::vector<std::vector<double>>& cum,
                                        std::vector<std::pair<double, double>>& windows) {
    if (windows.empty()) return;
    const auto& pts_self = pieces[self].points;
    if (pts_self.empty()) return;

    double station = 0.0;
    if (FindNeighborArcDepartureStation(pieces, self, cum, pts_self.front(), station)) {
        windows.front().first = std::min(windows.front().first, station);
    }
    if (FindNeighborArcDepartureStation(pieces, self, cum, pts_self.back(), station)) {
        windows.back().second = std::max(windows.back().second, station);
    }
}

bool FindLastArcRunEnd(const std::vector<overlume::Vec3>& pts, size_t& hi_idx) {
    if (pts.size() < 3) return false;
    bool found = false;
    bool in_run = false;
    double run_turn_deg = 0.0;

    auto close_run = [&](size_t run_end_vertex) {
        if (in_run && run_turn_deg >= kArcMinTotalTurnDeg) {
            hi_idx = run_end_vertex;
            found = true;
        }
        in_run = false;
        run_turn_deg = 0.0;
    };

    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        if (CircumradiusXY(pts[i - 1], pts[i], pts[i + 1]) < kArcRadiusThresholdM) {
            if (!in_run) {
                in_run = true;
                run_turn_deg = 0.0;
            }
            run_turn_deg += TurnAngleDeg(pts[i - 1], pts[i], pts[i + 1]);
        } else {
            close_run(i - 1);
        }
    }
    close_run(pts.size() - 2);
    return found;
}

void TrimRedundantArcTails(const std::vector<PendingRoadEdge>& pieces, size_t self,
                           const std::vector<std::vector<double>>& cum,
                           std::vector<std::pair<double, double>>& windows) {
    const auto& pts = pieces[self].points;
    size_t hi_idx = 0;
    if (!FindLastArcRunEnd(pts, hi_idx)) return;

    const overlume::Vec3& far = pts.back();
    const overlume::Vec3& penult = pts[pts.size() - 2];

    bool duplicate_found = false;
    for (size_t j = 0; j < pieces.size() && !duplicate_found; ++j) {
        if (j == self || pieces[j].lane_id == pieces[self].lane_id || pieces[j].points.size() < 2)
            continue;
        const auto& other = pieces[j].points;
        const double cond_a = std::min(std::hypot(far.x - other.front().x, far.y - other.front().y),
                                       std::hypot(far.x - other.back().x, far.y - other.back().y));
        if (cond_a >= kSharedNodeEpsM) continue;
        if (PointToPolylineDist2D(penult, other) < kRoadEdgeCoincidenceThresholdM) {
            duplicate_found = true;
        }
    }
    if (!duplicate_found) return;

    const double trim_start = cum[self][hi_idx];
    const double piece_end = cum[self].back();
    if (!windows.empty() && windows.back().second >= trim_start) {
        windows.back().second = piece_end;
    } else {
        windows.emplace_back(trim_start, piece_end);
    }
}

bool IsBoundaryKind(overlume::MapKind kind) {
    return kind == overlume::MapKind::LEFT_BOUNDARY || kind == overlume::MapKind::RIGHT_BOUNDARY;
}

bool PointInPolygonEvenOdd(const overlume::Vec3& p, const std::vector<overlume::Vec3>& poly) {
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const overlume::Vec3& a = poly[i];
        const overlume::Vec3& b = poly[j];
        if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool PointInAnyPolygon(const overlume::Vec3& p,
                       const std::vector<const std::vector<overlume::Vec3>*>& polys) {
    for (const auto* poly : polys) {
        if (poly != nullptr && poly->size() >= 3 && PointInPolygonEvenOdd(p, *poly)) return true;
    }
    return false;
}

std::vector<std::vector<overlume::Vec3>> ClipAgainstJunctions(
    const std::vector<overlume::Vec3>& pts,
    const std::vector<const std::vector<overlume::Vec3>*>& polys) {
    std::vector<std::vector<overlume::Vec3>> out;
    if (polys.empty() || pts.size() < 2) {
        out.push_back(pts);
        return out;
    }

    auto lerp = [](const overlume::Vec3& a, const overlume::Vec3& b, double t) {
        return overlume::Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    };

    std::vector<overlume::Vec3> current;
    bool a_in = PointInAnyPolygon(pts.front(), polys);
    if (!a_in) current.push_back(pts.front());
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const overlume::Vec3& a = pts[i];
        const overlume::Vec3& b = pts[i + 1];
        const bool b_in = PointInAnyPolygon(b, polys);
        if (a_in == b_in) {
            if (!b_in) current.push_back(b);
        } else {
            double lo = 0.0, hi = 1.0;
            for (int iter = 0; iter < 30; ++iter) {
                const double mid = 0.5 * (lo + hi);
                if (PointInAnyPolygon(lerp(a, b, mid), polys) == a_in)
                    lo = mid;
                else
                    hi = mid;
            }
            const overlume::Vec3 cross = lerp(a, b, 0.5 * (lo + hi));
            if (a_in && !b_in) {
                current.clear();
                current.push_back(cross);
                current.push_back(b);
            } else {
                current.push_back(cross);
                if (current.size() >= 2) out.push_back(current);
                current.clear();
            }
        }
        a_in = b_in;
    }
    if (current.size() >= 2) out.push_back(current);
    return out;
}

constexpr double kMinCrossingSinAngle = 0.25881904510252074;

bool SegSegIntersect2D(const overlume::Vec3& p1, const overlume::Vec3& p2, const overlume::Vec3& p3,
                       const overlume::Vec3& p4, double& t, double& u) {
    const double d1x = p2.x - p1.x, d1y = p2.y - p1.y;
    const double d2x = p4.x - p3.x, d2y = p4.y - p3.y;
    const double denom = d1x * d2y - d1y * d2x;
    const double len1 = std::hypot(d1x, d1y);
    const double len2 = std::hypot(d2x, d2y);
    if (len1 < 1e-9 || len2 < 1e-9) return false;
    if (std::abs(denom) < kMinCrossingSinAngle * len1 * len2) return false;
    const double dx = p3.x - p1.x, dy = p3.y - p1.y;
    t = (dx * d2y - dy * d2x) / denom;
    u = (dx * d1y - dy * d1x) / denom;
    return t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0;
}

void MergeWindows(std::vector<std::pair<double, double>>& windows) {
    if (windows.empty()) return;
    std::sort(windows.begin(), windows.end());
    std::vector<std::pair<double, double>> merged;
    merged.push_back(windows.front());
    for (size_t i = 1; i < windows.size(); ++i) {
        if (windows[i].first <= merged.back().second + kJunctionGapMergeM) {
            merged.back().second = std::max(merged.back().second, windows[i].second);
        } else {
            merged.push_back(windows[i]);
        }
    }
    windows = std::move(merged);
}

std::vector<std::vector<overlume::Vec3>> ApplyCutWindows(
    const std::vector<overlume::Vec3>& pts, const std::vector<double>& cum,
    const std::vector<std::pair<double, double>>& windows) {
    std::vector<std::vector<overlume::Vec3>> out;
    if (windows.empty()) {
        out.push_back(pts);
        return out;
    }
    const double total = cum.back();
    double pos = 0.0;
    auto append_kept_range = [&](double from, double to) {
        std::vector<overlume::Vec3> chain;
        chain.push_back(PointAtArcLength(pts, cum, from));
        for (size_t j = 0; j < pts.size(); ++j) {
            if (cum[j] > from && cum[j] < to) chain.push_back(pts[j]);
        }
        chain.push_back(PointAtArcLength(pts, cum, to));
        if (chain.size() >= 2) out.push_back(std::move(chain));
    };
    for (const auto& w : windows) {
        const double w_start = std::clamp(w.first, 0.0, total);
        const double w_end = std::clamp(w.second, 0.0, total);
        if (w_start > pos) append_kept_range(pos, w_start);
        pos = std::max(pos, w_end);
    }
    if (pos < total) append_kept_range(pos, total);
    return out;
}

}

HdMapAdapter::HdMapAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf) {}

void HdMapAdapter::ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec) {
    ++stats_.msgs;
    if (msg.markers.empty()) return;

    tf2::Transform xform;
    if (!tf_.lookup(msg.markers.front().header, xform)) {
        ++stats_.dropped_no_tf;
        return;
    }

    last_recv_sec_ = sim_time_sec;

    if (row_.max_rate_hz > 0.0 && last_rebuild_sec_ >= 0.0) {
        const double min_gap_sec = 1.0 / row_.max_rate_hz;
        if (sim_time_sec - last_rebuild_sec_ < min_gap_sec) return;
    }

    for (const auto& m : msg.markers) {
        if (m.action == kActionDeleteAll) {
            storage_.clear();
            continue;
        }
        if (m.action == kActionDelete) {
            storage_.erase(Key{m.ns, m.id});
            continue;
        }
        if (m.action != kActionAdd && m.action != kActionModify) continue;

        const NsRule* rule = match_rule(row_, m.ns);
        const NsRender verdict = rule != nullptr ? rule->render : row_.ns_default;
        if (verdict == NsRender::kDrop) {
            ++stats_.dropped_by_rule;
            continue;
        }

        if (m.type != kMarkerTypeLineStrip || m.points.size() < 2) {
            ++stats_.dropped_malformed;
            continue;
        }

        const bool identity_pose = MarkerPoseIsIdentity(m.pose);
        tf2::Transform marker_tf;
        if (!identity_pose) {
            if (MarkerPoseHasNan(m.pose)) {
                ++stats_.dropped_malformed;
                continue;
            }
            tf2::Quaternion q(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                              m.pose.orientation.w);
            if (q.length2() < 1e-12) q = tf2::Quaternion::getIdentity();
            marker_tf = tf2::Transform(
                q, tf2::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z));
        }

        std::vector<overlume::Vec3> pts;
        pts.reserve(m.points.size());
        bool ok = true;
        for (const auto& p : m.points) {
            const tf2::Vector3 local(p.x, p.y, p.z);
            const tf2::Vector3 posed = identity_pose ? local : marker_tf * local;
            const tf2::Vector3 tp = xform * posed;
            const overlume::Vec3 v{tp.x(), tp.y(), tf_.flatten_z() ? 0.0 : tp.z()};
            if (HasNan(v)) {
                ok = false;
                break;
            }
            pts.push_back(v);
        }
        if (!ok) {
            ++stats_.dropped_malformed;
            continue;
        }

        const uint8_t is_polygon = (verdict == NsRender::kPolygon) ? 1 : 0;

        if (is_polygon && pts.size() >= 2) {
            constexpr double kDedupEpsM = 1e-6;
            const auto& front = pts.front();
            const auto& back = pts.back();
            const double dx = back.x - front.x, dy = back.y - front.y, dz = back.z - front.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < kDedupEpsM) pts.pop_back();
        }

        const overlume::MapKind kind = rule != nullptr ? rule->kind : overlume::MapKind::OTHER;
        const uint32_t lane_id = KindCarriesLaneId(kind) ? static_cast<uint32_t>(m.id) : 0;

        StoredElement elem;
        elem.points = std::move(pts);
        elem.is_polygon = is_polygon;
        elem.kind = kind;
        elem.lane_id = lane_id;
        storage_[Key{m.ns, m.id}] = std::vector<StoredElement>{std::move(elem)};
    }

    last_rebuild_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void HdMapAdapter::fill(overlume::ros::SceneAssembly& out) const {
    road_surface_points_.clear();
    junction_cut_points_.clear();
    std::unordered_map<uint32_t, const std::vector<overlume::Vec3>*> left_by_lane, right_by_lane;
    std::vector<const std::vector<overlume::Vec3>*> junction_polys;

    for (const auto& [key, pieces] : storage_) {
        (void)key;
        for (const auto& elem : pieces) {
            if (elem.kind == overlume::MapKind::JUNCTION) junction_polys.push_back(&elem.points);
            if (elem.lane_id == 0) continue;
            if (elem.kind == overlume::MapKind::LEFT_BOUNDARY) {
                left_by_lane[elem.lane_id] = &elem.points;
            } else if (elem.kind == overlume::MapKind::RIGHT_BOUNDARY) {
                right_by_lane[elem.lane_id] = &elem.points;
            }
        }
    }

    std::vector<PendingRoadEdge> road_edge_pieces;

    for (const auto& [key, pieces] : storage_) {
        (void)key;
        for (const auto& elem : pieces) {
            overlume::MapElement e{};
            e.is_polygon = elem.is_polygon;
            e.kind = elem.kind;
            e.lane_id = elem.lane_id;
            e.last_update_sec = last_recv_sec_ + (row_.timeout_sec - kMapFadeWindowSec);
            if (e.kind == overlume::MapKind::LEFT_BOUNDARY &&
                IsRoadEdge(elem.lane_id, elem.points, right_by_lane)) {
                e.kind = overlume::MapKind::ROAD_EDGE;
            } else if (e.kind == overlume::MapKind::RIGHT_BOUNDARY &&
                       IsRoadEdge(elem.lane_id, elem.points, left_by_lane)) {
                e.kind = overlume::MapKind::ROAD_EDGE;
            }

            if (e.kind == overlume::MapKind::ROAD_EDGE) {
                for (auto& piece : ClipAgainstJunctions(elem.points, junction_polys)) {
                    road_edge_pieces.push_back(PendingRoadEdge{std::move(piece), elem.lane_id,
                                                               e.last_update_sec, e.is_polygon});
                }
                continue;
            }
            if (IsBoundaryKind(e.kind) && !row_.junction_interior_boundaries &&
                !junction_polys.empty()) {
                for (auto& piece : ClipAgainstJunctions(elem.points, junction_polys)) {
                    junction_cut_points_.push_back(std::move(piece));
                    overlume::MapElement be = e;
                    be.points = junction_cut_points_.back().data();
                    be.point_count = static_cast<uint32_t>(junction_cut_points_.back().size());
                    out.map_elements.push_back(be);
                }
                continue;
            }

            e.points = elem.points.data();
            e.point_count = static_cast<uint32_t>(elem.points.size());
            out.map_elements.push_back(e);
        }
    }

    std::vector<std::vector<double>> cum(road_edge_pieces.size());
    for (size_t i = 0; i < road_edge_pieces.size(); ++i) {
        cum[i] = CumulativeArcLength(road_edge_pieces[i].points);
    }
    std::vector<std::vector<std::pair<double, double>>> windows(road_edge_pieces.size());
    for (size_t i = 0; i < road_edge_pieces.size(); ++i) {
        const auto& pi = road_edge_pieces[i].points;
        for (size_t j = i + 1; j < road_edge_pieces.size(); ++j) {
            if (road_edge_pieces[i].lane_id == road_edge_pieces[j].lane_id) continue;
            const auto& pj = road_edge_pieces[j].points;
            for (size_t a = 0; a + 1 < pi.size(); ++a) {
                for (size_t b = 0; b + 1 < pj.size(); ++b) {
                    double t = 0.0, u = 0.0;
                    if (!SegSegIntersect2D(pi[a], pi[a + 1], pj[b], pj[b + 1], t, u)) continue;
                    const double s_i = cum[i][a] + t * (cum[i][a + 1] - cum[i][a]);
                    const double s_j = cum[j][b] + u * (cum[j][b + 1] - cum[j][b]);
                    windows[i].emplace_back(s_i - kJunctionCutBackoffM, s_i + kJunctionCutBackoffM);
                    windows[j].emplace_back(s_j - kJunctionCutBackoffM, s_j + kJunctionCutBackoffM);
                }
            }
        }
    }
    for (size_t i = 0; i < road_edge_pieces.size(); ++i) {
        MergeWindows(windows[i]);
        SnapWindowsToArcs(road_edge_pieces[i].points, cum[i], windows[i]);
        MergeWindows(windows[i]);
        SnapWindowsToNeighborArcDepartures(road_edge_pieces, i, cum, windows[i]);
        TrimRedundantArcTails(road_edge_pieces, i, cum, windows[i]);
        for (auto& piece : ApplyCutWindows(road_edge_pieces[i].points, cum[i], windows[i])) {
            junction_cut_points_.push_back(std::move(piece));
            overlume::MapElement e{};
            e.points = junction_cut_points_.back().data();
            e.point_count = static_cast<uint32_t>(junction_cut_points_.back().size());
            e.is_polygon = road_edge_pieces[i].is_polygon;
            e.kind = overlume::MapKind::ROAD_EDGE;
            e.lane_id = road_edge_pieces[i].lane_id;
            e.last_update_sec = road_edge_pieces[i].last_update_sec;
            out.map_elements.push_back(e);
        }
    }

    for (const auto& [lane_id, left_pts] : left_by_lane) {
        const auto it = right_by_lane.find(lane_id);
        if (it == right_by_lane.end()) continue;

        std::vector<overlume::Vec3> left_r = ResampleByArcLength(*left_pts, kRoadFillSamples);
        std::vector<overlume::Vec3> right_r = ResampleByArcLength(*it->second, kRoadFillSamples);
        if (left_r.empty() || right_r.empty()) continue;

        RoadFillCacheEntry& entry = road_fill_cache_[lane_id];
        entry.left = std::move(left_r);
        entry.right = std::move(right_r);
        entry.cached_recv_sec = last_recv_sec_;
    }

    for (auto cache_it = road_fill_cache_.begin(); cache_it != road_fill_cache_.end();) {
        const double age_sec = last_recv_sec_ - cache_it->second.cached_recv_sec;
        if (age_sec > row_.timeout_sec || age_sec < 0.0) {
            cache_it = road_fill_cache_.erase(cache_it);
            continue;
        }

        road_surface_points_.emplace_back();
        std::vector<overlume::Vec3>& combined = road_surface_points_.back();
        combined.reserve(2 * kRoadFillSamples);
        combined.insert(combined.end(), cache_it->second.left.begin(), cache_it->second.left.end());
        combined.insert(combined.end(), cache_it->second.right.begin(),
                        cache_it->second.right.end());

        overlume::MapElement e{};
        e.points = combined.data();
        e.point_count = static_cast<uint32_t>(combined.size());
        e.is_polygon = 0;
        e.kind = overlume::MapKind::ROAD_SURFACE;
        e.lane_id = cache_it->first;
        e.last_update_sec = last_recv_sec_ + (row_.timeout_sec - kMapFadeWindowSec);
        out.map_elements.push_back(e);
        ++cache_it;
    }
}

}
