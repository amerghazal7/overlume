// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

/** @file quality_governor.cpp
 *  @brief See quality_governor.hpp. Pure data transform over a render_ms
 *  stream -- no ROS node, no clock, no renderer -- unit-testable with
 *  nothing but doubles a test hands in directly (the AC's own test seam).
 */
#include "overlume_ros/quality_governor.hpp"

#include <algorithm>
#include <cmath>

namespace overlume_node {

namespace {

// Nearest-rank percentile, same shape tools/viz_benchmark.cpp already uses
// for its own render_ms p50/p99 reporting -- one definition of "p95" for
// this codebase's perf tooling, not a second one invented here.
double Percentile(std::vector<double> sorted_ms, double p) {
    if (sorted_ms.empty()) return 0.0;
    std::sort(sorted_ms.begin(), sorted_ms.end());
    size_t idx = static_cast<size_t>(std::ceil(p * static_cast<double>(sorted_ms.size())));
    if (idx > 0) --idx;
    if (idx >= sorted_ms.size()) idx = sorted_ms.size() - 1;
    return sorted_ms[idx];
}

}  // namespace

QualityGovernor::QualityGovernor(QualityGovernorParams params, uint32_t initial_preset)
    : params_(params),
      preset_(initial_preset > 2 ? 2 : initial_preset),
      windows_since_transition_(params_.min_dwell_windows) {
    window_.reserve(params_.window_size);
}

QualityTransition QualityGovernor::record_render_ms(double render_ms) {
    window_.push_back(render_ms);
    if (window_.size() < params_.window_size) return QualityTransition::NONE;

    const double p95 = Percentile(window_, 0.95);
    window_.clear();
    ++windows_since_transition_;
    const bool dwell_elapsed = windows_since_transition_ >= params_.min_dwell_windows;

    if (p95 > params_.drop_threshold_ms) {
        // Overloaded -- resets any accumulating recovery streak regardless
        // of whether a drop actually fires this window (still overloaded is
        // still not headroom).
        consecutive_good_windows_ = 0;
        if (dwell_elapsed && preset_ > 0) {
            --preset_;
            windows_since_transition_ = 0;
            return QualityTransition::DROPPED;
        }
        return QualityTransition::NONE;
    }

    if (p95 < params_.recover_threshold_ms) {
        ++consecutive_good_windows_;
        if (dwell_elapsed && preset_ < 2 &&
            consecutive_good_windows_ >= params_.recover_windows_required) {
            ++preset_;
            windows_since_transition_ = 0;
            consecutive_good_windows_ = 0;
            return QualityTransition::RECOVERED;
        }
        return QualityTransition::NONE;
    }

    // Inside the hysteresis gap (recover_threshold_ms <= p95 <=
    // drop_threshold_ms): neither overloaded nor headroom. Breaks a
    // recovery streak -- "sustained" headroom means every window in the
    // streak, not most of them.
    consecutive_good_windows_ = 0;
    return QualityTransition::NONE;
}

}  // namespace overlume_node
