// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/quality_governor.hpp"

#include <algorithm>
#include <cmath>

namespace overlume::ros {

namespace {

double Percentile(std::vector<double> sorted_ms, double p) {
    if (sorted_ms.empty()) return 0.0;
    std::sort(sorted_ms.begin(), sorted_ms.end());
    size_t idx = static_cast<size_t>(std::ceil(p * static_cast<double>(sorted_ms.size())));
    if (idx > 0) --idx;
    if (idx >= sorted_ms.size()) idx = sorted_ms.size() - 1;
    return sorted_ms[idx];
}

}

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

    consecutive_good_windows_ = 0;
    return QualityTransition::NONE;
}

}
