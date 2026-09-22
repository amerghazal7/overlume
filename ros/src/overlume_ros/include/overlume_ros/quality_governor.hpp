// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

namespace overlume::ros {

enum class QualityTransition : uint8_t {
    NONE = 0,
    DROPPED = 1,
    RECOVERED = 2,
};

struct QualityGovernorParams {
    uint32_t window_size{30};
    double drop_threshold_ms{28.0};
    double recover_threshold_ms{18.0};
    uint32_t recover_windows_required{3};
    uint32_t min_dwell_windows{3};
};

class QualityGovernor {
public:
    QualityGovernor(QualityGovernorParams params, uint32_t initial_preset);

    QualityTransition record_render_ms(double render_ms);

    uint32_t current_preset() const { return preset_; }

private:
    QualityGovernorParams params_;
    uint32_t preset_;
    std::vector<double> window_;
    uint32_t windows_since_transition_;
    uint32_t consecutive_good_windows_{0};
};

}
