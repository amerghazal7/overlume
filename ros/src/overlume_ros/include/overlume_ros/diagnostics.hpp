// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "overlume_ros/adapter_stats.hpp"

namespace overlume::ros {

struct RowStats {
    std::string topic;
    AdapterStats stats;
    double last_msg_age_sec{0.0};
    double timeout_sec{0.0};
};

diagnostic_msgs::msg::DiagnosticArray BuildDiagnostics(const std::vector<RowStats>& rows,
                                                       double render_ms);

}
