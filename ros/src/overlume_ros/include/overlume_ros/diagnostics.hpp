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

// Empty string == healthy. Otherwise names the param the operator must set.
std::string HybridStarvedReason(bool cloud_consumed, bool hybrid_enabled, bool has_cloud_sub);

// name "hybrid"; ERROR + message=reason when non-empty, OK "ok" otherwise.
diagnostic_msgs::msg::DiagnosticStatus BuildHybridStatus(const std::string& starved_reason);

}
