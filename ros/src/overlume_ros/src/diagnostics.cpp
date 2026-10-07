// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/diagnostics.hpp"

#include <sstream>

namespace overlume::ros {

namespace {

std::string to_str(double v) {
    std::ostringstream oss;
    oss << v;
    return oss.str();
}

diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue out;
    out.key = key;
    out.value = value;
    return out;
}

}

diagnostic_msgs::msg::DiagnosticArray BuildDiagnostics(const std::vector<RowStats>& rows,
                                                       double render_ms) {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.status.reserve(rows.size() + 1);

    for (const auto& row : rows) {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = row.topic;
        const bool absent = row.stats.msgs == 0;
        const bool stale =
            !absent && row.timeout_sec > 0.0 && row.last_msg_age_sec > row.timeout_sec;
        status.level = stale ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                             : diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = absent ? "no data yet" : (stale ? "stale" : "ok");
        if (!absent) {
            status.values.push_back(kv("last_msg_age_sec", to_str(row.last_msg_age_sec)));
        }
        status.values.push_back(kv("msgs", std::to_string(row.stats.msgs)));
        status.values.push_back(
            kv("dropped_malformed", std::to_string(row.stats.dropped_malformed)));
        status.values.push_back(kv("dropped_stale", std::to_string(row.stats.dropped_stale)));
        status.values.push_back(kv("dropped_no_tf", std::to_string(row.stats.dropped_no_tf)));
        status.values.push_back(kv("dropped_by_rule", std::to_string(row.stats.dropped_by_rule)));
        msg.status.push_back(std::move(status));
    }

    diagnostic_msgs::msg::DiagnosticStatus render;
    render.name = "render_ms";
    render.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    render.message = "ok";
    render.values.push_back(kv("render_ms", to_str(render_ms)));
    msg.status.push_back(std::move(render));

    return msg;
}

std::string HybridStarvedReason(bool cloud_consumed, bool hybrid_enabled, bool has_cloud_sub) {
    if (!cloud_consumed) return "";
    if (!hybrid_enabled) return "hybrid_enabled is false";
    if (!has_cloud_sub) return "pointcloud_topic is empty -- set pointcloud_topic and reconfigure";
    return "";
}

diagnostic_msgs::msg::DiagnosticStatus BuildHybridStatus(const std::string& starved_reason) {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "hybrid";
    st.level = starved_reason.empty() ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                      : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = starved_reason.empty() ? "ok" : starved_reason;
    return st;
}

}
