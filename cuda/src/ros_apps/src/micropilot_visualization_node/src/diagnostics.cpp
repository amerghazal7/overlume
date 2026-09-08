/** @file diagnostics.cpp
 *  @brief See diagnostics.hpp. Pure data transform -- no ROS node, no clock,
 *  no publisher -- so it is unit-testable with nothing but the AdapterStats
 *  values a test hand-builds.
 */
#include "micropilot_visualization_node/diagnostics.hpp"

#include <sstream>

namespace mpviz_node
{

namespace
{

std::string to_str(double v)
{
    std::ostringstream oss;
    oss << v;
    return oss.str();
}

diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value)
{
    diagnostic_msgs::msg::KeyValue out;
    out.key = key;
    out.value = value;
    return out;
}

}  // namespace

diagnostic_msgs::msg::DiagnosticArray BuildDiagnostics(const std::vector<RowStats>& rows,
                                                        double render_ms)
{
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.status.reserve(rows.size() + 1);

    for (const auto& row : rows)
    {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = row.topic;
        // ABSENT (never published) is not the same as STALE (published, then
        // went quiet past timeout_sec) -- mirrors visualization_node.cpp's
        // msgs==0 guard. For an absent row, last_msg_age_sec is really "how
        // long sim_clock_sec_ has run," not a real age; reporting it as
        // stale would be a bogus WARN on data that never existed.
        const bool absent = row.stats.msgs == 0;
        // WARN once this row's OWN timeout_sec is exceeded -- past it,
        // visualization_node.cpp's per-category gate stops calling fill(), so
        // this WARN signals exactly that, not a fixed threshold. timeout_sec
        // ==0.0 (unset) never WARNs.
        const bool stale =
            !absent && row.timeout_sec > 0.0 && row.last_msg_age_sec > row.timeout_sec;
        status.level = stale ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = absent ? "no data yet" : (stale ? "stale" : "ok");
        if (!absent)
        {
            status.values.push_back(kv("last_msg_age_sec", to_str(row.last_msg_age_sec)));
        }
        status.values.push_back(kv("msgs", std::to_string(row.stats.msgs)));
        status.values.push_back(kv("dropped_malformed", std::to_string(row.stats.dropped_malformed)));
        status.values.push_back(kv("dropped_stale", std::to_string(row.stats.dropped_stale)));
        status.values.push_back(kv("dropped_no_tf", std::to_string(row.stats.dropped_no_tf)));
        // Separate from dropped_malformed -- adapter_stats.hpp's own header
        // comment explains why (a rule-drop is intentional, not malformed).
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

}  // namespace mpviz_node
