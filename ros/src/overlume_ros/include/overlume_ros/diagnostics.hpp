#pragma once
/** @file diagnostics.hpp
 *  @brief Epic 3 Task 2 (VM-034): builds one `diagnostic_msgs/msg/
 *  DiagnosticArray` per tick from every profile row's `AdapterStats` plus
 *  the node's own `render_ms` (overlume_node.cpp's timer_callback()).
 *
 *  Message TYPE decided in the plan (Task 2 Step 0): `diagnostic_msgs::msg::
 *  DiagnosticArray`, not a bespoke struct -- it is the ROS-idiomatic type for
 *  exactly this shape (name + level + key/value list), already ships with
 *  this distro (no new interface-generation package needed -- this node
 *  still has zero .msg/.srv files of its own), and costs one `<depend>`
 *  line. One `DiagnosticStatus` per row (`name` = topic, `values` = every
 *  `AdapterStats` field verbatim -- `msgs` alongside the drop counters --
 *  `dropped_by_rule` kept separate from `dropped_malformed` per that
 *  struct's own header comment), plus one node-level status named
 *  "render_ms" carrying that single value.
 *
 *  ABSENT vs STALE: a row whose `stats.msgs == 0` has never published --
 *  mirrors overlume_node.cpp's own `msgs == 0` guard (its
 *  dynamic_objects/path/ogm/collision/generic_marker loops, "a topic that
 *  has NEVER published is absent, not stale") -- and is reported as level
 *  OK / message "no data yet" with no `last_msg_age_sec` value (the
 *  caller-computed age would just be sim_clock_sec_, a bogus number for
 *  data that was never received), never as a stale WARN.
 *
 *  Deliberately header-only-dependency: `RowStats` needs nothing but
 *  `AdapterStats` (already POD, already node-internal) -- no ROS clock, no
 *  profile row type -- so this file/its .cpp stay a standalone, easily
 *  unit-tested translation unit, same "small shared header" shape as
 *  adapter_stats.hpp itself.
 */

#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "overlume_ros/adapter_stats.hpp"

namespace overlume_node
{

// One profile row's diagnostics input. `last_msg_age_sec`/`timeout_sec` are
// caller-computed (overlume_node.cpp already has sim_clock_sec_ and the
// row's own timeout_sec at the call site) rather than re-derived here --
// this header takes no ROS clock and no profile-row type, on purpose.
// Default-initialized trailing fields let a caller (or a test) brace-init
// with just {topic, stats} when age/timeout aren't the point of that case.
struct RowStats
{
    std::string topic;             // named exactly as it appears in the profile row
    AdapterStats stats;
    double last_msg_age_sec{0.0};  // sim_clock_sec_ - stats.last_msg_sec
    double timeout_sec{0.0};       // the row's own timeout_sec; 0.0 = "never WARN on age"
};

// One DiagnosticStatus per row (in `rows`' order) plus one trailing
// node-level status named "render_ms". `render_ms` is published verbatim
// even when 0.0 (modes 1-2, or mode 3's first tick before any render) --
// the caller decides whether 0.0 means "not measured this tick", this
// function does not guess.
diagnostic_msgs::msg::DiagnosticArray BuildDiagnostics(const std::vector<RowStats>& rows,
                                                        double render_ms);

}  // namespace overlume_node
