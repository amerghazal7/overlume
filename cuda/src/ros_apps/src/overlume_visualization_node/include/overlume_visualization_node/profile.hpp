#pragma once
/** @file profile.hpp
 *  @brief Profile YAML loader (Epic 2 Task 1 / VM-020).
 *
 *  Deliberately rclcpp-free -- test_profile.cpp needs no ROS graph, and
 *  neither does anything downstream that only wants to know "what does this
 *  row say". `visualization_node.cpp` is the only place that turns a
 *  `SubSpec` into an actual `create_subscription` call.
 *
 *  See docs/superpowers/plans/2026-08-18-visual-mode-epic2.md, "Task 1
 *  (VM-020)" and the header sections it points back to ("Node: adapter
 *  shape", "Staleness") for the full rationale behind every field below.
 */

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mpviz_node
{

// The ONE namespace rule, used by every marker adapter: longest matching
// prefix wins; no match -> row.ns_default.
enum class NsRender : uint8_t { kDrop = 0, kPolyline = 1, kPolygon = 2 };

struct NsRule
{
    std::string prefix;
    NsRender render{NsRender::kDrop};
};

struct ProfileRow
{
    std::string topic;        // required (EXCEPT adapter: tf_axes)
    std::string type;         // required (EXCEPT adapter: tf_axes)
    std::string adapter;      // required: dynamic_objects|path|hd_map|ogm|
                               //           collision|generic|tf_axes
    std::string role;         // required; closed set per adapter (see profile.cpp)
    std::string update_topic; // optional, `ogm` rows ONLY

    double timeout_sec{2.0};  // optional, >= 1.0 (see epic2 plan, "Staleness")
    double max_rate_hz{0.0};  // optional, 0 = no limit

    std::vector<NsRule> namespaces;              // optional; empty = everything, as ns_default
    NsRender ns_default{NsRender::kPolyline};     // optional

    bool transient_local{false}; // optional; latched publishers (/sim/hd_map/markers)
    bool best_effort{false};     // optional; BEST_EFFORT publishers (/sim/ground_truth/boxes)
};

struct Profile
{
    std::string name;
    std::vector<ProfileRow> rows;
};

// What the node must subscribe to for one row, as a pure function of the
// row -- testable with no ROS graph. `tf_axes` -> {} (nothing publishes TF
// as markers; it's a producer). `ogm` -> 2 entries (topic +
// update_topic, the second typed map_msgs/msg/OccupancyGridUpdate).
// Everything else -> 1.
struct SubSpec
{
    std::string topic;
    std::string type;   // determines which ingest() overload the node binds
    bool best_effort;    // -> rclcpp::QoS(...).best_effort()
    bool transient_local; // -> .transient_local()
};

// Parses a profile from a file / from a literal. Returns std::nullopt and
// appends every validation problem found (not just the first) to `errors`
// on failure -- each formatted "<file>: row <i> (topic <t>): <what and
// why>". Unknown extra keys are a WARNING appended to `errors` but do NOT
// fail the load (profiles are hand-edited by the autonomy team; a typo'd
// optional key must not take the node down).
std::optional<Profile> load_profile(const std::string& path, std::vector<std::string>& errors);
std::optional<Profile> load_profile_string(std::string_view yaml, std::vector<std::string>& errors);

// First row in `profile` whose `topic` matches, or nullptr.
const ProfileRow* find_row(const Profile& profile, std::string_view topic);

// Longest matching prefix in `row.namespaces` wins; no match -> row.ns_default.
NsRender classify(const ProfileRow& row, std::string_view ns);

// subscriptions_for() is the pure function the node walks to build every
// create_subscription() call -- see profile.cpp for the ogm/tf_axes special
// cases.
std::vector<SubSpec> subscriptions_for(const ProfileRow& row);

}  // namespace mpviz_node
