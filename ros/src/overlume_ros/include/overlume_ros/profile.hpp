#pragma once
/** @file profile.hpp
 *  @brief Profile YAML loader (Epic 2 Task 1 / VM-020).
 *
 *  Deliberately rclcpp-free -- test_profile.cpp needs no ROS graph, and
 *  neither does anything downstream that only wants to know "what does this
 *  row say". `overlume_node.cpp` is the only place that turns a
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

#include "overlume/scene.h"

namespace overlume_node
{

// The ONE namespace rule, used by every marker adapter: longest matching
// prefix wins; no match -> row.ns_default.
enum class NsRender : uint8_t { kDrop = 0, kPolyline = 1, kPolygon = 2 };

struct NsRule
{
    std::string prefix;
    NsRender render{NsRender::kDrop};
    // Epic 3 Task 1 (VM-036, ADR-0004): which MapElement::kind a marker
    // matching this rule gets. Optional, defaults to OTHER; the validator
    // (profile.cpp's ValidateRow) restricts each non-OTHER value to the
    // render verdict it's actually legal on (kind: crosswalk only on
    // render: polygon; every lane-geometry kind only on render: polyline).
    // kind: road_surface is never a legal YAML value -- ROAD_SURFACE is
    // synthesized by HdMapAdapter itself (paired boundary rails), never
    // present on the wire.
    overlume::MapKind kind{overlume::MapKind::OTHER};
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

    // Junction-cleanup knob: adapter: hd_map rows only (ParseRow rejects it
    // elsewhere). true (default, matches every shipped profile): LEFT_/
    // RIGHT_BOUNDARY elements render through a JUNCTION polygon untouched --
    // interior lane-separator guidance stays visible by default. false:
    // HdMapAdapter::fill() drops a boundary element's segments that fall
    // inside a JUNCTION polygon, the same clip ROAD_EDGE always gets.
    // KNOWN LIMITATION, stated honestly: this flag is INERT when the row's
    // own message carries no JUNCTION-kind elements (the urban local/global
    // feed never does -- see hd_map.cpp) -- there is no polygon to clip
    // against, so a boundary crossing another lane's connector inside an
    // unmapped junction keeps rendering uncut even with this set to false.
    // Never applies the mutual-crossing cut (ROAD_EDGE-only) -- interior
    // separators legitimately cross connector geometry.
    bool junction_interior_boundaries{true};

    // adapter: point_cloud only (Epic 3 Task 6 / VM-035; ParseRow rejects
    // these on any other adapter, same restriction shape as
    // junction_interior_boundaries above). color_mode picks the per-point
    // bake tier (auto|rgb|intensity|height|flat -- see PointCloudAdapter's
    // own header comment for the auto-tier fallback order); max_points is
    // a decimation CEILING applied after stride (0 = no cap, same "0 = no
    // limit" convention as max_rate_hz); stride keeps every Nth point
    // (default 1 = no decimation; 0 is invalid, ValidateRow rejects it).
    std::string color_mode{"auto"};
    uint32_t max_points{0};
    uint32_t stride{1};
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

// The matched rule itself (classify()'s same longest-prefix-wins search),
// or nullptr when nothing matches -- callers needing more than the render
// verdict (HdMapAdapter wants `kind`) use this instead of classify(). No
// match means "row.ns_default applies", and ns_default carries no kind
// (kind is only ever set on an explicit namespace rule; no match -> OTHER).
const NsRule* match_rule(const ProfileRow& row, std::string_view ns);

// subscriptions_for() is the pure function the node walks to build every
// create_subscription() call -- see profile.cpp for the ogm/tf_axes special
// cases.
std::vector<SubSpec> subscriptions_for(const ProfileRow& row);

}  // namespace overlume_node
