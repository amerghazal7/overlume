#pragma once
/** @file dynamic_objects.hpp
 *  @brief DynamicObjectsAdapter (Epic 2 Task 3 / VM-021): MarkerArray ->
 *  mpviz::TrackedObject, plus the config-driven class-inference table.
 *
 *  Ctor shape is the (row, tf) every Epic 2 adapter carries, PLUS this
 *  adapter's one node-side extra (epic2 plan, "Node: adapter shape"):
 *  DynamicObjectsAdapter(row, tf, const ClassInferenceTable&).
 *
 *  Five namespaces arrive on the wire per track id: dynamic_objects_bbox,
 *  _text, _arrow, _hd_map_path, _hd_map_path_dots. The first FOUR fuse by
 *  marker id into one TrackedObject; the fifth is dropped by the row's
 *  `namespaces:` rules via classify() (longest-prefix-wins --
 *  "dynamic_objects_hd_map_path" is a prefix of "..._dots") and counted in
 *  dropped_by_rule, never dropped_malformed and never warned (epic2 plan,
 *  Task 3, "Five namespaces on the wire, not four"). classify() is called
 *  for that drop decision ONLY -- once a namespace is not kDrop, routing
 *  to bbox/text/arrow/path is by the literal namespace string, not by
 *  classify()'s render verdict.
 *
 *  Field sources (epic2 plan, Task 3 Step 1a -- restated here because
 *  getting either backwards is the documented failure mode):
 *    - heading_rad: yaw of the BBOX marker's pose.orientation. NEVER the
 *      arrow (the arrow's pose merely orients its own points -- it is NOT
 *      the object's heading; an arrow-less or zero-length-arrow frame
 *      would otherwise corrupt heading).
 *    - velocity: arrow points[0]->points[1] ONLY. Both points are first
 *      composed with the ARROW marker's OWN pose (Marker.msg semantics:
 *      points[] are relative to marker.pose) before the frame transform;
 *      the difference cancels both the arrow pose's and the frame
 *      transform's translation, so a
 *      pure-translation arrow pose leaves velocity unchanged and only a
 *      rotation actually rotates it. |p1-p0| < kMinArrowM -> {0,0,0}, never
 *      a NaN normalize. Missing arrow -> object still renders, zero
 *      velocity, correct bbox heading.
 *    - predicted_path: hd_map_path is Marker::type LINE_LIST -- points[]
 *      is independent segment PAIRS, pairwise-duplicated on the wire for a
 *      contiguous path, and (same rviz-parity fix) relative to the PATH
 *      marker's own pose, composed BEFORE line_list_to_polyline() collapses
 *      them. line_list_to_polyline() collapses n points to the n/2+1
 *      unique polyline vertices TrackedObject::predicted_path expects;
 *      genuinely disjoint pairs return false (that one path is dropped,
 *      ++dropped_malformed, the object still renders pathless). This
 *      conversion is adapter-side on purpose -- extrude_polyline (Task 4)
 *      consumes polylines and the renderer has never seen a ROS message
 *      type; teaching polyline.cpp about Marker::type would leak a ROS
 *      concept into the library.
 *    - per-vertex colors[] on path markers are discarded (styling is
 *      theme-driven, spec §7); TrackedObject has no color field for this
 *      category to leak into.
 *
 *  Malformed handling (spec §9, "drop the one primitive, count it, never
 *  propagate"): a NaN bbox pose or a zero-extent bbox drops that BBOX
 *  marker immediately (++dropped_malformed at ingest time). A text marker
 *  with an empty string is simply not stored as a label -- it is NOT
 *  double-counted against the bbox-with-no-matching-text case below;
 *  those are the same underlying defect (an object whose bbox has no
 *  usable label) and get exactly one bump. At the end of each ingest()
 *  call, every track id TOUCHED BY THIS MESSAGE that ended up with a bbox
 *  but no usable text is counted (++dropped_malformed) and evicted --
 *  this is checked once per ingest() call, not in fill() (fill() runs
 *  every render tick; bumping there would recount the same defect dozens
 *  of times a second). ponytail: this assumes bbox+text co-arrive in the
 *  same MarkerArray, true of every recorded frame; a publisher that split
 *  them across messages would need the check widened to "still missing
 *  after N ticks" -- add that if a real publisher ever does this.
 *
 *  Typed-topic seam (spec §5): a future typed perception topic would add
 *  a new `ingest(const SomeTypedMsg&, double)` overload here -- the
 *  renderer only ever sees TrackedObject[], so nothing downstream would
 *  change. No `ObjectSource` interface with one implementation (YAGNI);
 *  add one only when a second source actually exists.
 *
 *  Frames/staleness follow every other Epic 2 adapter exactly (see
 *  hd_map.hpp's header comment and the epic2 plan's "Frames"/"Staleness"
 *  sections) -- one FrameTransformer lookup per ingest() call, node-side
 *  timeout_sec enforcement, last_update_sec stamped from the node's own
 *  sim clock.
 */

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "micropilot_visualization_node/adapter_stats.hpp"
#include "micropilot_visualization_node/frame_transform.hpp"
#include "micropilot_visualization_node/profile.hpp"
#include "micropilot_visualization_node/scene_assembly.hpp"
#include "visual_renderer/scene.h"

namespace mpviz_node
{

// One footprint band (class_inference.yaml's `footprint:` list) -- first
// band whose length/width/height all satisfy wins; an absent bound is the
// widest-open value (max_*_m defaults to "no limit", min_height_m to "no
// floor"), matching a band that only names the fields it actually needs
// to discriminate (e.g. the CAR/TRUCK_VAN/BUS bands only bound length).
struct FootprintBand
{
    double max_length_m{1e300};
    double max_width_m{1e300};
    double min_height_m{0.0};
    mpviz::ObjectClass cls{mpviz::ObjectClass::UNKNOWN};
};

struct ClassInferenceTable
{
    std::unordered_map<std::string, mpviz::ObjectClass> prefix;
    std::vector<FootprintBand> footprint;
    mpviz::ObjectClass default_cls{mpviz::ObjectClass::UNKNOWN};
};

// Parses config/class_inference.yaml. Mirrors load_profile()'s shape
// (path + errors out-param) but is deliberately smaller -- this table has
// no cross-row validation and no adapter-specific role/type sets to check.
std::optional<ClassInferenceTable> load_class_inference(const std::string& path,
                                                         std::vector<std::string>& errors);

// Prefix wins over footprint; scale is ALWAYS the caller's bbox dims
// regardless of which class this returns (epic2 plan, Task 3 Step 2 --
// "that is the whole reason this table is allowed to be a heuristic").
// `label` may be nullptr (no text marker yet); dims are length(x)/
// width(y)/height(z), meters. Free function, no ROS types -- testable
// standalone.
mpviz::ObjectClass infer(const ClassInferenceTable& cfg, const char* label, mpviz::Vec3 dims);

// Collapses a LINE_LIST's pairwise-duplicated segment points (epic2 plan,
// Task 3 Step 1a) into the unique polyline vertices TrackedObject::
// predicted_path expects: n points -> n/2 + 1 vertices. Returns false
// (out left untouched) if `in.size()` is odd/less than 2, contains a NaN,
// or a consecutive pair fails to chain (p[2i+1] != p[2i+2] past 1e-6 m) --
// a genuinely disjoint LINE_LIST is not a polyline and must not be
// pretended into one.
bool line_list_to_polyline(const std::vector<geometry_msgs::msg::Point>& in,
                           std::vector<mpviz::Vec3>& out);

class DynamicObjectsAdapter
{
public:
    DynamicObjectsAdapter(const ProfileRow& row,
                          const micropilot::visualization_app::FrameTransformer& tf,
                          const ClassInferenceTable& classes);

    // ROS callback thread. See this file's header comment for the full
    // fusion/malformed/drop rules.
    void ingest(const visualization_msgs::msg::MarkerArray& msg, double sim_time_sec);

    // Timer thread, before set_scene(). APPENDS into out.objects (see
    // scene_assembly.hpp's own comment on why: multiple dynamic_objects
    // rows could ship, and the last one to run must not erase the
    // others). Returned TrackedObject::label/predicted_path alias this
    // object's OWN storage_ and stay valid exactly as long as this
    // instance is not destroyed and does not run another ingest().
    void fill(micropilot::visualization_app::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    // Node-side staleness bookkeeping (epic2 plan, "Staleness" -- the node
    // owns timeout_sec, not the adapter): the node calls this once per
    // timer tick that it withholds this row's objects for being past
    // row.timeout_sec, instead of calling fill(). ponytail: counts
    // withheld TICKS, not withheld objects -- the simplest reading of a
    // counter no test pins to an exact granularity; widen to "objects
    // withheld" if VM-034 ever needs that resolution.
    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    struct Track
    {
        bool has_bbox{false};
        bool has_text{false};
        mpviz::Vec3 position{0.0, 0.0, 0.0};
        double heading_rad{0.0};
        mpviz::Vec3 dimensions{0.0, 0.0, 0.0};
        mpviz::Vec3 velocity{0.0, 0.0, 0.0};
        std::string label;
        std::vector<mpviz::Vec3> predicted_path;
        double last_update_sec{0.0};
    };

    ProfileRow row_;
    const micropilot::visualization_app::FrameTransformer& tf_;
    const ClassInferenceTable& classes_;
    std::unordered_map<int32_t, Track> tracks_;
    AdapterStats stats_;
};

}  // namespace mpviz_node
