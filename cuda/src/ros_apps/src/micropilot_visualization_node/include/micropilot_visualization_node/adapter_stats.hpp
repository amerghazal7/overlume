#pragma once
/** @file adapter_stats.hpp
 *  @brief Diagnostics counters shared by every Epic 2 marker/path/grid
 *  adapter (epic2 plan, "Node: adapter shape" -- "Diagnostics counters").
 *
 *  Promoted out of hd_map.hpp (that file's own comment named this exact
 *  moment: "Promote to its own shared header the day a second adapter
 *  (Task 3's DynamicObjectsAdapter) needs the identical shape rather than
 *  copy it a second time" -- this is that day).
 *
 *  `dropped_by_rule` is a SEPARATE counter from `dropped_malformed` on
 *  purpose: every marker a profile `namespaces:` rule says `render: drop`
 *  is intentionally discarded, not malformed (dynamic_objects_hd_map_path_dots
 *  alone is 5894 of 42428 object markers in the recorded bag). Folding
 *  rule-drops into dropped_malformed would make a healthy system look
 *  broken; leaving them uncounted would make VM-034 report zero drops on
 *  data the node deliberately threw away.
 */

#include <cstdint>

namespace mpviz_node
{

struct AdapterStats
{
    double last_msg_sec{0.0};
    uint64_t msgs{0};
    uint64_t dropped_malformed{0};
    uint64_t dropped_stale{0};
    uint64_t dropped_no_tf{0};
    uint64_t dropped_by_rule{0};
};

}  // namespace mpviz_node
