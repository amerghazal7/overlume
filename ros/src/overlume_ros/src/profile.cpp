// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/profile.hpp"

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace overlume::ros {
namespace {

std::optional<NsRender> ParseNsRender(const std::string& s) {
    if (s == "drop") return NsRender::kDrop;
    if (s == "polyline") return NsRender::kPolyline;
    if (s == "polygon") return NsRender::kPolygon;
    return std::nullopt;
}

// YAML spelling -> MapKind. "road_surface" is deliberately NOT accepted here
// -- ROAD_SURFACE is adapter-synthesized (paired boundary rails), never a
// value a profile author can request.
std::optional<overlume::MapKind> ParseMapKind(const std::string& s) {
    if (s == "other") return overlume::MapKind::OTHER;
    if (s == "centerline") return overlume::MapKind::CENTERLINE;
    if (s == "left_boundary") return overlume::MapKind::LEFT_BOUNDARY;
    if (s == "right_boundary") return overlume::MapKind::RIGHT_BOUNDARY;
    if (s == "crosswalk") return overlume::MapKind::CROSSWALK;
    if (s == "stopline") return overlume::MapKind::STOPLINE;
    if (s == "junction") return overlume::MapKind::JUNCTION;
    if (s == "road_edge") return overlume::MapKind::ROAD_EDGE;
    return std::nullopt;
}

// Which render verdicts a non-OTHER kind is legal on (ValidateRow's
// cross-field check, same shape as the retired adapter-side chop-vs-render
// check it replaces). CROSSWALK is polygon-only; every lane-geometry kind is
// polyline-only.
bool KindIsLegalOnRender(overlume::MapKind kind, NsRender render) {
    if (kind == overlume::MapKind::OTHER) return true;
    if (kind == overlume::MapKind::CROSSWALK) return render == NsRender::kPolygon;
    return render == NsRender::kPolyline;
}

// Adapter -> its closed role set (see profile.hpp struct comment).
const std::map<std::string, std::set<std::string>>& RoleSets() {
    static const std::map<std::string, std::set<std::string>> kRoles = {
        {"path", {"behavior", "global", "local"}},
        {"hd_map", {"lane"}},
        {"ogm", {"dynamic_ogm", "gradient_ogm"}},
        {"collision", {"collision", "predicted", "merged_object", "sweep", "merged_ego"}},
        {"dynamic_objects", {"tracked"}},
        {"generic", {"neutral"}},
        {"tf_axes", {"debug"}},
        {"point_cloud", {"points"}},
        {"trajectory_carpet", {"carpet"}},
    };
    return kRoles;
}

// Adapter -> the message type(s) its rows may carry. tf_axes has none (it
// generates its markers from the tf2 buffer -- no subscription at all).
const std::map<std::string, std::set<std::string>>& TypeSets() {
    static const std::map<std::string, std::set<std::string>> kTypes = {
        {"path", {"nav_msgs/msg/Path"}},
        {"hd_map", {"visualization_msgs/msg/MarkerArray"}},
        {"ogm", {"nav_msgs/msg/OccupancyGrid"}},
        {"collision", {"visualization_msgs/msg/MarkerArray"}},
        {"dynamic_objects", {"visualization_msgs/msg/MarkerArray"}},
        {"generic", {"visualization_msgs/msg/MarkerArray"}},
        {"point_cloud", {"sensor_msgs/msg/PointCloud2"}},
        {"trajectory_carpet", {"visualization_msgs/msg/MarkerArray"}},
    };
    return kTypes;
}

const std::set<std::string>& KnownAdapters() {
    static const std::set<std::string> kAdapters = {
        "dynamic_objects", "path",        "hd_map",           "ogm", "collision", "generic",
        "tf_axes",         "point_cloud", "trajectory_carpet"};
    return kAdapters;
}

const std::set<std::string>& KnownRowKeys() {
    static const std::set<std::string> kKeys = {"topic",        "type",
                                                "adapter",      "role",
                                                "update_topic", "timeout_sec",
                                                "max_rate_hz",  "namespaces",
                                                "ns_default",   "transient_local",
                                                "best_effort",  "junction_interior_boundaries",
                                                "color_mode",   "max_points",
                                                "stride"};
    return kKeys;
}

std::string RowTag(const std::string& file, size_t idx, const std::string& topic) {
    std::ostringstream os;
    os << file << ": row " << idx << " (topic " << (topic.empty() ? "<empty>" : topic) << "): ";
    return os.str();
}

// Builds one ProfileRow from its YAML node. Returns false on a malformed enum
// value (namespaces[].render / ns_default) -- those can't be represented in
// ProfileRow at all, so they must fail here, before validate_row() ever runs.
// Unknown extra keys are a warning only (hand-edited by the autonomy team).
bool ParseRow(const YAML::Node& node, const std::string& file, size_t idx, ProfileRow& out,
              std::vector<std::string>& errors) {
    if (!node.IsMap()) {
        errors.push_back(RowTag(file, idx, "") + "row must be a mapping, not a scalar/sequence");
        return false;
    }

    bool ok = true;
    const auto get_str = [&](const char* key, const std::string& def) {
        return node[key] ? node[key].as<std::string>() : def;
    };

    out.topic = get_str("topic", "");
    out.type = get_str("type", "");
    out.adapter = get_str("adapter", "");
    out.role = get_str("role", "");
    out.update_topic = get_str("update_topic", "");
    out.timeout_sec = node["timeout_sec"] ? node["timeout_sec"].as<double>() : 2.0;
    out.max_rate_hz = node["max_rate_hz"] ? node["max_rate_hz"].as<double>() : 0.0;
    out.transient_local = node["transient_local"] ? node["transient_local"].as<bool>() : false;
    out.best_effort = node["best_effort"] ? node["best_effort"].as<bool>() : false;

    // adapter: hd_map only -- the key's presence is what's restricted, not its
    // value (same "typo'd key" concern update_topic's own adapter check
    // guards against). Checked here, not in ValidateRow, because ValidateRow
    // only sees the parsed bool -- it can't tell "explicitly true" from "left
    // at default" -- and out.adapter is already parsed by the time this key
    // is read.
    if (node["junction_interior_boundaries"]) {
        if (out.adapter != "hd_map") {
            errors.push_back(RowTag(file, idx, out.topic) +
                             "junction_interior_boundaries is only valid on adapter: hd_map rows");
            ok = false;
        } else {
            out.junction_interior_boundaries = node["junction_interior_boundaries"].as<bool>();
        }
    }

    // adapter: point_cloud only (Epic 3 Task 6 / VM-035) -- same
    // presence-restriction shape as junction_interior_boundaries above.
    if (node["color_mode"]) {
        if (out.adapter != "point_cloud") {
            errors.push_back(RowTag(file, idx, out.topic) +
                             "color_mode is only valid on adapter: point_cloud rows");
            ok = false;
        } else {
            out.color_mode = node["color_mode"].as<std::string>();
        }
    }
    if (node["max_points"]) {
        if (out.adapter != "point_cloud") {
            errors.push_back(RowTag(file, idx, out.topic) +
                             "max_points is only valid on adapter: point_cloud rows");
            ok = false;
        } else {
            out.max_points = node["max_points"].as<uint32_t>();
        }
    }
    if (node["stride"]) {
        if (out.adapter != "point_cloud") {
            errors.push_back(RowTag(file, idx, out.topic) +
                             "stride is only valid on adapter: point_cloud rows");
            ok = false;
        } else {
            out.stride = node["stride"].as<uint32_t>();
        }
    }

    const std::string ns_default_str = get_str("ns_default", "polyline");
    if (auto r = ParseNsRender(ns_default_str)) {
        out.ns_default = *r;
    } else {
        errors.push_back(RowTag(file, idx, out.topic) + "ns_default '" + ns_default_str +
                         "' must be one of drop|polyline|polygon");
        ok = false;
    }

    if (node["namespaces"]) {
        for (const auto& item : node["namespaces"]) {
            if (!item.IsMap()) {
                errors.push_back(RowTag(file, idx, out.topic) +
                                 "namespaces[] entry must be a mapping with 'prefix' and 'render'");
                ok = false;
                continue;
            }
            NsRule rule;
            rule.prefix = item["prefix"] ? item["prefix"].as<std::string>() : "";
            if (rule.prefix.empty()) {
                // An empty prefix matches every namespace (classify()'s
                // prefix compare is trivially true against ""), silently
                // overriding ns_default for everything -- almost always a
                // missing/misspelled 'prefix' key, never intentional.
                errors.push_back(RowTag(file, idx, out.topic) +
                                 "namespaces[] entry missing non-empty 'prefix'");
                ok = false;
                continue;
            }
            const std::string render_str = item["render"] ? item["render"].as<std::string>() : "";
            if (auto r = ParseNsRender(render_str)) {
                rule.render = *r;
            } else {
                errors.push_back(RowTag(file, idx, out.topic) + "namespaces[].render '" +
                                 render_str + "' must be one of drop|polyline|polygon");
                ok = false;
                continue;
            }
            if (item["kind"]) {
                const std::string kind_str = item["kind"].as<std::string>();
                if (auto k = ParseMapKind(kind_str)) {
                    rule.kind = *k;
                } else {
                    errors.push_back(RowTag(file, idx, out.topic) + "namespaces[].kind '" +
                                     kind_str +
                                     "' must be one of other|centerline|left_boundary|"
                                     "right_boundary|crosswalk|stopline|junction|road_edge");
                    ok = false;
                    continue;
                }
            }
            // kind vs. render is a cross-field semantic check, not a
            // can't-represent-it-at-all parse failure -- see ValidateRow.
            for (auto it = item.begin(); it != item.end(); ++it) {
                const std::string key = it->first.as<std::string>();
                if (key != "prefix" && key != "render" && key != "kind") {
                    errors.push_back(RowTag(file, idx, out.topic) + "namespaces[] unknown key '" +
                                     key + "' (ignored)");
                }
            }
            out.namespaces.push_back(std::move(rule));
        }
    }

    for (auto it = node.begin(); it != node.end(); ++it) {
        const std::string key = it->first.as<std::string>();
        if (!KnownRowKeys().count(key)) {
            errors.push_back(RowTag(file, idx, out.topic) + "unknown key '" + key + "' (ignored)");
        }
    }

    return ok;
}

// One problem per row is enough to report; the row is dropped either way
// once it's invalid, so piling up every downstream consequence of the
// first mistake would just be noise on top of the fix the user needs to make.
bool ValidateRow(const ProfileRow& row, const std::string& file, size_t idx,
                 std::vector<std::string>& errors) {
    const auto fail = [&](const std::string& msg) {
        errors.push_back(RowTag(file, idx, row.topic) + msg);
        return false;
    };

    if (row.adapter.empty()) return fail("missing required key 'adapter'");
    if (!KnownAdapters().count(row.adapter)) return fail("unknown adapter '" + row.adapter + "'");

    const bool is_tf_axes = (row.adapter == "tf_axes");
    if (is_tf_axes) {
        if (!row.topic.empty())
            return fail(
                "adapter: tf_axes rows must not set 'topic' (nothing publishes TF as markers)");
        if (!row.type.empty()) return fail("adapter: tf_axes rows must not set 'type'");
    } else {
        if (row.topic.empty()) return fail("missing required key 'topic'");
        if (row.type.empty()) return fail("missing required key 'type'");
        if (!TypeSets().at(row.adapter).count(row.type))
            return fail("type '" + row.type + "' is not valid for adapter '" + row.adapter + "'");
    }

    if (row.role.empty()) return fail("missing required key 'role'");
    if (!RoleSets().at(row.adapter).count(row.role))
        return fail("role '" + row.role + "' is not valid for adapter '" + row.adapter + "'");

    if (!row.update_topic.empty() && row.adapter != "ogm")
        return fail("update_topic is only valid on adapter: ogm rows");

    if (row.timeout_sec < 1.0)
        return fail(
            "timeout_sec must be >= 1.0 (the renderer's staleness fade runs 0.5..1.0s "
            "past it -- a smaller value yanks the entity before the fade finishes)");

    if (row.max_rate_hz < 0.0) return fail("max_rate_hz must be >= 0");

    if (row.adapter == "point_cloud") {
        static const std::set<std::string> kColorModes = {"auto", "rgb", "intensity", "height",
                                                          "flat"};
        if (!kColorModes.count(row.color_mode))
            return fail("color_mode '" + row.color_mode +
                        "' must be one of auto|rgb|intensity|height|flat");
        if (row.stride < 1)
            return fail("stride must be >= 1 (0 would divide/mod by zero in the adapter)");
    }

    std::set<std::string> seen;
    for (const auto& rule : row.namespaces) {
        if (!seen.insert(rule.prefix).second)
            return fail("duplicate namespace prefix '" + rule.prefix + "'");
        if (!KindIsLegalOnRender(rule.kind, rule.render))
            return fail("namespaces[] prefix '" + rule.prefix +
                        "': kind is not legal on this rule's render verdict");
    }

    return true;
}

std::optional<Profile> BuildProfile(const YAML::Node& root, const std::string& file,
                                    std::vector<std::string>& errors) {
    Profile profile;
    profile.name = root["name"] ? root["name"].as<std::string>() : "";

    bool hard_fail = false;
    if (!root["rows"] || !root["rows"].IsSequence()) {
        errors.push_back(file + ": missing required key 'rows' (must be a sequence)");
        return std::nullopt;
    }

    size_t idx = 0;
    // Parallel to profile.rows: the row's index in the FILE, not among
    // surviving rows -- a dropped earlier row must not shift later rows'
    // reported numbers, or the duplicate-pair message below names the wrong row.
    std::vector<size_t> row_file_idx;
    for (const auto& node : root["rows"]) {
        ProfileRow row;
        if (!ParseRow(node, file, idx, row, errors)) {
            hard_fail = true;
            ++idx;
            continue;
        }
        if (!ValidateRow(row, file, idx, errors)) {
            hard_fail = true;
            ++idx;
            continue;
        }
        profile.rows.push_back(std::move(row));
        row_file_idx.push_back(idx);
        ++idx;
    }

    // Cross-row: duplicate (topic, adapter) pairs. Meaningless for tf_axes
    // rows (no topic at all -- any number of them is fine, they're just
    // config for the same generated debug layer) so they're excluded.
    std::set<std::pair<std::string, std::string>> seen_topic_adapter;
    for (size_t i = 0; i < profile.rows.size(); ++i) {
        const auto& row = profile.rows[i];
        if (row.adapter == "tf_axes") continue;
        auto key = std::make_pair(row.topic, row.adapter);
        if (!seen_topic_adapter.insert(key).second) {
            errors.push_back(RowTag(file, row_file_idx[i], row.topic) +
                             "duplicate (topic, adapter) pair -- another row already " +
                             "subscribes '" + row.topic + "' with adapter '" + row.adapter + "'");
            hard_fail = true;
        }
    }

    if (hard_fail) return std::nullopt;
    return profile;
}

}  // namespace

std::optional<Profile> load_profile(const std::string& path, std::vector<std::string>& errors) {
    errors.clear();
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
        return BuildProfile(root, path, errors);
    } catch (const YAML::Exception& e) {
        errors.push_back(path + ": yaml parse error: " + e.what());
        return std::nullopt;
    } catch (const std::exception& e) {
        errors.push_back(path + ": " + e.what());
        return std::nullopt;
    }
}

std::optional<Profile> load_profile_string(std::string_view yaml,
                                           std::vector<std::string>& errors) {
    errors.clear();
    YAML::Node root;
    try {
        root = YAML::Load(std::string(yaml));
        return BuildProfile(root, "<string>", errors);
    } catch (const YAML::Exception& e) {
        errors.push_back(std::string("<string>: yaml parse error: ") + e.what());
        return std::nullopt;
    }
}

const ProfileRow* find_row(const Profile& profile, std::string_view topic) {
    for (const auto& row : profile.rows) {
        if (row.topic == topic) return &row;
    }
    return nullptr;
}

const NsRule* match_rule(const ProfileRow& row, std::string_view ns) {
    const NsRule* best = nullptr;
    for (const auto& rule : row.namespaces) {
        if (ns.size() < rule.prefix.size()) continue;
        if (ns.compare(0, rule.prefix.size(), rule.prefix) != 0) continue;
        if (best == nullptr || rule.prefix.size() > best->prefix.size()) best = &rule;
    }
    return best;
}

NsRender classify(const ProfileRow& row, std::string_view ns) {
    const NsRule* best = match_rule(row, ns);
    return best != nullptr ? best->render : row.ns_default;
}

std::vector<SubSpec> subscriptions_for(const ProfileRow& row) {
    if (row.adapter == "tf_axes") return {};

    std::vector<SubSpec> specs;
    specs.push_back(SubSpec{row.topic, row.type, row.best_effort, row.transient_local});
    if (row.adapter == "ogm" && !row.update_topic.empty()) {
        // best_effort is safe to propagate (a BEST_EFFORT subscriber still
        // matches a RELIABLE publisher); transient_local is NOT -- an update
        // stream is inherently VOLATILE (each patch supersedes the last), so a
        // TRANSIENT_LOCAL subscriber would never match it and go silently,
        // permanently dead.
        specs.push_back(
            SubSpec{row.update_topic, "map_msgs/msg/OccupancyGridUpdate", row.best_effort, false});
    }
    return specs;
}

}  // namespace overlume::ros
