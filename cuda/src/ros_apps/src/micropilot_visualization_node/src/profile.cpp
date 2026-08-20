#include "micropilot_visualization_node/profile.hpp"

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace mpviz_node
{
namespace
{

std::optional<NsRender> ParseNsRender(const std::string& s)
{
    if (s == "drop") return NsRender::kDrop;
    if (s == "polyline") return NsRender::kPolyline;
    if (s == "polygon") return NsRender::kPolygon;
    return std::nullopt;
}

// Adapter -> its closed role set (epic2 plan, profile.hpp struct comment).
const std::map<std::string, std::set<std::string>>& RoleSets()
{
    static const std::map<std::string, std::set<std::string>> kRoles = {
        {"path", {"behavior", "global", "local"}},
        {"hd_map", {"lane"}},
        {"ogm", {"dynamic_ogm", "gradient_ogm"}},
        {"collision", {"collision", "predicted", "merged_object", "sweep", "merged_ego"}},
        {"dynamic_objects", {"tracked"}},
        {"generic", {"neutral"}},
        {"tf_axes", {"debug"}},
    };
    return kRoles;
}

// Adapter -> the message type(s) its rows may carry. tf_axes has none (it
// generates its markers from the tf2 buffer -- no subscription at all).
const std::map<std::string, std::set<std::string>>& TypeSets()
{
    static const std::map<std::string, std::set<std::string>> kTypes = {
        {"path", {"nav_msgs/msg/Path"}},
        {"hd_map", {"visualization_msgs/msg/MarkerArray"}},
        {"ogm", {"nav_msgs/msg/OccupancyGrid"}},
        {"collision", {"visualization_msgs/msg/MarkerArray"}},
        {"dynamic_objects", {"visualization_msgs/msg/MarkerArray"}},
        {"generic", {"visualization_msgs/msg/MarkerArray"}},
    };
    return kTypes;
}

const std::set<std::string>& KnownAdapters()
{
    static const std::set<std::string> kAdapters = {
        "dynamic_objects", "path", "hd_map", "ogm", "collision", "generic", "tf_axes"};
    return kAdapters;
}

const std::set<std::string>& KnownRowKeys()
{
    static const std::set<std::string> kKeys = {
        "topic",     "type",       "adapter",         "role",     "update_topic",
        "timeout_sec", "max_rate_hz", "namespaces",    "ns_default", "transient_local",
        "best_effort"};
    return kKeys;
}

std::string RowTag(const std::string& file, size_t idx, const std::string& topic)
{
    std::ostringstream os;
    os << file << ": row " << idx << " (topic " << (topic.empty() ? "<empty>" : topic) << "): ";
    return os.str();
}

// Builds one ProfileRow from its YAML node. Returns false (and appends an
// error) on a malformed enum value (namespaces[].render / ns_default) --
// those can't be represented in ProfileRow at all, so they must fail here,
// before validate_row() ever runs. Unknown extra keys are a WARNING
// appended to `errors`, never a hard failure (VM-042: hand-edited by the
// autonomy team).
bool ParseRow(const YAML::Node& node, const std::string& file, size_t idx, ProfileRow& out,
              std::vector<std::string>& errors)
{
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

    const std::string ns_default_str = get_str("ns_default", "polyline");
    if (auto r = ParseNsRender(ns_default_str)) {
        out.ns_default = *r;
    } else {
        errors.push_back(RowTag(file, idx, out.topic) +
                          "ns_default '" + ns_default_str + "' must be one of drop|polyline|polygon");
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
                errors.push_back(RowTag(file, idx, out.topic) +
                                  "namespaces[].render '" + render_str +
                                  "' must be one of drop|polyline|polygon");
                ok = false;
                continue;
            }
            rule.dashed = item["dashed"] ? item["dashed"].as<bool>() : false;
            // dashed vs. render is a cross-field semantic check, not a
            // can't-represent-it-at-all parse failure -- see ValidateRow.
            for (auto it = item.begin(); it != item.end(); ++it) {
                const std::string key = it->first.as<std::string>();
                if (key != "prefix" && key != "render" && key != "dashed") {
                    errors.push_back(RowTag(file, idx, out.topic) +
                                      "namespaces[] unknown key '" + key + "' (ignored)");
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
                  std::vector<std::string>& errors)
{
    const auto fail = [&](const std::string& msg) {
        errors.push_back(RowTag(file, idx, row.topic) + msg);
        return false;
    };

    if (row.adapter.empty()) return fail("missing required key 'adapter'");
    if (!KnownAdapters().count(row.adapter)) return fail("unknown adapter '" + row.adapter + "'");

    const bool is_tf_axes = (row.adapter == "tf_axes");
    if (is_tf_axes) {
        if (!row.topic.empty())
            return fail("adapter: tf_axes rows must not set 'topic' (nothing publishes TF as markers)");
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
        return fail("timeout_sec must be >= 1.0 (the renderer's staleness fade runs 0.5..1.0s "
                    "past it -- a smaller value yanks the entity before the fade finishes)");

    if (row.max_rate_hz < 0.0) return fail("max_rate_hz must be >= 0");

    std::set<std::string> seen;
    for (const auto& rule : row.namespaces) {
        if (!seen.insert(rule.prefix).second)
            return fail("duplicate namespace prefix '" + rule.prefix + "'");
        if (rule.dashed && rule.render != NsRender::kPolyline)
            return fail("namespaces[] prefix '" + rule.prefix +
                        "': dashed: true is only legal on render: polyline (got '" +
                        (rule.render == NsRender::kPolygon ? "polygon" : "drop") + "')");
    }

    return true;
}

std::optional<Profile> BuildProfile(const YAML::Node& root, const std::string& file,
                                     std::vector<std::string>& errors)
{
    Profile profile;
    profile.name = root["name"] ? root["name"].as<std::string>() : "";

    bool hard_fail = false;
    if (!root["rows"] || !root["rows"].IsSequence()) {
        errors.push_back(file + ": missing required key 'rows' (must be a sequence)");
        return std::nullopt;
    }

    size_t idx = 0;
    // Parallel to profile.rows: the row's index in the FILE, not its index
    // among the surviving rows -- an earlier row that failed ParseRow/
    // ValidateRow is dropped from profile.rows but must not shift every
    // later row's reported number, or the duplicate-(topic,adapter) message
    // below would name the wrong row.
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

std::optional<Profile> load_profile(const std::string& path, std::vector<std::string>& errors)
{
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

std::optional<Profile> load_profile_string(std::string_view yaml, std::vector<std::string>& errors)
{
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

const ProfileRow* find_row(const Profile& profile, std::string_view topic)
{
    for (const auto& row : profile.rows) {
        if (row.topic == topic) return &row;
    }
    return nullptr;
}

const NsRule* match_rule(const ProfileRow& row, std::string_view ns)
{
    const NsRule* best = nullptr;
    for (const auto& rule : row.namespaces) {
        if (ns.size() < rule.prefix.size()) continue;
        if (ns.compare(0, rule.prefix.size(), rule.prefix) != 0) continue;
        if (best == nullptr || rule.prefix.size() > best->prefix.size()) best = &rule;
    }
    return best;
}

NsRender classify(const ProfileRow& row, std::string_view ns)
{
    const NsRule* best = match_rule(row, ns);
    return best != nullptr ? best->render : row.ns_default;
}

std::vector<SubSpec> subscriptions_for(const ProfileRow& row)
{
    if (row.adapter == "tf_axes") return {};

    std::vector<SubSpec> specs;
    specs.push_back(SubSpec{row.topic, row.type, row.best_effort, row.transient_local});
    if (row.adapter == "ogm" && !row.update_topic.empty()) {
        // best_effort is safe to propagate (a BEST_EFFORT subscriber still
        // matches a RELIABLE publisher); transient_local is NOT -- an update
        // stream is inherently VOLATILE (each patch supersedes the last, a
        // late joiner needs the base grid, not old patches), so a
        // TRANSIENT_LOCAL-requesting subscriber would never match it and the
        // patch stream would go silently, permanently dead.
        specs.push_back(SubSpec{row.update_topic, "map_msgs/msg/OccupancyGridUpdate",
                                 row.best_effort, false});
    }
    return specs;
}

}  // namespace mpviz_node
