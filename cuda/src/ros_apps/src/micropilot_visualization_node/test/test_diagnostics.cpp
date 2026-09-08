// test_diagnostics.cpp — Epic 3 Task 2 (VM-034) Step 0. BuildDiagnostics() is
// a pure data transform (diagnostics.hpp's own header comment) -- no ROS
// node, no clock, no fixture -- so these are plain value-in/value-out
// assertions on diagnostic_msgs::msg::DiagnosticArray.
#include "micropilot_visualization_node/diagnostics.hpp"

#include <gtest/gtest.h>

namespace
{

mpviz_node::AdapterStats StatsWithSomeDrops()
{
    mpviz_node::AdapterStats s;
    s.last_msg_sec = 12.0;
    s.msgs = 40;
    s.dropped_malformed = 1;
    s.dropped_stale = 2;
    s.dropped_no_tf = 3;
    s.dropped_by_rule = 4;
    return s;
}

}  // namespace

TEST(Diagnostics, OneStatusPerRowPlusRenderMs)
{
    std::vector<mpviz_node::RowStats> rows(1);
    rows[0].topic = "/hd_map_local_elements";
    rows[0].stats = StatsWithSomeDrops();
    rows[0].last_msg_age_sec = 0.5;
    rows[0].timeout_sec = 2.0;

    const auto msg = mpviz_node::BuildDiagnostics(rows, /*render_ms=*/4.2);

    ASSERT_EQ(msg.status.size(), 2u);  // 1 row + 1 node-level
    EXPECT_EQ(msg.status[0].name, "/hd_map_local_elements");
    EXPECT_EQ(msg.status[1].name, "render_ms");
}

TEST(Diagnostics, ValuesCarryEveryAdapterStatsCounterVerbatim)
{
    std::vector<mpviz_node::RowStats> rows(1);
    rows[0].topic = "/perception/dynamic_objects_list";
    rows[0].stats = StatsWithSomeDrops();
    rows[0].last_msg_age_sec = 0.5;
    rows[0].timeout_sec = 2.0;

    const auto msg = mpviz_node::BuildDiagnostics(rows, 1.0);
    const auto& status = msg.status[0];

    auto find = [&](const std::string& key) -> std::string {
        for (const auto& v : status.values) {
            if (v.key == key) return v.value;
        }
        return "<missing>";
    };
    // dropped_by_rule stays a SEPARATE counter from dropped_malformed
    // (adapter_stats.hpp's own header comment) -- both must be present and
    // must carry their own distinct value, not one folded into the other.
    // "verbatim" means every AdapterStats field, msgs included -- this test's
    // name used to claim that coverage while never checking msgs at all.
    EXPECT_EQ(find("msgs"), "40");
    EXPECT_EQ(find("dropped_malformed"), "1");
    EXPECT_EQ(find("dropped_stale"), "2");
    EXPECT_EQ(find("dropped_no_tf"), "3");
    EXPECT_EQ(find("dropped_by_rule"), "4");
    EXPECT_EQ(find("last_msg_age_sec"), "0.5");
}

TEST(Diagnostics, NeverPublishedRowReportsOkNoDataYetWithNoBogusAge)
{
    // stats defaults to msgs == 0 (never published) -- last_msg_sec also
    // defaults to 0.0, so a caller computing last_msg_age_sec as
    // sim_clock_sec_ - last_msg_sec (visualization_node.cpp's own formula)
    // hands BuildDiagnostics a large, meaningless "age" here, same as it
    // would for a row that has been silent since node start. This must not
    // read as stale.
    std::vector<mpviz_node::RowStats> rows(1);
    rows[0].topic = "/never_published";
    rows[0].last_msg_age_sec = 123.0;  // sim_clock_sec_ - 0.0, bogus for an absent row
    rows[0].timeout_sec = 2.0;

    const auto msg = mpviz_node::BuildDiagnostics(rows, 0.0);
    const auto& status = msg.status[0];

    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(status.message, "no data yet");
    for (const auto& v : status.values) {
        EXPECT_NE(v.key, "last_msg_age_sec") << "absent row must not report an age";
    }
}

TEST(Diagnostics, PublishedThenSilentRowStillReportsItsRealAgeAndStaleLevel)
{
    std::vector<mpviz_node::RowStats> rows(1);
    rows[0].topic = "/went_quiet";
    rows[0].stats.msgs = 5;  // published before, so this is genuine staleness
    rows[0].last_msg_age_sec = 5.0;
    rows[0].timeout_sec = 2.0;

    const auto msg = mpviz_node::BuildDiagnostics(rows, 0.0);
    const auto& status = msg.status[0];

    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_EQ(status.message, "stale");
    auto find = [&](const std::string& key) -> std::string {
        for (const auto& v : status.values) {
            if (v.key == key) return v.value;
        }
        return "<missing>";
    };
    EXPECT_EQ(find("last_msg_age_sec"), "5");
}

TEST(Diagnostics, RowPastItsOwnTimeoutSecIsWarnEverythingElseIsOk)
{
    std::vector<mpviz_node::RowStats> rows(2);
    rows[0].topic = "/fresh";
    rows[0].stats.msgs = 1;  // published, and recently -- not the msgs==0 absent case
    rows[0].last_msg_age_sec = 0.1;
    rows[0].timeout_sec = 2.0;
    rows[1].topic = "/stale";
    rows[1].stats.msgs = 1;  // published before, then went quiet past timeout_sec
    rows[1].last_msg_age_sec = 5.0;
    rows[1].timeout_sec = 2.0;

    const auto msg = mpviz_node::BuildDiagnostics(rows, 0.0);

    EXPECT_EQ(msg.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(msg.status[1].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(Diagnostics, RenderMsNodeLevelStatusCarriesTheValueVerbatim)
{
    const auto msg = mpviz_node::BuildDiagnostics({}, 7.75);
    ASSERT_EQ(msg.status.size(), 1u);  // 0 rows + 1 node-level
    ASSERT_EQ(msg.status[0].values.size(), 1u);
    EXPECT_EQ(msg.status[0].values[0].key, "render_ms");
    EXPECT_EQ(msg.status[0].values[0].value, "7.75");
}
