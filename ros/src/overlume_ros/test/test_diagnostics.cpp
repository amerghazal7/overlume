// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/diagnostics.hpp"

#include <gtest/gtest.h>

namespace {

overlume::ros::AdapterStats StatsWithSomeDrops() {
    overlume::ros::AdapterStats s;
    s.last_msg_sec = 12.0;
    s.msgs = 40;
    s.dropped_malformed = 1;
    s.dropped_stale = 2;
    s.dropped_no_tf = 3;
    s.dropped_by_rule = 4;
    return s;
}

}

TEST(Diagnostics, OneStatusPerRowPlusRenderMs) {
    std::vector<overlume::ros::RowStats> rows(1);
    rows[0].topic = "/hd_map_local_elements";
    rows[0].stats = StatsWithSomeDrops();
    rows[0].last_msg_age_sec = 0.5;
    rows[0].timeout_sec = 2.0;

    const auto msg = overlume::ros::BuildDiagnostics(rows, 4.2);

    ASSERT_EQ(msg.status.size(), 2u);
    EXPECT_EQ(msg.status[0].name, "/hd_map_local_elements");
    EXPECT_EQ(msg.status[1].name, "render_ms");
}

TEST(Diagnostics, ValuesCarryEveryAdapterStatsCounterVerbatim) {
    std::vector<overlume::ros::RowStats> rows(1);
    rows[0].topic = "/perception/dynamic_objects_list";
    rows[0].stats = StatsWithSomeDrops();
    rows[0].last_msg_age_sec = 0.5;
    rows[0].timeout_sec = 2.0;

    const auto msg = overlume::ros::BuildDiagnostics(rows, 1.0);
    const auto& status = msg.status[0];

    auto find = [&](const std::string& key) -> std::string {
        for (const auto& v : status.values) {
            if (v.key == key) return v.value;
        }
        return "<missing>";
    };
    EXPECT_EQ(find("msgs"), "40");
    EXPECT_EQ(find("dropped_malformed"), "1");
    EXPECT_EQ(find("dropped_stale"), "2");
    EXPECT_EQ(find("dropped_no_tf"), "3");
    EXPECT_EQ(find("dropped_by_rule"), "4");
    EXPECT_EQ(find("last_msg_age_sec"), "0.5");
}

TEST(Diagnostics, NeverPublishedRowReportsOkNoDataYetWithNoBogusAge) {
    std::vector<overlume::ros::RowStats> rows(1);
    rows[0].topic = "/never_published";
    rows[0].last_msg_age_sec = 123.0;
    rows[0].timeout_sec = 2.0;

    const auto msg = overlume::ros::BuildDiagnostics(rows, 0.0);
    const auto& status = msg.status[0];

    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(status.message, "no data yet");
    for (const auto& v : status.values) {
        EXPECT_NE(v.key, "last_msg_age_sec") << "absent row must not report an age";
    }
}

TEST(Diagnostics, PublishedThenSilentRowStillReportsItsRealAgeAndStaleLevel) {
    std::vector<overlume::ros::RowStats> rows(1);
    rows[0].topic = "/went_quiet";
    rows[0].stats.msgs = 5;
    rows[0].last_msg_age_sec = 5.0;
    rows[0].timeout_sec = 2.0;

    const auto msg = overlume::ros::BuildDiagnostics(rows, 0.0);
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

TEST(Diagnostics, RowPastItsOwnTimeoutSecIsWarnEverythingElseIsOk) {
    std::vector<overlume::ros::RowStats> rows(2);
    rows[0].topic = "/fresh";
    rows[0].stats.msgs = 1;
    rows[0].last_msg_age_sec = 0.1;
    rows[0].timeout_sec = 2.0;
    rows[1].topic = "/stale";
    rows[1].stats.msgs = 1;
    rows[1].last_msg_age_sec = 5.0;
    rows[1].timeout_sec = 2.0;

    const auto msg = overlume::ros::BuildDiagnostics(rows, 0.0);

    EXPECT_EQ(msg.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(msg.status[1].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(Diagnostics, RenderMsNodeLevelStatusCarriesTheValueVerbatim) {
    const auto msg = overlume::ros::BuildDiagnostics({}, 7.75);
    ASSERT_EQ(msg.status.size(), 1u);
    ASSERT_EQ(msg.status[0].values.size(), 1u);
    EXPECT_EQ(msg.status[0].values[0].key, "render_ms");
    EXPECT_EQ(msg.status[0].values[0].value, "7.75");
}

TEST(Diagnostics, HybridStarvedReasonTruthTable) {
    using overlume::ros::HybridStarvedReason;
    // (consumed, enabled, has_sub)
    EXPECT_EQ(HybridStarvedReason(false, false, false), "");
    EXPECT_EQ(HybridStarvedReason(false, true, false), "");
    EXPECT_EQ(HybridStarvedReason(true, true, true), "");
    EXPECT_NE(HybridStarvedReason(true, false, true), "");
    EXPECT_NE(HybridStarvedReason(true, true, false), "");
    EXPECT_NE(HybridStarvedReason(true, false, false), "");
}

TEST(Diagnostics, HybridStarvedReasonNamesTheParamToSet) {
    using overlume::ros::HybridStarvedReason;
    EXPECT_NE(HybridStarvedReason(true, true, false).find("pointcloud_topic"), std::string::npos);
    EXPECT_NE(HybridStarvedReason(true, false, true).find("hybrid_enabled"), std::string::npos);
    // disabled wins over missing subscription: the operator's first fix is the flag
    EXPECT_NE(HybridStarvedReason(true, false, false).find("hybrid_enabled"), std::string::npos);
}

TEST(Diagnostics, BuildHybridStatusErrorAndOk) {
    const auto bad = overlume::ros::BuildHybridStatus("pointcloud_topic is empty");
    EXPECT_EQ(bad.name, "hybrid");
    EXPECT_EQ(bad.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_EQ(bad.message, "pointcloud_topic is empty");
    const auto ok = overlume::ros::BuildHybridStatus("");
    EXPECT_EQ(ok.name, "hybrid");
    EXPECT_EQ(ok.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(ok.message, "ok");
}
