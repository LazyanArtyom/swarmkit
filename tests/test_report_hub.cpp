// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "../src/agent/report_hub.h"

namespace swarmkit::agent::internal {
namespace {

[[nodiscard]] swarmkit::v1::AgentReport MakeReport(std::string drone_id, std::string message) {
    swarmkit::v1::AgentReport report;
    report.set_drone_id(std::move(drone_id));
    report.set_type(swarmkit::v1::GOAL_REPORT);
    report.set_severity(swarmkit::v1::REPORT_INFO);
    report.set_message(std::move(message));
    return report;
}

TEST_CASE("ReportHub finalizes identity and replays its bounded in-memory history",
          "[agent][reports]") {
    std::int64_t wall_time = 1000;
    ReportHub hub({.max_in_memory_backlog = 2,
                   .wall_time_ms = [&wall_time] { return ++wall_time; },
                   .agent_session_id = "agent-session-test"});
    std::vector<swarmkit::v1::AgentReport> observed;
    hub.SetFinalizedReportObserver(
        [&observed](const swarmkit::v1::AgentReport& report) { observed.push_back(report); });

    hub.Publish(MakeReport("drone-1", "evicted"));
    hub.Publish(MakeReport("drone-2", "retained-2"));
    hub.Publish(MakeReport("drone-1", "retained-3"));

    REQUIRE(observed.size() == 3);
    CHECK(observed[0].sequence() == 1);
    CHECK(observed[2].sequence() == 3);
    CHECK(observed[2].unix_time_ms() == 1003);
    CHECK(observed[2].agent_session_id() == "agent-session-test");

    auto queue = std::make_shared<ReportQueue>();
    const ReportWatchToken token = hub.Watch("all", 0, queue);
    swarmkit::v1::AgentReport first;
    swarmkit::v1::AgentReport second;
    REQUIRE(queue->Pop(&first, std::chrono::milliseconds{10}));
    REQUIRE(queue->Pop(&second, std::chrono::milliseconds{10}));
    CHECK(first.sequence() == 2);
    CHECK(second.sequence() == 3);
    hub.Unwatch(token);
}

TEST_CASE("ReportHub filters replay and delivers live reports after retained evidence",
          "[agent][reports]") {
    ReportHub hub({.max_in_memory_backlog = 8,
                   .wall_time_ms = [] { return 42; },
                   .agent_session_id = "session"});
    hub.Publish(MakeReport("drone-1", "one"));
    hub.Publish(MakeReport("drone-2", "two"));

    auto queue = std::make_shared<ReportQueue>();
    const ReportWatchToken token = hub.Watch("drone-2", 1, queue);
    hub.Publish(MakeReport("drone-2", "three"));

    swarmkit::v1::AgentReport replayed;
    swarmkit::v1::AgentReport live;
    REQUIRE(queue->Pop(&replayed, std::chrono::milliseconds{10}));
    REQUIRE(queue->Pop(&live, std::chrono::milliseconds{10}));
    CHECK(replayed.sequence() == 2);
    CHECK(live.sequence() == 3);
    hub.Unwatch(token);
}

}  // namespace
}  // namespace swarmkit::agent::internal
