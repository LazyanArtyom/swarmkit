// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>

#include "../src/agent/report_hub.h"

namespace swarmkit::agent::internal {
namespace {

namespace fs = std::filesystem;

[[nodiscard]] fs::path UniqueReportPath(const std::string& name) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return fs::temp_directory_path() / (name + "-" + std::to_string(now) + ".jsonl");
}

[[nodiscard]] swarmkit::v1::AgentReport MakeReport(std::string drone_id, std::string message) {
    swarmkit::v1::AgentReport report;
    report.set_drone_id(std::move(drone_id));
    report.set_type(swarmkit::v1::GOAL_REPORT);
    report.set_severity(swarmkit::v1::REPORT_INFO);
    report.set_message(std::move(message));
    return report;
}

void RemoveReportFiles(const fs::path& log_file) {
    std::error_code error;
    fs::remove(log_file, error);
    fs::remove(log_file.string() + ".seq", error);
    fs::remove(log_file.string() + ".seq.tmp", error);
    for (int index = 1; index <= 5; ++index) {
        fs::remove(log_file.string() + "." + std::to_string(index), error);
    }
}

TEST_CASE("ReportHub persists sequence and replays reports after restart", "[agent][reports]") {
    const fs::path log_file = UniqueReportPath("swarmkit-report-hub-replay");
    RemoveReportFiles(log_file);

    ReportHubOptions options;
    options.report_log_file = log_file.string();
    options.sequence_state_file = log_file.string() + ".seq";
    options.max_in_memory_backlog = 1;
    options.flush_each_write = true;

    {
        ReportHub hub(options);
        hub.Publish(MakeReport("drone-1", "first"));
        hub.Publish(MakeReport("drone-2", "second"));
    }

    ReportHub restarted(options);
    auto replay_queue = std::make_shared<ReportQueue>();
    const ReportWatchToken token = restarted.Watch("drone-2", 1, replay_queue);

    swarmkit::v1::AgentReport replayed;
    REQUIRE(replay_queue->Pop(&replayed, std::chrono::milliseconds{100}));
    CHECK(replayed.sequence() == 2);
    CHECK(replayed.drone_id() == "drone-2");
    CHECK(replayed.message() == "second");
    restarted.Unwatch(token);

    auto live_queue = std::make_shared<ReportQueue>();
    const ReportWatchToken live_token = restarted.Watch("all", 2, live_queue);
    restarted.Publish(MakeReport("drone-1", "third"));

    swarmkit::v1::AgentReport live;
    REQUIRE(live_queue->Pop(&live, std::chrono::milliseconds{100}));
    CHECK(live.sequence() == 3);
    CHECK(live.message() == "third");
    restarted.Unwatch(live_token);

    RemoveReportFiles(log_file);
}

TEST_CASE("ReportHub rotates JSONL logs while preserving sequence state", "[agent][reports]") {
    const fs::path log_file = UniqueReportPath("swarmkit-report-hub-rotate");
    RemoveReportFiles(log_file);

    ReportHubOptions options;
    options.report_log_file = log_file.string();
    options.sequence_state_file = log_file.string() + ".seq";
    options.max_in_memory_backlog = 0;
    options.max_log_file_size_bytes = 180;
    options.max_log_files = 1;
    options.flush_each_write = true;

    {
        ReportHub hub(options);
        for (int index = 0; index < 6; ++index) {
            hub.Publish(MakeReport("drone-1", "rotated report " + std::to_string(index)));
        }
    }

    CHECK(fs::exists(log_file));
    CHECK(fs::exists(log_file.string() + ".1"));
    CHECK(fs::exists(options.sequence_state_file));

    ReportHub restarted(options);
    auto queue = std::make_shared<ReportQueue>();
    const ReportWatchToken token = restarted.Watch("all", 6, queue);
    restarted.Publish(MakeReport("drone-1", "after restart"));

    swarmkit::v1::AgentReport report;
    REQUIRE(queue->Pop(&report, std::chrono::milliseconds{100}));
    CHECK(report.sequence() == 7);
    CHECK(report.message() == "after restart");
    restarted.Unwatch(token);

    RemoveReportFiles(log_file);
}

}  // namespace
}  // namespace swarmkit::agent::internal
