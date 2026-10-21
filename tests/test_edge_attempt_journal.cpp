// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "../src/agent/edge_attempt_journal.h"

namespace {

[[nodiscard]] std::filesystem::path JournalPath(const char* scenario) {
    return std::filesystem::temp_directory_path() /
           (std::string("swarmkit-vecr-journal-") + scenario + "-" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
}

[[nodiscard]] swarmkit::agent::internal::EdgeAttemptJournal::Entry Record(
    swarmkit::v1::EdgePhase phase, bool accepted) {
    swarmkit::agent::internal::EdgeAttemptJournal::Entry entry;
    entry.operation.set_drone_id("drone-1");
    entry.operation.set_attempt_id("attempt-1");
    entry.operation.set_mission_id("mission");
    entry.operation.set_edge_id("A-B");
    entry.client_id = "client-1";
    entry.report.set_drone_id("drone-1");
    entry.report.set_attempt_id("attempt-1");
    entry.report.set_phase(phase);
    entry.report.set_reason("recorded outcome");
    entry.dispatch_accepted = accepted;
    entry.physical_navigation_unresolved =
        phase != swarmkit::v1::EDGE_FULFILLED && phase != swarmkit::v1::EDGE_INTERRUPTED;
    return entry;
}

}  // namespace

TEST_CASE("VECR attempt journal preserves terminal outcome and unknown restart state",
          "[vecr][journal]") {
    const auto path = JournalPath("restart");
    {
        swarmkit::agent::internal::EdgeAttemptJournal journal(path.string());
        REQUIRE(journal.Ready().IsOk());
        REQUIRE(journal.Append(Record(swarmkit::v1::EDGE_UNKNOWN, false)).IsOk());
        const auto locked = swarmkit::agent::internal::EdgeAttemptJournal(path.string());
        CHECK_FALSE(locked.Ready().IsOk());
    }
    {
        swarmkit::agent::internal::EdgeAttemptJournal journal(path.string());
        REQUIRE(journal.Ready().IsOk());
        const auto unknown = journal.Get("drone-1", "attempt-1");
        REQUIRE(unknown);
        CHECK(unknown->report.phase() == swarmkit::v1::EDGE_UNKNOWN);
        CHECK_FALSE(unknown->dispatch_accepted);
        CHECK(unknown->report.reason().find("agent restarted") != std::string::npos);
        REQUIRE(journal.Unresolved("drone-1"));
        REQUIRE(journal.Append(Record(swarmkit::v1::EDGE_FULFILLED, true)).IsOk());
        CHECK_FALSE(journal.Unresolved("drone-1"));
    }
    {
        swarmkit::agent::internal::EdgeAttemptJournal journal(path.string());
        REQUIRE(journal.Ready().IsOk());
        const auto terminal = journal.Get("drone-1", "attempt-1");
        REQUIRE(terminal);
        CHECK(terminal->report.phase() == swarmkit::v1::EDGE_FULFILLED);
        CHECK(terminal->dispatch_accepted);
        CHECK_FALSE(journal.Unresolved("drone-1"));
        CHECK_FALSE(journal.Append(Record(swarmkit::v1::EDGE_INTERRUPTED, true)).IsOk());
        const auto retained = journal.Get("drone-1", "attempt-1");
        REQUIRE(retained);
        CHECK(retained->report.phase() == swarmkit::v1::EDGE_FULFILLED);
        auto conflicting = Record(swarmkit::v1::EDGE_FULFILLED, true);
        conflicting.operation.set_edge_id("different-edge");
        CHECK_FALSE(journal.Append(conflicting).IsOk());
    }
    std::filesystem::remove(path);
}

TEST_CASE("VECR attempt journal fails closed on a truncated record", "[vecr][journal]") {
    const auto path = JournalPath("truncated");
    {
        swarmkit::agent::internal::EdgeAttemptJournal journal(path.string());
        REQUIRE(journal.Ready().IsOk());
        REQUIRE(journal.Append(Record(swarmkit::v1::EDGE_MOVING, true)).IsOk());
    }
    std::filesystem::resize_file(path, std::filesystem::file_size(path) - 3);
    {
        swarmkit::agent::internal::EdgeAttemptJournal journal(path.string());
        CHECK_FALSE(journal.Ready().IsOk());
        CHECK_FALSE(journal.Append(Record(swarmkit::v1::EDGE_MOVING, true)).IsOk());
    }
    std::filesystem::remove(path);
}

TEST_CASE("VECR journal storage faults never acknowledge a terminal commit", "[vecr][journal]") {
    using Journal = swarmkit::agent::internal::EdgeAttemptJournal;
    for (const auto fault :
         {Journal::Fault::kNoSpace, Journal::Fault::kQuota, Journal::Fault::kReadOnly,
          Journal::Fault::kPartialWrite, Journal::Fault::kFsync}) {
        const auto path = JournalPath("write-fault");
        {
            Journal journal(path.string(), [fault](const Journal::Entry& entry) {
                return entry.report.phase() == swarmkit::v1::EDGE_FULFILLED ? fault
                                                                            : Journal::Fault::kNone;
            });
            REQUIRE(journal.Ready().IsOk());
            REQUIRE(journal.Append(Record(swarmkit::v1::EDGE_MOVING, true)).IsOk());
            CHECK_FALSE(journal.Append(Record(swarmkit::v1::EDGE_FULFILLED, true)).IsOk());
            CHECK_FALSE(journal.Ready().IsOk());
            CHECK_FALSE(journal.Append(Record(swarmkit::v1::EDGE_FULFILLED, true)).IsOk());
        }
        {
            Journal restarted(path.string());
            if (fault == Journal::Fault::kPartialWrite) {
                CHECK_FALSE(restarted.Ready().IsOk());
            } else {
                REQUIRE(restarted.Ready().IsOk());
                const auto recovered = restarted.Get("drone-1", "attempt-1");
                REQUIRE(recovered);
                CHECK(recovered->report.phase() == (fault == Journal::Fault::kFsync
                                                        ? swarmkit::v1::EDGE_FULFILLED
                                                        : swarmkit::v1::EDGE_UNKNOWN));
            }
        }
        std::filesystem::remove(path);
    }
}

TEST_CASE("Interrupted obligation retains an unresolved physical-navigation gate",
          "[vecr][journal]") {
    using Journal = swarmkit::agent::internal::EdgeAttemptJournal;
    const auto path = JournalPath("interrupted-navigation");
    auto interrupted = Record(swarmkit::v1::EDGE_INTERRUPTED, true);
    interrupted.physical_navigation_unresolved = true;
    {
        Journal journal(path.string());
        REQUIRE(journal.Ready().IsOk());
        REQUIRE(journal.Append(Record(swarmkit::v1::EDGE_MOVING, true)).IsOk());
        REQUIRE(journal.Append(interrupted).IsOk());
        REQUIRE(journal.Unresolved("drone-1"));
    }
    {
        Journal journal(path.string());
        REQUIRE(journal.Ready().IsOk());
        REQUIRE(journal.Unresolved("drone-1"));
        interrupted.physical_navigation_unresolved = false;
        interrupted.report.set_reason("physical simulator hold confirmed");
        REQUIRE(journal.Append(interrupted).IsOk());
        CHECK_FALSE(journal.Unresolved("drone-1"));
    }
    std::filesystem::remove(path);
}
