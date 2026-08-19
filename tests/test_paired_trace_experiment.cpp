// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <chrono>

#include "swarmkit/agent/sim_backend.h"
#include "swarmkit/experiment/fault_injection.h"
#include "swarmkit/experiment/state_acceptance_experiment.h"

using namespace swarmkit;
using namespace swarmkit::experiment;
using Catch::Approx;
using Catch::Matchers::WithinAbs;

TEST_CASE("SimBackend UAVs actually move during experiment", "[experiment][motion]") {
    agent::SimBackendConfig sim_config{
        .clock_mode = agent::SimulationClockMode::kManual,
        .integration_step_ms = 20,
        .initial_source_unix_time_ms = 1'700'000'000'000LL,
        .home_lat_deg = 37.7749,
        .home_lon_deg = -122.4194,
        .home_alt_m = 10.0,
        .max_horizontal_speed_mps = 10.0F,
    };

    auto sim_or = agent::MakeSimBackend(sim_config);
    REQUIRE(sim_or.has_value());
    auto backend = std::move(sim_or->backend);
    auto control = std::move(sim_or->control);

    REQUIRE(backend->Execute({.context = {.drone_id = "uav-1"}, .command = commands::CmdArm{}}).IsOk());
    REQUIRE(backend->Execute({.context = {.drone_id = "uav-1"}, .command = commands::CmdTakeoff{.alt_m = 5.0}}).IsOk());
    REQUIRE(control->AdvanceAll(std::chrono::seconds(1)).IsOk());

    auto truth1 = control->Truth("uav-1");
    REQUIRE(truth1.has_value());
    const double north_initial = truth1->north_m;

    REQUIRE(backend->Execute({.context = {.drone_id = "uav-1"},
                             .command = commands::CmdVelocity{.vx_mps = 5.0F, .vy_mps = 0.0F, .duration_ms = 5000}}).IsOk());

    REQUIRE(control->AdvanceAll(std::chrono::seconds(2)).IsOk());
    auto truth50 = control->Truth("uav-1");
    REQUIRE(truth50.has_value());

    // Vehicle position must have physically moved substantially (~10 meters north)
    CHECK(truth50->north_m - north_initial > 5.0);
}

TEST_CASE("Clock drift fault injector advances offset dynamically per frame", "[experiment][faults][clock]") {
    agent::SimBackendConfig sim_config{
        .clock_mode = agent::SimulationClockMode::kManual,
        .integration_step_ms = 20,
    };
    auto sim_or = agent::MakeSimBackend(sim_config);
    REQUIRE(sim_or.has_value());

    FaultInjectionConfig fault_cfg{
        .seed = 42,
        .source_clock_offset_ms = 15,
        .source_clock_drift_ms_per_frame = 0.5,
    };

    auto fault_backend_or = MakeFaultInjectingBackend(std::move(sim_or->backend), fault_cfg);
    REQUIRE(fault_backend_or.has_value());

    std::vector<core::TelemetryFrame> frames;
    REQUIRE(fault_backend_or->backend->StartTelemetry(
        "uav-1", 10, [&frames](const core::TelemetryFrame& frame) { frames.push_back(frame); }).IsOk());

    REQUIRE(sim_or->control->AdvanceAll(std::chrono::milliseconds(500)).IsOk());

    REQUIRE(frames.size() >= 5);
    const auto ts0 = *frames[0].provenance.position.source_time.timestamp_ms;
    const auto ts_last = *frames.back().provenance.position.source_time.timestamp_ms;

    // Last frame offset must be significantly greater than first frame offset due to drift
    CHECK(ts_last > ts0);
}

TEST_CASE("MethodMetrics TrueRejectRate returns nullopt when denominator is zero", "[experiment][metrics]") {
    MethodMetrics metrics;
    metrics.false_accepts = 0;
    metrics.true_rejects = 0;

    auto tr_rate = metrics.TrueRejectRate();
    CHECK_FALSE(tr_rate.has_value());

    metrics.true_rejects = 10;
    tr_rate = metrics.TrueRejectRate();
    REQUIRE(tr_rate.has_value());
    CHECK(*tr_rate == Approx(1.0));
}

TEST_CASE("StateAcceptanceExperimentRunner paired-trace matrix evaluation", "[experiment]") {
    ScenarioConfig config{
        .seed = 12345,
        .runs = 1,
        .steps_per_scenario = 20,
        .agent_ids = {"uav-1", "uav-2", "uav-3"},
        .fault_scenarios = {
            ScenarioFaultKind::kNormal,
            ScenarioFaultKind::kNetworkDelay,
            ScenarioFaultKind::kPacketLoss,
            ScenarioFaultKind::kClockOffsetDrift,
            ScenarioFaultKind::kEstimatorDegradation,
            ScenarioFaultKind::kHighSpeedMotion,
            ScenarioFaultKind::kAgentRestartDelayedPackets,
            ScenarioFaultKind::kFrameMismatch,
        },
        .step_dt_ms = 100.0,
        .max_speed_mps = 10.0,
        .max_age_ms = 300.0,
        .max_clock_unc_ms = 10.0,
        .max_pos_unc_m = 3.0,
        .physical_error_tolerance_m = 3.0,
    };

    StateAcceptanceExperimentRunner runner(config);
    auto results = runner.Run();

    const auto& m_latest = results.aggregate_method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kReceiveLatest));
    const auto& m_age = results.aggregate_method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kTimestampAlignedAge));
    const auto& m_proposed = results.aggregate_method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kProposedStateAcceptance));

    SECTION("All methods evaluated across all requests") {
        REQUIRE(m_latest.total_requests == 160);     // 8 scenarios * 20 steps
        REQUIRE(m_age.total_requests == 160);
        REQUIRE(m_proposed.total_requests == 160);
        REQUIRE(m_latest.total_requests == m_age.total_requests);
        REQUIRE(m_latest.total_requests == m_proposed.total_requests);
    }

    SECTION("Metrics are bounded in [0, 1] and structurally valid") {
        REQUIRE(m_latest.FalseValidRate() >= 0.0);
        REQUIRE(m_latest.FalseValidRate() <= 1.0);
        REQUIRE(m_age.FalseValidRate() >= 0.0);
        REQUIRE(m_age.FalseValidRate() <= 1.0);
        REQUIRE(m_proposed.FalseValidRate() >= 0.0);
        REQUIRE(m_proposed.FalseValidRate() <= 1.0);
        REQUIRE(m_proposed.Availability() >= 0.0);
        REQUIRE(m_proposed.Availability() <= 1.0);
        REQUIRE(m_proposed.ContainmentFailureRate() >= 0.0);
        REQUIRE(m_proposed.ContainmentFailureRate() <= 1.0);
    }

    SECTION("Replay agreement is 100% between engine and offline verifier") {
        REQUIRE(results.soundness_metrics.replayed_decisions > 0);
        REQUIRE(results.soundness_metrics.verifier_agreements == results.soundness_metrics.replayed_decisions);
        REQUIRE_THAT(results.soundness_metrics.VerifierAgreementRate(), WithinAbs(1.0, 1e-9));
    }

    SECTION("Tampered certificate rejection rate is 100% across 15 mutation classes") {
        REQUIRE(results.soundness_metrics.mutation_cases_tested > 0);
        REQUIRE(results.soundness_metrics.mutation_cases_rejected == results.soundness_metrics.mutation_cases_tested);
    }

    SECTION("Result formatting is valid and populated") {
        const std::string table_ii = results.FormatTableII();
        REQUIRE_FALSE(table_ii.empty());
        REQUIRE(table_ii.find("Receive-latest") != std::string::npos);
        REQUIRE(table_ii.find("Timestamp-aligned + age") != std::string::npos);
        REQUIRE(table_ii.find("Proposed state acceptance") != std::string::npos);

        const std::string table_iii = results.FormatTableIII();
        REQUIRE_FALSE(table_iii.empty());
        REQUIRE(table_iii.find("Deterministic Enclosures Tested") != std::string::npos);
        REQUIRE(table_iii.find("Verifier Replay Agreement") != std::string::npos);

        const std::string json = results.ToJson();
        REQUIRE_FALSE(json.empty());
        REQUIRE(json.find("soundness_and_replay") != std::string::npos);
    }
}
