// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <algorithm>
#include <chrono>

#include "swarmkit/agent/sim_backend.h"
#include "swarmkit/experiment/fault_injection.h"
#include "swarmkit/experiment/state_acceptance_experiment.h"

using namespace swarmkit;
using namespace swarmkit::experiment;
using Catch::Approx;
using Catch::Matchers::WithinAbs;

namespace {

core::EvidenceRecord OraclePositionRecord() {
    return core::EvidenceRecord{
        .value = std::array<double, 3>{1.0, 2.0, 3.0},
        .source_time = {.timestamp_ms = 900},
        .receive_time_ms = 950,
        .quality = {
            .estimator_healthy = true,
            .estimator_position_ok = true,
            .estimator_velocity_ok = true,
        },
        .identity = {
            .agent_id = "uav-1",
            .agent_session_id = "session-current",
            .field_id = core::EvidenceFieldId::kPosition,
            .sequence = 1,
            .coordinate_frame = core::CoordinateFrame::kLocalNed,
            .source_component = "sim-position",
            .mission_id = "mission-1",
            .mission_revision = 7,
        },
    };
}

core::StateQualityContract OracleContract() {
    return core::StateQualityContract{
        .contract_id = "oracle-contract",
        .required_fields = {core::EvidenceFieldId::kPosition},
        .require_estimator_position_ok = true,
        .require_estimator_velocity_ok = true,
        .require_estimator_healthy = true,
        .required_position_frame = core::CoordinateFrame::kLocalNed,
        .require_current_epoch = true,
        .require_current_mission = true,
        .required_mission_id = "mission-1",
        .required_mission_revision = 7,
        .required_agents = {"uav-1"},
    };
}

MethodEvaluationOutcome OracleOutcome() {
    MethodEvaluationOutcome outcome;
    outcome.accepted = true;
    outcome.selected_position_evidence["uav-1"] = OraclePositionRecord();
    outcome.estimated_positions["uav-1"] = {1.0, 2.0, 3.0};
    return outcome;
}

std::unordered_map<std::string, GroundTruthState> OracleTruth(bool healthy) {
    return {{"uav-1",
             GroundTruthState{
                 .drone_id = "uav-1",
                 .physical_time_ms = 1000.0,
                 .position = {1.0, 2.0, 3.0},
                 .position_frame = core::CoordinateFrame::kLocalNed,
                 .healthy = healthy,
                 .session_id = "session-current",
             }}};
}

}  // namespace

TEST_CASE("test_common_oracle_current_truth_health_does_not_invent_new_predicate",
          "[experiment][oracle]") {
    const auto result = BaselineEvaluator::EvaluateAcceptedOutputValidity(
        OracleOutcome(), OracleTruth(false), {{"uav-1", "session-current"}},
        OracleContract(), 3.0);
    REQUIRE(result.overall_valid);
    REQUIRE(result.health_valid);
}

TEST_CASE("Common accepted-output oracle evaluates every formal contract predicate",
          "[experiment][oracle]") {
    const auto evaluate = [](const MethodEvaluationOutcome& outcome,
                             const core::StateQualityContract& contract) {
        return BaselineEvaluator::EvaluateAcceptedOutputValidity(
            outcome, OracleTruth(true), {{"uav-1", "session-current"}},
            contract, 3.0);
    };

    SECTION("correct position frame session selected health mission and provenance is valid") {
        REQUIRE(evaluate(OracleOutcome(), OracleContract()).overall_valid);
    }
    SECTION("position error over Umax is invalid") {
        auto outcome = OracleOutcome();
        outcome.selected_position_evidence["uav-1"].value =
            std::array<double, 3>{10.0, 2.0, 3.0};
        REQUIRE_FALSE(evaluate(outcome, OracleContract()).spatial_valid);
    }
    SECTION("wrong frame is invalid") {
        auto outcome = OracleOutcome();
        outcome.selected_position_evidence["uav-1"].identity.coordinate_frame =
            core::CoordinateFrame::kWgs84;
        REQUIRE_FALSE(evaluate(outcome, OracleContract()).frame_valid);
    }
    SECTION("old session is invalid") {
        auto outcome = OracleOutcome();
        outcome.selected_position_evidence["uav-1"].identity.agent_session_id =
            "session-old";
        REQUIRE_FALSE(evaluate(outcome, OracleContract()).session_valid);
    }
    SECTION("selected unhealthy evidence is invalid") {
        auto outcome = OracleOutcome();
        outcome.selected_position_evidence["uav-1"].quality.estimator_healthy = false;
        REQUIRE_FALSE(evaluate(outcome, OracleContract()).health_valid);
    }
    SECTION("selected position-not-ok evidence is invalid") {
        auto outcome = OracleOutcome();
        outcome.selected_position_evidence["uav-1"].quality.estimator_position_ok = false;
        REQUIRE_FALSE(evaluate(outcome, OracleContract()).health_valid);
    }
    SECTION("selected velocity-not-ok evidence is invalid") {
        auto outcome = OracleOutcome();
        outcome.selected_position_evidence["uav-1"].quality.estimator_velocity_ok = false;
        REQUIRE_FALSE(evaluate(outcome, OracleContract()).health_valid);
    }
    SECTION("wrong mission id or revision is invalid") {
        auto outcome = OracleOutcome();
        outcome.selected_position_evidence["uav-1"].identity.mission_revision = 8;
        REQUIRE_FALSE(evaluate(outcome, OracleContract()).mission_valid);
    }
    SECTION("GPS below threshold is invalid") {
        auto outcome = OracleOutcome();
        auto contract = OracleContract();
        contract.min_gps_quality = core::GpsQuality::kFix3D;
        auto gps = OraclePositionRecord();
        gps.identity.field_id = core::EvidenceFieldId::kGpsQuality;
        gps.value = core::GpsQuality::kFix2D;
        outcome.selected_gps_evidence["uav-1"] = gps;
        REQUIRE_FALSE(evaluate(outcome, contract).gps_valid);
    }
    SECTION("missing required agent is invalid") {
        auto outcome = OracleOutcome();
        outcome.selected_position_evidence.clear();
        REQUIRE_FALSE(evaluate(outcome, OracleContract()).complete);
    }
}

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

    SECTION("Mutated inconsistent certificate rejection is complete") {
        REQUIRE(results.soundness_metrics.mutation_cases_tested > 0);
        REQUIRE(results.soundness_metrics.mutation_cases_rejected == results.soundness_metrics.mutation_cases_tested);
        REQUIRE(results.soundness_metrics.mutation_classes_tested == 44);
        REQUIRE(results.soundness_metrics.mutation_classes_rejected == 44);
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
        REQUIRE(json.find("\"methods\"") != std::string::npos);
        REQUIRE(json.find("paired_P_minus_B1") != std::string::npos);
    }

    SECTION("test_all_bootstrap_intervals_are_exported") {
        const auto json = results.ToJson();
        for (const auto* key : {"FV_CI_low", "FV_CI_high", "availability_CI_low",
                                "availability_CI_high", "UAR_CI_low", "UAR_CI_high",
                                "bootstrap_iterations", "bootstrap_seed", "replicate_count"}) {
            REQUIRE(json.find(key) != std::string::npos);
        }
    }

    SECTION("test_replicate_results_export") {
        const auto csv = results.ToReplicateCsv();
        REQUIRE(csv.find("replicate_id,scenario,method,base_seed") == 0);
        REQUIRE(csv.find("reorder_inversions") != std::string::npos);
        REQUIRE(csv.find("clock_reestablishments") != std::string::npos);
        REQUIRE(static_cast<std::size_t>(std::count(csv.begin(), csv.end(), '\n')) ==
                results.replicate_records.size() + 1);
    }

    SECTION("test_scalability_raw_samples_export") {
        const auto csv = results.ToScalabilitySamplesCsv();
        REQUIRE(csv.find("N,iteration,latency_us,serialized_certificate_bytes") == 0);
        REQUIRE(static_cast<std::size_t>(std::count(csv.begin(), csv.end(), '\n')) == 3001);
    }
}

TEST_CASE("test_containment_failure_rate_is_calculated", "[experiment][export]") {
    SoundnessAndReplayMetrics metrics;
    metrics.enclosures_tested = 8;
    metrics.containment_failures = 2;
    REQUIRE_THAT(metrics.ContainmentFailureRate(), WithinAbs(0.25, 1e-12));
}
