// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "swarmkit/experiment/state_acceptance_experiment.h"

using namespace swarmkit::experiment;
using Catch::Matchers::WithinAbs;

TEST_CASE("StateAcceptanceExperimentRunner paired-trace matrix evaluation", "[experiment]") {
    ScenarioConfig config{
        .seed = 12345,
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
        .steps_per_scenario = 20,
        .step_dt_ms = 100.0,
        .max_speed_mps = 10.0,
        .max_age_ms = 300.0,
        .max_clock_unc_ms = 10.0,
        .max_pos_unc_m = 3.0,
    };

    StateAcceptanceExperimentRunner runner(config);
    auto results = runner.Run();

    const auto& m_latest = results.method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kReceiveLatest));
    const auto& m_age = results.method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kTimestampAlignedAge));
    const auto& m_proposed = results.method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kProposedStateAcceptance));

    SECTION("All methods evaluated across all requests") {
        REQUIRE(m_latest.total_requests == 160);     // 8 scenarios * 20 steps
        REQUIRE(m_age.total_requests == 160);
        REQUIRE(m_proposed.total_requests == 160);
    }

    SECTION("Proposed state acceptance achieves zero containment failures") {
        REQUIRE(results.soundness_metrics.enclosures_tested > 0);
        REQUIRE(results.soundness_metrics.containment_failures == 0);
        REQUIRE_THAT(m_proposed.ContainmentFailureRate(), WithinAbs(0.0, 1e-9));
    }

    SECTION("Proposed method false-valid rate is superior to naive baselines") {
        // In the presence of delays, restarts, and estimator degradation:
        // Receive-latest exposes invalid/stale states frequently.
        REQUIRE(m_latest.FalseValidRate() > m_proposed.FalseValidRate());
        REQUIRE(m_proposed.FalseValidRate() == 0.0);
    }

    SECTION("Replay agreement is 100% between engine and verifier") {
        REQUIRE(results.soundness_metrics.replayed_decisions > 0);
        REQUIRE(results.soundness_metrics.verifier_agreements == results.soundness_metrics.replayed_decisions);
        REQUIRE_THAT(results.soundness_metrics.VerifierAgreementRate(), WithinAbs(1.0, 1e-9));
    }

    SECTION("Tampered certificate rejection rate is 100%") {
        REQUIRE(results.soundness_metrics.tampered_certificates_tested > 0);
        REQUIRE(results.soundness_metrics.tampered_certificates_rejected == results.soundness_metrics.tampered_certificates_tested);
    }

    SECTION("Result formatting is valid and populated") {
        const std::string table_ii = results.FormatTableII();
        REQUIRE_FALSE(table_ii.empty());
        REQUIRE(table_ii.find("Receive-latest") != std::string::npos);
        REQUIRE(table_ii.find("Timestamp-aligned + age") != std::string::npos);
        REQUIRE(table_ii.find("Proposed state acceptance") != std::string::npos);

        const std::string table_iii = results.FormatTableIII();
        REQUIRE_FALSE(table_iii.empty());
        REQUIRE(table_iii.find("Containment failures") != std::string::npos);
        REQUIRE(table_iii.find("Runtime/verifier agreement") != std::string::npos);

        const std::string json = results.ToJson();
        REQUIRE_FALSE(json.empty());
        REQUIRE(json.find("table_ii") != std::string::npos);
        REQUIRE(json.find("table_iii") != std::string::npos);
    }
}
