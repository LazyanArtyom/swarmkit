// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarmkit/core/clock_quality.h"
#include "swarmkit/core/evidence_record.h"
#include "swarmkit/core/evidence_store.h"
#include "swarmkit/core/state_acceptance_certificate.h"
#include "swarmkit/core/state_acceptance_engine.h"
#include "swarmkit/core/state_acceptance_verifier.h"
#include "swarmkit/core/state_quality_contract.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::experiment {

/// Scenario fault condition (§21.1).
enum class ScenarioFaultKind : std::uint8_t {
    kNormal,
    kNetworkDelay,
    kNetworkReorder,
    kPacketLoss,
    kClockOffsetDrift,
    kEstimatorDegradation,
    kHighSpeedMotion,
    kAgentRestartDelayedPackets,
    kFrameMismatch,
};

[[nodiscard]] std::string ScenarioFaultKindToString(ScenarioFaultKind kind);

/// Evaluation baseline / method (§21.1).
enum class EvaluationMethod : std::uint8_t {
    /// Baseline 1: Newest received value for each field.
    kReceiveLatest,
    /// Baseline 2: Timestamp-aligned proximity + age rejection.
    kTimestampAlignedAge,
    /// Proposed: Full common-time State-Quality Contract acceptance.
    kProposedStateAcceptance,
};

[[nodiscard]] std::string EvaluationMethodToString(EvaluationMethod method);

/// Ground-truth physical vehicle state at a given physical reference time.
struct GroundTruthState {
    std::string drone_id;
    double physical_time_ms{};
    std::array<double, 3> position{};   // WGS84 lat, lon, alt (or NED)
    std::array<float, 3> velocity{};    // NED vx, vy, vz
    core::CoordinateFrame position_frame{core::CoordinateFrame::kWgs84};
    core::CoordinateFrame velocity_frame{core::CoordinateFrame::kLocalNed};
    bool healthy{true};
    std::string session_id;
};

/// Method snapshot evaluation outcome for one evaluation query.
struct MethodEvaluationOutcome {
    bool accepted{false};
    /// Per-agent estimated positions when accepted.
    std::unordered_map<std::string, std::array<double, 3>> estimated_positions;
    /// Per-agent estimated position enclosures (radius in meters) when accepted.
    std::unordered_map<std::string, double> position_enclosures;
    /// Ground-truth validity: whether accepted state satisfied physical truth.
    bool ground_truth_valid{false};
    /// Rejection reason string if rejected.
    std::string rejection_reason;
    /// Certificate if produced by proposed method.
    std::optional<core::StateAcceptanceCertificate> certificate;
};

/// Aggregated metrics for one evaluation method (§21.2, §23 Table II, §25 Table III).
struct MethodMetrics {
    EvaluationMethod method{EvaluationMethod::kReceiveLatest};
    std::size_t total_requests{};
    std::size_t accepted_count{};
    std::size_t invalid_accepted_count{};
    std::size_t deterministic_enclosures_tested{};
    std::size_t containment_failures{};

    /// False-valid rate: FV = N_{accepted and invalid} / N_{accepted}
    [[nodiscard]] double FalseValidRate() const {
        if (accepted_count == 0) return 0.0;
        return static_cast<double>(invalid_accepted_count) / static_cast<double>(accepted_count);
    }

    /// Availability: Availability = N_{accepted} / N_{requests}
    [[nodiscard]] double Availability() const {
        if (total_requests == 0) return 0.0;
        return static_cast<double>(accepted_count) / static_cast<double>(total_requests);
    }

    /// Unsafe acceptance per request: UAR = N_{accepted and invalid} / N_{requests}
    [[nodiscard]] double UnsafeAcceptancePerRequest() const {
        if (total_requests == 0) return 0.0;
        return static_cast<double>(invalid_accepted_count) / static_cast<double>(total_requests);
    }

    /// Containment failure rate: CF = N_{truth outside enclosure} / N_{enclosures}
    [[nodiscard]] double ContainmentFailureRate() const {
        if (deterministic_enclosures_tested == 0) return 0.0;
        return static_cast<double>(containment_failures) / static_cast<double>(deterministic_enclosures_tested);
    }
};

/// Soundness and replay metrics for the proposed method (§25 Table III).
struct SoundnessAndReplayMetrics {
    std::size_t enclosures_tested{};
    std::size_t containment_failures{};
    std::size_t replayed_decisions{};
    std::size_t verifier_agreements{};
    std::size_t tampered_certificates_tested{};
    std::size_t tampered_certificates_rejected{};
    double p95_latency_ms{};
    std::size_t median_certificate_size_bytes{};

    [[nodiscard]] double VerifierAgreementRate() const {
        if (replayed_decisions == 0) return 1.0;
        return static_cast<double>(verifier_agreements) / static_cast<double>(replayed_decisions);
    }
};

/// Complete paired-trace experiment results.
struct ExperimentResults {
    std::unordered_map<uint8_t, MethodMetrics> method_metrics;
    SoundnessAndReplayMetrics soundness_metrics;
    std::vector<double> latencies_ms;
    std::vector<std::size_t> certificate_sizes_bytes;

    [[nodiscard]] std::string FormatTableII() const;
    [[nodiscard]] std::string FormatTableIII() const;
    [[nodiscard]] std::string ToJson() const;
};

/// Configuration for paired-trace scenario generation.
struct ScenarioConfig {
    std::uint64_t seed{42};
    std::vector<std::string> agent_ids{"uav-1", "uav-2", "uav-3"};
    std::vector<ScenarioFaultKind> fault_scenarios{
        ScenarioFaultKind::kNormal,
        ScenarioFaultKind::kNetworkDelay,
        ScenarioFaultKind::kNetworkReorder,
        ScenarioFaultKind::kPacketLoss,
        ScenarioFaultKind::kClockOffsetDrift,
        ScenarioFaultKind::kEstimatorDegradation,
        ScenarioFaultKind::kHighSpeedMotion,
        ScenarioFaultKind::kAgentRestartDelayedPackets,
        ScenarioFaultKind::kFrameMismatch,
    };
    std::size_t steps_per_scenario{50};
    double step_dt_ms{100.0};  // 10 Hz
    double max_speed_mps{10.0};
    double max_age_ms{300.0};
    double max_clock_unc_ms{10.0};
    double max_pos_unc_m{3.0};
};

/// Evaluator baseline implementations.
class BaselineEvaluator {
   public:
    /// Evaluate Baseline 1: Receive-latest.
    [[nodiscard]] static MethodEvaluationOutcome EvaluateReceiveLatest(
        const core::StateQualityContract& contract,
        double evaluation_time_ms,
        const core::EvidenceStore& store,
        const std::unordered_map<std::string, GroundTruthState>& truth);

    /// Evaluate Baseline 2: Timestamp-aligned + age.
    [[nodiscard]] static MethodEvaluationOutcome EvaluateTimestampAlignedAge(
        const core::StateQualityContract& contract,
        double evaluation_time_ms,
        const core::EvidenceStore& store,
        double max_age_ms,
        const std::unordered_map<std::string, GroundTruthState>& truth);

    /// Evaluate Proposed: StateAcceptanceEngine + StateQualityContract.
    [[nodiscard]] static MethodEvaluationOutcome EvaluateProposed(
        const core::StateAcceptanceEngine& engine,
        const core::StateQualityContract& contract,
        double evaluation_time_ms,
        const core::EvidenceStore& store,
        const std::unordered_map<std::string, core::ClockQualityState>& clock_states,
        const std::unordered_map<std::string, GroundTruthState>& truth);
};

/// Paired-Trace Experiment Runner (§21, §24).
class StateAcceptanceExperimentRunner {
   public:
    explicit StateAcceptanceExperimentRunner(ScenarioConfig config = {});

    /// Run full paired-trace evaluation matrix and compute Tables II and III.
    [[nodiscard]] ExperimentResults Run();

   private:
    ScenarioConfig config_;
};

}  // namespace swarmkit::experiment
