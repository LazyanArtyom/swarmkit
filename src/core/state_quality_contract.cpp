// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/state_quality_contract.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

#include "sha256.h"

namespace swarmkit::core {

std::string ComputeContractHash(const StateQualityContract& contract) {
    // Serialize all configuration fields into a deterministic string
    // for hashing.  This ensures certificate binding captures the
    // exact contract semantics.
    std::ostringstream oss;
    oss.imbue(std::locale::classic());
    oss << std::setprecision(std::numeric_limits<double>::max_digits10);
    oss << "contract_id:" << contract.contract_id << "\n";
    oss << "schema_version:" << contract.schema_version << "\n";
    oss << "content_version:" << contract.content_version << "\n";

    oss << "required_fields:";
    for (const auto f : contract.required_fields) {
        oss << static_cast<int>(f) << ",";
    }
    oss << "\n";

    if (contract.max_evidence_age_ms.has_value())
        oss << "max_evidence_age_ms:" << *contract.max_evidence_age_ms << "\n";
    if (contract.max_clock_uncertainty_ms.has_value())
        oss << "max_clock_uncertainty_ms:" << *contract.max_clock_uncertainty_ms << "\n";
    if (contract.max_position_uncertainty_m.has_value())
        oss << "max_position_uncertainty_m:" << *contract.max_position_uncertainty_m << "\n";
    if (contract.max_velocity_uncertainty_mps.has_value())
        oss << "max_velocity_uncertainty_mps:" << *contract.max_velocity_uncertainty_mps << "\n";

    oss << "require_estimator_position_ok:" << contract.require_estimator_position_ok << "\n";
    oss << "require_estimator_velocity_ok:" << contract.require_estimator_velocity_ok << "\n";
    oss << "require_estimator_healthy:" << contract.require_estimator_healthy << "\n";
    oss << "min_gps_quality:" << static_cast<int>(contract.min_gps_quality) << "\n";

    oss << "required_position_frame:" << static_cast<int>(contract.required_position_frame) << "\n";
    oss << "required_velocity_frame:" << static_cast<int>(contract.required_velocity_frame) << "\n";
    oss << "require_current_epoch:" << contract.require_current_epoch << "\n";
    oss << "require_current_mission:" << contract.require_current_mission << "\n";
    oss << "required_mission_id:" << contract.required_mission_id << "\n";
    oss << "required_mission_revision:" << contract.required_mission_revision << "\n";

    oss << "required_agents:";
    // Sort for deterministic ordering.
    std::vector<std::string> sorted_agents(contract.required_agents.begin(),
                                           contract.required_agents.end());
    std::sort(sorted_agents.begin(), sorted_agents.end());
    for (const auto& a : sorted_agents) {
        oss << a << ",";
    }
    oss << "\n";

    oss << "completeness:" << static_cast<int>(contract.completeness) << "\n";
    oss << "min_required_agents:" << contract.min_required_agents << "\n";
    oss << "require_deterministic_bounds:" << contract.require_deterministic_bounds << "\n";
    oss << "propagation_model_id:" << contract.propagation_model_id << "\n";
    oss << "propagation_model_version:" << contract.propagation_model_version << "\n";
    oss << "max_horizontal_speed_mps:" << contract.max_horizontal_speed_mps << "\n";
    oss << "max_vertical_speed_mps:" << contract.max_vertical_speed_mps << "\n";

    const std::string content = oss.str();
    return swarmkit::core::internal::Sha256Hex(content);
}

core::Result ValidateStateQualityContract(const StateQualityContract& contract) {
    if (contract.contract_id.empty()) {
        return core::Result::Rejected("Contract validation error: contract_id cannot be empty");
    }
    if (contract.required_fields.empty()) {
        return core::Result::Rejected("Contract validation error: required_fields cannot be empty");
    }
    if (contract.required_agents.empty()) {
        return core::Result::Rejected("Contract validation error: required_agents cannot be empty");
    }
    if (contract.completeness == CompletenessRule::kMinimumCount) {
        if (contract.min_required_agents < 1 ||
            contract.min_required_agents > contract.required_agents.size()) {
            return core::Result::Rejected(
                "Contract validation error: min_required_agents out of range [1, required_agents.size()]");
        }
    }
    if (contract.max_evidence_age_ms.has_value()) {
        if (!std::isfinite(*contract.max_evidence_age_ms) || *contract.max_evidence_age_ms < 0.0) {
            return core::Result::Rejected(
                "Contract validation error: max_evidence_age_ms must be finite and non-negative");
        }
    }
    if (contract.max_clock_uncertainty_ms.has_value()) {
        if (!std::isfinite(*contract.max_clock_uncertainty_ms) || *contract.max_clock_uncertainty_ms < 0.0) {
            return core::Result::Rejected(
                "Contract validation error: max_clock_uncertainty_ms must be finite and non-negative");
        }
    }
    if (contract.max_position_uncertainty_m.has_value()) {
        if (!std::isfinite(*contract.max_position_uncertainty_m) || *contract.max_position_uncertainty_m < 0.0) {
            return core::Result::Rejected(
                "Contract validation error: max_position_uncertainty_m must be finite and non-negative");
        }
    }
    if (contract.max_velocity_uncertainty_mps.has_value()) {
        if (!std::isfinite(*contract.max_velocity_uncertainty_mps) || *contract.max_velocity_uncertainty_mps < 0.0) {
            return core::Result::Rejected(
                "Contract validation error: max_velocity_uncertainty_mps must be finite and non-negative");
        }
    }
    if (!std::isfinite(contract.max_horizontal_speed_mps) || contract.max_horizontal_speed_mps < 0.0f) {
        return core::Result::Rejected(
            "Contract validation error: max_horizontal_speed_mps must be finite and non-negative");
    }
    if (!std::isfinite(contract.max_vertical_speed_mps) || contract.max_vertical_speed_mps < 0.0f) {
        return core::Result::Rejected(
            "Contract validation error: max_vertical_speed_mps must be finite and non-negative");
    }
    if (contract.propagation_model_id.empty()) {
        return core::Result::Rejected(
            "Contract validation error: propagation_model_id cannot be empty");
    }
    return core::Result::Success();
}

}  // namespace swarmkit::core
