// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/state_acceptance_certificate.h"

#include <algorithm>
#include <sstream>

#include "sha256.h"

namespace swarmkit::core {

std::string ComputeCertificateHash(const StateAcceptanceCertificate& cert) {
    std::ostringstream oss;
    oss << "cert_id:" << cert.certificate_id << "\n";
    oss << "contract_id:" << cert.contract_id << "\n";
    oss << "contract_schema_version:" << cert.contract_schema_version << "\n";
    oss << "contract_content_version:" << cert.contract_content_version << "\n";
    oss << "contract_hash:" << cert.contract_hash << "\n";
    oss << "evaluation_time_ms:" << cert.evaluation_time_ms << "\n";

    // Evidence entries in deterministic order.
    for (const auto& entry : cert.evidence_entries) {
        oss << "evidence:" << entry.agent_id
            << ":" << static_cast<int>(entry.field)
            << ":" << entry.sequence
            << ":" << entry.agent_session_id
            << ":" << entry.source_component;
        if (entry.source_time_ms.has_value()) {
            oss << ":" << *entry.source_time_ms;
        }
        oss << ":" << entry.generation_interval.lower_ms
            << ":" << entry.generation_interval.upper_ms
            << ":" << entry.conservative_elapsed_ms
            << ":" << entry.clock_uncertainty_ms
            << ":" << entry.observation_uncertainty
            << ":" << entry.propagated_uncertainty
            << "\n";
    }

    oss << "max_clock_uncertainty_ms:" << cert.max_clock_uncertainty_ms << "\n";
    oss << "max_conservative_elapsed_ms:" << cert.max_conservative_elapsed_ms << "\n";
    oss << "max_propagated_position_uncertainty_m:"
        << cert.max_propagated_position_uncertainty_m << "\n";
    oss << "max_propagated_velocity_uncertainty_mps:"
        << cert.max_propagated_velocity_uncertainty_mps << "\n";

    oss << "model_id:" << cert.propagation_model_id << "\n";
    oss << "model_version:" << cert.propagation_model_version << "\n";
    oss << "max_horizontal_speed_mps:" << cert.max_horizontal_speed_mps << "\n";
    oss << "max_vertical_speed_mps:" << cert.max_vertical_speed_mps << "\n";

    oss << "accepted_agents:";
    for (const auto& a : cert.accepted_agents) {
        oss << a << ",";
    }
    oss << "\n";

    oss << "acceptance_semantics_version:" << cert.acceptance_semantics_version << "\n";
    oss << "produced_at_ms:" << cert.produced_at_ms << "\n";

    return internal::Sha256Hex(oss.str());
}

bool VerifyCertificateIntegrity(const StateAcceptanceCertificate& cert) {
    const std::string expected = ComputeCertificateHash(cert);
    return expected == cert.certificate_hash;
}

StateAcceptanceCertificate BuildCertificate(
    const AcceptedSnapshot& snapshot,
    const StateQualityContract& contract) {

    StateAcceptanceCertificate cert;
    cert.certificate_id = "cert-" + snapshot.snapshot_id;
    cert.contract_id = snapshot.contract_id;
    cert.contract_schema_version = contract.schema_version;
    cert.contract_content_version = snapshot.contract_version;
    cert.contract_hash = snapshot.contract_hash;
    cert.evaluation_time_ms = snapshot.evaluation_time_ms;

    cert.propagation_model_id = snapshot.model_id;
    cert.propagation_model_version = snapshot.model_version;
    cert.max_horizontal_speed_mps = contract.max_horizontal_speed_mps;
    cert.max_vertical_speed_mps = contract.max_vertical_speed_mps;

    cert.accepted_agents = snapshot.accepted_agents;
    cert.produced_at_ms = snapshot.produced_at_ms;

    double max_clock_unc = 0.0;
    double max_elapsed = 0.0;
    double max_pos_unc = 0.0;
    double max_vel_unc = 0.0;

    // Build evidence entries from snapshot, sorted by agent then field
    // for deterministic ordering.
    struct SortKey {
        std::string agent_id;
        std::uint8_t field_key;
    };
    std::vector<std::pair<SortKey, CertificateEvidenceEntry>> sorted_entries;

    for (const auto& [agent_id, fields] : snapshot.agent_states) {
        for (const auto& [field_key, field_state] : fields) {
            const auto field = static_cast<EvidenceFieldId>(field_key);

            CertificateEvidenceEntry entry;
            entry.agent_id = agent_id;
            entry.field = field;
            entry.sequence = field_state.evidence.identity.sequence;
            entry.agent_session_id = field_state.evidence.identity.agent_session_id;
            entry.source_component = field_state.evidence.identity.source_component;
            entry.source_time_ms = field_state.evidence.source_time.timestamp_ms;
            entry.generation_interval = field_state.generation_interval;
            entry.conservative_elapsed_ms = field_state.conservative_elapsed_ms;
            entry.clock_uncertainty_ms = field_state.clock_uncertainty_ms;
            entry.observation_uncertainty = field_state.observation_uncertainty;
            entry.propagated_uncertainty = field_state.propagated_uncertainty;

            max_clock_unc = std::max(max_clock_unc, field_state.clock_uncertainty_ms);
            max_elapsed = std::max(max_elapsed, field_state.conservative_elapsed_ms);

            if (field == EvidenceFieldId::kPosition) {
                max_pos_unc = std::max(max_pos_unc, field_state.propagated_uncertainty);
            }
            if (field == EvidenceFieldId::kVelocity) {
                max_vel_unc = std::max(max_vel_unc, field_state.propagated_uncertainty);
            }

            sorted_entries.push_back({
                {.agent_id = agent_id, .field_key = field_key},
                std::move(entry),
            });
        }
    }

    // Sort for deterministic hash.
    std::sort(sorted_entries.begin(), sorted_entries.end(),
              [](const auto& a, const auto& b) {
                  if (a.first.agent_id != b.first.agent_id)
                      return a.first.agent_id < b.first.agent_id;
                  return a.first.field_key < b.first.field_key;
              });

    for (auto& [_, entry] : sorted_entries) {
        cert.evidence_entries.push_back(std::move(entry));
    }

    cert.max_clock_uncertainty_ms = max_clock_unc;
    cert.max_conservative_elapsed_ms = max_elapsed;
    cert.max_propagated_position_uncertainty_m = max_pos_unc;
    cert.max_propagated_velocity_uncertainty_mps = max_vel_unc;

    // Compute the tamper-evident hash h_K last.
    cert.certificate_hash = ComputeCertificateHash(cert);

    return cert;
}

}  // namespace swarmkit::core
