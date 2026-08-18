// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/core/state_acceptance_certificate.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include "sha256.h"

namespace swarmkit::core {

namespace {

std::string CanonicalSerializeEvidenceValue(const EvidenceValue& val) {
    std::ostringstream oss;
    oss << std::setprecision(10);
    std::visit([&oss](const auto& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::array<double, 3>>) {
            oss << "arr3d:" << v[0] << "," << v[1] << "," << v[2];
        } else if constexpr (std::is_same_v<T, std::array<float, 3>>) {
            oss << "arr3f:" << v[0] << "," << v[1] << "," << v[2];
        } else if constexpr (std::is_same_v<T, double>) {
            oss << "d:" << v;
        } else if constexpr (std::is_same_v<T, float>) {
            oss << "f:" << v;
        } else if constexpr (std::is_same_v<T, bool>) {
            oss << "b:" << (v ? "1" : "0");
        } else if constexpr (std::is_same_v<T, std::int32_t>) {
            oss << "i32:" << v;
        } else if constexpr (std::is_same_v<T, std::uint32_t>) {
            oss << "u32:" << v;
        } else if constexpr (std::is_same_v<T, std::int64_t>) {
            oss << "i64:" << v;
        } else if constexpr (std::is_same_v<T, std::uint64_t>) {
            oss << "u64:" << v;
        } else if constexpr (std::is_same_v<T, std::string>) {
            oss << "str:" << v;
        } else if constexpr (std::is_same_v<T, EstimatorState>) {
            oss << "est:" << static_cast<int>(v);
        } else if constexpr (std::is_same_v<T, GpsQuality>) {
            oss << "gps:" << static_cast<int>(v);
        }
    }, val);
    return oss.str();
}

}  // namespace

std::string ComputeEvidenceHash(const EvidenceRecord& record) {
    std::ostringstream oss;
    oss << std::setprecision(10);
    oss << "agent:" << record.identity.agent_id << "\n";
    oss << "field:" << static_cast<int>(record.identity.field_id) << "\n";
    oss << "seq:" << record.identity.sequence << "\n";
    oss << "session:" << record.identity.agent_session_id << "\n";
    oss << "source:" << record.identity.source_component << "\n";
    oss << "frame:" << static_cast<int>(record.identity.coordinate_frame) << "\n";
    oss << "est_id:" << record.identity.estimator_id << "\n";
    oss << "mission_id:" << record.identity.mission_id << "\n";
    oss << "mission_rev:" << record.identity.mission_revision << "\n";

    if (record.source_time.timestamp_ms.has_value()) {
        oss << "src_time:" << *record.source_time.timestamp_ms << "\n";
    }
    if (record.source_time.clock_uncertainty_ms.has_value()) {
        oss << "src_clk_unc:" << *record.source_time.clock_uncertainty_ms << "\n";
    }
    oss << "receive_time_ms:" << record.receive_time_ms << "\n";

    oss << "est_healthy:" << (record.quality.estimator_healthy ? "1" : "0") << "\n";
    oss << "est_pos_ok:" << (record.quality.estimator_position_ok ? "1" : "0") << "\n";
    oss << "est_vel_ok:" << (record.quality.estimator_velocity_ok ? "1" : "0") << "\n";

    if (record.quality.uncertainty.has_value()) {
        oss << "unc_val:" << record.quality.uncertainty->value << "\n";
        oss << "unc_sem:" << static_cast<int>(record.quality.uncertainty->descriptor.semantics) << "\n";
    }

    oss << "val:" << CanonicalSerializeEvidenceValue(record.value) << "\n";

    return internal::Sha256Hex(oss.str());
}

std::string ComputeCertificateHash(const StateAcceptanceCertificate& cert) {
    std::ostringstream oss;
    oss << std::setprecision(10);
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
        } else {
            oss << ":absent";
        }
        oss << ":" << entry.generation_interval.lower_ms
            << ":" << entry.generation_interval.upper_ms
            << ":" << entry.conservative_elapsed_ms
            << ":" << entry.clock_uncertainty_ms
            << ":" << entry.observation_uncertainty
            << ":" << entry.propagated_uncertainty
            << ":" << entry.evidence_hash
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
    std::vector<std::string> sorted_agents = cert.accepted_agents;
    std::sort(sorted_agents.begin(), sorted_agents.end());
    for (const auto& a : sorted_agents) {
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
            entry.evidence_hash = ComputeEvidenceHash(field_state.evidence);

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

    // Sort for deterministic ordering.
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

std::string SerializeCertificate(const StateAcceptanceCertificate& cert) {
    std::ostringstream oss;
    oss << std::setprecision(10);
    oss << "CERT_V1\n";
    oss << cert.certificate_id << "\n";
    oss << cert.contract_id << "\n";
    oss << cert.contract_schema_version << "\n";
    oss << cert.contract_content_version << "\n";
    oss << cert.contract_hash << "\n";
    oss << cert.evaluation_time_ms << "\n";
    oss << cert.max_clock_uncertainty_ms << "\n";
    oss << cert.max_conservative_elapsed_ms << "\n";
    oss << cert.max_propagated_position_uncertainty_m << "\n";
    oss << cert.max_propagated_velocity_uncertainty_mps << "\n";
    oss << cert.propagation_model_id << "\n";
    oss << cert.propagation_model_version << "\n";
    oss << cert.max_horizontal_speed_mps << "\n";
    oss << cert.max_vertical_speed_mps << "\n";
    oss << cert.acceptance_semantics_version << "\n";
    oss << cert.produced_at_ms << "\n";
    oss << cert.certificate_hash << "\n";

    oss << cert.accepted_agents.size() << "\n";
    for (const auto& a : cert.accepted_agents) {
        oss << a << "\n";
    }

    oss << cert.evidence_entries.size() << "\n";
    for (const auto& e : cert.evidence_entries) {
        oss << e.agent_id << "\n";
        oss << static_cast<int>(e.field) << "\n";
        oss << e.sequence << "\n";
        oss << e.agent_session_id << "\n";
        oss << e.source_component << "\n";
        oss << (e.source_time_ms.has_value() ? *e.source_time_ms : -1) << "\n";
        oss << e.generation_interval.lower_ms << "\n";
        oss << e.generation_interval.upper_ms << "\n";
        oss << e.conservative_elapsed_ms << "\n";
        oss << e.clock_uncertainty_ms << "\n";
        oss << e.observation_uncertainty << "\n";
        oss << e.propagated_uncertainty << "\n";
        oss << e.evidence_hash << "\n";
    }

    return oss.str();
}

std::optional<StateAcceptanceCertificate> DeserializeCertificate(std::string_view data) {
    std::istringstream iss{std::string(data)};
    std::string header;
    if (!(iss >> header) || header != "CERT_V1") return std::nullopt;

    StateAcceptanceCertificate cert;
    if (!(iss >> cert.certificate_id >> cert.contract_id
              >> cert.contract_schema_version >> cert.contract_content_version
              >> cert.contract_hash >> cert.evaluation_time_ms
              >> cert.max_clock_uncertainty_ms >> cert.max_conservative_elapsed_ms
              >> cert.max_propagated_position_uncertainty_m
              >> cert.max_propagated_velocity_uncertainty_mps
              >> cert.propagation_model_id >> cert.propagation_model_version
              >> cert.max_horizontal_speed_mps >> cert.max_vertical_speed_mps
              >> cert.acceptance_semantics_version >> cert.produced_at_ms
              >> cert.certificate_hash)) {
        return std::nullopt;
    }

    std::size_t num_agents = 0;
    if (!(iss >> num_agents)) return std::nullopt;
    cert.accepted_agents.resize(num_agents);
    for (std::size_t i = 0; i < num_agents; ++i) {
        if (!(iss >> cert.accepted_agents[i])) return std::nullopt;
    }

    std::size_t num_entries = 0;
    if (!(iss >> num_entries)) return std::nullopt;
    cert.evidence_entries.resize(num_entries);
    for (std::size_t i = 0; i < num_entries; ++i) {
        auto& e = cert.evidence_entries[i];
        int field_int = 0;
        std::int64_t src_time = -1;
        if (!(iss >> e.agent_id >> field_int >> e.sequence >> e.agent_session_id
                  >> e.source_component >> src_time
                  >> e.generation_interval.lower_ms >> e.generation_interval.upper_ms
                  >> e.conservative_elapsed_ms >> e.clock_uncertainty_ms
                  >> e.observation_uncertainty >> e.propagated_uncertainty
                  >> e.evidence_hash)) {
            return std::nullopt;
        }
        e.field = static_cast<EvidenceFieldId>(field_int);
        if (src_time >= 0) {
            e.source_time_ms = src_time;
        }
    }

    return cert;
}

}  // namespace swarmkit::core
