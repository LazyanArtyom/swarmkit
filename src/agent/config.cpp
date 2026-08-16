// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <algorithm>
#include <expected>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include "config_yaml.h"
#include "env_utils.h"
#include "security_utils.h"
#include "swarmkit/agent/server.h"

namespace swarmkit::agent {
namespace {

using core::internal::GetEnvValue;
using core::internal::LooksLikeAddress;

constexpr std::string_view kAgentEnvId = "AGENT_ID";
constexpr std::string_view kAgentEnvBindAddr = "BIND_ADDR";
constexpr std::string_view kAgentEnvDefaultAuthorityTtlMs = "DEFAULT_AUTHORITY_TTL_MS";
constexpr std::string_view kAgentEnvDefaultTelemetryRateHz = "DEFAULT_TELEMETRY_RATE_HZ";
constexpr std::string_view kAgentEnvMinTelemetryRateHz = "MIN_TELEMETRY_RATE_HZ";
constexpr std::string_view kAgentEnvTelemetryRetentionFrames = "TELEMETRY_RETENTION_FRAMES";
constexpr std::string_view kAgentEnvAllowUnsafeBenchCommands = "ALLOW_UNSAFE_BENCH_COMMANDS";
constexpr std::string_view kAgentEnvReportBacklogSize = "REPORT_BACKLOG_SIZE";
constexpr std::string_view kAgentEnvEvidenceFile = "EVIDENCE_FILE";
constexpr std::string_view kAgentEnvEvidenceRunId = "EVIDENCE_RUN_ID";
constexpr std::string_view kAgentEnvEvidenceScenarioId = "EVIDENCE_SCENARIO_ID";
constexpr std::string_view kAgentEnvEvidenceMaxSegmentBytes = "EVIDENCE_MAX_SEGMENT_BYTES";
constexpr std::string_view kAgentEnvEvidenceMaxSegments = "EVIDENCE_MAX_SEGMENTS";
constexpr std::string_view kAgentEnvEvidenceLossPolicy = "EVIDENCE_LOSS_POLICY";
constexpr std::string_view kAgentEnvEvidenceRandomSeed = "EVIDENCE_RANDOM_SEED";
constexpr std::string_view kAgentEnvEvidenceFlushEachRecord = "EVIDENCE_FLUSH_EACH_RECORD";
constexpr std::string_view kAgentEnvEvidenceFsyncEachRecord = "EVIDENCE_FSYNC_EACH_RECORD";
constexpr std::string_view kAgentEnvEvidenceOverwrite = "EVIDENCE_OVERWRITE";
constexpr std::string_view kAgentEnvEvidenceCalibrationProfile = "EVIDENCE_CALIBRATION_PROFILE";
constexpr std::string_view kAgentEnvEvidenceCalibrationVersion = "EVIDENCE_CALIBRATION_VERSION";
constexpr std::string_view kAgentEnvArtifactDir = "ARTIFACT_DIR";
constexpr std::string_view kAgentEnvDataMessageBacklogSize = "DATA_MESSAGE_BACKLOG_SIZE";
constexpr std::string_view kAgentEnvDataMaxMessagePayloadBytes = "DATA_MAX_MESSAGE_PAYLOAD_BYTES";
constexpr std::string_view kAgentEnvDataArtifactChunkBytes = "DATA_ARTIFACT_CHUNK_BYTES";
constexpr std::string_view kAgentEnvDataMaxArtifactBytes = "DATA_MAX_ARTIFACT_BYTES";
constexpr std::string_view kAgentEnvDataMaxConcurrentArtifactTransfers =
    "DATA_MAX_CONCURRENT_ARTIFACT_TRANSFERS";
constexpr std::string_view kAgentEnvDataMaxQueuedArtifactTransfers =
    "DATA_MAX_QUEUED_ARTIFACT_TRANSFERS";
constexpr std::string_view kAgentEnvDataMaxConcurrentArtifactTransfersPerPeer =
    "DATA_MAX_CONCURRENT_ARTIFACT_TRANSFERS_PER_PEER";
constexpr std::string_view kAgentEnvDataArtifactPeerBytesPerSecond =
    "DATA_ARTIFACT_PEER_BYTES_PER_SECOND";
constexpr std::string_view kAgentEnvRootCaCertPath = "ROOT_CA_CERT_PATH";
constexpr std::string_view kAgentEnvCertChainPath = "CERT_CHAIN_PATH";
constexpr std::string_view kAgentEnvPrivateKeyPath = "PRIVATE_KEY_PATH";
constexpr std::string_view kAgentEnvAllowedClientIds = "ALLOWED_CLIENT_IDS";
constexpr std::string_view kAgentEnvTransportSecurity = "TRANSPORT_SECURITY";

[[nodiscard]] std::vector<std::string> SplitCsvList(std::string_view value) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start < value.size()) {
        const std::size_t end = value.find(',', start);
        const std::string_view token =
            end == std::string_view::npos ? value.substr(start) : value.substr(start, end - start);
        const std::string trimmed = core::internal::TrimWhitespace(token);
        if (!trimmed.empty()) {
            out.push_back(trimmed);
        }
        if (end == std::string_view::npos) {
            break;
        }
        start = end + 1;
    }
    return out;
}

}  // namespace

core::Result VehicleProfile::Validate() const {
    if (profile_id.empty()) {
        return core::Result::Rejected("vehicle_profile.profile_id must not be empty");
    }
    if (cruise_speed_mps <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.cruise_speed_mps must be > 0");
    }
    if (climb_speed_mps <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.climb_speed_mps must be > 0");
    }
    if (descent_speed_mps <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.descent_speed_mps must be > 0");
    }
    if (max_altitude_m <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.max_altitude_m must be > 0");
    }
    if (min_battery_percent < 0.0F || min_battery_percent > 100.0F) {
        return core::Result::Rejected("vehicle_profile.min_battery_percent must be 0..100");
    }
    if (battery_reserve_percent < 0.0F || battery_reserve_percent > 100.0F) {
        return core::Result::Rejected("vehicle_profile.battery_reserve_percent must be 0..100");
    }
    if (battery_reserve_percent > min_battery_percent) {
        return core::Result::Rejected(
            "vehicle_profile.battery_reserve_percent must be <= min_battery_percent");
    }
    if (tracking_tolerance_m <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.tracking_tolerance_m must be > 0");
    }
    if (goal_margin_ms < 0) {
        return core::Result::Rejected("vehicle_profile.goal_margin_ms must be >= 0");
    }
    if (takeoff_timeout_margin_ms < 0) {
        return core::Result::Rejected("vehicle_profile.takeoff_timeout_margin_ms must be >= 0");
    }
    if (land_timeout_margin_ms < 0) {
        return core::Result::Rejected("vehicle_profile.land_timeout_margin_ms must be >= 0");
    }
    if (max_goal_timeout_ms <= 0) {
        return core::Result::Rejected("vehicle_profile.max_goal_timeout_ms must be > 0");
    }
    if (min_gps_fix_type < 0) {
        return core::Result::Rejected("vehicle_profile.min_gps_fix_type must be >= 0");
    }
    if (min_satellites_visible < 0) {
        return core::Result::Rejected("vehicle_profile.min_satellites_visible must be >= 0");
    }
    if (max_gps_hdop <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.max_gps_hdop must be > 0");
    }
    return core::Result::Success();
}

core::TransportSecurityMode AgentSecurityConfig::EffectiveTransportSecurity() const {
    if (transport_security != core::TransportSecurityMode::kAuto) {
        return transport_security;
    }
    if (root_ca_cert_path.empty() && cert_chain_path.empty() && private_key_path.empty()) {
        return core::TransportSecurityMode::kInsecure;
    }
    if (!cert_chain_path.empty() && !private_key_path.empty() && root_ca_cert_path.empty()) {
        return core::TransportSecurityMode::kTls;
    }
    return core::TransportSecurityMode::kMutualTls;
}

core::Result AgentSecurityConfig::Validate() const {
    const core::TransportSecurityMode mode = EffectiveTransportSecurity();
    if (mode == core::TransportSecurityMode::kInsecure) {
        if (!allowed_client_ids.empty()) {
            return core::Result::Rejected(
                "security.allowed_client_ids requires mTLS transport security");
        }
        return core::Result::Success();
    }
    if (!core::internal::FileExists(cert_chain_path)) {
        return core::Result::Rejected(
            "security.cert_chain_path must point to an existing file for TLS/mTLS");
    }
    if (!core::internal::FileExists(private_key_path)) {
        return core::Result::Rejected(
            "security.private_key_path must point to an existing file for TLS/mTLS");
    }
    if (mode == core::TransportSecurityMode::kMutualTls &&
        !core::internal::FileExists(root_ca_cert_path)) {
        return core::Result::Rejected(
            "security.root_ca_cert_path must point to an existing file for mTLS");
    }
    if (mode != core::TransportSecurityMode::kMutualTls && !allowed_client_ids.empty()) {
        return core::Result::Rejected(
            "security.allowed_client_ids requires mTLS transport security");
    }
    return core::Result::Success();
}

core::Result ReportStreamConfig::Validate() const {
    if (backlog_size < 0) {
        return core::Result::Rejected("reports.backlog_size must be >= 0");
    }
    return core::Result::Success();
}

core::Result ExecutionRecorderConfig::Validate() const {
    if (file_path.empty()) {
        return core::Result::Success();
    }
    if (max_segment_bytes <= 1024) {
        return core::Result::Rejected("execution_recorder.max_segment_bytes must be > 1024");
    }
    if (max_segments <= 0) {
        return core::Result::Rejected("execution_recorder.max_segments must be > 0");
    }
    if (loss_policy == EvidenceLossPolicy::kInvalidateRun && max_segments != 1) {
        return core::Result::Rejected(
            "execution_recorder.max_segments must be 1 when loss_policy is invalidate_run");
    }
    return core::Result::Success();
}

core::Result TelemetryRetentionConfig::Validate() const {
    if (max_frames_per_drone <= 0) {
        return core::Result::Rejected("telemetry_retention.max_frames_per_drone must be > 0");
    }
    return core::Result::Success();
}

core::Result DataPeerConfig::Validate() const {
    if (drone_id.empty()) {
        return core::Result::Rejected("data.peers[].drone_id must not be empty");
    }
    if (!LooksLikeAddress(address)) {
        return core::Result::Rejected("data.peers[].address must be in host:port format");
    }
    return core::Result::Success();
}

core::Result DataPlaneConfig::Validate() const {
    if (artifact_dir.empty()) {
        return core::Result::Rejected("data.artifact_dir must not be empty");
    }
    if (message_backlog_size < 0) {
        return core::Result::Rejected("data.message_backlog_size must be >= 0");
    }
    if (max_message_payload_bytes <= 0) {
        return core::Result::Rejected("data.max_message_payload_bytes must be > 0");
    }
    if (artifact_chunk_bytes <= 0) {
        return core::Result::Rejected("data.artifact_chunk_bytes must be > 0");
    }
    if (max_artifact_bytes <= 0) {
        return core::Result::Rejected("data.max_artifact_bytes must be > 0");
    }
    if (max_concurrent_artifact_transfers <= 0) {
        return core::Result::Rejected("data.max_concurrent_artifact_transfers must be > 0");
    }
    if (max_queued_artifact_transfers < 0) {
        return core::Result::Rejected("data.max_queued_artifact_transfers must be >= 0");
    }
    if (max_concurrent_artifact_transfers_per_peer <= 0) {
        return core::Result::Rejected(
            "data.max_concurrent_artifact_transfers_per_peer must be > 0");
    }
    if (artifact_peer_bytes_per_second < 0) {
        return core::Result::Rejected("data.artifact_peer_bytes_per_second must be >= 0");
    }
    std::vector<std::string> seen;
    for (const auto& peer : peers) {
        if (const core::Result result = peer.Validate(); !result.IsOk()) {
            return result;
        }
        if (std::ranges::find(seen, peer.drone_id) != seen.end()) {
            return core::Result::Rejected("data.peers contains duplicate drone_id: " +
                                          peer.drone_id);
        }
        seen.push_back(peer.drone_id);
    }
    return core::Result::Success();
}

std::optional<DataPeerConfig> DataPlaneConfig::FindPeer(std::string_view drone_id) const {
    const auto iter = std::ranges::find_if(
        peers, [drone_id](const DataPeerConfig& peer) { return peer.drone_id == drone_id; });
    if (iter == peers.end()) {
        return std::nullopt;
    }
    return *iter;
}

core::Result AgentConfig::Validate() const {
    if (agent_id.empty()) {
        return core::Result::Rejected("agent_id must not be empty");
    }
    if (!LooksLikeAddress(bind_addr)) {
        return core::Result::Rejected("bind_addr must be in host:port format");
    }
    if (default_authority_ttl_ms <= 0) {
        return core::Result::Rejected("default_authority_ttl_ms must be > 0");
    }
    if (default_telemetry_rate_hz <= 0) {
        return core::Result::Rejected("default_telemetry_rate_hz must be > 0");
    }
    if (min_telemetry_rate_hz <= 0) {
        return core::Result::Rejected("min_telemetry_rate_hz must be > 0");
    }
    if (min_telemetry_rate_hz > default_telemetry_rate_hz) {
        return core::Result::Rejected("min_telemetry_rate_hz must be <= default_telemetry_rate_hz");
    }
    if (const core::Result telemetry_result = telemetry_retention.Validate();
        !telemetry_result.IsOk()) {
        return telemetry_result;
    }
    if (const core::Result vehicle_result = vehicle_profile.Validate(); !vehicle_result.IsOk()) {
        return vehicle_result;
    }
    if (const core::Result report_result = reports.Validate(); !report_result.IsOk()) {
        return report_result;
    }
    if (const core::Result recorder_result = execution_recorder.Validate();
        !recorder_result.IsOk()) {
        return recorder_result;
    }
    if (const core::Result data_result = data.Validate(); !data_result.IsOk()) {
        return data_result;
    }
    return security.Validate();
}

void AgentConfig::ApplyEnvironment(std::string_view prefix) {
    using core::internal::ApplyBoolEnv;
    using core::internal::ApplyIntEnv;
    using core::internal::ApplyStringEnv;

    ApplyStringEnv(prefix, kAgentEnvId, &agent_id);
    ApplyStringEnv(prefix, kAgentEnvBindAddr, &bind_addr);
    ApplyIntEnv(prefix, kAgentEnvDefaultAuthorityTtlMs, &default_authority_ttl_ms);
    ApplyIntEnv(prefix, kAgentEnvDefaultTelemetryRateHz, &default_telemetry_rate_hz);
    ApplyIntEnv(prefix, kAgentEnvMinTelemetryRateHz, &min_telemetry_rate_hz);
    ApplyIntEnv(prefix, kAgentEnvTelemetryRetentionFrames,
                &telemetry_retention.max_frames_per_drone);
    ApplyBoolEnv(prefix, kAgentEnvAllowUnsafeBenchCommands, &safety.allow_unsafe_bench_commands);
    ApplyIntEnv(prefix, kAgentEnvReportBacklogSize, &reports.backlog_size);
    ApplyStringEnv(prefix, kAgentEnvEvidenceFile, &execution_recorder.file_path);
    ApplyStringEnv(prefix, kAgentEnvEvidenceRunId, &execution_recorder.run_id);
    ApplyStringEnv(prefix, kAgentEnvEvidenceScenarioId, &execution_recorder.scenario_id);
    ApplyIntEnv(prefix, kAgentEnvEvidenceMaxSegments, &execution_recorder.max_segments);
    ApplyBoolEnv(prefix, kAgentEnvEvidenceFlushEachRecord, &execution_recorder.flush_each_record);
    ApplyBoolEnv(prefix, kAgentEnvEvidenceFsyncEachRecord, &execution_recorder.fsync_each_record);
    ApplyBoolEnv(prefix, kAgentEnvEvidenceOverwrite, &execution_recorder.overwrite_existing);
    ApplyStringEnv(prefix, kAgentEnvEvidenceCalibrationProfile,
                   &execution_recorder.calibration_profile_id);
    ApplyStringEnv(prefix, kAgentEnvEvidenceCalibrationVersion,
                   &execution_recorder.calibration_version);
    if (const auto value =
            GetEnvValue(std::string(prefix) + std::string(kAgentEnvEvidenceMaxSegmentBytes));
        value.has_value()) {
        try {
            execution_recorder.max_segment_bytes = std::stoll(*value);
        } catch (const std::exception&) {
            execution_recorder.max_segment_bytes = 0;
        }
    }
    if (const auto value =
            GetEnvValue(std::string(prefix) + std::string(kAgentEnvEvidenceRandomSeed));
        value.has_value()) {
        try {
            execution_recorder.random_seed = std::stoull(*value);
        } catch (const std::exception&) {
            execution_recorder.max_segment_bytes = 0;
        }
    }
    if (const auto value =
            GetEnvValue(std::string(prefix) + std::string(kAgentEnvEvidenceLossPolicy));
        value.has_value()) {
        if (*value == "invalidate_run") {
            execution_recorder.loss_policy = EvidenceLossPolicy::kInvalidateRun;
        } else if (*value == "rotate_oldest") {
            execution_recorder.loss_policy = EvidenceLossPolicy::kRotateOldest;
        } else {
            execution_recorder.max_segments = 0;
        }
    }
    ApplyStringEnv(prefix, kAgentEnvArtifactDir, &data.artifact_dir);
    ApplyIntEnv(prefix, kAgentEnvDataMessageBacklogSize, &data.message_backlog_size);
    ApplyIntEnv(prefix, kAgentEnvDataMaxMessagePayloadBytes, &data.max_message_payload_bytes);
    ApplyIntEnv(prefix, kAgentEnvDataArtifactChunkBytes, &data.artifact_chunk_bytes);
    ApplyIntEnv(prefix, kAgentEnvDataMaxArtifactBytes, &data.max_artifact_bytes);
    ApplyIntEnv(prefix, kAgentEnvDataMaxConcurrentArtifactTransfers,
                &data.max_concurrent_artifact_transfers);
    ApplyIntEnv(prefix, kAgentEnvDataMaxQueuedArtifactTransfers,
                &data.max_queued_artifact_transfers);
    ApplyIntEnv(prefix, kAgentEnvDataMaxConcurrentArtifactTransfersPerPeer,
                &data.max_concurrent_artifact_transfers_per_peer);
    ApplyIntEnv(prefix, kAgentEnvDataArtifactPeerBytesPerSecond,
                &data.artifact_peer_bytes_per_second);

    ApplyStringEnv(prefix, kAgentEnvRootCaCertPath, &security.root_ca_cert_path);
    ApplyStringEnv(prefix, kAgentEnvCertChainPath, &security.cert_chain_path);
    ApplyStringEnv(prefix, kAgentEnvPrivateKeyPath, &security.private_key_path);
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kAgentEnvTransportSecurity));
        kValue.has_value()) {
        const auto parsed = core::ParseTransportSecurityMode(*kValue);
        if (parsed.has_value()) {
            security.transport_security = *parsed;
        }
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kAgentEnvAllowedClientIds));
        kValue.has_value()) {
        security.allowed_client_ids = SplitCsvList(*kValue);
    }
}

std::expected<AgentConfig, core::Result> LoadAgentConfigFromFile(const std::string& path) {
    const auto loaded_yaml = core::yaml::LoadYamlFile(path);
    if (!loaded_yaml.has_value()) {
        return std::unexpected(loaded_yaml.error());
    }

    AgentConfig config;
    const YAML::Node root = core::yaml::SelectSection(*loaded_yaml, "agent");
    if (!root || !root.IsMap()) {
        return std::unexpected(core::Result::Rejected("agent YAML config must be a map"));
    }

    const auto agent_id = core::yaml::ReadOptionalScalar<std::string>(root, "agent_id");
    if (!agent_id.has_value()) {
        return std::unexpected(agent_id.error());
    }
    if (agent_id->has_value()) {
        config.agent_id = agent_id->value_or(config.agent_id);
    }

    const auto bind_addr = core::yaml::ReadOptionalScalar<std::string>(root, "bind_addr");
    if (!bind_addr.has_value()) {
        return std::unexpected(bind_addr.error());
    }
    if (bind_addr->has_value()) {
        config.bind_addr = bind_addr->value_or(config.bind_addr);
    }

    const auto default_authority_ttl_ms =
        core::yaml::ReadOptionalScalar<int>(root, "default_authority_ttl_ms");
    if (!default_authority_ttl_ms.has_value()) {
        return std::unexpected(default_authority_ttl_ms.error());
    }
    if (default_authority_ttl_ms->has_value()) {
        config.default_authority_ttl_ms =
            default_authority_ttl_ms->value_or(config.default_authority_ttl_ms);
    }

    const auto default_telemetry_rate_hz =
        core::yaml::ReadOptionalScalar<int>(root, "default_telemetry_rate_hz");
    if (!default_telemetry_rate_hz.has_value()) {
        return std::unexpected(default_telemetry_rate_hz.error());
    }
    if (default_telemetry_rate_hz->has_value()) {
        config.default_telemetry_rate_hz =
            default_telemetry_rate_hz->value_or(config.default_telemetry_rate_hz);
    }

    const auto min_telemetry_rate_hz =
        core::yaml::ReadOptionalScalar<int>(root, "min_telemetry_rate_hz");
    if (!min_telemetry_rate_hz.has_value()) {
        return std::unexpected(min_telemetry_rate_hz.error());
    }
    if (min_telemetry_rate_hz->has_value()) {
        config.min_telemetry_rate_hz =
            min_telemetry_rate_hz->value_or(config.min_telemetry_rate_hz);
    }

    if (const YAML::Node telemetry = root["telemetry"]; telemetry) {
        if (!telemetry.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.telemetry must be a map"));
        }
        const auto retention_frames =
            core::yaml::ReadOptionalScalar<int>(telemetry, "retention_frames_per_drone");
        if (!retention_frames.has_value()) {
            return std::unexpected(retention_frames.error());
        }
        if (retention_frames->has_value()) {
            config.telemetry_retention.max_frames_per_drone =
                retention_frames->value_or(config.telemetry_retention.max_frames_per_drone);
        }
    }

    if (const YAML::Node safety = root["safety"]; safety) {
        if (!safety.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.safety must be a map"));
        }
        const auto allow_unsafe_bench_commands =
            core::yaml::ReadOptionalScalar<bool>(safety, "allow_unsafe_bench_commands");
        if (!allow_unsafe_bench_commands.has_value()) {
            return std::unexpected(allow_unsafe_bench_commands.error());
        }
        if (allow_unsafe_bench_commands->has_value()) {
            config.safety.allow_unsafe_bench_commands =
                allow_unsafe_bench_commands->value_or(false);
        }
    }

    if (const YAML::Node reports = root["reports"]; reports) {
        if (!reports.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.reports must be a map"));
        }
        const auto backlog_size = core::yaml::ReadOptionalScalar<int>(reports, "backlog_size");
        if (!backlog_size.has_value()) {
            return std::unexpected(backlog_size.error());
        }
        if (backlog_size->has_value()) {
            config.reports.backlog_size = backlog_size->value_or(config.reports.backlog_size);
        }
    }

    if (const YAML::Node recorder = root["execution_recorder"]; recorder) {
        if (!recorder.IsMap()) {
            return std::unexpected(
                core::Result::Rejected("agent.execution_recorder must be a map"));
        }
        const auto file_path = core::yaml::ReadOptionalScalar<std::string>(recorder, "file_path");
        const auto max_segment_bytes =
            core::yaml::ReadOptionalScalar<std::int64_t>(recorder, "max_segment_bytes");
        const auto max_segments = core::yaml::ReadOptionalScalar<int>(recorder, "max_segments");
        const auto loss_policy =
            core::yaml::ReadOptionalScalar<std::string>(recorder, "loss_policy");
        const auto flush_each_record =
            core::yaml::ReadOptionalScalar<bool>(recorder, "flush_each_record");
        const auto fsync_each_record =
            core::yaml::ReadOptionalScalar<bool>(recorder, "fsync_each_record");
        const auto overwrite_existing =
            core::yaml::ReadOptionalScalar<bool>(recorder, "overwrite_existing");
        const auto run_id = core::yaml::ReadOptionalScalar<std::string>(recorder, "run_id");
        const auto scenario_id =
            core::yaml::ReadOptionalScalar<std::string>(recorder, "scenario_id");
        const auto random_seed =
            core::yaml::ReadOptionalScalar<std::uint64_t>(recorder, "random_seed");
        const auto calibration_profile_id =
            core::yaml::ReadOptionalScalar<std::string>(recorder, "calibration_profile_id");
        const auto calibration_version =
            core::yaml::ReadOptionalScalar<std::string>(recorder, "calibration_version");
        if (!file_path || !max_segment_bytes || !max_segments || !loss_policy ||
            !flush_each_record || !fsync_each_record || !overwrite_existing || !run_id ||
            !scenario_id || !random_seed || !calibration_profile_id || !calibration_version) {
            return std::unexpected(
                core::Result::Rejected("agent.execution_recorder contains an invalid scalar"));
        }
        if (file_path->has_value()) {
            config.execution_recorder.file_path = file_path->value_or("");
        }
        if (max_segment_bytes->has_value()) {
            config.execution_recorder.max_segment_bytes =
                max_segment_bytes->value_or(config.execution_recorder.max_segment_bytes);
        }
        if (max_segments->has_value()) {
            config.execution_recorder.max_segments =
                max_segments->value_or(config.execution_recorder.max_segments);
        }
        if (loss_policy->has_value()) {
            const std::string value = loss_policy->value_or("");
            if (value == "invalidate_run") {
                config.execution_recorder.loss_policy = EvidenceLossPolicy::kInvalidateRun;
            } else if (value == "rotate_oldest") {
                config.execution_recorder.loss_policy = EvidenceLossPolicy::kRotateOldest;
            } else {
                return std::unexpected(core::Result::Rejected(
                    "execution_recorder.loss_policy must be invalidate_run|rotate_oldest"));
            }
        }
        if (flush_each_record->has_value()) {
            config.execution_recorder.flush_each_record = flush_each_record->value_or(true);
        }
        if (fsync_each_record->has_value()) {
            config.execution_recorder.fsync_each_record = fsync_each_record->value_or(false);
        }
        if (overwrite_existing->has_value()) {
            config.execution_recorder.overwrite_existing = overwrite_existing->value_or(false);
        }
        if (run_id->has_value()) {
            config.execution_recorder.run_id = run_id->value_or("");
        }
        if (scenario_id->has_value()) {
            config.execution_recorder.scenario_id = scenario_id->value_or("");
        }
        if (random_seed->has_value()) {
            config.execution_recorder.random_seed = *random_seed;
        }
        if (calibration_profile_id->has_value()) {
            config.execution_recorder.calibration_profile_id = calibration_profile_id->value_or("");
        }
        if (calibration_version->has_value()) {
            config.execution_recorder.calibration_version = calibration_version->value_or("");
        }
        if (const YAML::Node labels = recorder["labels"]; labels) {
            if (!labels.IsMap()) {
                return std::unexpected(
                    core::Result::Rejected("agent.execution_recorder.labels must be a map"));
            }
            for (const auto& entry : labels) {
                if (!entry.first.IsScalar() || !entry.second.IsScalar()) {
                    return std::unexpected(core::Result::Rejected(
                        "agent.execution_recorder.labels entries must be scalar"));
                }
                config.execution_recorder.labels.emplace(entry.first.as<std::string>(),
                                                         entry.second.as<std::string>());
            }
        }
    }

    const YAML::Node data = root["data"] ? root["data"] : root["data_plane"];
    if (data) {
        if (!data.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.data must be a map"));
        }

        const auto artifact_dir = core::yaml::ReadOptionalScalar<std::string>(data, "artifact_dir");
        if (!artifact_dir.has_value()) {
            return std::unexpected(artifact_dir.error());
        }
        if (artifact_dir->has_value()) {
            config.data.artifact_dir = artifact_dir->value_or(config.data.artifact_dir);
        }

        const auto message_backlog_size =
            core::yaml::ReadOptionalScalar<int>(data, "message_backlog_size");
        if (!message_backlog_size.has_value()) {
            return std::unexpected(message_backlog_size.error());
        }
        if (message_backlog_size->has_value()) {
            config.data.message_backlog_size =
                message_backlog_size->value_or(config.data.message_backlog_size);
        }

        const auto max_message_payload_bytes =
            core::yaml::ReadOptionalScalar<int>(data, "max_message_payload_bytes");
        if (!max_message_payload_bytes.has_value()) {
            return std::unexpected(max_message_payload_bytes.error());
        }
        if (max_message_payload_bytes->has_value()) {
            config.data.max_message_payload_bytes =
                max_message_payload_bytes->value_or(config.data.max_message_payload_bytes);
        }

        const auto artifact_chunk_bytes =
            core::yaml::ReadOptionalScalar<int>(data, "artifact_chunk_bytes");
        if (!artifact_chunk_bytes.has_value()) {
            return std::unexpected(artifact_chunk_bytes.error());
        }
        if (artifact_chunk_bytes->has_value()) {
            config.data.artifact_chunk_bytes =
                artifact_chunk_bytes->value_or(config.data.artifact_chunk_bytes);
        }

        const auto max_artifact_bytes =
            core::yaml::ReadOptionalScalar<int>(data, "max_artifact_bytes");
        if (!max_artifact_bytes.has_value()) {
            return std::unexpected(max_artifact_bytes.error());
        }
        if (max_artifact_bytes->has_value()) {
            config.data.max_artifact_bytes =
                max_artifact_bytes->value_or(config.data.max_artifact_bytes);
        }

        const auto max_concurrent_artifact_transfers =
            core::yaml::ReadOptionalScalar<int>(data, "max_concurrent_artifact_transfers");
        if (!max_concurrent_artifact_transfers.has_value()) {
            return std::unexpected(max_concurrent_artifact_transfers.error());
        }
        if (max_concurrent_artifact_transfers->has_value()) {
            config.data.max_concurrent_artifact_transfers =
                max_concurrent_artifact_transfers->value_or(
                    config.data.max_concurrent_artifact_transfers);
        }

        const auto max_queued_artifact_transfers =
            core::yaml::ReadOptionalScalar<int>(data, "max_queued_artifact_transfers");
        if (!max_queued_artifact_transfers.has_value()) {
            return std::unexpected(max_queued_artifact_transfers.error());
        }
        if (max_queued_artifact_transfers->has_value()) {
            config.data.max_queued_artifact_transfers =
                max_queued_artifact_transfers->value_or(config.data.max_queued_artifact_transfers);
        }

        const auto max_concurrent_artifact_transfers_per_peer =
            core::yaml::ReadOptionalScalar<int>(data, "max_concurrent_artifact_transfers_per_peer");
        if (!max_concurrent_artifact_transfers_per_peer.has_value()) {
            return std::unexpected(max_concurrent_artifact_transfers_per_peer.error());
        }
        if (max_concurrent_artifact_transfers_per_peer->has_value()) {
            config.data.max_concurrent_artifact_transfers_per_peer =
                max_concurrent_artifact_transfers_per_peer->value_or(
                    config.data.max_concurrent_artifact_transfers_per_peer);
        }

        const auto artifact_peer_bytes_per_second =
            core::yaml::ReadOptionalScalar<int>(data, "artifact_peer_bytes_per_second");
        if (!artifact_peer_bytes_per_second.has_value()) {
            return std::unexpected(artifact_peer_bytes_per_second.error());
        }
        if (artifact_peer_bytes_per_second->has_value()) {
            config.data.artifact_peer_bytes_per_second = artifact_peer_bytes_per_second->value_or(
                config.data.artifact_peer_bytes_per_second);
        }

        const YAML::Node peers = data["peers"];
        if (peers) {
            if (!peers.IsSequence()) {
                return std::unexpected(
                    core::Result::Rejected("agent.data.peers must be a sequence"));
            }
            config.data.peers.clear();
            config.data.peers.reserve(peers.size());
            for (const auto& entry : peers) {
                if (!entry.IsMap()) {
                    return std::unexpected(
                        core::Result::Rejected("agent.data.peers entries must be maps"));
                }
                DataPeerConfig peer;
                const auto drone_id =
                    core::yaml::ReadOptionalScalar<std::string>(entry, "drone_id");
                if (!drone_id.has_value()) {
                    return std::unexpected(drone_id.error());
                }
                if (drone_id->has_value()) {
                    peer.drone_id = drone_id->value_or(peer.drone_id);
                }

                const auto address = core::yaml::ReadOptionalScalar<std::string>(entry, "address");
                if (!address.has_value()) {
                    return std::unexpected(address.error());
                }
                if (address->has_value()) {
                    peer.address = address->value_or(peer.address);
                }

                const auto transport_security =
                    core::yaml::ReadOptionalScalar<std::string>(entry, "transport_security");
                if (!transport_security.has_value()) {
                    return std::unexpected(transport_security.error());
                }
                if (transport_security->has_value()) {
                    const auto parsed = core::ParseTransportSecurityMode(
                        transport_security->value_or(std::string{}));
                    if (!parsed.has_value()) {
                        return std::unexpected(core::Result::Rejected(parsed.error()));
                    }
                    peer.transport_security = *parsed;
                }

                const auto root_ca_cert_path =
                    core::yaml::ReadOptionalScalar<std::string>(entry, "root_ca_cert_path");
                if (!root_ca_cert_path.has_value()) {
                    return std::unexpected(root_ca_cert_path.error());
                }
                if (root_ca_cert_path->has_value()) {
                    peer.root_ca_cert_path = core::internal::ResolveConfigRelativePath(
                        path, root_ca_cert_path->value_or(std::string{}));
                }

                const auto cert_chain_path =
                    core::yaml::ReadOptionalScalar<std::string>(entry, "cert_chain_path");
                if (!cert_chain_path.has_value()) {
                    return std::unexpected(cert_chain_path.error());
                }
                if (cert_chain_path->has_value()) {
                    peer.cert_chain_path = core::internal::ResolveConfigRelativePath(
                        path, cert_chain_path->value_or(std::string{}));
                }

                const auto private_key_path =
                    core::yaml::ReadOptionalScalar<std::string>(entry, "private_key_path");
                if (!private_key_path.has_value()) {
                    return std::unexpected(private_key_path.error());
                }
                if (private_key_path->has_value()) {
                    peer.private_key_path = core::internal::ResolveConfigRelativePath(
                        path, private_key_path->value_or(std::string{}));
                }

                const auto server_authority_override =
                    core::yaml::ReadOptionalScalar<std::string>(entry, "server_authority_override");
                if (!server_authority_override.has_value()) {
                    return std::unexpected(server_authority_override.error());
                }
                if (server_authority_override->has_value()) {
                    peer.server_authority_override =
                        server_authority_override->value_or(peer.server_authority_override);
                }

                config.data.peers.push_back(std::move(peer));
            }
        }
    }

    if (const YAML::Node vehicle_profile = root["vehicle_profile"]; vehicle_profile) {
        if (!vehicle_profile.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.vehicle_profile must be a map"));
        }

        const auto profile_id =
            core::yaml::ReadOptionalScalar<std::string>(vehicle_profile, "profile_id");
        if (!profile_id.has_value()) {
            return std::unexpected(profile_id.error());
        }
        if (profile_id->has_value()) {
            config.vehicle_profile.profile_id =
                profile_id->value_or(config.vehicle_profile.profile_id);
        }

        const auto cruise_speed_mps =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "cruise_speed_mps");
        if (!cruise_speed_mps.has_value()) {
            return std::unexpected(cruise_speed_mps.error());
        }
        if (cruise_speed_mps->has_value()) {
            config.vehicle_profile.cruise_speed_mps =
                cruise_speed_mps->value_or(config.vehicle_profile.cruise_speed_mps);
        }

        const auto climb_speed_mps =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "climb_speed_mps");
        if (!climb_speed_mps.has_value()) {
            return std::unexpected(climb_speed_mps.error());
        }
        if (climb_speed_mps->has_value()) {
            config.vehicle_profile.climb_speed_mps =
                climb_speed_mps->value_or(config.vehicle_profile.climb_speed_mps);
        }

        const auto descent_speed_mps =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "descent_speed_mps");
        if (!descent_speed_mps.has_value()) {
            return std::unexpected(descent_speed_mps.error());
        }
        if (descent_speed_mps->has_value()) {
            config.vehicle_profile.descent_speed_mps =
                descent_speed_mps->value_or(config.vehicle_profile.descent_speed_mps);
        }

        const auto max_altitude_m =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "max_altitude_m");
        if (!max_altitude_m.has_value()) {
            return std::unexpected(max_altitude_m.error());
        }
        if (max_altitude_m->has_value()) {
            config.vehicle_profile.max_altitude_m =
                max_altitude_m->value_or(config.vehicle_profile.max_altitude_m);
        }

        const auto min_battery_percent =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "min_battery_percent");
        if (!min_battery_percent.has_value()) {
            return std::unexpected(min_battery_percent.error());
        }
        if (min_battery_percent->has_value()) {
            config.vehicle_profile.min_battery_percent =
                min_battery_percent->value_or(config.vehicle_profile.min_battery_percent);
        }

        const auto battery_reserve_percent =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "battery_reserve_percent");
        if (!battery_reserve_percent.has_value()) {
            return std::unexpected(battery_reserve_percent.error());
        }
        if (battery_reserve_percent->has_value()) {
            config.vehicle_profile.battery_reserve_percent =
                battery_reserve_percent->value_or(config.vehicle_profile.battery_reserve_percent);
        }

        const auto tracking_tolerance_m =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "tracking_tolerance_m");
        if (!tracking_tolerance_m.has_value()) {
            return std::unexpected(tracking_tolerance_m.error());
        }
        if (tracking_tolerance_m->has_value()) {
            config.vehicle_profile.tracking_tolerance_m =
                tracking_tolerance_m->value_or(config.vehicle_profile.tracking_tolerance_m);
        }

        const auto goal_margin_ms =
            core::yaml::ReadOptionalScalar<int>(vehicle_profile, "goal_margin_ms");
        if (!goal_margin_ms.has_value()) {
            return std::unexpected(goal_margin_ms.error());
        }
        if (goal_margin_ms->has_value()) {
            config.vehicle_profile.goal_margin_ms =
                goal_margin_ms->value_or(config.vehicle_profile.goal_margin_ms);
        }

        const auto takeoff_timeout_margin_ms =
            core::yaml::ReadOptionalScalar<int>(vehicle_profile, "takeoff_timeout_margin_ms");
        if (!takeoff_timeout_margin_ms.has_value()) {
            return std::unexpected(takeoff_timeout_margin_ms.error());
        }
        if (takeoff_timeout_margin_ms->has_value()) {
            config.vehicle_profile.takeoff_timeout_margin_ms = takeoff_timeout_margin_ms->value_or(
                config.vehicle_profile.takeoff_timeout_margin_ms);
        }

        const auto land_timeout_margin_ms =
            core::yaml::ReadOptionalScalar<int>(vehicle_profile, "land_timeout_margin_ms");
        if (!land_timeout_margin_ms.has_value()) {
            return std::unexpected(land_timeout_margin_ms.error());
        }
        if (land_timeout_margin_ms->has_value()) {
            config.vehicle_profile.land_timeout_margin_ms =
                land_timeout_margin_ms->value_or(config.vehicle_profile.land_timeout_margin_ms);
        }

        const auto max_goal_timeout_ms =
            core::yaml::ReadOptionalScalar<int>(vehicle_profile, "max_goal_timeout_ms");
        if (!max_goal_timeout_ms.has_value()) {
            return std::unexpected(max_goal_timeout_ms.error());
        }
        if (max_goal_timeout_ms->has_value()) {
            config.vehicle_profile.max_goal_timeout_ms =
                max_goal_timeout_ms->value_or(config.vehicle_profile.max_goal_timeout_ms);
        }

        const auto min_gps_fix_type =
            core::yaml::ReadOptionalScalar<int>(vehicle_profile, "min_gps_fix_type");
        if (!min_gps_fix_type.has_value()) {
            return std::unexpected(min_gps_fix_type.error());
        }
        if (min_gps_fix_type->has_value()) {
            config.vehicle_profile.min_gps_fix_type =
                min_gps_fix_type->value_or(config.vehicle_profile.min_gps_fix_type);
        }

        const auto min_satellites_visible =
            core::yaml::ReadOptionalScalar<int>(vehicle_profile, "min_satellites_visible");
        if (!min_satellites_visible.has_value()) {
            return std::unexpected(min_satellites_visible.error());
        }
        if (min_satellites_visible->has_value()) {
            config.vehicle_profile.min_satellites_visible =
                min_satellites_visible->value_or(config.vehicle_profile.min_satellites_visible);
        }

        const auto max_gps_hdop =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "max_gps_hdop");
        if (!max_gps_hdop.has_value()) {
            return std::unexpected(max_gps_hdop.error());
        }
        if (max_gps_hdop->has_value()) {
            config.vehicle_profile.max_gps_hdop =
                max_gps_hdop->value_or(config.vehicle_profile.max_gps_hdop);
        }
    }

    if (const YAML::Node security = root["security"]; security) {
        if (!security.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.security must be a map"));
        }

        const auto transport_security =
            core::yaml::ReadOptionalScalar<std::string>(security, "transport_security");
        if (!transport_security.has_value()) {
            return std::unexpected(transport_security.error());
        }
        if (transport_security->has_value()) {
            const auto parsed =
                core::ParseTransportSecurityMode(transport_security->value_or(std::string{}));
            if (!parsed.has_value()) {
                return std::unexpected(core::Result::Rejected(parsed.error()));
            }
            config.security.transport_security = *parsed;
        }

        const auto root_ca_cert_path =
            core::yaml::ReadOptionalScalar<std::string>(security, "root_ca_cert_path");
        if (!root_ca_cert_path.has_value()) {
            return std::unexpected(root_ca_cert_path.error());
        }
        if (root_ca_cert_path->has_value()) {
            config.security.root_ca_cert_path = core::internal::ResolveConfigRelativePath(
                path, root_ca_cert_path->value_or(std::string{}));
        }

        const auto cert_chain_path =
            core::yaml::ReadOptionalScalar<std::string>(security, "cert_chain_path");
        if (!cert_chain_path.has_value()) {
            return std::unexpected(cert_chain_path.error());
        }
        if (cert_chain_path->has_value()) {
            config.security.cert_chain_path = core::internal::ResolveConfigRelativePath(
                path, cert_chain_path->value_or(std::string{}));
        }

        const auto private_key_path =
            core::yaml::ReadOptionalScalar<std::string>(security, "private_key_path");
        if (!private_key_path.has_value()) {
            return std::unexpected(private_key_path.error());
        }
        if (private_key_path->has_value()) {
            config.security.private_key_path = core::internal::ResolveConfigRelativePath(
                path, private_key_path->value_or(std::string{}));
        }

        const YAML::Node allowed_client_ids = security["allowed_client_ids"];
        if (allowed_client_ids) {
            if (!allowed_client_ids.IsSequence()) {
                return std::unexpected(
                    core::Result::Rejected("agent.security.allowed_client_ids must be a sequence"));
            }
            config.security.allowed_client_ids.clear();
            config.security.allowed_client_ids.reserve(allowed_client_ids.size());
            for (const auto& entry : allowed_client_ids) {
                if (!entry.IsScalar()) {
                    return std::unexpected(core::Result::Rejected(
                        "agent.security.allowed_client_ids entries must be scalars"));
                }
                config.security.allowed_client_ids.push_back(entry.as<std::string>());
            }
        }
    }

    if (const core::Result kValidation = config.Validate(); !kValidation.IsOk()) {
        return std::unexpected(kValidation);
    }
    return config;
}
}  // namespace swarmkit::agent
