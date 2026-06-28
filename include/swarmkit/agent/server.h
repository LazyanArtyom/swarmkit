// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/security.h"
namespace swarmkit::agent {

inline constexpr int kDefaultAuthorityTtlMs = 5000;
inline constexpr int kDefaultTelemetryRateHz = 5;
inline constexpr int kMinimumTelemetryRateHz = 1;
inline constexpr float kDefaultCruiseSpeedMps = 4.0F;
inline constexpr float kDefaultClimbSpeedMps = 1.5F;
inline constexpr float kDefaultDescentSpeedMps = 1.0F;
inline constexpr int kDefaultGoalMarginMs = 15000;
inline constexpr int kDefaultTakeoffTimeoutMarginMs = 20000;
inline constexpr int kDefaultLandTimeoutMarginMs = 30000;
inline constexpr int kDefaultMaxGoalTimeoutMs = 300000;
inline constexpr float kDefaultMaxAltitudeM = 120.0F;
inline constexpr float kDefaultMinBatteryPercent = 20.0F;
inline constexpr float kDefaultBatteryReservePercent = 15.0F;
inline constexpr float kDefaultTrackingToleranceM = 2.0F;
inline constexpr int kDefaultMinGpsFixType = 3;
inline constexpr int kDefaultMinSatellitesVisible = 6;
inline constexpr float kDefaultMaxGpsHdop = 2.5F;
inline constexpr int kDefaultReportBacklogSize = 1000;
inline constexpr int kDefaultReportLogMaxFileSizeBytes = 10 * 1024 * 1024;
inline constexpr int kDefaultReportLogMaxFiles = 5;
inline constexpr int kDefaultDataMessageBacklogSize = 1000;
inline constexpr int kDefaultDataMessageMaxPayloadBytes = 256 * 1024;
inline constexpr int kDefaultArtifactChunkBytes = 64 * 1024;
inline constexpr int kDefaultMaxArtifactBytes = 100 * 1024 * 1024;
inline constexpr int kDefaultMaxConcurrentArtifactTransfers = 2;
inline constexpr int kDefaultMaxQueuedArtifactTransfers = 16;
inline constexpr int kDefaultMaxConcurrentArtifactTransfersPerPeer = 1;
inline constexpr int kDefaultArtifactPeerBytesPerSecond = 0;

/// @brief Vehicle limits and readiness thresholds used by agent-side verification.
struct VehicleProfile {
    std::string profile_id{"generic-quad"};            ///< Human-readable profile id.
    float cruise_speed_mps{kDefaultCruiseSpeedMps};    ///< Horizontal cruise speed in m/s.
    float climb_speed_mps{kDefaultClimbSpeedMps};      ///< Climb speed in m/s.
    float descent_speed_mps{kDefaultDescentSpeedMps};  ///< Descent speed in m/s.
    float max_altitude_m{kDefaultMaxAltitudeM};  ///< Maximum normal target altitude in metres.
    float min_battery_percent{kDefaultMinBatteryPercent};  ///< Minimum battery for readiness.
    float battery_reserve_percent{kDefaultBatteryReservePercent};  ///< Reserved battery margin.
    float tracking_tolerance_m{kDefaultTrackingToleranceM};  ///< Goal tracking tolerance in metres.
    int goal_margin_ms{kDefaultGoalMarginMs};  ///< Added timeout margin for goal estimates.
    int takeoff_timeout_margin_ms{kDefaultTakeoffTimeoutMarginMs};  ///< Takeoff timeout margin.
    int land_timeout_margin_ms{kDefaultLandTimeoutMarginMs};        ///< Landing timeout margin.
    int max_goal_timeout_ms{kDefaultMaxGoalTimeoutMs};  ///< Maximum computed goal timeout.
    int min_gps_fix_type{kDefaultMinGpsFixType};  ///< Minimum backend GPS fix type for readiness.
    int min_satellites_visible{kDefaultMinSatellitesVisible};  ///< Minimum satellite count.
    float max_gps_hdop{kDefaultMaxGpsHdop};  ///< Maximum acceptable horizontal dilution.

    /// @brief Validate speed, altitude, battery, timeout, and GPS thresholds.
    [[nodiscard]] core::Result Validate() const;
};

/// @brief TLS/mTLS configuration for the agent gRPC server.
struct AgentSecurityConfig {
    core::TransportSecurityMode transport_security{core::TransportSecurityMode::kAuto};
    std::string root_ca_cert_path;                ///< Root CA for verifying mTLS clients.
    std::string cert_chain_path;                  ///< Server certificate chain path.
    std::string private_key_path;                 ///< Server private key path.
    std::vector<std::string> allowed_client_ids;  ///< Optional mTLS client allow-list.

    /// @brief Resolve kAuto into the effective server transport mode.
    [[nodiscard]] core::TransportSecurityMode EffectiveTransportSecurity() const;

    /// @brief Validate certificate paths and allowed-client constraints.
    [[nodiscard]] core::Result Validate() const;
};

/// @brief Persistent report stream storage configuration.
struct ReportPersistenceConfig {
    std::string log_file;             ///< JSONL report log path; empty disables file persistence.
    std::string sequence_state_file;  ///< Optional cursor/sequence state path.
    int backlog_size{kDefaultReportBacklogSize};  ///< In-memory replay backlog size.
    int max_log_file_size_bytes{kDefaultReportLogMaxFileSizeBytes};  ///< Rotation threshold.
    int max_log_files{kDefaultReportLogMaxFiles};  ///< Number of rotated report logs to keep.
    bool flush_each_write{true};   ///< Flush stream buffers after every report write.
    bool fsync_each_write{false};  ///< Force fsync after every write; expensive but durable.
    bool replay_from_log{true};    ///< Replay persisted reports at startup when possible.

    /// @brief Validate backlog and rotation limits.
    [[nodiscard]] core::Result Validate() const;
};

struct SafetyPolicy {
    /// Permit autonomous commands without normal flight-readiness checks.
    /// Intended only for restrained, propeller-off bench testing.
    bool allow_unsafe_bench_commands{false};
};

/// @brief Runtime data-plane peer configuration for inter-agent routing.
struct DataPeerConfig {
    std::string drone_id;  ///< Peer drone identifier.
    std::string address;   ///< Peer agent address in host:port format.
    core::TransportSecurityMode transport_security{core::TransportSecurityMode::kAuto};
    std::string root_ca_cert_path;          ///< Root CA path for TLS/mTLS peer connections.
    std::string cert_chain_path;            ///< Client certificate chain for mTLS peer connections.
    std::string private_key_path;           ///< Client private key for mTLS peer connections.
    std::string server_authority_override;  ///< Optional TLS authority override.

    /// @brief Validate drone id and address fields.
    [[nodiscard]] core::Result Validate() const;
};

/// @brief Agent-side message and artifact transfer configuration.
struct DataPlaneConfig {
    std::string artifact_dir{"/tmp/swarmkit-artifacts"};       ///< Stored artifact root directory.
    int message_backlog_size{kDefaultDataMessageBacklogSize};  ///< In-memory message backlog.
    int max_message_payload_bytes{kDefaultDataMessageMaxPayloadBytes};  ///< Message size limit.
    int artifact_chunk_bytes{kDefaultArtifactChunkBytes};  ///< Default artifact chunk size.
    int max_artifact_bytes{kDefaultMaxArtifactBytes};      ///< Maximum accepted artifact size.
    int max_concurrent_artifact_transfers{kDefaultMaxConcurrentArtifactTransfers};
    int max_queued_artifact_transfers{kDefaultMaxQueuedArtifactTransfers};
    int max_concurrent_artifact_transfers_per_peer{kDefaultMaxConcurrentArtifactTransfersPerPeer};
    int artifact_peer_bytes_per_second{kDefaultArtifactPeerBytesPerSecond};
    std::vector<DataPeerConfig> peers;  ///< Configured peers for routed data/artifact traffic.

    /// @brief Validate data-plane limits and peer definitions.
    [[nodiscard]] core::Result Validate() const;

    /// @brief Find a configured data peer by drone id.
    /// @param drone_id Peer drone identifier.
    /// @return Peer config when present.
    [[nodiscard]] std::optional<DataPeerConfig> FindPeer(std::string_view drone_id) const;
};

/// ---------------------------------------------------------------------------
/// AgentConfig -- startup parameters for the gRPC agent server.
/// ---------------------------------------------------------------------------
struct AgentConfig {
    std::string agent_id{"agent-1"};         ///< Unique identifier for this agent.
    std::string bind_addr{"0.0.0.0:50061"};  ///< gRPC listen address.
    int default_authority_ttl_ms{kDefaultAuthorityTtlMs};
    int default_telemetry_rate_hz{kDefaultTelemetryRateHz};
    int min_telemetry_rate_hz{kMinimumTelemetryRateHz};
    ReportPersistenceConfig reports{};
    DataPlaneConfig data{};
    VehicleProfile vehicle_profile{};
    SafetyPolicy safety{};
    AgentSecurityConfig security{};

    /// @brief Validate address, telemetry, authority, profile, data, reports, and security.
    [[nodiscard]] core::Result Validate() const;

    /// @brief Overlay matching environment variables onto this config.
    /// @param prefix Environment prefix, normally "SWARMKIT_AGENT_".
    void ApplyEnvironment(std::string_view prefix = "SWARMKIT_AGENT_");
};

/// @brief Load agent configuration from a YAML file.
///
/// Supports either a root-level agent map or an `agent:` section.
///
/// @param path YAML file path.
/// @return Parsed AgentConfig, or a validation/loading error.
[[nodiscard]] std::expected<AgentConfig, core::Result> LoadAgentConfigFromFile(
    const std::string& path);

/// @brief Start the agent gRPC server.
///
/// Blocks until the server shuts down.  Returns 0 on clean exit, non-zero on
/// error (e.g. port already in use, null backend).
///
/// @param config Validated agent server configuration.
/// @param backend Owning backend pointer. Must not be null.
/// @return Process-style exit code.
[[nodiscard]] int RunAgentServer(const AgentConfig& config, DroneBackendPtr backend);

}  // namespace swarmkit::agent
