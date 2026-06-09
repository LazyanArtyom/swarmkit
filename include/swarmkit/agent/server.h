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

struct VehicleProfile {
    std::string profile_id{"generic-quad"};
    float cruise_speed_mps{kDefaultCruiseSpeedMps};
    float climb_speed_mps{kDefaultClimbSpeedMps};
    float descent_speed_mps{kDefaultDescentSpeedMps};
    float max_altitude_m{kDefaultMaxAltitudeM};
    float min_battery_percent{kDefaultMinBatteryPercent};
    float battery_reserve_percent{kDefaultBatteryReservePercent};
    float tracking_tolerance_m{kDefaultTrackingToleranceM};
    int goal_margin_ms{kDefaultGoalMarginMs};
    int takeoff_timeout_margin_ms{kDefaultTakeoffTimeoutMarginMs};
    int land_timeout_margin_ms{kDefaultLandTimeoutMarginMs};
    int max_goal_timeout_ms{kDefaultMaxGoalTimeoutMs};
    int min_gps_fix_type{kDefaultMinGpsFixType};
    int min_satellites_visible{kDefaultMinSatellitesVisible};
    float max_gps_hdop{kDefaultMaxGpsHdop};

    [[nodiscard]] core::Result Validate() const;
};

struct AgentSecurityConfig {
    core::TransportSecurityMode transport_security{core::TransportSecurityMode::kAuto};
    std::string root_ca_cert_path;
    std::string cert_chain_path;
    std::string private_key_path;
    std::vector<std::string> allowed_client_ids;

    [[nodiscard]] core::TransportSecurityMode EffectiveTransportSecurity() const;
    [[nodiscard]] core::Result Validate() const;
};

struct ReportPersistenceConfig {
    std::string log_file;
    std::string sequence_state_file;
    int backlog_size{kDefaultReportBacklogSize};
    int max_log_file_size_bytes{kDefaultReportLogMaxFileSizeBytes};
    int max_log_files{kDefaultReportLogMaxFiles};
    bool flush_each_write{true};
    bool fsync_each_write{false};
    bool replay_from_log{true};

    [[nodiscard]] core::Result Validate() const;
};

struct SafetyPolicy {
    /// Permit autonomous commands without normal flight-readiness checks.
    /// Intended only for restrained, propeller-off bench testing.
    bool allow_unsafe_bench_commands{false};
};

struct DataPeerConfig {
    std::string drone_id;
    std::string address;
    core::TransportSecurityMode transport_security{core::TransportSecurityMode::kAuto};
    std::string root_ca_cert_path;
    std::string cert_chain_path;
    std::string private_key_path;
    std::string server_authority_override;

    [[nodiscard]] core::Result Validate() const;
};

struct DataPlaneConfig {
    std::string artifact_dir{"/tmp/swarmkit-artifacts"};
    int message_backlog_size{kDefaultDataMessageBacklogSize};
    int max_message_payload_bytes{kDefaultDataMessageMaxPayloadBytes};
    int artifact_chunk_bytes{kDefaultArtifactChunkBytes};
    int max_artifact_bytes{kDefaultMaxArtifactBytes};
    int max_concurrent_artifact_transfers{kDefaultMaxConcurrentArtifactTransfers};
    int max_queued_artifact_transfers{kDefaultMaxQueuedArtifactTransfers};
    int max_concurrent_artifact_transfers_per_peer{
        kDefaultMaxConcurrentArtifactTransfersPerPeer};
    int artifact_peer_bytes_per_second{kDefaultArtifactPeerBytesPerSecond};
    std::vector<DataPeerConfig> peers;

    [[nodiscard]] core::Result Validate() const;
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

    [[nodiscard]] core::Result Validate() const;
    void ApplyEnvironment(std::string_view prefix = "SWARMKIT_AGENT_");
};

/// @brief Load agent configuration from a YAML file.
///
/// Supports either a root-level agent map or an `agent:` section.
[[nodiscard]] std::expected<AgentConfig, core::Result> LoadAgentConfigFromFile(
    const std::string& path);

/// @brief Start the agent gRPC server.
///
/// Blocks until the server shuts down.  Returns 0 on clean exit, non-zero on
/// error (e.g. port already in use, null backend).
[[nodiscard]] int RunAgentServer(const AgentConfig& config, DroneBackendPtr backend);

}  // namespace swarmkit::agent
