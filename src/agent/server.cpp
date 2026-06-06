// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/server.h"

#include <grpcpp/grpcpp.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <expected>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "active_goal_supervisor.h"
#include "command_preconditions.h"
#include "config_yaml.h"
#include "env_utils.h"
#include "report_hub.h"
#include "runtime_counters.h"
#include "security_utils.h"
#include "server_test_support.h"
#include "sha256.h"
#include "swarmkit/agent/arbiter.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/version.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"
#include "telemetry_manager.h"
#include "trajectory_execution_manager.h"

namespace swarmkit::agent {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)
using core::internal::GetEnvValue;
using core::internal::IsValidPriority;
using core::internal::LooksLikeAddress;
using core::internal::MakeCorrelationId;
using core::internal::ParseIntValue;

namespace {
using grpc::Status;

constexpr int kMillisecondsPerSecond = 1000;
constexpr std::string_view kCorrelationMetadataKey = "x-correlation-id";
constexpr std::string_view kAgentEnvId = "AGENT_ID";
constexpr std::string_view kAgentEnvBindAddr = "BIND_ADDR";
constexpr std::string_view kAgentEnvDefaultAuthorityTtlMs = "DEFAULT_AUTHORITY_TTL_MS";
constexpr std::string_view kAgentEnvDefaultTelemetryRateHz = "DEFAULT_TELEMETRY_RATE_HZ";
constexpr std::string_view kAgentEnvMinTelemetryRateHz = "MIN_TELEMETRY_RATE_HZ";
constexpr std::string_view kAgentEnvAllowUnsafeBenchCommands = "ALLOW_UNSAFE_BENCH_COMMANDS";
constexpr std::string_view kAgentEnvReportLogFile = "REPORT_LOG_FILE";
constexpr std::string_view kAgentEnvReportSequenceStateFile = "REPORT_SEQUENCE_STATE_FILE";
constexpr std::string_view kAgentEnvReportBacklogSize = "REPORT_BACKLOG_SIZE";
constexpr std::string_view kAgentEnvReportLogMaxFileSizeBytes = "REPORT_LOG_MAX_FILE_SIZE_BYTES";
constexpr std::string_view kAgentEnvReportLogMaxFiles = "REPORT_LOG_MAX_FILES";
constexpr std::string_view kAgentEnvReportFlushEachWrite = "REPORT_FLUSH_EACH_WRITE";
constexpr std::string_view kAgentEnvReportFsyncEachWrite = "REPORT_FSYNC_EACH_WRITE";
constexpr std::string_view kAgentEnvReportReplayFromLog = "REPORT_REPLAY_FROM_LOG";
constexpr std::string_view kAgentEnvArtifactDir = "ARTIFACT_DIR";
constexpr std::string_view kAgentEnvDataMessageBacklogSize = "DATA_MESSAGE_BACKLOG_SIZE";
constexpr std::string_view kAgentEnvDataMaxMessagePayloadBytes =
    "DATA_MAX_MESSAGE_PAYLOAD_BYTES";
constexpr std::string_view kAgentEnvDataArtifactChunkBytes = "DATA_ARTIFACT_CHUNK_BYTES";
constexpr std::string_view kAgentEnvDataMaxArtifactBytes = "DATA_MAX_ARTIFACT_BYTES";
constexpr std::string_view kAgentEnvRootCaCertPath = "ROOT_CA_CERT_PATH";
constexpr std::string_view kAgentEnvCertChainPath = "CERT_CHAIN_PATH";
constexpr std::string_view kAgentEnvPrivateKeyPath = "PRIVATE_KEY_PATH";
constexpr std::string_view kAgentEnvAllowedClientIds = "ALLOWED_CLIENT_IDS";
constexpr std::string_view kAgentEnvTransportSecurity = "TRANSPORT_SECURITY";

/// @brief Watcher poll interval while blocking inside WatchAuthority RPC.
constexpr auto kWatchPollInterval = std::chrono::milliseconds{100};
constexpr auto kTelemetryWaitTimeout = std::chrono::milliseconds{200};

/// @name Time helpers
/// @{

[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] std::string SanitizedPathComponent(std::string value) {
    if (value.empty()) {
        return "artifact";
    }
    for (char& character : value) {
        const bool ok = (character >= 'a' && character <= 'z') ||
                        (character >= 'A' && character <= 'Z') ||
                        (character >= '0' && character <= '9') || character == '.' ||
                        character == '-' || character == '_';
        if (!ok) {
            character = '_';
        }
    }
    return value;
}

[[nodiscard]] bool MessageExpired(const swarmkit::v1::DataMessage& message,
                                  std::int64_t now_ms) {
    return message.ttl_ms() > 0 && message.unix_time_ms() > 0 &&
           now_ms - message.unix_time_ms() > message.ttl_ms();
}

[[nodiscard]] bool ArtifactExpired(const swarmkit::v1::ArtifactDescriptor& descriptor,
                                   std::int64_t now_ms) {
    return descriptor.ttl_ms() > 0 && descriptor.created_unix_ms() > 0 &&
           now_ms - descriptor.created_unix_ms() > descriptor.ttl_ms();
}

[[nodiscard]] core::TransportSecurityMode EffectivePeerTransportSecurity(
    const DataPeerConfig& peer, const AgentSecurityConfig& fallback) {
    if (peer.transport_security != core::TransportSecurityMode::kAuto) {
        return peer.transport_security;
    }
    return fallback.EffectiveTransportSecurity();
}

[[nodiscard]] std::shared_ptr<grpc::ChannelCredentials> MakePeerChannelCredentials(
    const DataPeerConfig& peer, const AgentSecurityConfig& fallback) {
    const core::TransportSecurityMode mode = EffectivePeerTransportSecurity(peer, fallback);
    if (mode == core::TransportSecurityMode::kInsecure) {
        return grpc::InsecureChannelCredentials();
    }

    grpc::SslCredentialsOptions options;
    const std::string root_ca =
        peer.root_ca_cert_path.empty() ? fallback.root_ca_cert_path : peer.root_ca_cert_path;
    const std::string cert_chain =
        peer.cert_chain_path.empty() ? fallback.cert_chain_path : peer.cert_chain_path;
    const std::string private_key =
        peer.private_key_path.empty() ? fallback.private_key_path : peer.private_key_path;
    static_cast<void>(core::internal::ReadTextFile(root_ca, &options.pem_root_certs));
    if (mode == core::TransportSecurityMode::kMutualTls) {
        static_cast<void>(core::internal::ReadTextFile(private_key, &options.pem_private_key));
        static_cast<void>(core::internal::ReadTextFile(cert_chain, &options.pem_cert_chain));
    }
    return grpc::SslCredentials(options);
}

[[nodiscard]] std::shared_ptr<grpc::Channel> MakePeerChannel(
    const DataPeerConfig& peer, const AgentSecurityConfig& fallback) {
    const auto credentials = MakePeerChannelCredentials(peer, fallback);
    if (peer.server_authority_override.empty() ||
        EffectivePeerTransportSecurity(peer, fallback) == core::TransportSecurityMode::kInsecure) {
        return grpc::CreateChannel(peer.address, credentials);
    }
    grpc::ChannelArguments arguments;
    arguments.SetSslTargetNameOverride(peer.server_authority_override);
    return grpc::CreateCustomChannel(peer.address, credentials, arguments);
}

[[nodiscard]] bool MessageMatchesSubscription(const swarmkit::v1::DataMessage& message,
                                              const swarmkit::v1::MessageSubscription& sub) {
    const bool message_is_broadcast = message.target_id().empty();
    if (!sub.target_id().empty() && !message_is_broadcast &&
        message.target_id() != sub.target_id() && message.source_id() != sub.target_id()) {
        return false;
    }
    if (sub.target_id().empty() && !sub.subscriber_id().empty() && !message_is_broadcast &&
        message.target_id() != sub.subscriber_id()) {
        return false;
    }
    if (sub.topics().empty()) {
        return true;
    }
    return std::ranges::any_of(sub.topics(), [&message](const std::string& topic) {
        return topic == message.topic();
    });
}

[[nodiscard]] bool HasFreshTimestamp(std::int64_t unix_time_ms) {
    return unix_time_ms > 0;
}

void AddReadinessCheck(swarmkit::v1::HealthReply* reply, std::string_view name, bool ok,
                       std::string detail, swarmkit::v1::ReadinessCheckSeverity severity) {
    auto* check = reply->add_readiness_checks();
    check->set_name(std::string(name));
    check->set_ok(ok);
    check->set_detail(std::move(detail));
    check->set_severity(ok ? swarmkit::v1::READINESS_INFO : severity);
}

void PopulateReadiness(const BackendHealth& health, bool ready, swarmkit::v1::HealthReply* reply) {
    if (reply == nullptr) {
        return;
    }

    const bool heartbeat_ok = HasFreshTimestamp(health.last_heartbeat_unix_ms);
    const bool telemetry_ok = HasFreshTimestamp(health.last_telemetry_unix_ms);
    const bool not_failsafe = !health.failsafe;
    const bool checks_ok =
        ready && heartbeat_ok && telemetry_ok && health.gps_ok && health.ekf_ok && not_failsafe;
    reply->set_autonomous_ready(checks_ok);

    AddReadinessCheck(reply, "backend", ready, ready ? health.message : "not ready: " + health.message,
                      swarmkit::v1::READINESS_ERROR);
    AddReadinessCheck(reply, "heartbeat", heartbeat_ok,
                      heartbeat_ok ? "heartbeat observed" : "no vehicle heartbeat observed",
                      swarmkit::v1::READINESS_ERROR);
    AddReadinessCheck(reply, "telemetry", telemetry_ok,
                      telemetry_ok ? "telemetry observed" : "no vehicle telemetry observed",
                      swarmkit::v1::READINESS_ERROR);
    AddReadinessCheck(reply, "gps", health.gps_ok,
                      "fix_type=" + std::to_string(health.gps_fix_type) + " sats=" +
                          std::to_string(health.satellites_visible) + " hdop=" +
                          std::to_string(health.gps_hdop),
                      swarmkit::v1::READINESS_ERROR);
    AddReadinessCheck(reply, "ekf", health.ekf_ok,
                      health.ekf_ok ? "healthy" : "unhealthy", swarmkit::v1::READINESS_ERROR);
    AddReadinessCheck(reply, "failsafe", not_failsafe,
                      health.failsafe ? "failsafe active" : "inactive",
                      swarmkit::v1::READINESS_ERROR);
    AddReadinessCheck(reply, "armed", !health.armed,
                      health.armed ? "vehicle already armed" : "vehicle disarmed",
                      swarmkit::v1::READINESS_WARNING);
    AddReadinessCheck(reply, "landed", health.landed,
                      std::string{"landed="} + (health.landed ? "true" : "false"),
                      swarmkit::v1::READINESS_WARNING);

    if (!ready) {
        reply->add_arming_blockers("backend_not_ready");
    }
    if (!heartbeat_ok) {
        reply->add_arming_blockers("heartbeat_missing");
    }
    if (!telemetry_ok) {
        reply->add_arming_blockers("telemetry_missing");
    }
    if (!health.gps_ok) {
        reply->add_arming_blockers("gps_unhealthy");
    }
    if (!health.ekf_ok) {
        reply->add_arming_blockers("ekf_unhealthy");
    }
    if (health.failsafe) {
        reply->add_arming_blockers("failsafe_active");
    }
    if (health.armed) {
        reply->add_arming_blockers("already_armed");
    }
    if (!health.landed) {
        reply->add_arming_blockers("not_landed");
    }
}

[[nodiscard]] bool GoalUsesLocalTarget(const swarmkit::v1::ActiveGoal& goal) {
    return goal.use_local_target() || goal.target_frame() == "local-ned";
}

[[nodiscard]] internal::ReportHubOptions MakeReportHubOptions(
    const ReportPersistenceConfig& config) {
    return {
        .report_log_file = config.log_file,
        .sequence_state_file = config.sequence_state_file,
        .max_in_memory_backlog = static_cast<std::size_t>(config.backlog_size),
        .max_log_file_size_bytes = static_cast<std::size_t>(config.max_log_file_size_bytes),
        .max_log_files = config.max_log_files,
        .flush_each_write = config.flush_each_write,
        .fsync_each_write = config.fsync_each_write,
        .replay_from_log = config.replay_from_log,
    };
}

[[nodiscard]] swarmkit::v1::TelemetryCoordinateFrame ToProtoCoordinateFrame(
    core::CoordinateFrame frame) {
    switch (frame) {
        case core::CoordinateFrame::kWgs84:
            return swarmkit::v1::TELEMETRY_COORDINATE_FRAME_WGS84;
        case core::CoordinateFrame::kLocalNed:
            return swarmkit::v1::TELEMETRY_COORDINATE_FRAME_LOCAL_NED;
        case core::CoordinateFrame::kBodyNed:
            return swarmkit::v1::TELEMETRY_COORDINATE_FRAME_BODY_NED;
        case core::CoordinateFrame::kUnknown:
        default:
            return swarmkit::v1::TELEMETRY_COORDINATE_FRAME_UNSPECIFIED;
    }
}

[[nodiscard]] swarmkit::v1::TelemetryGpsQuality ToProtoGpsQuality(core::GpsQuality quality) {
    switch (quality) {
        case core::GpsQuality::kNoFix:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_NO_FIX;
        case core::GpsQuality::kFix2D:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_2D_FIX;
        case core::GpsQuality::kFix3D:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_3D_FIX;
        case core::GpsQuality::kDgps:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_DGPS;
        case core::GpsQuality::kRtkFloat:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_RTK_FLOAT;
        case core::GpsQuality::kRtkFixed:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_RTK_FIXED;
        case core::GpsQuality::kStatic:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_STATIC;
        case core::GpsQuality::kPpp:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_PPP;
        case core::GpsQuality::kUnknown:
        default:
            return swarmkit::v1::TELEMETRY_GPS_QUALITY_UNSPECIFIED;
    }
}

[[nodiscard]] swarmkit::v1::TelemetryEstimatorState ToProtoEstimatorState(
    core::EstimatorState state) {
    switch (state) {
        case core::EstimatorState::kInitializing:
            return swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_INITIALIZING;
        case core::EstimatorState::kHealthy:
            return swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_HEALTHY;
        case core::EstimatorState::kDegraded:
            return swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_DEGRADED;
        case core::EstimatorState::kFault:
            return swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_FAULT;
        case core::EstimatorState::kUnknown:
        default:
            return swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_UNSPECIFIED;
    }
}

void PopulateTelemetryProto(const core::TelemetryFrame& frame,
                            swarmkit::v1::TelemetryFrame* out) {
    if (out == nullptr) {
        return;
    }
    out->set_drone_id(frame.drone_id);
    out->set_unix_time_ms(frame.unix_time_ms);
    out->set_lat_deg(frame.lat_deg);
    out->set_lon_deg(frame.lon_deg);
    out->set_rel_alt_m(frame.rel_alt_m);
    out->set_abs_alt_m(frame.abs_alt_m);
    out->set_vx_mps(frame.vx_mps);
    out->set_vy_mps(frame.vy_mps);
    out->set_vz_mps(frame.vz_mps);
    out->set_roll_deg(frame.roll_deg);
    out->set_pitch_deg(frame.pitch_deg);
    out->set_yaw_deg(frame.yaw_deg);
    out->set_battery_percent(frame.battery_percent);
    out->set_mode(frame.mode);
    out->set_armed(frame.armed);
    out->set_landed(frame.landed);
    out->set_failsafe(frame.failsafe);
    out->set_ekf_ok(frame.ekf_ok);
    out->set_gps_fix_type(frame.gps_fix_type);
    out->set_satellites_visible(frame.satellites_visible);
    out->set_gps_hdop(frame.gps_hdop);
    out->set_link_quality_percent(frame.link_quality_percent);
    out->set_source_unix_time_ms(frame.source_unix_time_ms);
    out->set_source_time_boot_ms(frame.source_time_boot_ms);
    out->set_position_frame(ToProtoCoordinateFrame(frame.position_frame));
    out->set_velocity_frame(ToProtoCoordinateFrame(frame.velocity_frame));

    auto* validity = out->mutable_validity();
    validity->set_position(frame.validity.position);
    validity->set_relative_altitude(frame.validity.relative_altitude);
    validity->set_absolute_altitude(frame.validity.absolute_altitude);
    validity->set_velocity(frame.validity.velocity);
    validity->set_attitude(frame.validity.attitude);
    validity->set_battery(frame.validity.battery);
    validity->set_mode(frame.validity.mode);
    validity->set_armed(frame.validity.armed);
    validity->set_landed(frame.validity.landed);
    validity->set_failsafe(frame.validity.failsafe);
    validity->set_gps(frame.validity.gps);
    validity->set_gps_hdop(frame.validity.gps_hdop);
    validity->set_link_quality(frame.validity.link_quality);
    validity->set_estimator(frame.validity.estimator);
    validity->set_home_origin(frame.validity.home_origin);

    auto* accuracy = out->mutable_accuracy();
    accuracy->set_horizontal_position_valid(frame.accuracy.horizontal_position_valid);
    accuracy->set_horizontal_position_m(frame.accuracy.horizontal_position_m);
    accuracy->set_vertical_position_valid(frame.accuracy.vertical_position_valid);
    accuracy->set_vertical_position_m(frame.accuracy.vertical_position_m);
    accuracy->set_velocity_valid(frame.accuracy.velocity_valid);
    accuracy->set_velocity_mps(frame.accuracy.velocity_mps);
    accuracy->set_heading_valid(frame.accuracy.heading_valid);
    accuracy->set_heading_deg(frame.accuracy.heading_deg);
    accuracy->set_attitude_valid(frame.accuracy.attitude_valid);
    accuracy->set_attitude_deg(frame.accuracy.attitude_deg);
    accuracy->set_position_covariance_valid(frame.accuracy.position_covariance_valid);
    if (frame.accuracy.position_covariance_valid) {
        for (float value : frame.accuracy.position_covariance) {
            accuracy->add_position_covariance(value);
        }
    }
    accuracy->set_velocity_covariance_valid(frame.accuracy.velocity_covariance_valid);
    if (frame.accuracy.velocity_covariance_valid) {
        for (float value : frame.accuracy.velocity_covariance) {
            accuracy->add_velocity_covariance(value);
        }
    }

    if (frame.validity.home_origin) {
        auto* home = out->mutable_home_origin();
        home->set_frame(ToProtoCoordinateFrame(frame.home_origin.frame));
        home->set_lat_deg(frame.home_origin.lat_deg);
        home->set_lon_deg(frame.home_origin.lon_deg);
        home->set_alt_m(frame.home_origin.alt_m);
        home->set_north_m(frame.home_origin.north_m);
        home->set_east_m(frame.home_origin.east_m);
        home->set_down_m(frame.home_origin.down_m);
    }

    out->set_gps_quality(ToProtoGpsQuality(frame.gps_quality));
    out->set_estimator_state(ToProtoEstimatorState(frame.estimator_state));
    out->set_estimator_flags(frame.estimator_flags);
    out->set_estimator_position_ok(frame.estimator_position_ok);
    out->set_estimator_velocity_ok(frame.estimator_velocity_ok);
    out->set_estimator_attitude_ok(frame.estimator_attitude_ok);
    out->set_active_command_id(frame.active_command_id);
    out->set_active_goal_id(frame.active_goal_id);
    out->set_active_execution_id(frame.active_execution_id);
    out->set_correlation_id(frame.correlation_id);
}

/// @}

/// @name Correlation ID resolution
/// @{

[[nodiscard]] std::string ResolveCorrelationId(grpc::ServerContext* ctx,
                                               std::string_view request_correlation_id,
                                               std::string_view fallback_prefix) {
    if (!request_correlation_id.empty()) {
        return std::string(request_correlation_id);
    }
    if (ctx != nullptr) {
        const auto kMetadataIter =
            ctx->client_metadata().find(std::string(kCorrelationMetadataKey));
        if (kMetadataIter != ctx->client_metadata().end()) {
            return std::string(kMetadataIter->second.data(), kMetadataIter->second.length());
        }
    }
    return MakeCorrelationId(fallback_prefix);
}

/// @}

/// @name Validation helpers
/// @{

[[nodiscard]] core::Result ValidateCommandContext(const CommandContext& context) {
    if (context.drone_id.empty()) {
        return core::Result::Rejected("ctx.drone_id must not be empty");
    }
    if (context.client_id.empty()) {
        return core::Result::Rejected("ctx.client_id must not be empty");
    }
    if (!IsValidPriority(context.priority)) {
        return core::Result::Rejected("ctx.priority is not a supported CommandPriority");
    }
    const auto kEpoch = std::chrono::system_clock::time_point{};
    if (context.deadline != kEpoch && context.deadline <= std::chrono::system_clock::now()) {
        return core::Result::Failed("command deadline already expired");
    }
    return core::Result::Success();
}

[[nodiscard]] core::Result ValidateLockRequest(const CommandContext& context, std::int64_t ttl_ms) {
    if (ttl_ms < 0) {
        return core::Result::Rejected("ttl_ms must be >= 0");
    }
    return ValidateCommandContext(context);
}

[[nodiscard]] core::Result ValidateTelemetryRequest(const swarmkit::v1::TelemetryRequest* req) {
    if (req == nullptr) {
        return core::Result::Success();
    }
    if (req->rate_hz() < 0) {
        return core::Result::Rejected("telemetry rate_hz must be >= 0");
    }
    return core::Result::Success();
}

[[nodiscard]] std::vector<std::string> SplitCsvList(std::string_view value) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start < value.size()) {
        const std::size_t end = value.find(',', start);
        const std::string_view token =
            end == std::string_view::npos ? value.substr(start) : value.substr(start, end - start);
        const std::string kTrimmed = core::internal::TrimWhitespace(token);
        if (!kTrimmed.empty()) {
            out.push_back(kTrimmed);
        }
        if (end == std::string_view::npos) {
            break;
        }
        start = end + 1;
    }
    return out;
}

[[nodiscard]] std::optional<std::string> FindPeerAuthProperty(grpc::ServerContext* ctx,
                                                              std::string_view property_name) {
    if (ctx == nullptr) {
        return std::nullopt;
    }

    const auto kAuthContext = ctx->auth_context();
    if (!kAuthContext || !kAuthContext->IsPeerAuthenticated()) {
        return std::nullopt;
    }

    const auto kValues = kAuthContext->FindPropertyValues(std::string(property_name));
    if (kValues.empty()) {
        return std::nullopt;
    }
    return std::string(kValues.front().data(), kValues.front().size());
}

[[nodiscard]] std::optional<std::string> ResolvePeerIdentity(grpc::ServerContext* ctx) {
    if (const auto kCommonName = FindPeerAuthProperty(ctx, "x509_common_name");
        kCommonName.has_value()) {
        return kCommonName;
    }
    if (const auto kSubjectAltName = FindPeerAuthProperty(ctx, "x509_subject_alternative_name");
        kSubjectAltName.has_value()) {
        return kSubjectAltName;
    }
    return std::nullopt;
}

[[nodiscard]] core::Result AuthorizePeer(grpc::ServerContext* ctx,
                                         const AgentSecurityConfig& security,
                                         std::string* requested_client_id) {
    if (security.EffectiveTransportSecurity() != core::TransportSecurityMode::kMutualTls) {
        return core::Result::Success();
    }

    const auto kPeerIdentity = ResolvePeerIdentity(ctx);
    if (!kPeerIdentity.has_value()) {
        return core::Result::Rejected("authenticated peer identity is required");
    }

    if (!security.allowed_client_ids.empty() &&
        std::ranges::find(security.allowed_client_ids, *kPeerIdentity) ==
            security.allowed_client_ids.end()) {
        return core::Result::Rejected("peer identity '" + *kPeerIdentity + "' is not authorized");
    }

    if (requested_client_id == nullptr) {
        return core::Result::Success();
    }
    if (requested_client_id->empty()) {
        *requested_client_id = *kPeerIdentity;
        return core::Result::Success();
    }
    if (*requested_client_id != *kPeerIdentity) {
        return core::Result::Rejected(
            "request client_id must match the authenticated peer identity");
    }
    return core::Result::Success();
}

[[nodiscard]] std::shared_ptr<grpc::ServerCredentials> MakeServerCredentials(
    const AgentSecurityConfig& security, core::Result* out_error) {
    if (out_error != nullptr) {
        *out_error = core::Result::Success();
    }

    const core::TransportSecurityMode mode = security.EffectiveTransportSecurity();
    if (mode == core::TransportSecurityMode::kInsecure) {
        return grpc::InsecureServerCredentials();
    }

    std::string cert_chain;
    if (const core::Result kResult =
            core::internal::ReadTextFile(security.cert_chain_path, &cert_chain);
        !kResult.IsOk()) {
        if (out_error != nullptr) {
            *out_error = kResult;
        }
        return {};
    }

    std::string private_key;
    if (const core::Result kResult =
            core::internal::ReadTextFile(security.private_key_path, &private_key);
        !kResult.IsOk()) {
        if (out_error != nullptr) {
            *out_error = kResult;
        }
        return {};
    }

    grpc::SslServerCredentialsOptions options;
    options.pem_key_cert_pairs.push_back(grpc::SslServerCredentialsOptions::PemKeyCertPair{
        .private_key = private_key,
        .cert_chain = cert_chain,
    });

    if (mode == core::TransportSecurityMode::kMutualTls) {
        if (const core::Result kResult =
                core::internal::ReadTextFile(security.root_ca_cert_path, &options.pem_root_certs);
            !kResult.IsOk()) {
            if (out_error != nullptr) {
                *out_error = kResult;
            }
            return {};
        }
        options.client_certificate_request =
            GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY;
    } else {
        options.client_certificate_request = GRPC_SSL_DONT_REQUEST_CLIENT_CERTIFICATE;
    }

    return grpc::SslServerCredentials(options);
}

/// @}

/// @name Proto conversion helpers
/// @{

[[nodiscard]] swarmkit::v1::ErrorCode ToProtoErrorCode(core::StatusCode code) {
    using ProtoCode = swarmkit::v1::ErrorCode;
    switch (code) {
        case core::StatusCode::kOk:
            return ProtoCode::ERROR_CODE_NONE;
        case core::StatusCode::kRejected:
            return ProtoCode::ERROR_CODE_REJECTED;
        case core::StatusCode::kFailed:
            return ProtoCode::ERROR_CODE_INTERNAL;
    }
    return ProtoCode::ERROR_CODE_INTERNAL;
}

[[nodiscard]] swarmkit::v1::CommandReply::Status ToProtoStatus(core::StatusCode code) {
    using ReplyStatus = swarmkit::v1::CommandReply::Status;
    switch (code) {
        case core::StatusCode::kOk:
            return ReplyStatus::CommandReply_Status_OK;
        case core::StatusCode::kRejected:
            return ReplyStatus::CommandReply_Status_REJECTED;
        case core::StatusCode::kFailed:
            return ReplyStatus::CommandReply_Status_FAILED;
    }
    return ReplyStatus::CommandReply_Status_STATUS_UNSPECIFIED;
}

[[nodiscard]] CommandContext ToCoreContext(const swarmkit::v1::CommandContext& proto) {
    CommandContext context;
    context.drone_id = proto.drone_id();
    context.client_id = proto.client_id();
    context.priority = static_cast<CommandPriority>(proto.priority());
    context.correlation_id = proto.correlation_id();

    if (proto.deadline_unix_ms() > 0) {
        context.deadline = std::chrono::system_clock::time_point{
            std::chrono::milliseconds{proto.deadline_unix_ms()}};
    }

    return context;
}

[[nodiscard]] std::string ProtoCommandName(const swarmkit::v1::Command& proto) {
    switch (proto.kind_case()) {
        case swarmkit::v1::Command::kArm:
            return "ARM";
        case swarmkit::v1::Command::kForceArm:
            return "FORCE_ARM";
        case swarmkit::v1::Command::kDisarm:
            return "DISARM";
        case swarmkit::v1::Command::kLand:
            return "LAND";
        case swarmkit::v1::Command::kSetMode:
            return "SET_MODE(" + proto.set_mode().mode() + ")";
        case swarmkit::v1::Command::kForceDisarm:
            return "FORCE_DISARM";
        case swarmkit::v1::Command::kFlightTerminate:
            return "FLIGHT_TERMINATE";
        case swarmkit::v1::Command::kTakeoff:
            return "TAKEOFF(" + std::to_string(static_cast<int>(proto.takeoff().alt_m())) + "m)";
        case swarmkit::v1::Command::kSetWaypoint:
            return "WAYPOINT(lat=" + std::to_string(proto.set_waypoint().lat_deg()) +
                   " lon=" + std::to_string(proto.set_waypoint().lon_deg()) +
                   " alt=" + std::to_string(static_cast<int>(proto.set_waypoint().alt_m())) + "m)";
        case swarmkit::v1::Command::kReturnHome:
            return "RETURN_HOME";
        case swarmkit::v1::Command::kHoldPosition:
            return "HOLD";
        case swarmkit::v1::Command::kSetSpeed:
            return "SET_SPEED";
        case swarmkit::v1::Command::kGotoPosition:
            return "GOTO";
        case swarmkit::v1::Command::kPause:
            return "PAUSE";
        case swarmkit::v1::Command::kResume:
            return "RESUME";
        case swarmkit::v1::Command::kSetYaw:
            return "SET_YAW";
        case swarmkit::v1::Command::kVelocity:
            return "VELOCITY";
        case swarmkit::v1::Command::kSetHome:
            return "SET_HOME";
        case swarmkit::v1::Command::kUploadMission:
            return "UPLOAD_MISSION";
        case swarmkit::v1::Command::kClearMission:
            return "CLEAR_MISSION";
        case swarmkit::v1::Command::kStartMission:
            return "START_MISSION";
        case swarmkit::v1::Command::kPauseMission:
            return "PAUSE_MISSION";
        case swarmkit::v1::Command::kResumeMission:
            return "RESUME_MISSION";
        case swarmkit::v1::Command::kSetCurrentMissionItem:
            return "SET_CURRENT_MISSION_ITEM";
        case swarmkit::v1::Command::kPhoto:
            return "PHOTO";
        case swarmkit::v1::Command::kPhotoIntervalStart:
            return "PHOTO_INTERVAL_START";
        case swarmkit::v1::Command::kPhotoIntervalStop:
            return "PHOTO_INTERVAL_STOP";
        case swarmkit::v1::Command::kVideoStart:
            return "VIDEO_START";
        case swarmkit::v1::Command::kVideoStop:
            return "VIDEO_STOP";
        case swarmkit::v1::Command::kGimbalPoint:
            return "GIMBAL_POINT";
        case swarmkit::v1::Command::kRoiLocation:
            return "ROI_LOCATION";
        case swarmkit::v1::Command::kRoiClear:
            return "ROI_CLEAR";
        case swarmkit::v1::Command::kServo:
            return "SERVO";
        case swarmkit::v1::Command::kRelay:
            return "RELAY";
        case swarmkit::v1::Command::kGripper:
            return "GRIPPER";
        case swarmkit::v1::Command::kBackendCommand:
            return "BACKEND_COMMAND(" + proto.backend_command().backend_namespace() + "." +
                   proto.backend_command().name() + ")";
        default:
            return "UNKNOWN";
    }
}

[[nodiscard]] swarmkit::v1::AuthorityEvent::Kind ToProtoEventKind(AuthorityEvent::Kind kind) {
    using ProtoKind = swarmkit::v1::AuthorityEvent::Kind;
    switch (kind) {
        case AuthorityEvent::Kind::kGranted:
            return ProtoKind::AuthorityEvent_Kind_GRANTED;
        case AuthorityEvent::Kind::kPreempted:
            return ProtoKind::AuthorityEvent_Kind_PREEMPTED;
        case AuthorityEvent::Kind::kResumed:
            return ProtoKind::AuthorityEvent_Kind_RESUMED;
        case AuthorityEvent::Kind::kExpired:
            return ProtoKind::AuthorityEvent_Kind_EXPIRED;
    }
    return ProtoKind::AuthorityEvent_Kind_KIND_UNSPECIFIED;
}

[[nodiscard]] MissionItemType ToCoreMissionItemType(swarmkit::v1::MissionItemType type) {
    switch (type) {
        case swarmkit::v1::MISSION_ITEM_TAKEOFF:
            return MissionItemType::kTakeoff;
        case swarmkit::v1::MISSION_ITEM_LAND:
            return MissionItemType::kLand;
        case swarmkit::v1::MISSION_ITEM_LOITER:
            return MissionItemType::kLoiter;
        case swarmkit::v1::MISSION_ITEM_DELAY:
            return MissionItemType::kDelay;
        case swarmkit::v1::MISSION_ITEM_ACTION:
            return MissionItemType::kAction;
        case swarmkit::v1::MISSION_ITEM_PAYLOAD_ACTION:
            return MissionItemType::kPayloadAction;
        case swarmkit::v1::MissionItemType_INT_MIN_SENTINEL_DO_NOT_USE_:
        case swarmkit::v1::MissionItemType_INT_MAX_SENTINEL_DO_NOT_USE_:
        case swarmkit::v1::MISSION_ITEM_WAYPOINT:
            return MissionItemType::kWaypoint;
    }
    return MissionItemType::kWaypoint;
}

[[nodiscard]] std::expected<Command, core::Result> ConvertProtoCommand(
    const swarmkit::v1::Command& proto) {
    switch (proto.kind_case()) {
        case swarmkit::v1::Command::kArm:
            return FlightCmd{CmdArm{}};
        case swarmkit::v1::Command::kForceArm:
            return FlightCmd{CmdForceArm{}};
        case swarmkit::v1::Command::kDisarm:
            return FlightCmd{CmdDisarm{}};
        case swarmkit::v1::Command::kLand:
            return FlightCmd{CmdLand{}};
        case swarmkit::v1::Command::kTakeoff:
            return FlightCmd{CmdTakeoff{proto.takeoff().alt_m()}};
        case swarmkit::v1::Command::kSetMode:
            return FlightCmd{CmdSetMode{
                .mode = proto.set_mode().mode(),
                .custom_mode = proto.set_mode().custom_mode(),
            }};
        case swarmkit::v1::Command::kForceDisarm:
            return FlightCmd{CmdForceDisarm{}};
        case swarmkit::v1::Command::kFlightTerminate:
            return FlightCmd{CmdFlightTerminate{}};

        case swarmkit::v1::Command::kSetWaypoint: {
            CmdSetWaypoint waypoint;
            waypoint.lat_deg = proto.set_waypoint().lat_deg();
            waypoint.lon_deg = proto.set_waypoint().lon_deg();
            waypoint.alt_m = proto.set_waypoint().alt_m();
            waypoint.speed_mps = proto.set_waypoint().speed_mps();
            return NavCmd{waypoint};
        }
        case swarmkit::v1::Command::kReturnHome:
            return NavCmd{CmdReturnHome{}};
        case swarmkit::v1::Command::kHoldPosition:
            return NavCmd{CmdHoldPosition{}};
        case swarmkit::v1::Command::kSetSpeed:
            return NavCmd{CmdSetSpeed{proto.set_speed().ground_mps()}};
        case swarmkit::v1::Command::kGotoPosition: {
            CmdGoto go_to;
            go_to.lat_deg = proto.goto_position().lat_deg();
            go_to.lon_deg = proto.goto_position().lon_deg();
            go_to.alt_m = proto.goto_position().alt_m();
            go_to.speed_mps = proto.goto_position().speed_mps();
            go_to.yaw_deg = proto.goto_position().yaw_deg();
            go_to.use_yaw = proto.goto_position().use_yaw();
            return NavCmd{go_to};
        }
        case swarmkit::v1::Command::kPause:
            return NavCmd{CmdPause{}};
        case swarmkit::v1::Command::kResume:
            return NavCmd{CmdResume{}};
        case swarmkit::v1::Command::kSetYaw:
            return NavCmd{CmdSetYaw{
                .yaw_deg = proto.set_yaw().yaw_deg(),
                .rate_deg_s = proto.set_yaw().rate_deg_s(),
                .relative = proto.set_yaw().relative(),
            }};
        case swarmkit::v1::Command::kVelocity:
            return NavCmd{CmdVelocity{
                .vx_mps = proto.velocity().vx_mps(),
                .vy_mps = proto.velocity().vy_mps(),
                .vz_mps = proto.velocity().vz_mps(),
                .duration_ms = proto.velocity().duration_ms(),
                .body_frame = proto.velocity().body_frame(),
            }};
        case swarmkit::v1::Command::kSetHome: {
            CmdSetHome home;
            home.use_current = proto.set_home().use_current();
            home.lat_deg = proto.set_home().lat_deg();
            home.lon_deg = proto.set_home().lon_deg();
            home.alt_m = proto.set_home().alt_m();
            return NavCmd{home};
        }

        case swarmkit::v1::Command::kUploadMission: {
            CmdUploadMission upload;
            upload.items.reserve(static_cast<std::size_t>(proto.upload_mission().items_size()));
            for (const auto& proto_item : proto.upload_mission().items()) {
                MissionItem item;
                item.type = ToCoreMissionItemType(proto_item.type());
                item.backend_command = static_cast<std::uint16_t>(proto_item.command());
                item.lat_deg = proto_item.lat_deg();
                item.lon_deg = proto_item.lon_deg();
                item.alt_m = proto_item.alt_m();
                item.hold_s = proto_item.hold_s();
                item.acceptance_radius_m = proto_item.acceptance_radius_m();
                item.yaw_deg = proto_item.yaw_deg();
                item.action_namespace = proto_item.action_namespace();
                item.action_name = proto_item.action_name();
                for (const auto& [key, value] : proto_item.params()) {
                    item.params.emplace(key, value);
                }
                item.param1 = proto_item.param1();
                item.param2 = proto_item.param2();
                item.param3 = proto_item.param3();
                item.param4 = proto_item.param4();
                item.current = proto_item.current();
                item.autocontinue = proto_item.autocontinue();
                upload.items.push_back(item);
            }
            return MissionCmd{std::move(upload)};
        }
        case swarmkit::v1::Command::kClearMission:
            return MissionCmd{CmdClearMission{}};
        case swarmkit::v1::Command::kStartMission:
            return MissionCmd{CmdStartMission{
                .first_item = proto.start_mission().first_item(),
                .last_item = proto.start_mission().last_item(),
            }};
        case swarmkit::v1::Command::kPauseMission:
            return MissionCmd{CmdPauseMission{}};
        case swarmkit::v1::Command::kResumeMission:
            return MissionCmd{CmdResumeMission{}};
        case swarmkit::v1::Command::kSetCurrentMissionItem:
            return MissionCmd{CmdSetCurrentMissionItem{proto.set_current_mission_item().seq()}};

        case swarmkit::v1::Command::kPhoto:
            return PayloadCmd{CmdPhoto{proto.photo().camera_id()}};
        case swarmkit::v1::Command::kPhotoIntervalStart:
            return PayloadCmd{CmdPhotoIntervalStart{
                .interval_s = proto.photo_interval_start().interval_s(),
                .count = proto.photo_interval_start().count(),
                .camera_id = proto.photo_interval_start().camera_id(),
            }};
        case swarmkit::v1::Command::kPhotoIntervalStop:
            return PayloadCmd{CmdPhotoIntervalStop{proto.photo_interval_stop().camera_id()}};
        case swarmkit::v1::Command::kVideoStart:
            return PayloadCmd{CmdVideoStart{
                .stream_id = proto.video_start().stream_id(),
                .camera_id = proto.video_start().camera_id(),
            }};
        case swarmkit::v1::Command::kVideoStop:
            return PayloadCmd{CmdVideoStop{
                .stream_id = proto.video_stop().stream_id(),
                .camera_id = proto.video_stop().camera_id(),
            }};
        case swarmkit::v1::Command::kGimbalPoint:
            return PayloadCmd{CmdGimbalPoint{
                .pitch_deg = proto.gimbal_point().pitch_deg(),
                .roll_deg = proto.gimbal_point().roll_deg(),
                .yaw_deg = proto.gimbal_point().yaw_deg(),
            }};
        case swarmkit::v1::Command::kRoiLocation: {
            CmdRoiLocation roi;
            roi.lat_deg = proto.roi_location().lat_deg();
            roi.lon_deg = proto.roi_location().lon_deg();
            roi.alt_m = proto.roi_location().alt_m();
            roi.gimbal_id = proto.roi_location().gimbal_id();
            return PayloadCmd{roi};
        }
        case swarmkit::v1::Command::kRoiClear:
            return PayloadCmd{CmdRoiClear{proto.roi_clear().gimbal_id()}};
        case swarmkit::v1::Command::kServo:
            return PayloadCmd{CmdServo{
                .servo = proto.servo().servo(),
                .pwm = proto.servo().pwm(),
            }};
        case swarmkit::v1::Command::kRelay:
            return PayloadCmd{CmdRelay{
                .relay = proto.relay().relay(),
                .enabled = proto.relay().enabled(),
            }};
        case swarmkit::v1::Command::kGripper:
            return PayloadCmd{CmdGripper{
                .gripper = proto.gripper().gripper(),
                .release = proto.gripper().release(),
            }};
        case swarmkit::v1::Command::kBackendCommand: {
            CmdBackendCommand command;
            command.backend_namespace = proto.backend_command().backend_namespace();
            command.name = proto.backend_command().name();
            for (const auto& [key, value] : proto.backend_command().params()) {
                command.params.emplace(key, value);
            }
            return BackendCmd{std::move(command)};
        }

        default:
            return std::unexpected(core::Result::Rejected("unknown command kind"));
    }
}

/// @}

/// @name AgentServiceImpl — gRPC service implementation
/// @{

/**
 * @brief Implements the AgentService gRPC service.
 *
 * Delegates telemetry management to TelemetryManager and runtime
 * counting to RuntimeCounters, keeping RPC handlers focused.
 */
class AgentServiceImpl final : public swarmkit::v1::AgentService::Service,
                               public swarmkit::v1::DataService::Service {
   public:
    AgentServiceImpl(AgentConfig config, DroneBackendPtr backend)
        : config_(std::move(config)),
          backend_(std::move(backend)),
          telemetry_(backend_.get(), config_.default_telemetry_rate_hz,
                     config_.min_telemetry_rate_hz),
          reports_(MakeReportHubOptions(config_.reports)),
          goals_(&telemetry_, &reports_, &config_),
          executions_(backend_.get(), &telemetry_, &reports_, &config_) {
        if (const core::Result start_result = backend_->Start(); !start_result.IsOk()) {
            ready_.store(false, std::memory_order_relaxed);
            startup_error_ = start_result.message;
            core::Logger::ErrorFmt("Agent backend failed to start: {}", start_result.message);
        }
    }

    // -- Unary RPCs -----------------------------------------------------------

    grpc::Status Ping(grpc::ServerContext* ctx, const swarmkit::v1::PingRequest* /*req*/,
                      swarmkit::v1::PingReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        counters_.IncrementPingRequests();
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "ping");

        reply->set_agent_id(config_.agent_id);
        reply->set_version(core::kSwarmkitVersionString);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_correlation_id(kCorrelationId);
        return grpc::Status::OK;
    }

    grpc::Status GetHealth(grpc::ServerContext* ctx, const swarmkit::v1::HealthRequest* /*req*/,
                           swarmkit::v1::HealthReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        counters_.IncrementHealthRequests();
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "health");
        BackendHealth backend_health = backend_->GetHealth();
        if (!startup_error_.empty()) {
            backend_health.ready = false;
            backend_health.message = startup_error_;
        }
        const bool kIsReady = ready_.load(std::memory_order_relaxed) && backend_health.ready;

        reply->set_ok(true);
        reply->set_ready(kIsReady);
        reply->set_agent_id(config_.agent_id);
        reply->set_version(core::kSwarmkitVersionString);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_message(kIsReady ? backend_health.message
                                    : "not ready: " + backend_health.message);
        reply->set_correlation_id(kCorrelationId);
        reply->set_backend_name(backend_health.backend_name);
        reply->set_protocol(backend_health.protocol);
        reply->set_last_heartbeat_unix_ms(backend_health.last_heartbeat_unix_ms);
        reply->set_last_telemetry_unix_ms(backend_health.last_telemetry_unix_ms);
        reply->set_armed(backend_health.armed);
        reply->set_landed(backend_health.landed);
        reply->set_mode(backend_health.mode);
        reply->set_custom_mode(backend_health.custom_mode);
        reply->set_failsafe(backend_health.failsafe);
        reply->set_gps_ok(backend_health.gps_ok);
        reply->set_gps_fix_type(backend_health.gps_fix_type);
        reply->set_satellites_visible(backend_health.satellites_visible);
        reply->set_gps_hdop(backend_health.gps_hdop);
        reply->set_ekf_ok(backend_health.ekf_ok);
        reply->set_has_relative_altitude(backend_health.has_relative_altitude);
        reply->set_relative_alt_m(backend_health.relative_alt_m);
        PopulateReadiness(backend_health, kIsReady, reply);
        if (backend_health.link_quality_percent.has_value()) {
            reply->set_link_quality_percent(*backend_health.link_quality_percent);
        }
        return grpc::Status::OK;
    }

    grpc::Status GetRuntimeStats(grpc::ServerContext* ctx,
                                 const swarmkit::v1::RuntimeStatsRequest* /*req*/,
                                 swarmkit::v1::RuntimeStatsReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        counters_.IncrementRuntimeStatsRequests();

        // Sync telemetry counters from the manager into the atomic counters.
        counters_.SetTelemetryCounters(telemetry_.TotalSubscriptionCount(),
                                       telemetry_.ActiveStreamCount(), telemetry_.FramesSentTotal(),
                                       telemetry_.BackendFailureCount());

        const internal::CounterSnapshot kSnap = counters_.Snapshot();
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "stats");

        reply->set_agent_id(config_.agent_id);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_correlation_id(kCorrelationId);
        reply->set_ping_requests_total(kSnap.ping_requests_total);
        reply->set_health_requests_total(kSnap.health_requests_total);
        reply->set_runtime_stats_requests_total(kSnap.runtime_stats_requests_total);
        reply->set_command_requests_total(kSnap.command_requests_total);
        reply->set_command_rejected_total(kSnap.command_rejected_total);
        reply->set_command_failed_total(kSnap.command_failed_total);
        reply->set_lock_requests_total(kSnap.lock_requests_total);
        reply->set_watch_requests_total(kSnap.watch_requests_total);
        reply->set_current_authority_watchers(kSnap.current_authority_watchers);
        reply->set_total_telemetry_subscriptions(kSnap.total_telemetry_subscriptions);
        reply->set_current_telemetry_streams(kSnap.current_telemetry_streams);
        reply->set_telemetry_frames_sent_total(kSnap.telemetry_frames_sent_total);
        reply->set_backend_failures_total(kSnap.backend_failures_total);
        reply->set_data_messages_published_total(kSnap.data_messages_published_total);
        reply->set_data_messages_rejected_total(kSnap.data_messages_rejected_total);
        reply->set_current_message_subscribers(kSnap.current_message_subscribers);
        reply->set_artifact_uploads_total(kSnap.artifact_uploads_total);
        reply->set_artifact_downloads_total(kSnap.artifact_downloads_total);
        reply->set_artifact_bytes_received_total(kSnap.artifact_bytes_received_total);
        reply->set_artifact_bytes_sent_total(kSnap.artifact_bytes_sent_total);
        reply->set_artifact_failures_total(kSnap.artifact_failures_total);
        reply->set_ready(ready_.load(std::memory_order_relaxed));
        return grpc::Status::OK;
    }

    grpc::Status GetCapabilities(grpc::ServerContext* ctx,
                                 const swarmkit::v1::CapabilitiesRequest* /*req*/,
                                 swarmkit::v1::CapabilitiesReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        const BackendCapabilities capabilities = backend_->GetCapabilities();
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "capabilities");
        reply->set_agent_id(config_.agent_id);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_correlation_id(kCorrelationId);
        reply->set_backend_name(capabilities.backend_name);
        reply->set_protocol(capabilities.protocol);
        reply->set_vehicle_class(capabilities.vehicle_class);
        reply->set_supports_mission_upload(capabilities.supports_mission_upload);
        reply->set_supports_payload_control(capabilities.supports_payload_control);
        reply->set_supports_velocity_control(capabilities.supports_velocity_control);
        reply->set_supports_flight_termination(capabilities.supports_flight_termination);
        reply->set_supports_backend_commands(capabilities.supports_backend_commands);
        reply->set_supports_time_sync(capabilities.supports_time_sync);
        reply->set_supports_trajectory_upload(capabilities.supports_trajectory_upload);
        reply->set_autopilot_type(capabilities.autopilot_type);
        for (const std::string& mode : capabilities.supported_modes) {
            reply->add_supported_modes(mode);
        }
        for (const std::string& command : capabilities.supported_commands) {
            reply->add_supported_commands(command);
        }
        for (const std::string& item : capabilities.supported_mission_items) {
            reply->add_supported_mission_items(item);
        }
        for (const std::string& payload : capabilities.supported_payloads) {
            reply->add_supported_payloads(payload);
        }
        for (const std::string& field : capabilities.supported_telemetry_fields) {
            reply->add_supported_telemetry_fields(field);
        }
        for (const std::string& command : capabilities.backend_command_names) {
            reply->add_backend_command_names(command);
        }
        if (capabilities.limits.max_horizontal_speed_mps.has_value()) {
            reply->set_max_horizontal_speed_mps(*capabilities.limits.max_horizontal_speed_mps);
        }
        if (capabilities.limits.max_climb_speed_mps.has_value()) {
            reply->set_max_climb_speed_mps(*capabilities.limits.max_climb_speed_mps);
        }
        if (capabilities.limits.max_descent_speed_mps.has_value()) {
            reply->set_max_descent_speed_mps(*capabilities.limits.max_descent_speed_mps);
        }
        if (capabilities.limits.max_altitude_m.has_value()) {
            reply->set_max_altitude_m(*capabilities.limits.max_altitude_m);
        }
        return grpc::Status::OK;
    }

    // -- Command RPC ----------------------------------------------------------

    grpc::Status SendCommand(grpc::ServerContext* ctx, const swarmkit::v1::CommandRequest* req,
                             swarmkit::v1::CommandReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }

        counters_.IncrementCommandRequests();

        if (!req->has_cmd()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing cmd field");
        }

        CommandEnvelope envelope;
        if (req->has_ctx()) {
            envelope.context = ToCoreContext(req->ctx());
        }
        envelope.context.correlation_id =
            ResolveCorrelationId(ctx, envelope.context.correlation_id, "command");
        if (const core::Result kAuthResult =
                AuthorizePeer(ctx, config_.security, &envelope.context.client_id);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        const std::string kCmdName = ProtoCommandName(req->cmd());

        if (const core::Result kValidation = ValidateCommandContext(envelope.context);
            !kValidation.IsOk()) {
            if (kValidation.code == core::StatusCode::kFailed) {
                reply->set_status(swarmkit::v1::CommandReply::FAILED);
                reply->set_message("command deadline already expired");
                reply->set_correlation_id(envelope.context.correlation_id);
                reply->set_error_code(swarmkit::v1::ERROR_CODE_DEADLINE_EXCEEDED);
                reply->set_debug_message(kValidation.message);
                counters_.IncrementCommandFailed();
                PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_FAILED,
                                     swarmkit::v1::REPORT_ERROR,
                                     "command deadline already expired: " + kValidation.message);
                return grpc::Status::OK;
            }
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, kValidation.message);
        }

        auto convert_result = ConvertProtoCommand(req->cmd());
        if (!convert_result.has_value()) {
            const auto& error = convert_result.error();
            counters_.IncrementCommandFailed();
            reply->set_status(swarmkit::v1::CommandReply::FAILED);
            reply->set_message("invalid command payload");
            reply->set_correlation_id(envelope.context.correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            reply->set_debug_message(error.message);
            PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_FAILED,
                                 swarmkit::v1::REPORT_ERROR,
                                 "invalid command payload: " + error.message);
            return grpc::Status::OK;
        }
        envelope.command = std::move(convert_result.value());

        const CommandPreconditionDecision kPrecondition =
            EvaluateCommandPreconditions(envelope, backend_->GetHealth(),
                                         config_.safety.allow_unsafe_bench_commands);
        if (kPrecondition.action != CommandPreconditionAction::kExecute) {
            reply->set_correlation_id(envelope.context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(kPrecondition.result.code));
            reply->set_debug_message(kPrecondition.result.message);
            if (kPrecondition.action == CommandPreconditionAction::kAlreadySatisfied) {
                reply->set_status(swarmkit::v1::CommandReply::OK);
                reply->set_message(kPrecondition.result.message);
                PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_ACKED,
                                     swarmkit::v1::REPORT_INFO, kPrecondition.result.message);
                core::Logger::InfoFmt(
                    "rpc=SendCommand corr={} agent={} drone={} client={} command={} "
                    "result=already_satisfied detail={}",
                    envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
                    envelope.context.client_id, kCmdName, kPrecondition.result.message);
            } else {
                counters_.IncrementCommandRejected();
                reply->set_status(swarmkit::v1::CommandReply::REJECTED);
                reply->set_message(kPrecondition.result.message);
                PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_REJECTED,
                                     swarmkit::v1::REPORT_WARNING, kPrecondition.result.message);
                core::Logger::WarnFmt(
                    "rpc=SendCommand corr={} agent={} drone={} client={} command={} "
                    "result=precondition_rejected detail={}",
                    envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
                    envelope.context.client_id, kCmdName, kPrecondition.result.message);
            }
            return grpc::Status::OK;
        }

        std::chrono::milliseconds ttl{config_.default_authority_ttl_ms};
        const auto kEpoch = std::chrono::system_clock::time_point{};
        if (envelope.context.deadline != kEpoch) {
            const auto kRemaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                envelope.context.deadline - std::chrono::system_clock::now());
            if (kRemaining.count() > 0) {
                ttl = kRemaining;
            }
        }

        const CommandArbiter::GrantResult kGrant = arbiter_.CheckAndGrantDetailed(envelope.context, ttl);
        if (!kGrant.IsOk()) {
            counters_.IncrementCommandRejected();
            core::Logger::WarnFmt(
                "rpc=SendCommand corr={} agent={} drone={} client={} priority={} peer={} "
                "result=rejected reason={}",
                envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
                envelope.context.client_id, static_cast<int>(envelope.context.priority),
                ctx->peer(), kGrant.result.message);
            reply->set_status(ToProtoStatus(kGrant.result.code));
            reply->set_message(kGrant.result.message);
            reply->set_correlation_id(envelope.context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(kGrant.result.code));
            reply->set_debug_message(kGrant.result.message);
            PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_REJECTED,
                                 swarmkit::v1::REPORT_WARNING, kGrant.result.message);
            return grpc::Status::OK;
        }

        core::Logger::InfoFmt(
            "rpc=SendCommand corr={} agent={} drone={} client={} priority={} peer={} command={}",
            envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
            envelope.context.client_id, static_cast<int>(envelope.context.priority), ctx->peer(),
            kCmdName);
        PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_ACCEPTED,
                             swarmkit::v1::REPORT_INFO, "authority granted; dispatching to backend");

        const core::Result kExecResult = backend_->Execute(envelope);
        reply->set_correlation_id(envelope.context.correlation_id);
        reply->set_error_code(ToProtoErrorCode(kExecResult.code));
        if (kExecResult.IsOk()) {
            reply->set_status(swarmkit::v1::CommandReply::OK);
            reply->set_message(kExecResult.message);
            PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_ACKED,
                                 swarmkit::v1::REPORT_INFO,
                                 kExecResult.message.empty() ? "command executed"
                                                             : kExecResult.message);
        } else {
            counters_.IncrementCommandFailed();
            counters_.IncrementBackendFailures();
            core::Logger::ErrorFmt(
                "rpc=SendCommand corr={} agent={} drone={} client={} result=backend_failure "
                "detail={}",
                envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
                envelope.context.client_id, kExecResult.message);
            reply->set_status(swarmkit::v1::CommandReply::FAILED);
            reply->set_message(kExecResult.message.empty() ? "command execution failed"
                                                           : kExecResult.message);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_BACKEND_FAILURE);
            reply->set_debug_message(kExecResult.message);
            PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_FAILED,
                                 swarmkit::v1::REPORT_ERROR,
                                 kExecResult.message.empty() ? "command execution failed"
                                                             : kExecResult.message);
        }
        if (kGrant.granted_for_call) {
            arbiter_.Release(envelope.context.drone_id, envelope.context.client_id);
            PublishSimpleReport(envelope.context.drone_id, envelope.context.correlation_id,
                                swarmkit::v1::AUTHORITY_RELEASED, swarmkit::v1::REPORT_INFO,
                                "one-shot command authority released");
        }
        return grpc::Status::OK;
    }

    grpc::Status SetActiveGoal(grpc::ServerContext* ctx,
                               const swarmkit::v1::SetActiveGoalRequest* req,
                               swarmkit::v1::SetActiveGoalReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx() || !req->has_goal()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx or goal field");
        }

        CommandContext context = ToCoreContext(req->ctx());
        context.correlation_id = ResolveCorrelationId(ctx, context.correlation_id, "goal");
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, &context.client_id);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        if (context.drone_id.empty()) {
            context.drone_id = req->goal().drone_id();
        }
        if (const core::Result validation = ValidateCommandContext(context); !validation.IsOk()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, validation.message);
        }

        swarmkit::v1::ActiveGoal goal = req->goal();
        if (goal.drone_id().empty()) {
            goal.set_drone_id(context.drone_id);
        }
        if (goal.drone_id() != context.drone_id) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "goal.drone_id must match ctx.drone_id");
        }
        if (const core::Result validation = internal::ActiveGoalSupervisor::ValidateGoal(goal);
            !validation.IsOk()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, validation.message);
        }
        if (GoalUsesLocalTarget(goal)) {
            const std::string message =
                "active goal local-ned target is supported by the API but current backend "
                "execution requires a global GPS target";
            reply->set_ok(false);
            reply->set_message(message);
            reply->set_correlation_id(context.correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_REJECTED);
            reply->set_debug_message(message);
            *reply->mutable_goal() = goal;
            PublishGoalFailure(goal, context.correlation_id, message);
            return grpc::Status::OK;
        }

        const core::Result arbiter_result =
            arbiter_.CheckAndGrant(context, std::chrono::milliseconds{config_.default_authority_ttl_ms});
        if (!arbiter_result.IsOk()) {
            reply->set_ok(false);
            reply->set_message(arbiter_result.message);
            reply->set_correlation_id(context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(arbiter_result.code));
            reply->set_debug_message(arbiter_result.message);
            PublishSimpleReport(context.drone_id, context.correlation_id,
                                swarmkit::v1::AUTHORITY_REJECTED,
                                swarmkit::v1::REPORT_WARNING, arbiter_result.message);
            return grpc::Status::OK;
        }

        CommandEnvelope envelope;
        envelope.context = context;
        envelope.command = NavCmd{CmdGoto{
            .lat_deg = goal.target().lat_deg(),
            .lon_deg = goal.target().lon_deg(),
            .alt_m = goal.target().alt_m(),
            .speed_mps = goal.speed_mps(),
        }};

        const core::Result readiness = ValidateAutonomousReadiness(
            backend_->GetHealth(), "active goal", config_.safety.allow_unsafe_bench_commands);
        if (!readiness.IsOk()) {
            arbiter_.Release(context.drone_id, context.client_id);
            reply->set_ok(false);
            reply->set_message(readiness.message);
            reply->set_correlation_id(context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(readiness.code));
            reply->set_debug_message(readiness.message);
            *reply->mutable_goal() = goal;
            PublishGoalFailure(goal, context.correlation_id, readiness.message);
            return grpc::Status::OK;
        }

        const core::Result exec_result = backend_->Execute(envelope);
        if (!exec_result.IsOk()) {
            arbiter_.Release(context.drone_id, context.client_id);
            counters_.IncrementBackendFailures();
            reply->set_ok(false);
            reply->set_message(exec_result.message.empty() ? "active goal command failed"
                                                           : exec_result.message);
            reply->set_correlation_id(context.correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_BACKEND_FAILURE);
            reply->set_debug_message(exec_result.message);
            *reply->mutable_goal() = goal;
            PublishGoalFailure(goal, context.correlation_id, exec_result.message);
            return grpc::Status::OK;
        }

        const std::int64_t computed_timeout_ms = goals_.StartGoal(goal, context.correlation_id);
        reply->set_ok(true);
        reply->set_message(exec_result.message);
        reply->set_correlation_id(context.correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        *reply->mutable_goal() = goal;
        reply->set_computed_timeout_ms(computed_timeout_ms);
        return grpc::Status::OK;
    }

    grpc::Status CancelGoal(grpc::ServerContext* ctx,
                            const swarmkit::v1::CancelGoalRequest* req,
                            swarmkit::v1::CancelGoalReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx field");
        }

        CommandContext context = ToCoreContext(req->ctx());
        context.correlation_id = ResolveCorrelationId(ctx, context.correlation_id, "cancel-goal");
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, &context.client_id);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        if (const core::Result validation = ValidateCommandContext(context); !validation.IsOk()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, validation.message);
        }

        const core::Result arbiter_result =
            arbiter_.CheckAndGrant(context, std::chrono::milliseconds{config_.default_authority_ttl_ms});
        if (!arbiter_result.IsOk()) {
            reply->set_ok(false);
            reply->set_message(arbiter_result.message);
            reply->set_correlation_id(context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(arbiter_result.code));
            reply->set_debug_message(arbiter_result.message);
            PublishSimpleReport(context.drone_id, context.correlation_id,
                                swarmkit::v1::AUTHORITY_REJECTED,
                                swarmkit::v1::REPORT_WARNING, arbiter_result.message);
            return grpc::Status::OK;
        }

        const core::Result result =
            goals_.CancelGoal(context.drone_id, req->goal_id(), context.correlation_id);
        reply->set_ok(result.IsOk());
        reply->set_message(result.message);
        reply->set_correlation_id(context.correlation_id);
        reply->set_error_code(ToProtoErrorCode(result.code));
        reply->set_debug_message(result.message);
        return grpc::Status::OK;
    }

    grpc::Status GetActiveGoal(grpc::ServerContext* ctx,
                               const swarmkit::v1::GetActiveGoalRequest* req,
                               swarmkit::v1::GetActiveGoalReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, nullptr);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        if (req->drone_id().empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "drone_id must not be empty");
        }

        swarmkit::v1::GoalStatus status = swarmkit::v1::GOAL_STATUS_UNSPECIFIED;
        const auto goal = goals_.GetGoal(req->drone_id(), &status);
        if (!goal.has_value()) {
            reply->set_goal_present(false);
            reply->set_status(swarmkit::v1::GOAL_STATUS_UNSPECIFIED);
            reply->set_message("no active goal");
            return grpc::Status::OK;
        }
        reply->set_goal_present(true);
        *reply->mutable_goal() = goal->first;
        reply->set_status(status);
        reply->set_computed_timeout_ms(goal->second);
        reply->set_message("goal found");
        return grpc::Status::OK;
    }

    grpc::Status UploadTrajectory(grpc::ServerContext* ctx,
                                  const swarmkit::v1::UploadTrajectoryRequest* req,
                                  swarmkit::v1::ExecutionReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx() || !req->has_plan()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx or plan field");
        }
        auto context_result = PrepareExecutionContext(ctx, req->ctx(), req->plan().drone_id(),
                                                      "trajectory-upload");
        if (!context_result.has_value()) {
            return context_result.error();
        }
        swarmkit::v1::TrajectoryPlan plan = req->plan();
        if (plan.drone_id().empty()) {
            plan.set_drone_id(context_result->drone_id);
        }
        if (plan.drone_id() != context_result->drone_id) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "plan.drone_id must match ctx.drone_id");
        }
        if (const core::Result arbiter_result = GrantExecutionAuthority(*context_result);
            !arbiter_result.IsOk()) {
            FillExecutionReply(reply, false, arbiter_result, context_result->correlation_id, {}, {});
            return grpc::Status::OK;
        }
        swarmkit::v1::ExecutionHandle handle;
        swarmkit::v1::ValidateTrajectoryResult validation;
        const core::Result result =
            executions_.Upload(std::move(plan), context_result->correlation_id, &handle, &validation);
        FillExecutionReply(reply, result.IsOk(), result, context_result->correlation_id, handle,
                           validation);
        return grpc::Status::OK;
    }

    grpc::Status ValidateTrajectory(grpc::ServerContext* ctx,
                                    const swarmkit::v1::ValidateTrajectoryRequest* req,
                                    swarmkit::v1::ValidateTrajectoryReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx() || !req->has_plan()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx or plan field");
        }
        auto context_result = PrepareExecutionContext(ctx, req->ctx(), req->plan().drone_id(),
                                                      "trajectory-validate");
        if (!context_result.has_value()) {
            return context_result.error();
        }
        const auto validation = executions_.ValidatePlan(req->plan());
        reply->set_ok(validation.ok());
        reply->set_message(validation.ok() ? "trajectory valid" : "trajectory validation failed");
        reply->set_correlation_id(context_result->correlation_id);
        reply->set_error_code(validation.ok() ? swarmkit::v1::ERROR_CODE_NONE
                                              : swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
        reply->set_debug_message(reply->message());
        *reply->mutable_validation() = validation;
        return grpc::Status::OK;
    }

    grpc::Status ClearTrajectory(grpc::ServerContext* ctx,
                                 const swarmkit::v1::ExecutionRequest* req,
                                 swarmkit::v1::ExecutionReply* reply) override {
        return HandleExecutionRequest(ctx, req, reply, "trajectory-clear",
                                      [this](const CommandContext& context,
                                             const std::string& execution_id,
                                             swarmkit::v1::ExecutionHandle* handle,
                                             swarmkit::v1::ValidateTrajectoryResult*) {
                                          return executions_.Clear(context.drone_id, execution_id,
                                                                   context.correlation_id, handle);
                                      });
    }

    grpc::Status PrepareTrajectory(grpc::ServerContext* ctx,
                                   const swarmkit::v1::ExecutionRequest* req,
                                   swarmkit::v1::ExecutionReply* reply) override {
        return HandleExecutionRequest(ctx, req, reply, "trajectory-prepare",
                                      [this](const CommandContext& context,
                                             const std::string& execution_id,
                                             swarmkit::v1::ExecutionHandle* handle,
                                             swarmkit::v1::ValidateTrajectoryResult* validation) {
                                          return executions_.Prepare(context.drone_id, execution_id,
                                                                     context.correlation_id, handle,
                                                                     validation);
                                      });
    }

    grpc::Status StartExecutionAt(grpc::ServerContext* ctx,
                                  const swarmkit::v1::StartExecutionAtRequest* req,
                                  swarmkit::v1::ExecutionReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx() || req->execution_id().empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "missing ctx or execution_id field");
        }
        auto context_result =
            PrepareExecutionContext(ctx, req->ctx(), req->ctx().drone_id(), "execution-start");
        if (!context_result.has_value()) {
            return context_result.error();
        }
        if (const core::Result arbiter_result = GrantExecutionAuthority(*context_result);
            !arbiter_result.IsOk()) {
            FillExecutionReply(reply, false, arbiter_result, context_result->correlation_id, {}, {});
            return grpc::Status::OK;
        }
        swarmkit::v1::ExecutionHandle handle;
        const core::Result result =
            executions_.StartAt(context_result->drone_id, req->execution_id(), req->unix_time_ms(),
                                context_result->correlation_id, &handle);
        FillExecutionReply(reply, result.IsOk(), result, context_result->correlation_id, handle, {});
        return grpc::Status::OK;
    }

    grpc::Status PauseExecution(grpc::ServerContext* ctx,
                                const swarmkit::v1::ExecutionRequest* req,
                                swarmkit::v1::ExecutionReply* reply) override {
        return HandleExecutionRequest(ctx, req, reply, "execution-pause",
                                      [this](const CommandContext& context,
                                             const std::string& execution_id,
                                             swarmkit::v1::ExecutionHandle* handle,
                                             swarmkit::v1::ValidateTrajectoryResult*) {
                                          return executions_.Pause(context.drone_id, execution_id,
                                                                   context.correlation_id, handle);
                                      });
    }

    grpc::Status ResumeExecution(grpc::ServerContext* ctx,
                                 const swarmkit::v1::ExecutionRequest* req,
                                 swarmkit::v1::ExecutionReply* reply) override {
        return HandleExecutionRequest(ctx, req, reply, "execution-resume",
                                      [this](const CommandContext& context,
                                             const std::string& execution_id,
                                             swarmkit::v1::ExecutionHandle* handle,
                                             swarmkit::v1::ValidateTrajectoryResult*) {
                                          return executions_.Resume(context.drone_id, execution_id,
                                                                    context.correlation_id, handle);
                                      });
    }

    grpc::Status AbortExecution(grpc::ServerContext* ctx,
                                const swarmkit::v1::ExecutionRequest* req,
                                swarmkit::v1::ExecutionReply* reply) override {
        return HandleExecutionRequest(ctx, req, reply, "execution-abort",
                                      [this](const CommandContext& context,
                                             const std::string& execution_id,
                                             swarmkit::v1::ExecutionHandle* handle,
                                             swarmkit::v1::ValidateTrajectoryResult*) {
                                          return executions_.Abort(context.drone_id, execution_id,
                                                                   context.correlation_id, handle);
                                      });
    }

    grpc::Status GetExecution(grpc::ServerContext* ctx,
                              const swarmkit::v1::GetExecutionRequest* req,
                              swarmkit::v1::GetExecutionReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, nullptr);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        if (req->drone_id().empty() || req->execution_id().empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "drone_id and execution_id must not be empty");
        }
        const auto execution = executions_.Get(req->drone_id(), req->execution_id());
        if (!execution.has_value()) {
            reply->set_found(false);
            reply->set_message("execution not found");
            return grpc::Status::OK;
        }
        reply->set_found(true);
        *reply->mutable_handle() = execution->first;
        *reply->mutable_plan() = execution->second;
        reply->set_message("execution found");
        return grpc::Status::OK;
    }

    grpc::Status ListExecutions(grpc::ServerContext* ctx,
                                const swarmkit::v1::ListExecutionsRequest* req,
                                swarmkit::v1::ListExecutionsReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, nullptr);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        const std::string drone_id = req == nullptr ? "all" : req->drone_id();
        for (const auto& handle : executions_.List(drone_id)) {
            *reply->add_executions() = handle;
        }
        return grpc::Status::OK;
    }

    grpc::Status GetTimeSyncState(grpc::ServerContext* ctx,
                                  const swarmkit::v1::TimeSyncRequest* req,
                                  swarmkit::v1::TimeSyncState* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, nullptr);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        *reply = executions_.GetTimeSyncState(req == nullptr ? "default" : req->drone_id());
        return grpc::Status::OK;
    }

    // -- Authority RPCs -------------------------------------------------------

    grpc::Status LockAuthority(grpc::ServerContext* ctx,
                               const swarmkit::v1::LockAuthorityRequest* req,
                               swarmkit::v1::LockAuthorityReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx field");
        }

        counters_.IncrementLockRequests();

        CommandContext lock_context = ToCoreContext(req->ctx());
        lock_context.correlation_id =
            ResolveCorrelationId(ctx, lock_context.correlation_id, "lock");
        if (const core::Result kAuthResult =
                AuthorizePeer(ctx, config_.security, &lock_context.client_id);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }
        const auto kTtlDuration = std::chrono::milliseconds{req->ttl_ms()};

        if (const core::Result kValidation = ValidateLockRequest(lock_context, req->ttl_ms());
            !kValidation.IsOk()) {
            if (kValidation.code == core::StatusCode::kFailed) {
                reply->set_ok(false);
                reply->set_message("command deadline already expired");
                reply->set_correlation_id(lock_context.correlation_id);
                reply->set_error_code(swarmkit::v1::ERROR_CODE_DEADLINE_EXCEEDED);
                reply->set_debug_message(kValidation.message);
                return grpc::Status::OK;
            }
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, kValidation.message);
        }

        const core::Result kResult = arbiter_.CheckAndGrant(lock_context, kTtlDuration);

        reply->set_ok(kResult.IsOk());
        reply->set_message(kResult.message);
        reply->set_correlation_id(lock_context.correlation_id);
        reply->set_error_code(ToProtoErrorCode(kResult.code));
        reply->set_debug_message(kResult.message);

        if (kResult.IsOk()) {
            core::Logger::InfoFmt(
                "rpc=LockAuthority corr={} agent={} drone={} client={} priority={} ttl_ms={} "
                "result=granted",
                lock_context.correlation_id, config_.agent_id, lock_context.drone_id,
                lock_context.client_id, static_cast<int>(lock_context.priority), req->ttl_ms());
        } else {
            core::Logger::WarnFmt(
                "rpc=LockAuthority corr={} agent={} drone={} client={} priority={} ttl_ms={} "
                "result=rejected reason={}",
                lock_context.correlation_id, config_.agent_id, lock_context.drone_id,
                lock_context.client_id, static_cast<int>(lock_context.priority), req->ttl_ms(),
                kResult.message);
        }
        return grpc::Status::OK;
    }

    grpc::Status ReleaseAuthority(grpc::ServerContext* ctx,
                                  const swarmkit::v1::ReleaseAuthorityRequest* req,
                                  swarmkit::v1::ReleaseAuthorityReply* /*reply*/) override {
        if (req == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request");
        }
        std::string client_id = req->client_id();
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, &client_id);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }
        if (req->drone_id().empty() || client_id.empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "drone_id and client_id must not be empty");
        }
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "unlock");
        arbiter_.Release(req->drone_id(), client_id);
        core::Logger::InfoFmt(
            "rpc=ReleaseAuthority corr={} agent={} drone={} client={} result=released",
            kCorrelationId, config_.agent_id, req->drone_id(), client_id);
        return grpc::Status::OK;
    }

    // -- Streaming RPCs -------------------------------------------------------

    grpc::Status StreamTelemetry(
        grpc::ServerContext* ctx, const swarmkit::v1::TelemetryRequest* req,
        grpc::ServerWriter<swarmkit::v1::TelemetryFrame>* writer) override {
        if (ctx == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/writer");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }
        if (const core::Result kValidation = ValidateTelemetryRequest(req); !kValidation.IsOk()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, kValidation.message);
        }

        const std::string kPeer = ctx->peer();
        const std::string kStreamId = ResolveCorrelationId(ctx, "", "telemetry");
        const std::string kDroneId =
            (req != nullptr && !req->drone_id().empty()) ? req->drone_id() : "default";
        const int kRequestedRateHz = (req != nullptr) ? req->rate_hz() : 0;
        const int kEffectiveRateHz =
            std::max(config_.min_telemetry_rate_hz,
                     kRequestedRateHz <= 0 ? config_.default_telemetry_rate_hz : kRequestedRateHz);
        const auto kEmitPeriod =
            std::chrono::milliseconds(kMillisecondsPerSecond / kEffectiveRateHz);

        internal::TelemetryLease lease;
        const core::Result kAcquireResult =
            telemetry_.AcquireLease(kDroneId, kEffectiveRateHz, &lease);
        if (!kAcquireResult.IsOk()) {
            core::Logger::ErrorFmt(
                "rpc=StreamTelemetry corr={} agent={} drone={} peer={} rate_hz={} "
                "result=acquire_failed detail={}",
                kStreamId, config_.agent_id, kDroneId, kPeer, kEffectiveRateHz,
                kAcquireResult.message);
            const auto status_code = kAcquireResult.code == core::StatusCode::kRejected
                                         ? grpc::StatusCode::FAILED_PRECONDITION
                                         : grpc::StatusCode::UNAVAILABLE;
            return grpc::Status(status_code, kAcquireResult.message);
        }

        core::Logger::InfoFmt(
            "rpc=StreamTelemetry corr={} agent={} drone={} peer={} rate_hz={} result=connected",
            kStreamId, config_.agent_id, kDroneId, kPeer, kEffectiveRateHz);

        std::uint64_t last_sequence = 0;
        auto next_emit_at = std::chrono::steady_clock::now();

        while (!ctx->IsCancelled()) {
            core::TelemetryFrame frame;
            if (!internal::TelemetryManager::WaitForFrame(lease, &last_sequence, &frame,
                                                          kTelemetryWaitTimeout)) {
                continue;
            }

            const auto kNow = std::chrono::steady_clock::now();
            if (kNow < next_emit_at) {
                continue;
            }

            if (frame.active_goal_id.empty()) {
                const auto active_goal = goals_.GetGoal(frame.drone_id, nullptr);
                if (active_goal.has_value()) {
                    frame.active_goal_id = active_goal->first.goal_id();
                }
            }

            swarmkit::v1::TelemetryFrame out;
            PopulateTelemetryProto(frame, &out);
            out.set_stream_id(kStreamId);

            if (!writer->Write(out)) {
                break;
            }
            telemetry_.IncrementFramesSent();
            next_emit_at = kNow + kEmitPeriod;
        }

        telemetry_.ReleaseLease(lease);
        core::Logger::InfoFmt(
            "rpc=StreamTelemetry corr={} agent={} drone={} peer={} result=disconnected", kStreamId,
            config_.agent_id, kDroneId, kPeer);
        return grpc::Status::OK;
    }

    grpc::Status WatchAuthority(grpc::ServerContext* ctx,
                                const swarmkit::v1::WatchAuthorityRequest* req,
                                grpc::ServerWriter<swarmkit::v1::AuthorityEvent>* writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/writer");
        }

        const std::string& drone_id = req->drone_id();
        std::string client_id = req->client_id();
        const auto kPriority = static_cast<CommandPriority>(req->priority());
        const std::string kStreamId = ResolveCorrelationId(ctx, "", "watch");
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, &client_id);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }
        if (drone_id.empty() || client_id.empty() || !IsValidPriority(kPriority)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "invalid watch authority request");
        }

        counters_.IncrementWatchRequests();
        counters_.IncrementAuthorityWatchers();
        auto queue = std::make_shared<EventQueue>();
        const WatchToken kToken = arbiter_.Watch(drone_id, client_id, kPriority, queue);

        core::Logger::InfoFmt(
            "rpc=WatchAuthority corr={} agent={} drone={} client={} priority={} result=connected",
            kStreamId, config_.agent_id, drone_id, client_id, static_cast<int>(kPriority));

        while (!ctx->IsCancelled()) {
            AuthorityEvent event;
            if (!queue->Pop(event, kWatchPollInterval)) {
                continue;
            }

            swarmkit::v1::AuthorityEvent proto_event;
            proto_event.set_kind(ToProtoEventKind(event.kind));
            proto_event.set_drone_id(event.drone_id);
            proto_event.set_holder_client_id(event.holder_client_id);
            proto_event.set_holder_priority(static_cast<int>(event.holder_priority));
            proto_event.set_correlation_id(kStreamId);

            if (!writer->Write(proto_event)) {
                break;
            }
        }

        queue->Shutdown();
        arbiter_.Unwatch(kToken);
        counters_.DecrementAuthorityWatchers();

        core::Logger::InfoFmt(
            "rpc=WatchAuthority corr={} agent={} drone={} client={} result=disconnected", kStreamId,
            config_.agent_id, drone_id, client_id);
        return grpc::Status::OK;
    }

    grpc::Status SubscribeReports(
        grpc::ServerContext* ctx, const swarmkit::v1::ReportSubscription* req,
        grpc::ServerWriter<swarmkit::v1::AgentReport>* writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/writer");
        }
        std::string client_id = req->client_id();
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, &client_id);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }

        const std::string stream_id = ResolveCorrelationId(ctx, "", "reports");
        auto queue = std::make_shared<internal::ReportQueue>();
        const internal::ReportWatchToken token =
            reports_.Watch(req->drone_id(), req->after_sequence(), queue);
        core::Logger::InfoFmt(
            "rpc=SubscribeReports corr={} agent={} drone={} client={} result=connected", stream_id,
            config_.agent_id, req->drone_id(), client_id);

        while (!ctx->IsCancelled()) {
            swarmkit::v1::AgentReport report;
            if (!queue->Pop(&report, kWatchPollInterval)) {
                continue;
            }
            if (!writer->Write(report)) {
                break;
            }
        }

        queue->Shutdown();
        reports_.Unwatch(token);
        core::Logger::InfoFmt(
            "rpc=SubscribeReports corr={} agent={} drone={} client={} result=disconnected",
            stream_id, config_.agent_id, req->drone_id(), client_id);
        return grpc::Status::OK;
    }

    grpc::Status PublishMessage(grpc::ServerContext* ctx,
                                const swarmkit::v1::PublishMessageRequest* req,
                                swarmkit::v1::PublishMessageReply* reply) override {
        if (req == nullptr || reply == nullptr || !req->has_message()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing message");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-message");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        swarmkit::v1::DataMessage message = req->message();
        if (message.topic().empty()) {
            counters_.IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("message.topic is required");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            return grpc::Status::OK;
        }
        if (message.payload().size() >
            static_cast<std::size_t>(config_.data.max_message_payload_bytes)) {
            counters_.IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("message payload exceeds max_message_payload_bytes");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_REJECTED);
            return grpc::Status::OK;
        }
        if (message.source_id().empty()) {
            message.set_source_id(config_.agent_id);
        }
        if (message.unix_time_ms() == 0) {
            message.set_unix_time_ms(NowUnixMs());
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            message.set_sequence(++data_sequence_);
            if (message.message_id().empty()) {
                message.set_message_id("msg-" + std::to_string(message.sequence()));
            }
            message_backlog_.push_back(message);
            while (static_cast<int>(message_backlog_.size()) > config_.data.message_backlog_size) {
                message_backlog_.pop_front();
            }
        }
        data_cv_.notify_all();
        counters_.IncrementDataMessagesPublished();

        reply->set_ok(true);
        reply->set_message("message published");
        reply->set_correlation_id(correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        reply->set_sequence(message.sequence());
        return grpc::Status::OK;
    }

    grpc::Status SendMessageToDrone(grpc::ServerContext* ctx,
                                    const swarmkit::v1::PublishMessageRequest* req,
                                    swarmkit::v1::PublishMessageReply* reply) override {
        if (req == nullptr || reply == nullptr || !req->has_message()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing message");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-route-message");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        swarmkit::v1::DataMessage message = req->message();
        if (message.target_id().empty()) {
            counters_.IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("message.target_id is required for routed sends");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            return grpc::Status::OK;
        }
        if (message.target_id() == config_.agent_id) {
            swarmkit::v1::PublishMessageRequest local_req;
            *local_req.mutable_message() = std::move(message);
            return PublishMessage(ctx, &local_req, reply);
        }
        const std::optional<DataPeerConfig> peer = config_.data.FindPeer(message.target_id());
        if (!peer.has_value()) {
            counters_.IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("no data peer configured for target_id=" + message.target_id());
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_UNAVAILABLE);
            return grpc::Status::OK;
        }
        if (message.source_id().empty()) {
            message.set_source_id(config_.agent_id);
        }
        if (message.unix_time_ms() == 0) {
            message.set_unix_time_ms(NowUnixMs());
        }

        auto channel = MakePeerChannel(*peer, config_.security);
        auto stub = swarmkit::v1::DataService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{30});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), correlation_id);
        swarmkit::v1::PublishMessageRequest peer_req;
        *peer_req.mutable_message() = std::move(message);
        swarmkit::v1::PublishMessageReply peer_reply;
        const grpc::Status peer_status = stub->PublishMessage(&peer_ctx, peer_req, &peer_reply);
        if (!peer_status.ok()) {
            counters_.IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("peer message delivery failed: " + peer_status.error_message());
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_UNAVAILABLE);
            return grpc::Status::OK;
        }
        *reply = peer_reply;
        if (reply->correlation_id().empty()) {
            reply->set_correlation_id(correlation_id);
        }
        if (reply->ok()) {
            reply->set_message("message delivered to " + peer->drone_id + ": " + reply->message());
        }
        return grpc::Status::OK;
    }

    grpc::Status SubscribeMessages(
        grpc::ServerContext* ctx, const swarmkit::v1::MessageSubscription* req,
        grpc::ServerWriter<swarmkit::v1::DataMessage>* writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/writer");
        }
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        counters_.IncrementMessageSubscribers();
        const auto decrement_subscribers = [this] {
            counters_.DecrementMessageSubscribers();
        };
        std::uint64_t cursor = req->after_sequence();
        while (!ctx->IsCancelled()) {
            std::vector<swarmkit::v1::DataMessage> pending;
            {
                std::unique_lock<std::mutex> lock(data_mutex_);
                data_cv_.wait_for(lock, std::chrono::milliseconds{250}, [&] {
                    return ctx->IsCancelled() || data_sequence_ > cursor;
                });
                const std::int64_t now_ms = NowUnixMs();
                for (const auto& message : message_backlog_) {
                    if (message.sequence() <= cursor || MessageExpired(message, now_ms) ||
                        !MessageMatchesSubscription(message, *req)) {
                        continue;
                    }
                    pending.push_back(message);
                }
            }
            for (const auto& message : pending) {
                cursor = std::max(cursor, message.sequence());
                if (!writer->Write(message)) {
                    decrement_subscribers();
                    return grpc::Status::OK;
                }
            }
        }
        decrement_subscribers();
        return grpc::Status::OK;
    }

    grpc::Status UploadArtifact(grpc::ServerContext* ctx,
                                grpc::ServerReader<swarmkit::v1::ArtifactChunk>* reader,
                                swarmkit::v1::ArtifactDescriptor* reply) override {
        if (ctx == nullptr || reader == nullptr || reply == nullptr) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/reader/reply");
        }
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        std::filesystem::path tmp_dir = std::filesystem::path(config_.data.artifact_dir) / "tmp";
        std::error_code fs_error;
        std::filesystem::create_directories(tmp_dir, fs_error);
        if (fs_error) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                "failed to create artifact temp dir");
        }

        swarmkit::v1::ArtifactDescriptor descriptor;
        std::string transfer_id;
        std::filesystem::path tmp_path;
        std::ofstream output;
        std::int64_t received_bytes = 0;
        bool saw_descriptor = false;
        bool saw_final = false;
        const auto fail_upload = [&](grpc::StatusCode code, std::string_view message) {
            if (output.is_open()) {
                output.close();
            }
            if (!tmp_path.empty()) {
                std::filesystem::remove(tmp_path, fs_error);
            }
            counters_.IncrementArtifactFailures();
            return grpc::Status(code, std::string(message));
        };

        swarmkit::v1::ArtifactChunk chunk;
        while (reader->Read(&chunk)) {
            if (!saw_descriptor) {
                if (!chunk.has_artifact()) {
                    return fail_upload(grpc::StatusCode::INVALID_ARGUMENT,
                                       "first artifact chunk must include descriptor");
                }
                descriptor = chunk.artifact();
                if (descriptor.source_id().empty()) {
                    descriptor.set_source_id(config_.agent_id);
                }
                if (descriptor.created_unix_ms() == 0) {
                    descriptor.set_created_unix_ms(NowUnixMs());
                }
                transfer_id = chunk.transfer_id().empty()
                                  ? "transfer-" + std::to_string(descriptor.created_unix_ms())
                                  : chunk.transfer_id();
                tmp_path = tmp_dir / (SanitizedPathComponent(transfer_id) + ".part");
                output.open(tmp_path, std::ios::binary | std::ios::trunc);
                if (!output.is_open()) {
                    return fail_upload(grpc::StatusCode::FAILED_PRECONDITION,
                                       "failed to open artifact temp file");
                }
                saw_descriptor = true;
            }
            if (chunk.offset() != received_bytes) {
                return fail_upload(grpc::StatusCode::INVALID_ARGUMENT,
                                   "artifact chunks must arrive in order with matching offsets");
            }
            if (received_bytes + static_cast<std::int64_t>(chunk.data().size()) >
                config_.data.max_artifact_bytes) {
                return fail_upload(grpc::StatusCode::RESOURCE_EXHAUSTED,
                                   "artifact exceeds max_artifact_bytes");
            }
            output.write(chunk.data().data(), static_cast<std::streamsize>(chunk.data().size()));
            received_bytes += static_cast<std::int64_t>(chunk.data().size());
            saw_final = saw_final || chunk.final_chunk();
        }
        if (!saw_descriptor) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "empty artifact upload");
        }
        output.flush();
        output.close();
        if (!saw_final) {
            std::filesystem::remove(tmp_path, fs_error);
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "artifact upload ended before final chunk");
        }

        const std::string sha256_hex = core::internal::Sha256FileHex(tmp_path);
        if (sha256_hex.empty()) {
            std::filesystem::remove(tmp_path, fs_error);
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INTERNAL, "failed to hash artifact");
        }
        if (!descriptor.sha256_hex().empty() && descriptor.sha256_hex() != sha256_hex) {
            std::filesystem::remove(tmp_path, fs_error);
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::DATA_LOSS, "artifact sha256 mismatch");
        }
        descriptor.set_sha256_hex(sha256_hex);
        if (descriptor.artifact_id().empty()) {
            descriptor.set_artifact_id(sha256_hex);
        }
        descriptor.set_size_bytes(received_bytes);
        if (descriptor.filename().empty()) {
            descriptor.set_filename(descriptor.artifact_id() + ".bin");
        }

        const std::filesystem::path final_dir =
            std::filesystem::path(config_.data.artifact_dir) /
            SanitizedPathComponent(descriptor.source_id()) /
            SanitizedPathComponent(descriptor.artifact_id());
        std::filesystem::create_directories(final_dir, fs_error);
        if (fs_error) {
            std::filesystem::remove(tmp_path, fs_error);
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                "failed to create artifact final dir");
        }
        const std::filesystem::path final_path =
            final_dir / SanitizedPathComponent(descriptor.filename());
        std::filesystem::rename(tmp_path, final_path, fs_error);
        if (fs_error) {
            std::filesystem::remove(final_path, fs_error);
            std::filesystem::rename(tmp_path, final_path, fs_error);
        }
        if (fs_error) {
            std::filesystem::remove(tmp_path, fs_error);
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                "failed to finalize artifact");
        }
        descriptor.set_storage_path(final_path.string());
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            artifacts_[descriptor.artifact_id()] = descriptor;
        }
        *reply = descriptor;
        counters_.IncrementArtifactUploads(static_cast<std::uint64_t>(received_bytes));
        return grpc::Status::OK;
    }

    grpc::Status SendArtifactToDrone(grpc::ServerContext* ctx,
                                     grpc::ServerReader<swarmkit::v1::ArtifactChunk>* reader,
                                     swarmkit::v1::ArtifactDescriptor* reply) override {
        if (ctx == nullptr || reader == nullptr || reply == nullptr) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/reader/reply");
        }
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-route");

        swarmkit::v1::ArtifactChunk chunk;
        if (!reader->Read(&chunk)) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "empty artifact send");
        }
        if (!chunk.has_artifact()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "first artifact chunk must include descriptor");
        }
        swarmkit::v1::ArtifactDescriptor descriptor = chunk.artifact();
        if (descriptor.target_id().empty()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "artifact.target_id is required for routed sends");
        }
        if (descriptor.target_id() == config_.agent_id) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "routed artifact target_id points to local agent");
        }
        const std::optional<DataPeerConfig> peer = config_.data.FindPeer(descriptor.target_id());
        if (!peer.has_value()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::NOT_FOUND,
                                "no data peer configured for target_id=" + descriptor.target_id());
        }
        if (descriptor.source_id().empty()) {
            descriptor.set_source_id(config_.agent_id);
        }
        if (descriptor.created_unix_ms() == 0) {
            descriptor.set_created_unix_ms(NowUnixMs());
        }
        *chunk.mutable_artifact() = descriptor;

        auto channel = MakePeerChannel(*peer, config_.security);
        auto stub = swarmkit::v1::DataService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{60});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), correlation_id);
        auto writer = stub->UploadArtifact(&peer_ctx, reply);
        std::uint64_t bytes_forwarded = static_cast<std::uint64_t>(chunk.data().size());
        bool wrote_all = writer->Write(chunk);
        while (wrote_all && reader->Read(&chunk)) {
            bytes_forwarded += static_cast<std::uint64_t>(chunk.data().size());
            wrote_all = writer->Write(chunk);
        }
        writer->WritesDone();
        const grpc::Status peer_status = writer->Finish();
        if (!wrote_all && peer_status.ok()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                "peer artifact stream closed before all chunks were accepted");
        }
        if (!peer_status.ok()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(peer_status.error_code(),
                                "peer artifact delivery failed: " + peer_status.error_message());
        }
        counters_.IncrementArtifactDownloads(bytes_forwarded);
        return grpc::Status::OK;
    }

    grpc::Status DownloadArtifact(
        grpc::ServerContext* ctx, const swarmkit::v1::ArtifactRequest* req,
        grpc::ServerWriter<swarmkit::v1::ArtifactChunk>* writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/writer");
        }
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        swarmkit::v1::ArtifactDescriptor descriptor;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const auto iter = artifacts_.find(req->artifact_id());
            if (iter == artifacts_.end()) {
                counters_.IncrementArtifactFailures();
                return grpc::Status(grpc::StatusCode::NOT_FOUND, "artifact not found");
            }
            descriptor = iter->second;
            if (ArtifactExpired(descriptor, NowUnixMs())) {
                artifacts_.erase(iter);
                counters_.IncrementArtifactFailures();
                return grpc::Status(grpc::StatusCode::NOT_FOUND, "artifact expired");
            }
        }
        std::ifstream input(descriptor.storage_path(), std::ios::binary);
        if (!input.is_open()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::NOT_FOUND, "artifact file not found");
        }
        if (descriptor.size_bytes() == 0) {
            swarmkit::v1::ArtifactChunk chunk;
            chunk.set_transfer_id("download-" + descriptor.artifact_id());
            *chunk.mutable_artifact() = descriptor;
            chunk.set_offset(0);
            chunk.set_chunk_index(0);
            chunk.set_final_chunk(true);
            static_cast<void>(writer->Write(chunk));
            counters_.IncrementArtifactDownloads(0);
            return grpc::Status::OK;
        }
        const std::size_t chunk_size =
            std::max<std::size_t>(1, static_cast<std::size_t>(config_.data.artifact_chunk_bytes));
        std::string buffer(chunk_size, '\0');
        std::int64_t offset = 0;
        int chunk_index = 0;
        while (!ctx->IsCancelled() && input.good()) {
            input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize read = input.gcount();
            if (read <= 0) {
                break;
            }
            swarmkit::v1::ArtifactChunk chunk;
            chunk.set_transfer_id("download-" + descriptor.artifact_id());
            if (chunk_index == 0) {
                *chunk.mutable_artifact() = descriptor;
            }
            chunk.set_offset(offset);
            chunk.set_chunk_index(chunk_index);
            chunk.set_data(buffer.data(), static_cast<std::size_t>(read));
            offset += read;
            chunk.set_final_chunk(offset >= descriptor.size_bytes());
            if (!writer->Write(chunk)) {
                counters_.IncrementArtifactDownloads(static_cast<std::uint64_t>(offset));
                return grpc::Status::OK;
            }
            ++chunk_index;
        }
        counters_.IncrementArtifactDownloads(static_cast<std::uint64_t>(offset));
        return grpc::Status::OK;
    }

    grpc::Status AnnounceArtifact(grpc::ServerContext* ctx,
                                  const swarmkit::v1::ArtifactDescriptor* req,
                                  swarmkit::v1::ArtifactReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-announce");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->artifact_id().empty()) {
            counters_.IncrementArtifactFailures();
            reply->set_ok(false);
            reply->set_message("artifact_id is required");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            return grpc::Status::OK;
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            swarmkit::v1::ArtifactDescriptor descriptor = *req;
            if (descriptor.source_id().empty()) {
                descriptor.set_source_id(config_.agent_id);
            }
            if (descriptor.created_unix_ms() == 0) {
                descriptor.set_created_unix_ms(NowUnixMs());
            }
            artifacts_[descriptor.artifact_id()] = descriptor;
            *reply->mutable_artifact() = descriptor;
        }
        reply->set_ok(true);
        reply->set_message("artifact announced");
        reply->set_correlation_id(correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        return grpc::Status::OK;
    }

   private:
    [[nodiscard]] std::expected<CommandContext, grpc::Status> PrepareExecutionContext(
        grpc::ServerContext* ctx, const swarmkit::v1::CommandContext& proto_context,
        std::string_view fallback_drone_id, std::string_view correlation_prefix) const {
        CommandContext context = ToCoreContext(proto_context);
        if (context.drone_id.empty()) {
            context.drone_id = std::string(fallback_drone_id);
        }
        context.correlation_id = ResolveCorrelationId(ctx, context.correlation_id,
                                                      correlation_prefix);
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, &context.client_id);
            !auth_result.IsOk()) {
            return std::unexpected(
                grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message));
        }
        if (const core::Result validation = ValidateCommandContext(context); !validation.IsOk()) {
            return std::unexpected(
                grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, validation.message));
        }
        return context;
    }

    [[nodiscard]] core::Result GrantExecutionAuthority(const CommandContext& context) {
        return arbiter_.CheckAndGrant(
            context, std::chrono::milliseconds{config_.default_authority_ttl_ms});
    }

    static void FillExecutionReply(swarmkit::v1::ExecutionReply* reply, bool succeeded,
                                   const core::Result& result,
                                   std::string_view correlation_id,
                                   const swarmkit::v1::ExecutionHandle& handle,
                                   const swarmkit::v1::ValidateTrajectoryResult& validation) {
        if (reply == nullptr) {
            return;
        }
        reply->set_ok(succeeded);
        reply->set_message(result.message);
        reply->set_correlation_id(std::string(correlation_id));
        reply->set_error_code(ToProtoErrorCode(result.code));
        reply->set_debug_message(result.message);
        if (!handle.execution_id().empty()) {
            *reply->mutable_handle() = handle;
        }
        if (validation.issues_size() > 0 || validation.ok()) {
            *reply->mutable_validation() = validation;
        }
    }

    using ExecutionOperation = std::function<core::Result(
        const CommandContext&, const std::string&, swarmkit::v1::ExecutionHandle*,
        swarmkit::v1::ValidateTrajectoryResult*)>;

    grpc::Status HandleExecutionRequest(grpc::ServerContext* ctx,
                                        const swarmkit::v1::ExecutionRequest* req,
                                        swarmkit::v1::ExecutionReply* reply,
                                        std::string_view correlation_prefix,
                                        const ExecutionOperation& operation) {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx() || req->execution_id().empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "missing ctx or execution_id field");
        }
        auto context_result =
            PrepareExecutionContext(ctx, req->ctx(), req->ctx().drone_id(), correlation_prefix);
        if (!context_result.has_value()) {
            return context_result.error();
        }
        if (const core::Result arbiter_result = GrantExecutionAuthority(*context_result);
            !arbiter_result.IsOk()) {
            FillExecutionReply(reply, false, arbiter_result, context_result->correlation_id, {}, {});
            return grpc::Status::OK;
        }

        swarmkit::v1::ExecutionHandle handle;
        swarmkit::v1::ValidateTrajectoryResult validation;
        const core::Result result =
            operation(*context_result, req->execution_id(), &handle, &validation);
        FillExecutionReply(reply, result.IsOk(), result, context_result->correlation_id, handle,
                           validation);
        return grpc::Status::OK;
    }

    void PublishSimpleReport(std::string_view drone_id, std::string_view correlation_id,
                             swarmkit::v1::AgentReportType type,
                             swarmkit::v1::ReportSeverity severity, std::string_view message) {
        swarmkit::v1::AgentReport report;
        report.set_drone_id(std::string(drone_id));
        report.set_correlation_id(std::string(correlation_id));
        report.set_type(type);
        report.set_severity(severity);
        report.set_message(std::string(message));
        reports_.Publish(std::move(report));
    }

    void PublishCommandReport(const CommandContext& context, std::string_view command_name,
                              swarmkit::v1::AgentReportType type,
                              swarmkit::v1::ReportSeverity severity, std::string_view detail) {
        std::string message = "command=" + std::string(command_name);
        if (!detail.empty()) {
            message += " ";
            message += detail;
        }
        PublishSimpleReport(context.drone_id, context.correlation_id, type, severity, message);
    }

    void PublishGoalFailure(const swarmkit::v1::ActiveGoal& goal, std::string_view correlation_id,
                            std::string_view message) {
        swarmkit::v1::AgentReport report;
        report.set_drone_id(goal.drone_id());
        report.set_correlation_id(std::string(correlation_id));
        report.set_type(swarmkit::v1::GOAL_REPORT);
        report.set_severity(swarmkit::v1::REPORT_ERROR);
        report.set_message(std::string(message));
        auto* goal_report = report.mutable_goal();
        goal_report->set_drone_id(goal.drone_id());
        goal_report->set_goal_id(goal.goal_id());
        goal_report->set_revision(goal.revision());
        goal_report->set_status(swarmkit::v1::GOAL_FAILED);
        goal_report->set_acceptance_radius_m(goal.acceptance_radius_m());
        goal_report->set_deviation_radius_m(goal.deviation_radius_m());
        goal_report->set_timeout_ms(goal.timeout_ms());
        goal_report->set_message(std::string(message));
        reports_.Publish(std::move(report));
    }

    AgentConfig config_;
    DroneBackendPtr backend_;
    CommandArbiter arbiter_;
    internal::TelemetryManager telemetry_;
    internal::ReportHub reports_;
    internal::ActiveGoalSupervisor goals_;
    internal::TrajectoryExecutionManager executions_;
    internal::RuntimeCounters counters_;
    std::atomic<bool> ready_{true};
    std::string startup_error_;
    mutable std::mutex data_mutex_;
    std::condition_variable data_cv_;
    std::deque<swarmkit::v1::DataMessage> message_backlog_;
    std::unordered_map<std::string, swarmkit::v1::ArtifactDescriptor> artifacts_;
    std::uint64_t data_sequence_{0};
};

/// @}

}  // namespace

// ---------------------------------------------------------------------------
// AgentConfig
// ---------------------------------------------------------------------------

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

core::Result ReportPersistenceConfig::Validate() const {
    if (backlog_size < 0) {
        return core::Result::Rejected("reports.backlog_size must be >= 0");
    }
    if (max_log_file_size_bytes < 0) {
        return core::Result::Rejected("reports.max_log_file_size_bytes must be >= 0");
    }
    if (max_log_files < 0) {
        return core::Result::Rejected("reports.max_log_files must be >= 0");
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
    const auto iter = std::ranges::find_if(peers, [drone_id](const DataPeerConfig& peer) {
        return peer.drone_id == drone_id;
    });
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
    if (const core::Result vehicle_result = vehicle_profile.Validate(); !vehicle_result.IsOk()) {
        return vehicle_result;
    }
    if (const core::Result report_result = reports.Validate(); !report_result.IsOk()) {
        return report_result;
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
    ApplyBoolEnv(prefix, kAgentEnvAllowUnsafeBenchCommands,
                 &safety.allow_unsafe_bench_commands);
    ApplyStringEnv(prefix, kAgentEnvReportLogFile, &reports.log_file);
    ApplyStringEnv(prefix, kAgentEnvReportSequenceStateFile, &reports.sequence_state_file);
    ApplyIntEnv(prefix, kAgentEnvReportBacklogSize, &reports.backlog_size);
    ApplyIntEnv(prefix, kAgentEnvReportLogMaxFileSizeBytes, &reports.max_log_file_size_bytes);
    ApplyIntEnv(prefix, kAgentEnvReportLogMaxFiles, &reports.max_log_files);
    ApplyBoolEnv(prefix, kAgentEnvReportFlushEachWrite, &reports.flush_each_write);
    ApplyBoolEnv(prefix, kAgentEnvReportFsyncEachWrite, &reports.fsync_each_write);
    ApplyBoolEnv(prefix, kAgentEnvReportReplayFromLog, &reports.replay_from_log);
    ApplyStringEnv(prefix, kAgentEnvArtifactDir, &data.artifact_dir);
    ApplyIntEnv(prefix, kAgentEnvDataMessageBacklogSize, &data.message_backlog_size);
    ApplyIntEnv(prefix, kAgentEnvDataMaxMessagePayloadBytes, &data.max_message_payload_bytes);
    ApplyIntEnv(prefix, kAgentEnvDataArtifactChunkBytes, &data.artifact_chunk_bytes);
    ApplyIntEnv(prefix, kAgentEnvDataMaxArtifactBytes, &data.max_artifact_bytes);

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

    const auto report_log_file = core::yaml::ReadOptionalScalar<std::string>(root, "report_log_file");
    if (!report_log_file.has_value()) {
        return std::unexpected(report_log_file.error());
    }
    if (report_log_file->has_value()) {
        config.reports.log_file = report_log_file->value_or(config.reports.log_file);
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
            config.safety.allow_unsafe_bench_commands = allow_unsafe_bench_commands->value_or(false);
        }
    }

        const YAML::Node report_persistence =
        root["report_persistence"] ? root["report_persistence"] : root["reports"];
    if (report_persistence) {
        if (!report_persistence.IsMap()) {
            return std::unexpected(
                core::Result::Rejected("agent.report_persistence must be a map"));
        }

        const auto log_file =
            core::yaml::ReadOptionalScalar<std::string>(report_persistence, "log_file");
        if (!log_file.has_value()) {
            return std::unexpected(log_file.error());
        }
        if (log_file->has_value()) {
            config.reports.log_file = log_file->value_or(config.reports.log_file);
        }

        const auto sequence_state_file = core::yaml::ReadOptionalScalar<std::string>(
            report_persistence, "sequence_state_file");
        if (!sequence_state_file.has_value()) {
            return std::unexpected(sequence_state_file.error());
        }
        if (sequence_state_file->has_value()) {
            config.reports.sequence_state_file =
                sequence_state_file->value_or(config.reports.sequence_state_file);
        }

        const auto backlog_size =
            core::yaml::ReadOptionalScalar<int>(report_persistence, "backlog_size");
        if (!backlog_size.has_value()) {
            return std::unexpected(backlog_size.error());
        }
        if (backlog_size->has_value()) {
            config.reports.backlog_size = backlog_size->value_or(config.reports.backlog_size);
        }

        const auto max_log_file_size_bytes = core::yaml::ReadOptionalScalar<int>(
            report_persistence, "max_log_file_size_bytes");
        if (!max_log_file_size_bytes.has_value()) {
            return std::unexpected(max_log_file_size_bytes.error());
        }
        if (max_log_file_size_bytes->has_value()) {
            config.reports.max_log_file_size_bytes =
                max_log_file_size_bytes->value_or(config.reports.max_log_file_size_bytes);
        }

        const auto max_log_files =
            core::yaml::ReadOptionalScalar<int>(report_persistence, "max_log_files");
        if (!max_log_files.has_value()) {
            return std::unexpected(max_log_files.error());
        }
        if (max_log_files->has_value()) {
            config.reports.max_log_files =
                max_log_files->value_or(config.reports.max_log_files);
        }

        const auto flush_each_write =
            core::yaml::ReadOptionalScalar<bool>(report_persistence, "flush_each_write");
        if (!flush_each_write.has_value()) {
            return std::unexpected(flush_each_write.error());
        }
        if (flush_each_write->has_value()) {
            config.reports.flush_each_write =
                flush_each_write->value_or(config.reports.flush_each_write);
        }

        const auto fsync_each_write =
            core::yaml::ReadOptionalScalar<bool>(report_persistence, "fsync_each_write");
        if (!fsync_each_write.has_value()) {
            return std::unexpected(fsync_each_write.error());
        }
        if (fsync_each_write->has_value()) {
            config.reports.fsync_each_write =
                fsync_each_write->value_or(config.reports.fsync_each_write);
        }

        const auto replay_from_log =
            core::yaml::ReadOptionalScalar<bool>(report_persistence, "replay_from_log");
        if (!replay_from_log.has_value()) {
            return std::unexpected(replay_from_log.error());
        }
        if (replay_from_log->has_value()) {
            config.reports.replay_from_log =
                replay_from_log->value_or(config.reports.replay_from_log);
        }
    }

    const YAML::Node data = root["data"] ? root["data"] : root["data_plane"];
    if (data) {
        if (!data.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.data must be a map"));
        }

        const auto artifact_dir =
            core::yaml::ReadOptionalScalar<std::string>(data, "artifact_dir");
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

                const auto address =
                    core::yaml::ReadOptionalScalar<std::string>(entry, "address");
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

                const auto server_authority_override = core::yaml::ReadOptionalScalar<std::string>(
                    entry, "server_authority_override");
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
            config.vehicle_profile.battery_reserve_percent = battery_reserve_percent->value_or(
                config.vehicle_profile.battery_reserve_percent);
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
            config.vehicle_profile.takeoff_timeout_margin_ms =
                takeoff_timeout_margin_ms->value_or(
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

// ---------------------------------------------------------------------------
// RunAgentServer
// ---------------------------------------------------------------------------

int RunAgentServer(const AgentConfig& config, DroneBackendPtr backend) {
    if (!backend) {
        core::Logger::Error("RunAgentServer: backend is null");
        return 1;
    }
    if (const core::Result kValidation = config.Validate(); !kValidation.IsOk()) {
        core::Logger::ErrorFmt("RunAgentServer: invalid config - {}", kValidation.message);
        return 1;
    }

    grpc::ServerBuilder builder;
    core::Result credentials_error = core::Result::Success();
    auto credentials = MakeServerCredentials(config.security, &credentials_error);
    if (!credentials) {
        core::Logger::ErrorFmt("Failed to configure server credentials: {}",
                               credentials_error.message);
        return 1;
    }
    builder.AddListeningPort(config.bind_addr, credentials);

    auto service = internal::MakeAgentServiceForTesting(config, std::move(backend));
    if (auto* agent_service = dynamic_cast<swarmkit::v1::AgentService::Service*>(service.get());
        agent_service != nullptr) {
        builder.RegisterService(agent_service);
    }
    if (auto* data_service = dynamic_cast<swarmkit::v1::DataService::Service*>(service.get());
        data_service != nullptr) {
        builder.RegisterService(data_service);
    }

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    if (!server) {
        core::Logger::ErrorFmt("Failed to start gRPC server on {}", config.bind_addr);
        return 1;
    }

    core::Logger::InfoFmt("Agent '{}' listening on {}", config.agent_id, config.bind_addr);
    server->Wait();
    return 0;
}

std::unique_ptr<swarmkit::v1::AgentService::Service> internal::MakeAgentServiceForTesting(
    const AgentConfig& config, DroneBackendPtr backend) {
    return std::make_unique<AgentServiceImpl>(config, std::move(backend));
}

}  // namespace swarmkit::agent
