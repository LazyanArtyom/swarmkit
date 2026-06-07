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
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <unordered_set>
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

[[nodiscard]] std::filesystem::path ArtifactStorageRoot(const AgentConfig& config) {
    std::filesystem::path root(config.data.artifact_dir);
    if (config.data.artifact_dir == DataPlaneConfig{}.artifact_dir) {
        root /= SanitizedPathComponent(config.agent_id);
    }
    return root;
}

[[nodiscard]] bool WriteBytes(std::ostream& output, std::span<const char> bytes) {
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return output.good();
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

[[nodiscard]] std::string TransportSecurityName(core::TransportSecurityMode mode) {
    switch (mode) {
        case core::TransportSecurityMode::kInsecure:
            return "insecure";
        case core::TransportSecurityMode::kTls:
            return "tls";
        case core::TransportSecurityMode::kMutualTls:
            return "mtls";
        case core::TransportSecurityMode::kAuto:
        default:
            return "auto";
    }
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
          goals_(&telemetry_, &reports_, &config_) {
        if (const core::Result start_result = backend_->Start(); !start_result.IsOk()) {
            ready_.store(false, std::memory_order_relaxed);
            startup_error_ = start_result.message;
            core::Logger::ErrorFmt("Agent backend failed to start: {}", start_result.message);
        }
    }

    ~AgentServiceImpl() override {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (auto& [unused, cancel] : artifact_transfer_cancel_) {
                static_cast<void>(unused);
                cancel->store(true, std::memory_order_relaxed);
            }
        }
        std::lock_guard<std::mutex> lock(artifact_transfer_threads_mutex_);
        for (std::thread& worker : artifact_transfer_threads_) {
            if (worker.joinable()) {
                worker.join();
            }
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
        reply->set_supports_payload_control(capabilities.supports_payload_control);
        reply->set_supports_velocity_control(capabilities.supports_velocity_control);
        reply->set_supports_flight_termination(capabilities.supports_flight_termination);
        reply->set_supports_backend_commands(capabilities.supports_backend_commands);
        reply->set_autopilot_type(capabilities.autopilot_type);
        for (const std::string& mode : capabilities.supported_modes) {
            reply->add_supported_modes(mode);
        }
        for (const std::string& command : capabilities.supported_commands) {
            reply->add_supported_commands(command);
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
                "requires a global GPS target";
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
            core::Logger::WarnFmt(
                "rpc=PublishMessage corr={} agent={} source={} target={} result=rejected "
                "reason=missing_topic",
                correlation_id, config_.agent_id, message.source_id(), message.target_id());
            return grpc::Status::OK;
        }
        if (message.payload().size() >
            static_cast<std::size_t>(config_.data.max_message_payload_bytes)) {
            counters_.IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("message payload exceeds max_message_payload_bytes");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_REJECTED);
            core::Logger::WarnFmt(
                "rpc=PublishMessage corr={} agent={} topic={} source={} target={} "
                "payload_bytes={} limit={} result=rejected reason=payload_too_large",
                correlation_id, config_.agent_id, message.topic(), message.source_id(),
                message.target_id(), message.payload().size(),
                config_.data.max_message_payload_bytes);
            return grpc::Status::OK;
        }
        if (message.source_id().empty()) {
            message.set_source_id(config_.agent_id);
        }
        if (message.message_id().empty()) {
            message.set_message_id(
                MakeCorrelationId("msg-" + SanitizedPathComponent(message.source_id())));
        }
        if (message.unix_time_ms() == 0) {
            message.set_unix_time_ms(NowUnixMs());
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            message.set_sequence(++data_sequence_);
            if (message.message_id().empty()) {
                message.set_message_id(
                    MakeCorrelationId("msg-" + SanitizedPathComponent(message.source_id())));
            }
            message_backlog_.push_back(message);
            while (static_cast<int>(message_backlog_.size()) > config_.data.message_backlog_size) {
                message_backlog_.pop_front();
            }
        }
        data_cv_.notify_all();
        counters_.IncrementDataMessagesPublished();
        core::Logger::InfoFmt(
            "rpc=PublishMessage corr={} agent={} seq={} message_id={} topic={} source={} "
            "target={} payload_bytes={} result=published",
            correlation_id, config_.agent_id, message.sequence(), message.message_id(),
            message.topic(), message.source_id(), message.target_id(), message.payload().size());

        reply->set_ok(true);
        reply->set_message("message published");
        reply->set_correlation_id(correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        reply->set_sequence(message.sequence());
        return grpc::Status::OK;
    }

    grpc::Status ListDataPeers(grpc::ServerContext* ctx,
                               const swarmkit::v1::DataPeerListRequest* req,
                               swarmkit::v1::DataPeerListReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-peers");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->refresh()) {
            RefreshPeerStatuses({});
        }
        FillPeerListReply(correlation_id, reply);
        core::Logger::InfoFmt("rpc=ListDataPeers corr={} agent={} peer_count={} refresh={}",
                              correlation_id, config_.agent_id, reply->peers_size(),
                              req->refresh());
        return grpc::Status::OK;
    }

    grpc::Status RefreshDataPeers(grpc::ServerContext* ctx,
                                  const swarmkit::v1::DataPeerRefreshRequest* req,
                                  swarmkit::v1::DataPeerListReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "data-peers-refresh");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        std::unordered_set<std::string> requested;
        for (const std::string& drone_id : req->drone_ids()) {
            requested.insert(drone_id);
        }
        RefreshPeerStatuses(requested);
        FillPeerListReply(correlation_id, reply);
        core::Logger::InfoFmt(
            "rpc=RefreshDataPeers corr={} agent={} requested={} peer_count={}", correlation_id,
            config_.agent_id, requested.size(), reply->peers_size());
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
            core::Logger::WarnFmt(
                "rpc=SendMessageToDrone corr={} agent={} target={} topic={} result=rejected "
                "reason=no_peer",
                correlation_id, config_.agent_id, message.target_id(), message.topic());
            return grpc::Status::OK;
        }
        if (message.source_id().empty()) {
            message.set_source_id(config_.agent_id);
        }
        if (message.unix_time_ms() == 0) {
            message.set_unix_time_ms(NowUnixMs());
        }

        const auto route_started = std::chrono::steady_clock::now();
        core::Logger::InfoFmt(
            "rpc=SendMessageToDrone corr={} agent={} peer={} address={} topic={} source={} "
            "target={} message_id={} payload_bytes={} result=routing",
            correlation_id, config_.agent_id, peer->drone_id, peer->address, message.topic(),
            message.source_id(), message.target_id(), message.message_id(), message.payload().size());
        auto channel = MakePeerChannel(*peer, config_.security);
        auto stub = swarmkit::v1::DataService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{30});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), correlation_id);
        swarmkit::v1::PublishMessageRequest peer_req;
        *peer_req.mutable_message() = std::move(message);
        swarmkit::v1::PublishMessageReply peer_reply;
        const grpc::Status peer_status = stub->PublishMessage(&peer_ctx, peer_req, &peer_reply);
        const auto route_latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now() - route_started)
                                          .count();
        if (!peer_status.ok()) {
            counters_.IncrementDataMessagesRejected();
            reply->set_ok(false);
            reply->set_message("peer message delivery failed: " + peer_status.error_message());
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_UNAVAILABLE);
            MarkPeerRouteResult(*peer, false, route_latency_ms, peer_status.error_message());
            core::Logger::WarnFmt(
                "rpc=SendMessageToDrone corr={} agent={} peer={} address={} topic={} target={} "
                "latency_ms={} result=failed error={}",
                correlation_id, config_.agent_id, peer->drone_id, peer->address,
                peer_req.message().topic(), peer_req.message().target_id(), route_latency_ms,
                peer_status.error_message());
            return grpc::Status::OK;
        }
        MarkPeerRouteResult(*peer, peer_reply.ok(), route_latency_ms, peer_reply.message());
        *reply = peer_reply;
        if (reply->correlation_id().empty()) {
            reply->set_correlation_id(correlation_id);
        }
        if (reply->ok()) {
            reply->set_message("message delivered to " + peer->drone_id + ": " + reply->message());
        }
        core::Logger::InfoFmt(
            "rpc=SendMessageToDrone corr={} agent={} peer={} address={} topic={} target={} "
            "remote_seq={} latency_ms={} result={}",
            correlation_id, config_.agent_id, peer->drone_id, peer->address,
            peer_req.message().topic(), peer_req.message().target_id(), reply->sequence(),
            route_latency_ms, reply->ok() ? "delivered" : "rejected");
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
        std::filesystem::path tmp_dir = ArtifactStorageRoot(config_) / "tmp";
        std::error_code fs_error;
        std::filesystem::create_directories(tmp_dir, fs_error);
        if (fs_error) {
            const std::string detail =
                "failed to create artifact temp dir path=" + tmp_dir.string() +
                " error=" + fs_error.message();
            core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id, detail);
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, detail);
        }

        swarmkit::v1::ArtifactDescriptor descriptor;
        std::string transfer_id;
        std::filesystem::path tmp_path;
        std::ofstream output;
        std::int64_t received_bytes = 0;
        int expected_chunk_index = 0;
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
            if (saw_final) {
                return fail_upload(grpc::StatusCode::INVALID_ARGUMENT,
                                   "artifact chunk received after final chunk");
            }
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
                transfer_id =
                    chunk.transfer_id().empty() ? MakeCorrelationId("artifact-upload")
                                                : chunk.transfer_id();
                tmp_path = tmp_dir / (SanitizedPathComponent(transfer_id) + ".part");
                output.open(tmp_path, std::ios::binary | std::ios::trunc);
                if (!output.is_open()) {
                    core::Logger::ErrorFmt(
                        "rpc=UploadArtifact agent={} failed to open artifact temp file path={}",
                        config_.agent_id, tmp_path.string());
                    return fail_upload(grpc::StatusCode::FAILED_PRECONDITION,
                                       "failed to open artifact temp file path=" +
                                           tmp_path.string());
                }
                saw_descriptor = true;
            }
            if (chunk.chunk_index() != expected_chunk_index) {
                return fail_upload(grpc::StatusCode::INVALID_ARGUMENT,
                                   "artifact chunks must arrive in order with matching indexes");
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
            if (!WriteBytes(output, std::span<const char>{chunk.data().data(),
                                                          chunk.data().size()})) {
                return fail_upload(grpc::StatusCode::FAILED_PRECONDITION,
                                   "failed to write artifact temp file");
            }
            received_bytes += static_cast<std::int64_t>(chunk.data().size());
            ++expected_chunk_index;
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

        const std::filesystem::path final_dir = ArtifactStorageRoot(config_) /
                                                SanitizedPathComponent(descriptor.source_id()) /
                                                SanitizedPathComponent(descriptor.artifact_id());
        std::filesystem::create_directories(final_dir, fs_error);
        if (fs_error) {
            const std::string detail =
                "failed to create artifact final dir path=" + final_dir.string() +
                " error=" + fs_error.message();
            core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id, detail);
            std::filesystem::remove(tmp_path, fs_error);
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, detail);
        }
        const std::filesystem::path final_path =
            final_dir / SanitizedPathComponent(descriptor.filename());
        std::filesystem::rename(tmp_path, final_path, fs_error);
        if (fs_error) {
            std::error_code exists_error;
            if (std::filesystem::exists(final_path, exists_error)) {
                const std::string existing_sha256 = core::internal::Sha256FileHex(final_path);
                if (existing_sha256 == sha256_hex) {
                    std::filesystem::remove(tmp_path, fs_error);
                    fs_error.clear();
                } else {
                    std::filesystem::remove(tmp_path, exists_error);
                    counters_.IncrementArtifactFailures();
                    const std::string detail =
                        "artifact final path exists with different sha256 path=" +
                        final_path.string() + " existing_sha256=" + existing_sha256 +
                        " incoming_sha256=" + sha256_hex;
                    core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id,
                                           detail);
                    return grpc::Status(grpc::StatusCode::ALREADY_EXISTS, detail);
                }
            }
        }
        if (fs_error) {
            const std::string detail =
                "failed to finalize artifact tmp_path=" + tmp_path.string() +
                " final_path=" + final_path.string() + " error=" + fs_error.message();
            core::Logger::ErrorFmt("rpc=UploadArtifact agent={} {}", config_.agent_id, detail);
            std::filesystem::remove(tmp_path, fs_error);
            counters_.IncrementArtifactFailures();
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, detail);
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            artifacts_[descriptor.artifact_id()] =
                StoredArtifact{.descriptor = descriptor, .storage_path = final_path};
        }
        PublishArtifactReceivedEvent(descriptor, "artifact-upload");
        *reply = descriptor;
        counters_.IncrementArtifactUploads(static_cast<std::uint64_t>(received_bytes));
        core::Logger::InfoFmt(
            "rpc=UploadArtifact agent={} artifact_id={} source={} target={} bytes={} path={} "
            "result=stored",
            config_.agent_id, descriptor.artifact_id(), descriptor.source_id(),
            descriptor.target_id(), received_bytes, final_path.string());
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
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} result=rejected reason=missing_target",
                correlation_id, config_.agent_id);
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "artifact.target_id is required for routed sends");
        }
        if (descriptor.target_id() == config_.agent_id) {
            counters_.IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} target={} result=rejected "
                "reason=local_target",
                correlation_id, config_.agent_id, descriptor.target_id());
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "routed artifact target_id points to local agent");
        }
        const std::optional<DataPeerConfig> peer = config_.data.FindPeer(descriptor.target_id());
        if (!peer.has_value()) {
            counters_.IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} target={} result=rejected "
                "reason=no_peer",
                correlation_id, config_.agent_id, descriptor.target_id());
            return grpc::Status(grpc::StatusCode::NOT_FOUND,
                                "route failed before upload: no data peer configured for target_id=" +
                                    descriptor.target_id());
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
            MarkPeerRouteResult(*peer, false, 0, "peer stream closed");
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} peer={} address={} result=failed "
                "error=peer_stream_closed",
                correlation_id, config_.agent_id, peer->drone_id, peer->address);
            return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                "peer artifact stream closed before all chunks were accepted");
        }
        if (!peer_status.ok()) {
            counters_.IncrementArtifactFailures();
            MarkPeerRouteResult(*peer, false, 0, peer_status.error_message());
            core::Logger::WarnFmt(
                "rpc=SendArtifactToDrone corr={} agent={} peer={} address={} result=failed "
                "error={}",
                correlation_id, config_.agent_id, peer->drone_id, peer->address,
                peer_status.error_message());
            return grpc::Status(peer_status.error_code(),
                                "peer artifact delivery failed: " + peer_status.error_message());
        }
        MarkPeerRouteResult(*peer, true, 0, "artifact delivered");
        counters_.IncrementArtifactDownloads(bytes_forwarded);
        core::Logger::InfoFmt(
            "rpc=SendArtifactToDrone corr={} agent={} peer={} address={} artifact_id={} "
            "bytes={} result=delivered",
            correlation_id, config_.agent_id, peer->drone_id, peer->address, reply->artifact_id(),
            bytes_forwarded);
        return grpc::Status::OK;
    }

    grpc::Status StartArtifactTransfer(grpc::ServerContext* ctx,
                                       const swarmkit::v1::ArtifactTransferStartRequest* req,
                                       swarmkit::v1::ArtifactTransferReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-transfer");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->source_path().empty()) {
            FillArtifactTransferReply(reply, false, "source_path is required", correlation_id,
                                      swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, {});
            return grpc::Status::OK;
        }
        if (req->route_to_target() && req->artifact().target_id().empty()) {
            FillArtifactTransferReply(reply, false,
                                      "artifact.target_id is required for routed transfer",
                                      correlation_id,
                                      swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, {});
            return grpc::Status::OK;
        }
        std::error_code fs_error;
        const auto file_size = std::filesystem::file_size(req->source_path(), fs_error);
        if (fs_error) {
            FillArtifactTransferReply(reply, false,
                                      "source_path cannot be read path=" + req->source_path() +
                                          " error=" + fs_error.message(),
                                      correlation_id,
                                      swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT, {});
            return grpc::Status::OK;
        }

        const std::string transfer_id = MakeCorrelationId("artifact-transfer");
        swarmkit::v1::ArtifactTransferStatus status;
        status.set_transfer_id(transfer_id);
        status.set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_QUEUED);
        *status.mutable_artifact() = req->artifact();
        status.set_bytes_total(static_cast<std::int64_t>(file_size));
        status.set_started_unix_ms(NowUnixMs());
        status.set_updated_unix_ms(status.started_unix_ms());
        status.set_message("queued");
        status.set_error_code(swarmkit::v1::ERROR_CODE_NONE);

        auto cancel_flag = std::make_shared<std::atomic<bool>>(false);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            artifact_transfers_[transfer_id] = status;
            artifact_transfer_cancel_[transfer_id] = cancel_flag;
        }
        swarmkit::v1::ArtifactTransferStartRequest request = *req;
        {
            std::lock_guard<std::mutex> lock(artifact_transfer_threads_mutex_);
            artifact_transfer_threads_.emplace_back(
                [this, transfer_id, request = std::move(request), cancel_flag] {
                    RunArtifactTransfer(transfer_id, request, cancel_flag);
                });
        }
        core::Logger::InfoFmt(
            "rpc=StartArtifactTransfer corr={} agent={} transfer_id={} source_path={} "
            "target={} route={} bytes={} result=queued",
            correlation_id, config_.agent_id, transfer_id, req->source_path(),
            req->artifact().target_id(), req->route_to_target(), file_size);
        FillArtifactTransferReply(reply, true, "artifact transfer queued", correlation_id,
                                  swarmkit::v1::ERROR_CODE_NONE, status);
        return grpc::Status::OK;
    }

    grpc::Status GetArtifactTransfer(grpc::ServerContext* ctx,
                                     const swarmkit::v1::ArtifactTransferRequest* req,
                                     swarmkit::v1::ArtifactTransferReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-transfer-get");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const auto status = FindArtifactTransfer(req->transfer_id());
        if (!status.has_value()) {
            FillArtifactTransferReply(reply, false, "artifact transfer not found", correlation_id,
                                      swarmkit::v1::ERROR_CODE_UNAVAILABLE, {});
            return grpc::Status::OK;
        }
        FillArtifactTransferReply(reply, true, status->message(), correlation_id,
                                  status->error_code(), *status);
        return grpc::Status::OK;
    }

    grpc::Status CancelArtifactTransfer(grpc::ServerContext* ctx,
                                        const swarmkit::v1::ArtifactTransferRequest* req,
                                        swarmkit::v1::ArtifactTransferReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id =
            ResolveCorrelationId(ctx, "", "artifact-transfer-cancel");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        std::optional<swarmkit::v1::ArtifactTransferStatus> status;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const auto cancel_iter = artifact_transfer_cancel_.find(req->transfer_id());
            const auto status_iter = artifact_transfers_.find(req->transfer_id());
            if (status_iter == artifact_transfers_.end()) {
                FillArtifactTransferReply(reply, false, "artifact transfer not found",
                                          correlation_id, swarmkit::v1::ERROR_CODE_UNAVAILABLE,
                                          {});
                return grpc::Status::OK;
            }
            if (cancel_iter != artifact_transfer_cancel_.end()) {
                cancel_iter->second->store(true, std::memory_order_relaxed);
            }
            if (status_iter->second.state() ==
                    swarmkit::v1::ARTIFACT_TRANSFER_STATE_COMPLETED ||
                status_iter->second.state() == swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED ||
                status_iter->second.state() == swarmkit::v1::ARTIFACT_TRANSFER_STATE_CANCELLED) {
                status = status_iter->second;
            } else {
                status_iter->second.set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_CANCELLED);
                status_iter->second.set_message("cancel requested");
                status_iter->second.set_updated_unix_ms(NowUnixMs());
                status = status_iter->second;
            }
        }
        core::Logger::InfoFmt("rpc=CancelArtifactTransfer corr={} agent={} transfer_id={}",
                              correlation_id, config_.agent_id, req->transfer_id());
        FillArtifactTransferReply(reply, true, status->message(), correlation_id,
                                  status->error_code(), *status);
        return grpc::Status::OK;
    }

    grpc::Status ListArtifacts(grpc::ServerContext* ctx,
                               const swarmkit::v1::ArtifactListRequest* req,
                               swarmkit::v1::ArtifactListReply* reply) override {
        if (ctx == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-list");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        const std::string source_id = req == nullptr ? "" : req->source_id();
        const std::string target_id = req == nullptr ? "" : req->target_id();
        const bool include_expired = req != nullptr && req->include_expired();
        const std::int64_t now_ms = NowUnixMs();
        std::vector<std::string> expired_ids;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (const auto& [artifact_id, stored] : artifacts_) {
                const auto& descriptor = stored.descriptor;
                const bool expired = ArtifactExpired(descriptor, now_ms);
                if (expired && !include_expired) {
                    expired_ids.push_back(artifact_id);
                    continue;
                }
                if (!source_id.empty() && descriptor.source_id() != source_id) {
                    continue;
                }
                if (!target_id.empty() && descriptor.target_id() != target_id) {
                    continue;
                }
                *reply->add_artifacts() = descriptor;
            }
            for (const std::string& artifact_id : expired_ids) {
                artifacts_.erase(artifact_id);
            }
        }
        reply->set_correlation_id(correlation_id);
        core::Logger::InfoFmt(
            "rpc=ListArtifacts corr={} agent={} count={} source_filter={} target_filter={}",
            correlation_id, config_.agent_id, reply->artifacts_size(), source_id, target_id);
        return grpc::Status::OK;
    }

    grpc::Status GetArtifact(grpc::ServerContext* ctx,
                             const swarmkit::v1::ArtifactRequest* req,
                             swarmkit::v1::ArtifactReply* reply) override {
        if (ctx == nullptr || req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/reply");
        }
        const std::string correlation_id = ResolveCorrelationId(ctx, "", "artifact-info");
        if (const core::Result auth = AuthorizePeer(ctx, config_.security, nullptr); !auth.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth.message);
        }
        if (req->artifact_id().empty()) {
            reply->set_ok(false);
            reply->set_message("artifact_id is required");
            reply->set_correlation_id(correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            return grpc::Status::OK;
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const auto iter = artifacts_.find(req->artifact_id());
            if (iter == artifacts_.end()) {
                reply->set_ok(false);
                reply->set_message("artifact not found");
                reply->set_correlation_id(correlation_id);
                reply->set_error_code(swarmkit::v1::ERROR_CODE_UNAVAILABLE);
                return grpc::Status::OK;
            }
            if (ArtifactExpired(iter->second.descriptor, NowUnixMs())) {
                artifacts_.erase(iter);
                reply->set_ok(false);
                reply->set_message("artifact expired");
                reply->set_correlation_id(correlation_id);
                reply->set_error_code(swarmkit::v1::ERROR_CODE_UNAVAILABLE);
                return grpc::Status::OK;
            }
            *reply->mutable_artifact() = iter->second.descriptor;
        }
        reply->set_ok(true);
        reply->set_message("artifact found");
        reply->set_correlation_id(correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
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
        std::filesystem::path storage_path;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const auto iter = artifacts_.find(req->artifact_id());
            if (iter == artifacts_.end()) {
                counters_.IncrementArtifactFailures();
                core::Logger::WarnFmt(
                    "rpc=DownloadArtifact agent={} artifact_id={} result=failed reason=not_found",
                    config_.agent_id, req->artifact_id());
                return grpc::Status(grpc::StatusCode::NOT_FOUND, "artifact not found");
            }
            descriptor = iter->second.descriptor;
            storage_path = iter->second.storage_path;
            if (ArtifactExpired(descriptor, NowUnixMs())) {
                artifacts_.erase(iter);
                counters_.IncrementArtifactFailures();
                core::Logger::WarnFmt(
                    "rpc=DownloadArtifact agent={} artifact_id={} result=failed reason=expired",
                    config_.agent_id, req->artifact_id());
                return grpc::Status(grpc::StatusCode::NOT_FOUND, "artifact expired");
            }
        }
        std::ifstream input(storage_path, std::ios::binary);
        if (!input.is_open()) {
            counters_.IncrementArtifactFailures();
            core::Logger::WarnFmt(
                "rpc=DownloadArtifact agent={} artifact_id={} path={} result=failed "
                "reason=file_not_found",
                config_.agent_id, req->artifact_id(), storage_path.string());
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
            core::Logger::InfoFmt(
                "rpc=DownloadArtifact agent={} artifact_id={} bytes=0 result=sent",
                config_.agent_id, descriptor.artifact_id());
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
        core::Logger::InfoFmt(
            "rpc=DownloadArtifact agent={} artifact_id={} bytes={} result=sent", config_.agent_id,
            descriptor.artifact_id(), offset);
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
            core::Logger::WarnFmt(
                "rpc=AnnounceArtifact corr={} agent={} result=rejected reason=missing_artifact_id",
                correlation_id, config_.agent_id);
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
            artifacts_[descriptor.artifact_id()] =
                StoredArtifact{.descriptor = descriptor, .storage_path = {}};
            *reply->mutable_artifact() = descriptor;
        }
        reply->set_ok(true);
        reply->set_message("artifact announced");
        reply->set_correlation_id(correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        core::Logger::InfoFmt(
            "rpc=AnnounceArtifact corr={} agent={} artifact_id={} source={} target={} "
            "result=announced",
            correlation_id, config_.agent_id, reply->artifact().artifact_id(),
            reply->artifact().source_id(), reply->artifact().target_id());
        return grpc::Status::OK;
    }

   private:
    struct StoredArtifact {
        swarmkit::v1::ArtifactDescriptor descriptor;
        std::filesystem::path storage_path;
    };

    [[nodiscard]] swarmkit::v1::DataPeerStatus DefaultPeerStatus(
        const DataPeerConfig& peer) const {
        swarmkit::v1::DataPeerStatus status;
        status.set_drone_id(peer.drone_id);
        status.set_address(peer.address);
        status.set_transport_security(
            TransportSecurityName(EffectivePeerTransportSecurity(peer, config_.security)));
        status.set_state(swarmkit::v1::DATA_PEER_STATE_UNKNOWN);
        status.set_message("not checked");
        return status;
    }

    [[nodiscard]] swarmkit::v1::DataPeerStatus RefreshPeerStatus(
        const DataPeerConfig& peer) const {
        swarmkit::v1::DataPeerStatus status = DefaultPeerStatus(peer);
        const std::int64_t started_ms = NowUnixMs();
        status.set_last_checked_unix_ms(started_ms);
        if (const core::Result validation = peer.Validate(); !validation.IsOk()) {
            status.set_state(swarmkit::v1::DATA_PEER_STATE_MISCONFIGURED);
            status.set_last_failure_unix_ms(started_ms);
            status.set_message(validation.message);
            return status;
        }

        auto channel = MakePeerChannel(peer, config_.security);
        auto stub = swarmkit::v1::AgentService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{3});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), MakeCorrelationId("peer-check"));
        swarmkit::v1::PingRequest request;
        request.set_agent_id(config_.agent_id);
        swarmkit::v1::PingReply reply;
        const grpc::Status ping_status = stub->Ping(&peer_ctx, request, &reply);
        const std::int64_t finished_ms = NowUnixMs();
        status.set_round_trip_ms(std::max<std::int64_t>(0, finished_ms - started_ms));
        if (!ping_status.ok()) {
            status.set_state(swarmkit::v1::DATA_PEER_STATE_UNREACHABLE);
            status.set_last_failure_unix_ms(finished_ms);
            status.set_message(ping_status.error_message().empty()
                                   ? "peer ping failed"
                                   : ping_status.error_message());
            return status;
        }
        status.set_state(swarmkit::v1::DATA_PEER_STATE_READY);
        status.set_last_success_unix_ms(finished_ms);
        status.set_message("ready agent_id=" + reply.agent_id());
        return status;
    }

    void RefreshPeerStatuses(const std::unordered_set<std::string>& requested) {
        for (const auto& peer : config_.data.peers) {
            if (!requested.empty() && !requested.contains(peer.drone_id)) {
                continue;
            }
            swarmkit::v1::DataPeerStatus status = RefreshPeerStatus(peer);
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                peer_statuses_[peer.drone_id] = status;
            }
            core::Logger::InfoFmt(
                "data_peer_refresh agent={} peer={} address={} state={} rtt_ms={} message={}",
                config_.agent_id, status.drone_id(), status.address(),
                static_cast<int>(status.state()), status.round_trip_ms(), status.message());
        }
    }

    void FillPeerListReply(std::string_view correlation_id,
                           swarmkit::v1::DataPeerListReply* reply) const {
        if (reply == nullptr) {
            return;
        }
        reply->set_correlation_id(std::string(correlation_id));
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (const auto& peer : config_.data.peers) {
            const auto iter = peer_statuses_.find(peer.drone_id);
            if (iter != peer_statuses_.end()) {
                *reply->add_peers() = iter->second;
            } else {
                *reply->add_peers() = DefaultPeerStatus(peer);
            }
        }
    }

    void MarkPeerRouteResult(const DataPeerConfig& peer, bool ok, long long latency_ms,
                             std::string_view message) {
        swarmkit::v1::DataPeerStatus status = DefaultPeerStatus(peer);
        const std::int64_t now_ms = NowUnixMs();
        status.set_last_checked_unix_ms(now_ms);
        status.set_round_trip_ms(std::max<long long>(0, latency_ms));
        status.set_message(std::string(message));
        if (ok) {
            status.set_state(swarmkit::v1::DATA_PEER_STATE_READY);
            status.set_last_success_unix_ms(now_ms);
        } else {
            status.set_state(swarmkit::v1::DATA_PEER_STATE_UNREACHABLE);
            status.set_last_failure_unix_ms(now_ms);
        }
        std::lock_guard<std::mutex> lock(data_mutex_);
        peer_statuses_[peer.drone_id] = std::move(status);
    }

    void PublishArtifactReceivedEvent(const swarmkit::v1::ArtifactDescriptor& descriptor,
                                      std::string_view origin = "artifact-transfer") {
        swarmkit::v1::DataMessage event;
        event.set_message_id(MakeCorrelationId("artifact-received"));
        event.set_source_id(config_.agent_id);
        event.set_topic("swarmkit.artifact.received");
        event.set_unix_time_ms(NowUnixMs());
        event.set_ttl_ms(descriptor.ttl_ms());
        auto& labels = *event.mutable_labels();
        labels["artifact_id"] = descriptor.artifact_id();
        labels["artifact_source_id"] = descriptor.source_id();
        labels["source_id"] = descriptor.source_id();
        labels["target_id"] = descriptor.target_id();
        labels["content_type"] = descriptor.content_type();
        labels["filename"] = descriptor.filename();
        labels["size_bytes"] = std::to_string(descriptor.size_bytes());
        labels["sha256_hex"] = descriptor.sha256_hex();
        labels["origin"] = std::string(origin);
        for (const auto& [key, value] : descriptor.labels()) {
            labels["artifact.label." + key] = value;
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            event.set_sequence(++data_sequence_);
            message_backlog_.push_back(event);
            while (static_cast<int>(message_backlog_.size()) > config_.data.message_backlog_size) {
                message_backlog_.pop_front();
            }
        }
        data_cv_.notify_all();
        counters_.IncrementDataMessagesPublished();
        core::Logger::InfoFmt(
            "event=artifact.received agent={} artifact_id={} artifact_source={} target={} "
            "content_type={} bytes={} origin={} result=published",
            config_.agent_id, descriptor.artifact_id(), descriptor.source_id(),
            descriptor.target_id(), descriptor.content_type(), descriptor.size_bytes(), origin);
    }

    struct ArtifactTransferOutcome {
        bool ok{false};
        swarmkit::v1::ErrorCode error_code{swarmkit::v1::ERROR_CODE_INTERNAL};
        std::string message;
        swarmkit::v1::ArtifactDescriptor descriptor;
    };

    void FillArtifactTransferReply(swarmkit::v1::ArtifactTransferReply* reply, bool ok,
                                   std::string_view message, std::string_view correlation_id,
                                   swarmkit::v1::ErrorCode error_code,
                                   const swarmkit::v1::ArtifactTransferStatus& status) const {
        if (reply == nullptr) {
            return;
        }
        reply->set_ok(ok);
        reply->set_message(std::string(message));
        reply->set_correlation_id(std::string(correlation_id));
        reply->set_error_code(error_code);
        reply->set_debug_message(std::string(message));
        if (!status.transfer_id().empty()) {
            *reply->mutable_transfer() = status;
        }
    }

    [[nodiscard]] std::optional<swarmkit::v1::ArtifactTransferStatus> FindArtifactTransfer(
        std::string_view transfer_id) const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto iter = artifact_transfers_.find(std::string(transfer_id));
        if (iter == artifact_transfers_.end()) {
            return std::nullopt;
        }
        return iter->second;
    }

    void UpdateArtifactTransfer(
        std::string_view transfer_id,
        const std::function<void(swarmkit::v1::ArtifactTransferStatus*)>& update) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto iter = artifact_transfers_.find(std::string(transfer_id));
        if (iter == artifact_transfers_.end()) {
            return;
        }
        update(&iter->second);
        iter->second.set_updated_unix_ms(NowUnixMs());
    }

    void CompleteArtifactTransfer(std::string_view transfer_id,
                                  swarmkit::v1::ArtifactTransferState state,
                                  swarmkit::v1::ErrorCode error_code, std::string_view message,
                                  const swarmkit::v1::ArtifactDescriptor& descriptor) {
        UpdateArtifactTransfer(transfer_id, [&](swarmkit::v1::ArtifactTransferStatus* status) {
            status->set_state(state);
            status->set_error_code(error_code);
            status->set_message(std::string(message));
            status->set_completed_unix_ms(NowUnixMs());
            if (!descriptor.artifact_id().empty()) {
                *status->mutable_artifact() = descriptor;
            }
        });
    }

    [[nodiscard]] swarmkit::v1::ArtifactDescriptor PrepareArtifactDescriptorForPath(
        const std::filesystem::path& source_path, swarmkit::v1::ArtifactDescriptor descriptor,
        std::int64_t file_size) const {
        if (descriptor.source_id().empty()) {
            descriptor.set_source_id(config_.agent_id);
        }
        if (descriptor.created_unix_ms() == 0) {
            descriptor.set_created_unix_ms(NowUnixMs());
        }
        if (descriptor.filename().empty()) {
            descriptor.set_filename(source_path.filename().string());
        }
        if (descriptor.content_type().empty()) {
            descriptor.set_content_type("application/octet-stream");
        }
        descriptor.set_size_bytes(file_size);
        return descriptor;
    }

    [[nodiscard]] ArtifactTransferOutcome StoreArtifactFileLocally(
        std::string_view transfer_id, const std::filesystem::path& source_path,
        swarmkit::v1::ArtifactDescriptor descriptor,
        const std::shared_ptr<std::atomic<bool>>& cancel_flag) {
        ArtifactTransferOutcome outcome;
        std::ifstream input(source_path, std::ios::binary);
        if (!input.is_open()) {
            outcome.message = "failed to open source_path=" + source_path.string();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT;
            return outcome;
        }
        std::filesystem::path tmp_dir = ArtifactStorageRoot(config_) / "tmp";
        std::error_code fs_error;
        std::filesystem::create_directories(tmp_dir, fs_error);
        if (fs_error) {
            outcome.message = "failed to create artifact temp dir path=" + tmp_dir.string() +
                              " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
            return outcome;
        }
        const std::filesystem::path tmp_path =
            tmp_dir / (SanitizedPathComponent(std::string(transfer_id)) + ".part");
        std::ofstream output(tmp_path, std::ios::binary | std::ios::trunc);
        if (!output.is_open()) {
            outcome.message = "failed to open artifact temp file path=" + tmp_path.string();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
            return outcome;
        }

        const std::size_t chunk_size =
            std::max<std::size_t>(1, static_cast<std::size_t>(config_.data.artifact_chunk_bytes));
        std::string buffer(chunk_size, '\0');
        std::int64_t bytes = 0;
        while (input.good()) {
            if (cancel_flag->load(std::memory_order_relaxed)) {
                output.close();
                std::filesystem::remove(tmp_path, fs_error);
                outcome.message = "artifact transfer cancelled";
                outcome.error_code = swarmkit::v1::ERROR_CODE_CANCELLED;
                return outcome;
            }
            input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize read = input.gcount();
            if (read <= 0) {
                break;
            }
            if (bytes + static_cast<std::int64_t>(read) > config_.data.max_artifact_bytes) {
                output.close();
                std::filesystem::remove(tmp_path, fs_error);
                outcome.message = "artifact exceeds max_artifact_bytes";
                outcome.error_code = swarmkit::v1::ERROR_CODE_REJECTED;
                return outcome;
            }
            if (!WriteBytes(output,
                            std::span<const char>{buffer.data(), static_cast<std::size_t>(read)})) {
                output.close();
                std::filesystem::remove(tmp_path, fs_error);
                outcome.message = "failed to write artifact temp file path=" + tmp_path.string();
                outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
                return outcome;
            }
            bytes += static_cast<std::int64_t>(read);
            UpdateArtifactTransfer(transfer_id, [bytes](swarmkit::v1::ArtifactTransferStatus* s) {
                s->set_bytes_transferred(bytes);
            });
        }
        output.flush();
        output.close();

        const std::string sha256_hex = core::internal::Sha256FileHex(tmp_path);
        if (sha256_hex.empty()) {
            std::filesystem::remove(tmp_path, fs_error);
            outcome.message = "failed to hash artifact";
            outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
            return outcome;
        }
        if (!descriptor.sha256_hex().empty() && descriptor.sha256_hex() != sha256_hex) {
            std::filesystem::remove(tmp_path, fs_error);
            outcome.message = "artifact sha256 mismatch";
            outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
            return outcome;
        }
        descriptor.set_sha256_hex(sha256_hex);
        if (descriptor.artifact_id().empty()) {
            descriptor.set_artifact_id(sha256_hex);
        }

        const std::filesystem::path final_dir = ArtifactStorageRoot(config_) /
                                                SanitizedPathComponent(descriptor.source_id()) /
                                                SanitizedPathComponent(descriptor.artifact_id());
        std::filesystem::create_directories(final_dir, fs_error);
        if (fs_error) {
            std::filesystem::remove(tmp_path, fs_error);
            outcome.message = "failed to create artifact final dir path=" + final_dir.string() +
                              " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
            return outcome;
        }
        const std::filesystem::path final_path =
            final_dir / SanitizedPathComponent(descriptor.filename());
        std::filesystem::rename(tmp_path, final_path, fs_error);
        if (fs_error) {
            std::error_code exists_error;
            if (std::filesystem::exists(final_path, exists_error) &&
                core::internal::Sha256FileHex(final_path) == sha256_hex) {
                std::filesystem::remove(tmp_path, exists_error);
                fs_error.clear();
            }
        }
        if (fs_error) {
            std::filesystem::remove(tmp_path, fs_error);
            outcome.message = "failed to finalize artifact tmp_path=" + tmp_path.string() +
                              " final_path=" + final_path.string() +
                              " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INTERNAL;
            return outcome;
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            artifacts_[descriptor.artifact_id()] =
                StoredArtifact{.descriptor = descriptor, .storage_path = final_path};
        }
        PublishArtifactReceivedEvent(descriptor);
        counters_.IncrementArtifactUploads(static_cast<std::uint64_t>(bytes));
        outcome.ok = true;
        outcome.error_code = swarmkit::v1::ERROR_CODE_NONE;
        outcome.message = "artifact stored";
        outcome.descriptor = descriptor;
        return outcome;
    }

    [[nodiscard]] ArtifactTransferOutcome RouteArtifactFileToPeer(
        std::string_view transfer_id, const std::filesystem::path& source_path,
        swarmkit::v1::ArtifactDescriptor descriptor, int chunk_bytes,
        const std::shared_ptr<std::atomic<bool>>& cancel_flag) {
        ArtifactTransferOutcome outcome;
        const std::optional<DataPeerConfig> peer = config_.data.FindPeer(descriptor.target_id());
        if (!peer.has_value()) {
            outcome.message =
                "route failed before upload: no data peer configured for target_id=" +
                descriptor.target_id();
            outcome.error_code = swarmkit::v1::ERROR_CODE_UNAVAILABLE;
            return outcome;
        }
        std::ifstream input(source_path, std::ios::binary);
        if (!input.is_open()) {
            outcome.message = "failed to open source_path=" + source_path.string();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT;
            return outcome;
        }

        const auto route_started = std::chrono::steady_clock::now();
        auto channel = MakePeerChannel(*peer, config_.security);
        auto stub = swarmkit::v1::DataService::NewStub(channel);
        grpc::ClientContext peer_ctx;
        peer_ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds{300});
        peer_ctx.AddMetadata(std::string(kCorrelationMetadataKey), std::string(transfer_id));
        swarmkit::v1::ArtifactDescriptor peer_reply;
        auto writer = stub->UploadArtifact(&peer_ctx, &peer_reply);

        const std::size_t chunk_size = std::max<std::size_t>(
            1, chunk_bytes > 0 ? static_cast<std::size_t>(chunk_bytes)
                               : static_cast<std::size_t>(config_.data.artifact_chunk_bytes));
        std::string buffer(chunk_size, '\0');
        std::int64_t offset = 0;
        int chunk_index = 0;
        bool wrote_all = true;
        while (input.good()) {
            if (cancel_flag->load(std::memory_order_relaxed)) {
                peer_ctx.TryCancel();
                writer->WritesDone();
                static_cast<void>(writer->Finish());
                outcome.message = "artifact transfer cancelled";
                outcome.error_code = swarmkit::v1::ERROR_CODE_CANCELLED;
                return outcome;
            }
            input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize read = input.gcount();
            if (read <= 0) {
                break;
            }
            swarmkit::v1::ArtifactChunk chunk;
            chunk.set_transfer_id(std::string(transfer_id));
            if (chunk_index == 0) {
                *chunk.mutable_artifact() = descriptor;
            }
            chunk.set_offset(offset);
            chunk.set_chunk_index(chunk_index);
            chunk.set_data(buffer.data(), static_cast<std::size_t>(read));
            offset += static_cast<std::int64_t>(read);
            chunk.set_final_chunk(offset >= descriptor.size_bytes());
            wrote_all = writer->Write(chunk);
            if (!wrote_all) {
                break;
            }
            UpdateArtifactTransfer(transfer_id, [offset](swarmkit::v1::ArtifactTransferStatus* s) {
                s->set_bytes_transferred(offset);
            });
            ++chunk_index;
        }
        writer->WritesDone();
        const grpc::Status peer_status = writer->Finish();
        const auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - route_started)
                                    .count();
        if (!wrote_all && peer_status.ok()) {
            MarkPeerRouteResult(*peer, false, latency_ms, "peer stream closed");
            outcome.message = "peer artifact stream closed before all chunks were accepted";
            outcome.error_code = swarmkit::v1::ERROR_CODE_UNAVAILABLE;
            return outcome;
        }
        if (!peer_status.ok()) {
            MarkPeerRouteResult(*peer, false, latency_ms, peer_status.error_message());
            outcome.message = "peer artifact delivery failed: " + peer_status.error_message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_UNAVAILABLE;
            return outcome;
        }
        MarkPeerRouteResult(*peer, true, latency_ms, "artifact delivered");
        counters_.IncrementArtifactDownloads(static_cast<std::uint64_t>(offset));
        outcome.ok = true;
        outcome.error_code = swarmkit::v1::ERROR_CODE_NONE;
        outcome.message = "artifact delivered";
        outcome.descriptor = peer_reply;
        return outcome;
    }

    void RunArtifactTransfer(std::string transfer_id,
                             swarmkit::v1::ArtifactTransferStartRequest request,
                             std::shared_ptr<std::atomic<bool>> cancel_flag) {
        UpdateArtifactTransfer(transfer_id, [](swarmkit::v1::ArtifactTransferStatus* status) {
            status->set_state(swarmkit::v1::ARTIFACT_TRANSFER_STATE_RUNNING);
            status->set_message("running");
        });

        std::error_code fs_error;
        const auto file_size = std::filesystem::file_size(request.source_path(), fs_error);
        swarmkit::v1::ArtifactDescriptor descriptor =
            PrepareArtifactDescriptorForPath(request.source_path(), request.artifact(),
                                             fs_error ? 0 : static_cast<std::int64_t>(file_size));
        UpdateArtifactTransfer(transfer_id, [&descriptor](swarmkit::v1::ArtifactTransferStatus* s) {
            *s->mutable_artifact() = descriptor;
            s->set_bytes_total(descriptor.size_bytes());
        });
        ArtifactTransferOutcome outcome;
        if (fs_error) {
            outcome.message = "source_path cannot be read path=" + request.source_path() +
                              " error=" + fs_error.message();
            outcome.error_code = swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT;
        } else if (request.route_to_target()) {
            outcome = RouteArtifactFileToPeer(transfer_id, request.source_path(), descriptor,
                                              request.chunk_bytes(), cancel_flag);
        } else {
            outcome = StoreArtifactFileLocally(transfer_id, request.source_path(), descriptor,
                                               cancel_flag);
        }

        const bool cancelled = outcome.error_code == swarmkit::v1::ERROR_CODE_CANCELLED;
        CompleteArtifactTransfer(
            transfer_id,
            outcome.ok ? swarmkit::v1::ARTIFACT_TRANSFER_STATE_COMPLETED
                       : (cancelled ? swarmkit::v1::ARTIFACT_TRANSFER_STATE_CANCELLED
                                    : swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED),
            outcome.error_code, outcome.message, outcome.descriptor);
        if (outcome.ok) {
            core::Logger::InfoFmt(
                "artifact_transfer agent={} transfer_id={} artifact_id={} bytes={} result={}",
                config_.agent_id, transfer_id, outcome.descriptor.artifact_id(),
                outcome.descriptor.size_bytes(),
                request.route_to_target() ? "delivered" : "stored");
        } else {
            core::Logger::WarnFmt(
                "artifact_transfer agent={} transfer_id={} source_path={} target={} result={} "
                "error={}",
                config_.agent_id, transfer_id, request.source_path(), request.artifact().target_id(),
                cancelled ? "cancelled" : "failed", outcome.message);
        }
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
    internal::RuntimeCounters counters_;
    std::atomic<bool> ready_{true};
    std::string startup_error_;
    mutable std::mutex data_mutex_;
    std::condition_variable data_cv_;
    std::deque<swarmkit::v1::DataMessage> message_backlog_;
    std::unordered_map<std::string, StoredArtifact> artifacts_;
    std::unordered_map<std::string, swarmkit::v1::DataPeerStatus> peer_statuses_;
    std::unordered_map<std::string, swarmkit::v1::ArtifactTransferStatus> artifact_transfers_;
    std::unordered_map<std::string, std::shared_ptr<std::atomic<bool>>> artifact_transfer_cancel_;
    std::mutex artifact_transfer_threads_mutex_;
    std::vector<std::thread> artifact_transfer_threads_;
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
