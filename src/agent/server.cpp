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
#include <exception>
#include <expected>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <stop_token>
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

/// @brief Watcher poll interval while blocking inside WatchAuthority RPC.
constexpr auto kWatchPollInterval = std::chrono::milliseconds{100};
constexpr auto kTelemetryWaitTimeout = std::chrono::milliseconds{200};
constexpr std::string_view kArtifactMetadataFilename = "artifact.pb";

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

[[nodiscard]] swarmkit::v1::ErrorCode ToProtoErrorCode(core::ErrorCode code) {
    using ProtoCode = swarmkit::v1::ErrorCode;
    switch (code) {
        case core::ErrorCode::kOk:
            return ProtoCode::ERROR_CODE_NONE;
        case core::ErrorCode::kInvalidArgument:
            return ProtoCode::ERROR_CODE_INVALID_ARGUMENT;
        case core::ErrorCode::kRejected:
            return ProtoCode::ERROR_CODE_REJECTED;
        case core::ErrorCode::kPermissionDenied:
            return ProtoCode::ERROR_CODE_PERMISSION_DENIED;
        case core::ErrorCode::kNotFound:
            return ProtoCode::ERROR_CODE_NOT_FOUND;
        case core::ErrorCode::kAlreadyExists:
            return ProtoCode::ERROR_CODE_ALREADY_EXISTS;
        case core::ErrorCode::kFailedPrecondition:
            return ProtoCode::ERROR_CODE_FAILED_PRECONDITION;
        case core::ErrorCode::kUnsupported:
            return ProtoCode::ERROR_CODE_UNSUPPORTED;
        case core::ErrorCode::kUnavailable:
            return ProtoCode::ERROR_CODE_UNAVAILABLE;
        case core::ErrorCode::kDeadlineExceeded:
            return ProtoCode::ERROR_CODE_DEADLINE_EXCEEDED;
        case core::ErrorCode::kCancelled:
            return ProtoCode::ERROR_CODE_CANCELLED;
        case core::ErrorCode::kBackendFailure:
            return ProtoCode::ERROR_CODE_BACKEND_FAILURE;
        case core::ErrorCode::kInternal:
            return ProtoCode::ERROR_CODE_INTERNAL;
        case core::ErrorCode::kUnknown:
            return ProtoCode::ERROR_CODE_UNKNOWN;
    }
    return ProtoCode::ERROR_CODE_UNKNOWN;
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
class AgentServiceImpl final : public swarmkit::v1::AgentService::Service {
   public:
    AgentServiceImpl(AgentConfig config, DroneBackendPtr backend,
                     std::shared_ptr<internal::RuntimeCounters> counters)
        : config_(std::move(config)),
          backend_(std::move(backend)),
          counters_(std::move(counters)),
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

    ~AgentServiceImpl() override = default;

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

        counters_->IncrementPingRequests();
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

        counters_->IncrementHealthRequests();
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

        counters_->IncrementRuntimeStatsRequests();

        // Sync telemetry counters from the manager into the atomic counters.
        counters_->SetTelemetryCounters(telemetry_.TotalSubscriptionCount(),
                                       telemetry_.ActiveStreamCount(), telemetry_.FramesSentTotal(),
                                       telemetry_.BackendFailureCount());

        const internal::CounterSnapshot kSnap = counters_->Snapshot();
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

        counters_->IncrementCommandRequests();

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
                counters_->IncrementCommandFailed();
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
            counters_->IncrementCommandFailed();
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
            reply->set_error_code(ToProtoErrorCode(kPrecondition.result.error.code));
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
                counters_->IncrementCommandRejected();
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
            counters_->IncrementCommandRejected();
            core::Logger::WarnFmt(
                "rpc=SendCommand corr={} agent={} drone={} client={} priority={} peer={} "
                "result=rejected reason={}",
                envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
                envelope.context.client_id, static_cast<int>(envelope.context.priority),
                ctx->peer(), kGrant.result.message);
            reply->set_status(ToProtoStatus(kGrant.result.code));
            reply->set_message(kGrant.result.message);
            reply->set_correlation_id(envelope.context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(kGrant.result.error.code));
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
        reply->set_error_code(ToProtoErrorCode(kExecResult.error.code));
        if (kExecResult.IsOk()) {
            reply->set_status(swarmkit::v1::CommandReply::OK);
            reply->set_message(kExecResult.message);
            PublishCommandReport(envelope.context, kCmdName, swarmkit::v1::COMMAND_ACKED,
                                 swarmkit::v1::REPORT_INFO,
                                 kExecResult.message.empty() ? "command executed"
                                                             : kExecResult.message);
        } else {
            counters_->IncrementCommandFailed();
            counters_->IncrementBackendFailures();
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
            reply->set_error_code(swarmkit::v1::ERROR_CODE_UNSUPPORTED);
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
            reply->set_error_code(ToProtoErrorCode(arbiter_result.error.code));
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
            reply->set_error_code(ToProtoErrorCode(readiness.error.code));
            reply->set_debug_message(readiness.message);
            *reply->mutable_goal() = goal;
            PublishGoalFailure(goal, context.correlation_id, readiness.message);
            return grpc::Status::OK;
        }

        const core::Result exec_result = backend_->Execute(envelope);
        if (!exec_result.IsOk()) {
            arbiter_.Release(context.drone_id, context.client_id);
            counters_->IncrementBackendFailures();
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
            reply->set_error_code(ToProtoErrorCode(arbiter_result.error.code));
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
        reply->set_error_code(ToProtoErrorCode(result.error.code));
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

        counters_->IncrementLockRequests();

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
        reply->set_error_code(ToProtoErrorCode(kResult.error.code));
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

        counters_->IncrementWatchRequests();
        counters_->IncrementAuthorityWatchers();
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
        counters_->DecrementAuthorityWatchers();

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

   private:
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
    std::shared_ptr<internal::RuntimeCounters> counters_;
    internal::TelemetryManager telemetry_;
    internal::ReportHub reports_;
    internal::ActiveGoalSupervisor goals_;
    std::atomic<bool> ready_{true};
    std::string startup_error_;
};

#include "data_service.inc"

/// @}

}  // namespace

// ---------------------------------------------------------------------------
// AgentConfig
// ---------------------------------------------------------------------------

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

    auto services = internal::MakeAgentServicesForTesting(config, std::move(backend));
    builder.RegisterService(services->agent_service.get());
    builder.RegisterService(services->data_service.get());

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
    auto counters = std::make_shared<internal::RuntimeCounters>();
    return std::make_unique<AgentServiceImpl>(config, std::move(backend), std::move(counters));
}

std::unique_ptr<internal::AgentServiceBundle> internal::MakeAgentServicesForTesting(
    const AgentConfig& config, DroneBackendPtr backend) {
    auto counters = std::make_shared<internal::RuntimeCounters>();
    auto bundle = std::make_unique<internal::AgentServiceBundle>();
    bundle->agent_service =
        std::make_unique<AgentServiceImpl>(config, std::move(backend), counters);
    bundle->data_service = std::make_unique<DataServiceImpl>(config, std::move(counters));
    return bundle;
}

}  // namespace swarmkit::agent
