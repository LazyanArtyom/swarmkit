// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/client/client.h"

#include <grpcpp/grpcpp.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <expected>
#include <functional>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>

#include "env_utils.h"
#include "security_utils.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/overloaded.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::client {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)
using core::internal::GetEnvValue;
using core::internal::IsValidPriority;
using core::internal::LooksLikeAddress;
using core::internal::MakeCorrelationId;
using core::internal::ParseBoolValue;
using core::internal::ParseIntValue;
using core::internal::ParsePriorityValue;

namespace {

constexpr std::string_view kClientEnvAddress = "ADDRESS";
constexpr std::string_view kClientEnvId = "CLIENT_ID";
constexpr std::string_view kClientEnvDeadlineMs = "DEADLINE_MS";
constexpr std::string_view kClientEnvPriority = "PRIORITY";
constexpr std::string_view kClientEnvRetryMaxAttempts = "RETRY_MAX_ATTEMPTS";
constexpr std::string_view kClientEnvRetryInitialBackoffMs = "RETRY_INITIAL_BACKOFF_MS";
constexpr std::string_view kClientEnvRetryMaxBackoffMs = "RETRY_MAX_BACKOFF_MS";
constexpr std::string_view kClientEnvStreamReconnectEnabled = "STREAM_RECONNECT_ENABLED";
constexpr std::string_view kClientEnvStreamReconnectInitialBackoffMs =
    "STREAM_RECONNECT_INITIAL_BACKOFF_MS";
constexpr std::string_view kClientEnvStreamReconnectMaxBackoffMs =
    "STREAM_RECONNECT_MAX_BACKOFF_MS";
constexpr std::string_view kClientEnvStreamReconnectMaxAttempts = "STREAM_RECONNECT_MAX_ATTEMPTS";
constexpr std::string_view kClientEnvRootCaCertPath = "ROOT_CA_CERT_PATH";
constexpr std::string_view kClientEnvClientCertChainPath = "CLIENT_CERT_CHAIN_PATH";
constexpr std::string_view kClientEnvClientPrivateKeyPath = "CLIENT_PRIVATE_KEY_PATH";
constexpr std::string_view kClientEnvServerAuthorityOverride = "SERVER_AUTHORITY_OVERRIDE";
constexpr std::string_view kClientEnvTransportSecurity = "TRANSPORT_SECURITY";
constexpr std::string_view kCorrelationMetadataKey = "x-correlation-id";
constexpr double kEarthRadiusM = 6371000.0;

[[nodiscard]] RpcStatusCode ToRpcStatusCode(const grpc::Status& status) {
    if (status.ok()) {
        return RpcStatusCode::kOk;
    }

    switch (status.error_code()) {
        case grpc::StatusCode::INVALID_ARGUMENT:
            return RpcStatusCode::kInvalidArgument;
        case grpc::StatusCode::PERMISSION_DENIED:
            return RpcStatusCode::kPermissionDenied;
        case grpc::StatusCode::NOT_FOUND:
            return RpcStatusCode::kNotFound;
        case grpc::StatusCode::ALREADY_EXISTS:
            return RpcStatusCode::kAlreadyExists;
        case grpc::StatusCode::FAILED_PRECONDITION:
            return RpcStatusCode::kFailedPrecondition;
        case grpc::StatusCode::UNIMPLEMENTED:
            return RpcStatusCode::kUnsupported;
        case grpc::StatusCode::UNAVAILABLE:
            return RpcStatusCode::kUnavailable;
        case grpc::StatusCode::DEADLINE_EXCEEDED:
            return RpcStatusCode::kDeadlineExceeded;
        case grpc::StatusCode::CANCELLED:
            return RpcStatusCode::kCancelled;
        case grpc::StatusCode::INTERNAL:
            return RpcStatusCode::kInternal;
        default:
            return RpcStatusCode::kUnknown;
    }
}

[[nodiscard]] core::CoordinateFrame ToCoreCoordinateFrame(
    swarmkit::v1::TelemetryCoordinateFrame frame) {
    switch (frame) {
        case swarmkit::v1::TELEMETRY_COORDINATE_FRAME_WGS84:
            return core::CoordinateFrame::kWgs84;
        case swarmkit::v1::TELEMETRY_COORDINATE_FRAME_LOCAL_NED:
            return core::CoordinateFrame::kLocalNed;
        case swarmkit::v1::TELEMETRY_COORDINATE_FRAME_BODY_NED:
            return core::CoordinateFrame::kBodyNed;
        case swarmkit::v1::TELEMETRY_COORDINATE_FRAME_UNSPECIFIED:
        default:
            return core::CoordinateFrame::kUnknown;
    }
}

[[nodiscard]] core::GpsQuality ToCoreGpsQuality(swarmkit::v1::TelemetryGpsQuality quality) {
    switch (quality) {
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_NO_FIX:
            return core::GpsQuality::kNoFix;
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_2D_FIX:
            return core::GpsQuality::kFix2D;
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_3D_FIX:
            return core::GpsQuality::kFix3D;
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_DGPS:
            return core::GpsQuality::kDgps;
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_RTK_FLOAT:
            return core::GpsQuality::kRtkFloat;
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_RTK_FIXED:
            return core::GpsQuality::kRtkFixed;
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_STATIC:
            return core::GpsQuality::kStatic;
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_PPP:
            return core::GpsQuality::kPpp;
        case swarmkit::v1::TELEMETRY_GPS_QUALITY_UNSPECIFIED:
        default:
            return core::GpsQuality::kUnknown;
    }
}

[[nodiscard]] core::EstimatorState ToCoreEstimatorState(
    swarmkit::v1::TelemetryEstimatorState state) {
    switch (state) {
        case swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_INITIALIZING:
            return core::EstimatorState::kInitializing;
        case swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_HEALTHY:
            return core::EstimatorState::kHealthy;
        case swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_DEGRADED:
            return core::EstimatorState::kDegraded;
        case swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_FAULT:
            return core::EstimatorState::kFault;
        case swarmkit::v1::TELEMETRY_ESTIMATOR_STATE_UNSPECIFIED:
        default:
            return core::EstimatorState::kUnknown;
    }
}

[[nodiscard]] std::shared_ptr<grpc::ChannelCredentials> MakeChannelCredentials(
    const ClientSecurityConfig& security) {
    const core::TransportSecurityMode mode = security.EffectiveTransportSecurity();
    if (mode == core::TransportSecurityMode::kInsecure) {
        return grpc::InsecureChannelCredentials();
    }

    grpc::SslCredentialsOptions options;
    static_cast<void>(
        core::internal::ReadTextFile(security.root_ca_cert_path, &options.pem_root_certs));
    if (mode == core::TransportSecurityMode::kMutualTls) {
        static_cast<void>(
            core::internal::ReadTextFile(security.private_key_path, &options.pem_private_key));
        static_cast<void>(
            core::internal::ReadTextFile(security.cert_chain_path, &options.pem_cert_chain));
    }
    return grpc::SslCredentials(options);
}

[[nodiscard]] std::shared_ptr<grpc::Channel> MakeChannel(const ClientConfig& config) {
    const auto kCredentials = MakeChannelCredentials(config.security);
    if (config.security.server_authority_override.empty() ||
        config.security.EffectiveTransportSecurity() == core::TransportSecurityMode::kInsecure) {
        return grpc::CreateChannel(config.address, kCredentials);
    }

    grpc::ChannelArguments arguments;
    arguments.SetSslTargetNameOverride(config.security.server_authority_override);
    return grpc::CreateCustomChannel(config.address, kCredentials, arguments);
}

[[nodiscard]] RpcStatusCode ToRpcStatusCode(swarmkit::v1::ErrorCode code) {
    using ProtoCode = swarmkit::v1::ErrorCode;
    switch (code) {
        case ProtoCode::ErrorCode_INT_MIN_SENTINEL_DO_NOT_USE_:
        case ProtoCode::ErrorCode_INT_MAX_SENTINEL_DO_NOT_USE_:
            return RpcStatusCode::kUnknown;
        case ProtoCode::ERROR_CODE_NONE:
            return RpcStatusCode::kOk;
        case ProtoCode::ERROR_CODE_INVALID_ARGUMENT:
            return RpcStatusCode::kInvalidArgument;
        case ProtoCode::ERROR_CODE_REJECTED:
            return RpcStatusCode::kRejected;
        case ProtoCode::ERROR_CODE_UNAVAILABLE:
            return RpcStatusCode::kUnavailable;
        case ProtoCode::ERROR_CODE_DEADLINE_EXCEEDED:
            return RpcStatusCode::kDeadlineExceeded;
        case ProtoCode::ERROR_CODE_CANCELLED:
            return RpcStatusCode::kCancelled;
        case ProtoCode::ERROR_CODE_INTERNAL:
            return RpcStatusCode::kInternal;
        case ProtoCode::ERROR_CODE_BACKEND_FAILURE:
            return RpcStatusCode::kBackendFailure;
        case ProtoCode::ERROR_CODE_UNSPECIFIED:
            return RpcStatusCode::kUnknown;
    }
    return RpcStatusCode::kUnknown;
}

[[nodiscard]] bool ShouldRetryUnary(const grpc::Status& status, const RetryPolicy& policy,
                                    int attempt_number) {
    if (attempt_number >= policy.max_attempts) {
        return false;
    }

    switch (status.error_code()) {
        case grpc::StatusCode::UNAVAILABLE:
        case grpc::StatusCode::DEADLINE_EXCEEDED:
        case grpc::StatusCode::CANCELLED:
            return true;
        default:
            return false;
    }
}

void ApplyUnaryClientContext(const ClientConfig& config, grpc::ClientContext* context,
                             std::string_view correlation_id) {
    if (context == nullptr) {
        return;
    }

    if (config.deadline_ms > 0) {
        context->set_deadline(std::chrono::system_clock::now() +
                              std::chrono::milliseconds(config.deadline_ms));
    }
    context->AddMetadata(std::string(kCorrelationMetadataKey), std::string(correlation_id));
}

[[nodiscard]] swarmkit::v1::MissionItemType ToProtoMissionItemType(commands::MissionItemType type) {
    using commands::MissionItemType;
    switch (type) {
        case MissionItemType::kWaypoint:
            return swarmkit::v1::MISSION_ITEM_WAYPOINT;
        case MissionItemType::kTakeoff:
            return swarmkit::v1::MISSION_ITEM_TAKEOFF;
        case MissionItemType::kLand:
            return swarmkit::v1::MISSION_ITEM_LAND;
        case MissionItemType::kLoiter:
            return swarmkit::v1::MISSION_ITEM_LOITER;
        case MissionItemType::kDelay:
            return swarmkit::v1::MISSION_ITEM_DELAY;
        case MissionItemType::kAction:
            return swarmkit::v1::MISSION_ITEM_ACTION;
        case MissionItemType::kPayloadAction:
            return swarmkit::v1::MISSION_ITEM_PAYLOAD_ACTION;
    }
    return swarmkit::v1::MISSION_ITEM_WAYPOINT;
}

template <typename Call>
grpc::Status InvokeUnaryWithRetry(const ClientConfig& config, std::string_view correlation_id,
                                  int* attempts_used, Call&& call) {
    int attempt_number = 0;
    int backoff_ms = std::max(1, config.retry_policy.initial_backoff_ms);

    while (true) {
        ++attempt_number;

        grpc::ClientContext context;
        ApplyUnaryClientContext(config, &context, correlation_id);
        const grpc::Status kStatus = call(&context);
        if (!ShouldRetryUnary(kStatus, config.retry_policy, attempt_number)) {
            if (attempts_used != nullptr) {
                *attempts_used = attempt_number;
            }
            return kStatus;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
        backoff_ms = std::min(config.retry_policy.max_backoff_ms, backoff_ms * 2);
    }
}

[[nodiscard]] core::ErrorSeverity SeverityForCode(RpcStatusCode code);
[[nodiscard]] core::ErrorRetryability RetryabilityForCode(RpcStatusCode code);
[[nodiscard]] std::string RemediationForCode(RpcStatusCode code);

void PopulateTransportError(RpcError* error, const grpc::Status& status,
                            std::string_view correlation_id, int attempt_count) {
    if (error == nullptr) {
        return;
    }

    const RpcStatusCode code = ToRpcStatusCode(status);
    *error = core::SwarmError::Make(core::ErrorDomain::kTransport, code, status.error_message(),
                                    SeverityForCode(code), RetryabilityForCode(code),
                                    RemediationForCode(code));
    error->debug_message = status.error_details();
    error->correlation_id = std::string(correlation_id);
    error->attempt_count = attempt_count;
    error->details["grpc_status_code"] = std::to_string(static_cast<int>(status.error_code()));
}

[[nodiscard]] core::ErrorSeverity SeverityForCode(RpcStatusCode code) {
    switch (code) {
        case RpcStatusCode::kOk:
            return core::ErrorSeverity::kInfo;
        case RpcStatusCode::kInvalidArgument:
        case RpcStatusCode::kRejected:
        case RpcStatusCode::kPermissionDenied:
        case RpcStatusCode::kNotFound:
        case RpcStatusCode::kAlreadyExists:
        case RpcStatusCode::kFailedPrecondition:
        case RpcStatusCode::kUnsupported:
            return core::ErrorSeverity::kWarning;
        case RpcStatusCode::kUnavailable:
        case RpcStatusCode::kDeadlineExceeded:
        case RpcStatusCode::kCancelled:
        case RpcStatusCode::kBackendFailure:
            return core::ErrorSeverity::kError;
        case RpcStatusCode::kInternal:
        case RpcStatusCode::kUnknown:
            return core::ErrorSeverity::kCritical;
    }
    return core::ErrorSeverity::kError;
}

[[nodiscard]] core::ErrorRetryability RetryabilityForCode(RpcStatusCode code) {
    switch (code) {
        case RpcStatusCode::kUnavailable:
        case RpcStatusCode::kDeadlineExceeded:
        case RpcStatusCode::kCancelled:
            return core::ErrorRetryability::kAfterBackoff;
        case RpcStatusCode::kInvalidArgument:
        case RpcStatusCode::kRejected:
        case RpcStatusCode::kPermissionDenied:
        case RpcStatusCode::kNotFound:
        case RpcStatusCode::kAlreadyExists:
        case RpcStatusCode::kFailedPrecondition:
        case RpcStatusCode::kUnsupported:
            return core::ErrorRetryability::kAfterRemediation;
        case RpcStatusCode::kInternal:
        case RpcStatusCode::kBackendFailure:
        case RpcStatusCode::kUnknown:
            return core::ErrorRetryability::kUnknown;
        case RpcStatusCode::kOk:
            return core::ErrorRetryability::kNever;
    }
    return core::ErrorRetryability::kUnknown;
}

[[nodiscard]] std::string RemediationForCode(RpcStatusCode code) {
    switch (code) {
        case RpcStatusCode::kInvalidArgument:
            return "Fix the request parameters before retrying";
        case RpcStatusCode::kRejected:
        case RpcStatusCode::kFailedPrecondition:
            return "Check authority, vehicle state, and command preconditions";
        case RpcStatusCode::kPermissionDenied:
            return "Check client identity, certificates, and allowed_client_ids";
        case RpcStatusCode::kNotFound:
            return "Verify the drone, goal, or execution identifier";
        case RpcStatusCode::kAlreadyExists:
            return "Use a unique identifier or update the existing resource";
        case RpcStatusCode::kUnsupported:
            return "Check backend capabilities before sending this operation";
        case RpcStatusCode::kUnavailable:
            return "Check agent/backend availability and retry with backoff";
        case RpcStatusCode::kDeadlineExceeded:
            return "Increase the deadline or retry when the vehicle is responsive";
        case RpcStatusCode::kCancelled:
            return "Retry if the operation was not intentionally cancelled";
        case RpcStatusCode::kBackendFailure:
            return "Inspect backend/autopilot diagnostics before retrying";
        case RpcStatusCode::kInternal:
        case RpcStatusCode::kUnknown:
            return "Inspect logs with the correlation ID and report the failure";
        case RpcStatusCode::kOk:
            return {};
    }
    return {};
}

[[nodiscard]] core::ErrorDomain DomainForReply(RpcStatusCode code,
                                               core::ErrorDomain requested_domain) {
    if (code == RpcStatusCode::kOk) {
        return core::ErrorDomain::kNone;
    }
    if (code == RpcStatusCode::kBackendFailure) {
        return core::ErrorDomain::kBackend;
    }
    if (code == RpcStatusCode::kInvalidArgument) {
        return core::ErrorDomain::kValidation;
    }
    return requested_domain;
}

void PopulateSuccessError(RpcError* error, std::string_view correlation_id, int attempt_count) {
    if (error == nullptr) {
        return;
    }
    *error = core::SwarmError::Ok();
    error->correlation_id = std::string(correlation_id);
    error->attempt_count = attempt_count;
}

void PopulateTypedError(RpcError* error, core::ErrorDomain domain, RpcStatusCode code,
                        std::string user_message, std::string debug_message,
                        std::string_view correlation_id, int attempt_count) {
    if (error == nullptr) {
        return;
    }
    if (code == RpcStatusCode::kOk) {
        PopulateSuccessError(error, correlation_id, attempt_count);
        return;
    }

    *error = core::SwarmError::Make(domain, code, std::move(user_message), SeverityForCode(code),
                                    RetryabilityForCode(code), RemediationForCode(code));
    error->debug_message = std::move(debug_message);
    error->correlation_id = std::string(correlation_id);
    error->attempt_count = attempt_count;
}

void PopulateReplyError(RpcError* error, swarmkit::v1::ErrorCode proto_code,
                        std::string user_message, std::string debug_message,
                        std::string_view correlation_id, int attempt_count,
                        core::ErrorDomain domain = core::ErrorDomain::kCommand) {
    const RpcStatusCode code = ToRpcStatusCode(proto_code);
    PopulateTypedError(error, DomainForReply(code, domain), code, std::move(user_message),
                       std::move(debug_message), correlation_id, attempt_count);
}

[[nodiscard]] AuthorityEventKind ToAuthorityEventKind(swarmkit::v1::AuthorityEvent::Kind kind) {
    using ProtoKind = swarmkit::v1::AuthorityEvent::Kind;
    switch (kind) {
        case ProtoKind::AuthorityEvent_Kind_AuthorityEvent_Kind_INT_MIN_SENTINEL_DO_NOT_USE_:
        case ProtoKind::AuthorityEvent_Kind_AuthorityEvent_Kind_INT_MAX_SENTINEL_DO_NOT_USE_:
            return AuthorityEventKind::kExpired;
        case ProtoKind::AuthorityEvent_Kind_GRANTED:
            return AuthorityEventKind::kGranted;
        case ProtoKind::AuthorityEvent_Kind_PREEMPTED:
            return AuthorityEventKind::kPreempted;
        case ProtoKind::AuthorityEvent_Kind_RESUMED:
            return AuthorityEventKind::kResumed;
        case ProtoKind::AuthorityEvent_Kind_EXPIRED:
        case ProtoKind::AuthorityEvent_Kind_KIND_UNSPECIFIED:
            return AuthorityEventKind::kExpired;
    }
    return AuthorityEventKind::kExpired;
}

/// @brief Serialise a CommandEnvelope into a CommandRequest proto.
void BuildProtoCommand(const commands::CommandEnvelope& envelope,
                       swarmkit::v1::CommandRequest& req) {
    auto* proto_ctx = req.mutable_ctx();
    proto_ctx->set_drone_id(envelope.context.drone_id);
    proto_ctx->set_client_id(envelope.context.client_id);
    proto_ctx->set_priority(static_cast<std::int32_t>(envelope.context.priority));
    proto_ctx->set_correlation_id(envelope.context.correlation_id);

    const auto kEpoch = std::chrono::system_clock::time_point{};
    if (envelope.context.deadline != kEpoch) {
        proto_ctx->set_deadline_unix_ms(std::chrono::duration_cast<std::chrono::milliseconds>(
                                            envelope.context.deadline.time_since_epoch())
                                            .count());
    }

    auto* proto_cmd = req.mutable_cmd();
    std::visit(
        core::Overloaded{

            [&](const commands::FlightCmd& flight) {
                std::visit(
                    core::Overloaded{
                        [&](const commands::CmdArm&) { proto_cmd->mutable_arm(); },
                        [&](const commands::CmdForceArm&) { proto_cmd->mutable_force_arm(); },
                        [&](const commands::CmdDisarm&) { proto_cmd->mutable_disarm(); },
                        [&](const commands::CmdTakeoff& takeoff) {
                            proto_cmd->mutable_takeoff()->set_alt_m(takeoff.alt_m);
                        },
                        [&](const commands::CmdLand&) { proto_cmd->mutable_land(); },
                        [&](const commands::CmdSetMode& mode) {
                            auto* proto = proto_cmd->mutable_set_mode();
                            proto->set_mode(mode.mode);
                            proto->set_custom_mode(mode.custom_mode);
                        },
                        [&](const commands::CmdForceDisarm&) { proto_cmd->mutable_force_disarm(); },
                        [&](const commands::CmdFlightTerminate&) {
                            proto_cmd->mutable_flight_terminate();
                        },
                    },
                    flight);
            },

            [&](const commands::NavCmd& nav) {
                std::visit(
                    core::Overloaded{
                        [&](const commands::CmdSetWaypoint& waypoint) {
                            auto* proto_wp = proto_cmd->mutable_set_waypoint();
                            proto_wp->set_lat_deg(waypoint.lat_deg);
                            proto_wp->set_lon_deg(waypoint.lon_deg);
                            proto_wp->set_alt_m(waypoint.alt_m);
                            proto_wp->set_speed_mps(waypoint.speed_mps);
                        },
                        [&](const commands::CmdReturnHome&) { proto_cmd->mutable_return_home(); },
                        [&](const commands::CmdHoldPosition&) {
                            proto_cmd->mutable_hold_position();
                        },
                        [&](const commands::CmdSetSpeed& speed) {
                            proto_cmd->mutable_set_speed()->set_ground_mps(speed.ground_mps);
                        },
                        [&](const commands::CmdGoto& go_to) {
                            auto* proto = proto_cmd->mutable_goto_position();
                            proto->set_lat_deg(go_to.lat_deg);
                            proto->set_lon_deg(go_to.lon_deg);
                            proto->set_alt_m(go_to.alt_m);
                            proto->set_speed_mps(go_to.speed_mps);
                            proto->set_yaw_deg(go_to.yaw_deg);
                            proto->set_use_yaw(go_to.use_yaw);
                        },
                        [&](const commands::CmdPause&) { proto_cmd->mutable_pause(); },
                        [&](const commands::CmdResume&) { proto_cmd->mutable_resume(); },
                        [&](const commands::CmdSetYaw& yaw) {
                            auto* proto = proto_cmd->mutable_set_yaw();
                            proto->set_yaw_deg(yaw.yaw_deg);
                            proto->set_rate_deg_s(yaw.rate_deg_s);
                            proto->set_relative(yaw.relative);
                        },
                        [&](const commands::CmdVelocity& velocity) {
                            auto* proto = proto_cmd->mutable_velocity();
                            proto->set_vx_mps(velocity.vx_mps);
                            proto->set_vy_mps(velocity.vy_mps);
                            proto->set_vz_mps(velocity.vz_mps);
                            proto->set_duration_ms(velocity.duration_ms);
                            proto->set_body_frame(velocity.body_frame);
                        },
                        [&](const commands::CmdSetHome& home) {
                            auto* proto = proto_cmd->mutable_set_home();
                            proto->set_use_current(home.use_current);
                            proto->set_lat_deg(home.lat_deg);
                            proto->set_lon_deg(home.lon_deg);
                            proto->set_alt_m(home.alt_m);
                        },
                    },
                    nav);
            },

            [&](const commands::MissionCmd& mission) {
                std::visit(
                    core::Overloaded{
                        [&](const commands::CmdUploadMission& upload) {
                            auto* proto = proto_cmd->mutable_upload_mission();
                            for (const auto& item : upload.items) {
                                auto* proto_item = proto->add_items();
                                proto_item->set_command(item.backend_command);
                                proto_item->set_lat_deg(item.lat_deg);
                                proto_item->set_lon_deg(item.lon_deg);
                                proto_item->set_alt_m(item.alt_m);
                                proto_item->set_type(ToProtoMissionItemType(item.type));
                                proto_item->set_hold_s(item.hold_s);
                                proto_item->set_acceptance_radius_m(item.acceptance_radius_m);
                                proto_item->set_yaw_deg(item.yaw_deg);
                                proto_item->set_action_namespace(item.action_namespace);
                                proto_item->set_action_name(item.action_name);
                                for (const auto& [key, value] : item.params) {
                                    (*proto_item->mutable_params())[key] = value;
                                }
                                proto_item->set_param1(item.param1);
                                proto_item->set_param2(item.param2);
                                proto_item->set_param3(item.param3);
                                proto_item->set_param4(item.param4);
                                proto_item->set_current(item.current);
                                proto_item->set_autocontinue(item.autocontinue);
                            }
                        },
                        [&](const commands::CmdClearMission&) {
                            proto_cmd->mutable_clear_mission();
                        },
                        [&](const commands::CmdStartMission& start) {
                            auto* proto = proto_cmd->mutable_start_mission();
                            proto->set_first_item(start.first_item);
                            proto->set_last_item(start.last_item);
                        },
                        [&](const commands::CmdPauseMission&) {
                            proto_cmd->mutable_pause_mission();
                        },
                        [&](const commands::CmdResumeMission&) {
                            proto_cmd->mutable_resume_mission();
                        },
                        [&](const commands::CmdSetCurrentMissionItem& current) {
                            proto_cmd->mutable_set_current_mission_item()->set_seq(current.seq);
                        },
                    },
                    mission);
            },
            [&](const commands::PayloadCmd& payload) {
                std::visit(core::Overloaded{
                               [&](const commands::CmdPhoto& photo) {
                                   proto_cmd->mutable_photo()->set_camera_id(photo.camera_id);
                               },
                               [&](const commands::CmdPhotoIntervalStart& photo) {
                                   auto* proto = proto_cmd->mutable_photo_interval_start();
                                   proto->set_interval_s(photo.interval_s);
                                   proto->set_count(photo.count);
                                   proto->set_camera_id(photo.camera_id);
                               },
                               [&](const commands::CmdPhotoIntervalStop& photo) {
                                   proto_cmd->mutable_photo_interval_stop()->set_camera_id(
                                       photo.camera_id);
                               },
                               [&](const commands::CmdVideoStart& video) {
                                   auto* proto = proto_cmd->mutable_video_start();
                                   proto->set_stream_id(video.stream_id);
                                   proto->set_camera_id(video.camera_id);
                               },
                               [&](const commands::CmdVideoStop& video) {
                                   auto* proto = proto_cmd->mutable_video_stop();
                                   proto->set_stream_id(video.stream_id);
                                   proto->set_camera_id(video.camera_id);
                               },
                               [&](const commands::CmdGimbalPoint& gimbal) {
                                   auto* proto = proto_cmd->mutable_gimbal_point();
                                   proto->set_pitch_deg(gimbal.pitch_deg);
                                   proto->set_roll_deg(gimbal.roll_deg);
                                   proto->set_yaw_deg(gimbal.yaw_deg);
                               },
                               [&](const commands::CmdRoiLocation& roi) {
                                   auto* proto = proto_cmd->mutable_roi_location();
                                   proto->set_lat_deg(roi.lat_deg);
                                   proto->set_lon_deg(roi.lon_deg);
                                   proto->set_alt_m(roi.alt_m);
                                   proto->set_gimbal_id(roi.gimbal_id);
                               },
                               [&](const commands::CmdRoiClear& roi) {
                                   proto_cmd->mutable_roi_clear()->set_gimbal_id(roi.gimbal_id);
                               },
                               [&](const commands::CmdServo& servo) {
                                   auto* proto = proto_cmd->mutable_servo();
                                   proto->set_servo(servo.servo);
                                   proto->set_pwm(servo.pwm);
                               },
                               [&](const commands::CmdRelay& relay) {
                                   auto* proto = proto_cmd->mutable_relay();
                                   proto->set_relay(relay.relay);
                                   proto->set_enabled(relay.enabled);
                               },
                               [&](const commands::CmdGripper& gripper) {
                                   auto* proto = proto_cmd->mutable_gripper();
                                   proto->set_gripper(gripper.gripper);
                                   proto->set_release(gripper.release);
                               },
                           },
                           payload);
            },

            [&](const commands::BackendCmd& backend) {
                std::visit(
                    core::Overloaded{
                        [&](const commands::CmdBackendCommand& command) {
                            auto* proto = proto_cmd->mutable_backend_command();
                            proto->set_backend_namespace(command.backend_namespace);
                            proto->set_name(command.name);
                            for (const auto& [key, value] : command.params) {
                                (*proto->mutable_params())[key] = value;
                            }
                        },
                    },
                    backend);
            },

        },
        envelope.command);
}

}  // namespace

struct StreamState {
    std::mutex mutex;
    std::unique_ptr<grpc::ClientContext> context;
    std::thread worker;
    std::thread callback_worker;
    std::atomic<bool> active{false};
    std::atomic<bool> stop_requested{false};
    std::atomic<std::uint64_t> generation{0};

    std::mutex callback_mutex;
    std::condition_variable callback_cv;
    std::deque<std::function<void()>> callback_queue;
    bool callback_shutdown{false};
    std::size_t dropped_callbacks{0};

    SubscriptionKind kind{SubscriptionKind::kTelemetry};
    std::string drone_id;
    std::string correlation_id;
    SubscriptionOptions options{};
    TelemetryErrorHandler on_error;
    SubscriptionEventHandler on_event;
};

struct Subscription::State {
    std::weak_ptr<StreamState> stream;
    SubscriptionKind kind{SubscriptionKind::kTelemetry};
    std::uint64_t generation{};
    std::atomic<bool> stopped{false};
};

/// @brief Holds the gRPC channel, stub, and streaming state.
struct Client::Impl {
    ClientConfig config;
    std::shared_ptr<grpc::Channel> channel;
    std::unique_ptr<swarmkit::v1::AgentService::Stub> stub;
    std::shared_ptr<StreamState> telemetry{std::make_shared<StreamState>()};
    std::shared_ptr<StreamState> authority{std::make_shared<StreamState>()};
    std::shared_ptr<StreamState> reports{std::make_shared<StreamState>()};

    explicit Impl(ClientConfig cfg)
        : config(std::move(cfg)),
          channel(MakeChannel(config)),
          stub(swarmkit::v1::AgentService::NewStub(channel)) {}
};

struct StreamRetryState {
    int attempt_number{0};
    int backoff_ms{1};
};

struct ClientRuntime {
    const ClientConfig& config;
    swarmkit::v1::AgentService::Stub& stub;
};

[[nodiscard]] bool IsStopRequested(const StreamState& stream_state) {
    return stream_state.stop_requested.load(std::memory_order_relaxed);
}

[[nodiscard]] StreamRetryState MakeStreamRetryState(const ClientConfig& config) {
    return StreamRetryState{
        .attempt_number = 0,
        .backoff_ms = std::max(1, config.stream_reconnect_policy.initial_backoff_ms),
    };
}

[[nodiscard]] grpc::ClientContext* InstallStreamContext(StreamState& stream_state,
                                                        std::string_view correlation_id) {
    auto context = std::make_unique<grpc::ClientContext>();
    context->AddMetadata(std::string(kCorrelationMetadataKey), std::string(correlation_id));
    grpc::ClientContext* context_ptr = context.get();

    std::lock_guard<std::mutex> lock(stream_state.mutex);
    stream_state.context = std::move(context);
    return context_ptr;
}

void ResetStreamContext(StreamState& stream_state) {
    std::lock_guard<std::mutex> lock(stream_state.mutex);
    stream_state.context.reset();
}

[[nodiscard]] core::ErrorDomain DomainForSubscriptionKind(SubscriptionKind kind) {
    switch (kind) {
        case SubscriptionKind::kTelemetry:
            return core::ErrorDomain::kTelemetry;
        case SubscriptionKind::kAuthority:
            return core::ErrorDomain::kAuthority;
        case SubscriptionKind::kReports:
            return core::ErrorDomain::kInternal;
    }
    return core::ErrorDomain::kInternal;
}

[[nodiscard]] std::string_view ToString(SubscriptionKind kind) {
    switch (kind) {
        case SubscriptionKind::kTelemetry:
            return "telemetry";
        case SubscriptionKind::kAuthority:
            return "authority";
        case SubscriptionKind::kReports:
            return "reports";
    }
    return "stream";
}

void SafeNotifyStreamError(const TelemetryErrorHandler& on_error, const std::string& message) {
    if (!on_error) {
        return;
    }
    try {
        on_error(message);
    } catch (const std::exception& exc) {
        core::Logger::WarnFmt("Client stream error callback threw: {}", exc.what());
    } catch (...) {
        core::Logger::Warn("Client stream error callback threw an unknown exception");
    }
}

void SafeNotifySubscriptionEvent(const SubscriptionEventHandler& on_event,
                                 const SubscriptionEvent& event) {
    if (!on_event) {
        return;
    }
    try {
        on_event(event);
    } catch (const std::exception& exc) {
        core::Logger::WarnFmt("Client subscription event callback threw: {}", exc.what());
    } catch (...) {
        core::Logger::Warn("Client subscription event callback threw an unknown exception");
    }
}

void HandleCallbackException(StreamState& stream_state, std::string_view callback_name,
                             const std::string& message) {
    const std::string detail = std::string(ToString(stream_state.kind)) + " " +
                               std::string(callback_name) + " callback threw: " + message;
    core::Logger::WarnFmt("Client stream callback exception: {}", detail);

    RpcError error = core::SwarmError::Make(
        DomainForSubscriptionKind(stream_state.kind), RpcStatusCode::kInternal, detail,
        core::ErrorSeverity::kError, core::ErrorRetryability::kAfterRemediation,
        "Catch exceptions inside subscription callbacks");
    error.correlation_id = stream_state.correlation_id;

    SafeNotifySubscriptionEvent(stream_state.on_event,
                                SubscriptionEvent{
                                    .kind = stream_state.kind,
                                    .state = SubscriptionLifecycleState::kCallbackError,
                                    .drone_id = stream_state.drone_id,
                                    .correlation_id = stream_state.correlation_id,
                                    .error = error,
                                    .message = detail,
                                    .dropped_callbacks = stream_state.dropped_callbacks,
                                });
    SafeNotifyStreamError(stream_state.on_error, detail);
}

void ResetCallbackQueue(StreamState& stream_state) {
    std::lock_guard<std::mutex> lock(stream_state.callback_mutex);
    stream_state.callback_queue.clear();
    stream_state.callback_shutdown = false;
    stream_state.dropped_callbacks = 0;
}

void ShutdownCallbackQueue(StreamState& stream_state) {
    {
        std::lock_guard<std::mutex> lock(stream_state.callback_mutex);
        stream_state.callback_shutdown = true;
    }
    stream_state.callback_cv.notify_all();
}

[[nodiscard]] bool EnqueueCallback(StreamState& stream_state, std::function<void()> callback,
                                   bool force = false) {
    const std::size_t max_pending =
        std::max<std::size_t>(1, stream_state.options.backpressure.max_pending_callbacks);

    std::unique_lock<std::mutex> lock(stream_state.callback_mutex);
    if (!force && stream_state.options.backpressure.policy == StreamBackpressurePolicy::kBlock) {
        stream_state.callback_cv.wait(lock, [&] {
            return stream_state.callback_shutdown ||
                   stream_state.callback_queue.size() < max_pending ||
                   stream_state.stop_requested.load(std::memory_order_relaxed);
        });
    }

    if (stream_state.callback_shutdown ||
        stream_state.stop_requested.load(std::memory_order_relaxed)) {
        return false;
    }

    if (!force && stream_state.callback_queue.size() >= max_pending) {
        if (stream_state.options.backpressure.policy == StreamBackpressurePolicy::kDropNewest) {
            ++stream_state.dropped_callbacks;
            return false;
        }
        if (stream_state.options.backpressure.policy == StreamBackpressurePolicy::kDropOldest) {
            stream_state.callback_queue.pop_front();
            ++stream_state.dropped_callbacks;
        }
    }

    stream_state.callback_queue.push_back(std::move(callback));
    lock.unlock();
    stream_state.callback_cv.notify_one();
    return true;
}

void RunCallbackDispatcher(StreamState& stream_state) {
    while (true) {
        std::function<void()> callback;
        {
            std::unique_lock<std::mutex> lock(stream_state.callback_mutex);
            stream_state.callback_cv.wait(lock, [&] {
                return stream_state.callback_shutdown || !stream_state.callback_queue.empty();
            });
            if (stream_state.callback_queue.empty() && stream_state.callback_shutdown) {
                return;
            }
            callback = std::move(stream_state.callback_queue.front());
            stream_state.callback_queue.pop_front();
        }
        stream_state.callback_cv.notify_all();

        try {
            callback();
        } catch (const std::exception& exc) {
            HandleCallbackException(stream_state, "application", exc.what());
        } catch (...) {
            HandleCallbackException(stream_state, "application", "unknown exception");
        }
    }
}

void EmitSubscriptionEvent(StreamState& stream_state, SubscriptionLifecycleState state,
                           std::string message, int attempt_number = 0, RpcError error = {}) {
    error.correlation_id =
        error.correlation_id.empty() ? stream_state.correlation_id : error.correlation_id;
    SubscriptionEvent event{
        .kind = stream_state.kind,
        .state = state,
        .drone_id = stream_state.drone_id,
        .correlation_id = stream_state.correlation_id,
        .attempt_number = attempt_number,
        .error = std::move(error),
        .message = std::move(message),
        .dropped_callbacks = stream_state.dropped_callbacks,
    };
    static_cast<void>(EnqueueCallback(
        stream_state,
        [handler = stream_state.on_event, event = std::move(event)]() {
            SafeNotifySubscriptionEvent(handler, event);
        },
        true));
}

void CancelAndJoinStream(StreamState& stream_state,
                         std::optional<std::uint64_t> generation = std::nullopt) {
    if (generation.has_value() &&
        stream_state.generation.load(std::memory_order_relaxed) != *generation) {
        return;
    }
    if (!stream_state.active.load(std::memory_order_relaxed) && !stream_state.worker.joinable() &&
        !stream_state.callback_worker.joinable()) {
        return;
    }

    stream_state.stop_requested.store(true, std::memory_order_relaxed);
    stream_state.callback_cv.notify_all();
    {
        std::lock_guard<std::mutex> lock(stream_state.mutex);
        if (stream_state.context) {
            stream_state.context->TryCancel();
        }
    }

    if (stream_state.worker.joinable()) {
        stream_state.worker.join();
    }

    ShutdownCallbackQueue(stream_state);
    if (stream_state.callback_worker.joinable()) {
        if (stream_state.callback_worker.get_id() == std::this_thread::get_id()) {
            stream_state.callback_worker.detach();
        } else {
            stream_state.callback_worker.join();
        }
    }

    ResetStreamContext(stream_state);
    stream_state.active.store(false, std::memory_order_relaxed);
}

Subscription::~Subscription() {
    Stop();
}

Subscription::Subscription(Subscription&&) noexcept = default;

Subscription& Subscription::operator=(Subscription&& other) noexcept {
    if (this == &other) {
        return *this;
    }
    Stop();
    state_ = std::move(other.state_);
    return *this;
}

void Subscription::Stop() noexcept {
    if (!state_) {
        return;
    }
    if (state_->stopped.exchange(true, std::memory_order_relaxed)) {
        return;
    }
    if (auto stream = state_->stream.lock()) {
        CancelAndJoinStream(*stream, state_->generation);
    }
}

bool Subscription::IsActive() const noexcept {
    if (!state_ || state_->stopped.load(std::memory_order_relaxed)) {
        return false;
    }
    const auto stream = state_->stream.lock();
    return stream && stream->generation.load(std::memory_order_relaxed) == state_->generation &&
           stream->active.load(std::memory_order_relaxed);
}

SubscriptionKind Subscription::Kind() const noexcept {
    return state_ ? state_->kind : SubscriptionKind::kTelemetry;
}

std::uint64_t Subscription::Id() const noexcept {
    return state_ ? state_->generation : 0;
}

[[nodiscard]] bool ShouldRetryStream(const StreamReconnectPolicy& policy, int attempt_number,
                                     const grpc::Status& final_status) {
    if (!policy.enabled) {
        return false;
    }
    if (final_status.ok()) {
        return false;
    }
    switch (final_status.error_code()) {
        case grpc::StatusCode::UNAVAILABLE:
        case grpc::StatusCode::DEADLINE_EXCEEDED:
        case grpc::StatusCode::CANCELLED:
            break;
        default:
            return false;
    }

    const bool kUnlimitedAttempts = policy.max_attempts == kUnlimitedStreamReconnectAttempts;
    return kUnlimitedAttempts || attempt_number < policy.max_attempts;
}

void SleepBeforeNextRetry(const StreamReconnectPolicy& policy, int* backoff_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(*backoff_ms));
    *backoff_ms = std::min(policy.max_backoff_ms, *backoff_ms * 2);
}

void MaybeReportStreamFailure(StreamState& stream_state, const grpc::Status& status) {
    if (stream_state.on_error && !status.ok()) {
        static_cast<void>(EnqueueCallback(
            stream_state,
            [on_error = stream_state.on_error, message = status.error_message()]() {
                SafeNotifyStreamError(on_error, message);
            },
            true));
    }
}

[[nodiscard]] bool ShouldLogStreamFailureAttempt(int attempt_number) {
    if (attempt_number <= 1) {
        return true;
    }

    constexpr int kStreamFailureLogInterval = 10;
    return attempt_number % kStreamFailureLogInterval == 0;
}

void LogStreamFailure(std::string_view stream_name, std::string_view drone_id,
                      std::string_view correlation_id, const grpc::Status& status,
                      int attempt_number) {
    if (status.ok()) {
        return;
    }
    if (!ShouldLogStreamFailureAttempt(attempt_number)) {
        return;
    }

    core::Logger::WarnFmt(
        "Client {} stream failed: drone={} corr={} attempt={} status={} message={}", stream_name,
        drone_id, correlation_id, attempt_number, static_cast<int>(status.error_code()),
        status.error_message());
}

[[nodiscard]] core::TelemetryFrame ToCoreTelemetryFrame(
    const swarmkit::v1::TelemetryFrame& proto_frame) {
    core::TelemetryFrame frame;
    frame.drone_id = proto_frame.drone_id();
    frame.unix_time_ms = proto_frame.unix_time_ms();
    frame.source_unix_time_ms = proto_frame.source_unix_time_ms();
    frame.source_time_boot_ms = proto_frame.source_time_boot_ms();
    frame.lat_deg = proto_frame.lat_deg();
    frame.lon_deg = proto_frame.lon_deg();
    frame.rel_alt_m = proto_frame.rel_alt_m();
    frame.abs_alt_m = proto_frame.abs_alt_m();
    frame.vx_mps = proto_frame.vx_mps();
    frame.vy_mps = proto_frame.vy_mps();
    frame.vz_mps = proto_frame.vz_mps();
    frame.roll_deg = proto_frame.roll_deg();
    frame.pitch_deg = proto_frame.pitch_deg();
    frame.yaw_deg = proto_frame.yaw_deg();
    frame.battery_percent = proto_frame.battery_percent();
    frame.mode = proto_frame.mode();
    frame.armed = proto_frame.armed();
    frame.landed = proto_frame.landed();
    frame.failsafe = proto_frame.failsafe();
    frame.ekf_ok = proto_frame.ekf_ok();
    frame.gps_fix_type = proto_frame.gps_fix_type();
    frame.satellites_visible = proto_frame.satellites_visible();
    frame.gps_hdop = proto_frame.gps_hdop();
    frame.link_quality_percent = proto_frame.link_quality_percent();
    frame.position_frame = ToCoreCoordinateFrame(proto_frame.position_frame());
    frame.velocity_frame = ToCoreCoordinateFrame(proto_frame.velocity_frame());
    if (proto_frame.has_validity()) {
        const auto& validity = proto_frame.validity();
        frame.validity.position = validity.position();
        frame.validity.relative_altitude = validity.relative_altitude();
        frame.validity.absolute_altitude = validity.absolute_altitude();
        frame.validity.velocity = validity.velocity();
        frame.validity.attitude = validity.attitude();
        frame.validity.battery = validity.battery();
        frame.validity.mode = validity.mode();
        frame.validity.armed = validity.armed();
        frame.validity.landed = validity.landed();
        frame.validity.failsafe = validity.failsafe();
        frame.validity.gps = validity.gps();
        frame.validity.gps_hdop = validity.gps_hdop();
        frame.validity.link_quality = validity.link_quality();
        frame.validity.estimator = validity.estimator();
        frame.validity.home_origin = validity.home_origin();
    }
    if (proto_frame.has_accuracy()) {
        const auto& accuracy = proto_frame.accuracy();
        frame.accuracy.horizontal_position_valid = accuracy.horizontal_position_valid();
        frame.accuracy.horizontal_position_m = accuracy.horizontal_position_m();
        frame.accuracy.vertical_position_valid = accuracy.vertical_position_valid();
        frame.accuracy.vertical_position_m = accuracy.vertical_position_m();
        frame.accuracy.velocity_valid = accuracy.velocity_valid();
        frame.accuracy.velocity_mps = accuracy.velocity_mps();
        frame.accuracy.heading_valid = accuracy.heading_valid();
        frame.accuracy.heading_deg = accuracy.heading_deg();
        frame.accuracy.attitude_valid = accuracy.attitude_valid();
        frame.accuracy.attitude_deg = accuracy.attitude_deg();
        frame.accuracy.position_covariance_valid = accuracy.position_covariance_valid();
        for (int index = 0;
             index < accuracy.position_covariance_size() &&
             index < static_cast<int>(frame.accuracy.position_covariance.size());
             ++index) {
            frame.accuracy.position_covariance[static_cast<std::size_t>(index)] =
                accuracy.position_covariance(index);
        }
        frame.accuracy.velocity_covariance_valid = accuracy.velocity_covariance_valid();
        for (int index = 0;
             index < accuracy.velocity_covariance_size() &&
             index < static_cast<int>(frame.accuracy.velocity_covariance.size());
             ++index) {
            frame.accuracy.velocity_covariance[static_cast<std::size_t>(index)] =
                accuracy.velocity_covariance(index);
        }
    }
    if (proto_frame.has_home_origin()) {
        const auto& home = proto_frame.home_origin();
        frame.home_origin.frame = ToCoreCoordinateFrame(home.frame());
        frame.home_origin.lat_deg = home.lat_deg();
        frame.home_origin.lon_deg = home.lon_deg();
        frame.home_origin.alt_m = home.alt_m();
        frame.home_origin.north_m = home.north_m();
        frame.home_origin.east_m = home.east_m();
        frame.home_origin.down_m = home.down_m();
    }
    frame.gps_quality = ToCoreGpsQuality(proto_frame.gps_quality());
    frame.estimator_state = ToCoreEstimatorState(proto_frame.estimator_state());
    frame.estimator_flags = proto_frame.estimator_flags();
    frame.estimator_position_ok = proto_frame.estimator_position_ok();
    frame.estimator_velocity_ok = proto_frame.estimator_velocity_ok();
    frame.estimator_attitude_ok = proto_frame.estimator_attitude_ok();
    frame.active_command_id = proto_frame.active_command_id();
    frame.active_goal_id = proto_frame.active_goal_id();
    frame.active_execution_id = proto_frame.active_execution_id();
    frame.correlation_id = proto_frame.correlation_id();
    return frame;
}

[[nodiscard]] double DegreesToRadians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] double DistanceMetres(double lat_one, double lon_one, double lat_two,
                                    double lon_two) {
    const double dlat = DegreesToRadians(lat_two - lat_one);
    const double dlon = DegreesToRadians(lon_two - lon_one);
    const double rlat_one = DegreesToRadians(lat_one);
    const double rlat_two = DegreesToRadians(lat_two);
    const double sin_half_lat = std::sin(dlat / 2.0);
    const double sin_half_lon = std::sin(dlon / 2.0);
    const double haversine =
        std::clamp((sin_half_lat * sin_half_lat) +
                       (std::cos(rlat_one) * std::cos(rlat_two) * sin_half_lon * sin_half_lon),
                   0.0, 1.0);
    return kEarthRadiusM * 2.0 *
           std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));
}

[[nodiscard]] AuthorityEventInfo ToAuthorityEventInfo(
    const swarmkit::v1::AuthorityEvent& proto_event) {
    return AuthorityEventInfo{
        .kind = ToAuthorityEventKind(proto_event.kind()),
        .drone_id = proto_event.drone_id(),
        .holder_client_id = proto_event.holder_client_id(),
        .holder_priority = static_cast<CommandPriority>(proto_event.holder_priority()),
        .correlation_id = proto_event.correlation_id(),
    };
}

[[nodiscard]] swarmkit::v1::GeoPoint ToProtoGeoPoint(const GeoPoint& point) {
    swarmkit::v1::GeoPoint proto;
    proto.set_lat_deg(point.lat_deg);
    proto.set_lon_deg(point.lon_deg);
    proto.set_alt_m(point.alt_m);
    return proto;
}

void PopulateProtoActiveGoal(const ActiveGoal& goal, swarmkit::v1::ActiveGoal* proto_goal) {
    if (proto_goal == nullptr) {
        return;
    }
    proto_goal->set_drone_id(goal.drone_id);
    proto_goal->set_goal_id(goal.goal_id);
    proto_goal->set_revision(goal.revision);
    *proto_goal->mutable_target() = ToProtoGeoPoint(goal.target);
    proto_goal->set_speed_mps(goal.speed_mps);
    proto_goal->set_acceptance_radius_m(goal.acceptance_radius_m);
    proto_goal->set_deviation_radius_m(goal.deviation_radius_m);
    proto_goal->set_timeout_ms(goal.timeout_ms);
    for (const auto& [key, value] : goal.labels) {
        (*proto_goal->mutable_labels())[key] = value;
    }
}

[[nodiscard]] ActiveGoal ToActiveGoal(const swarmkit::v1::ActiveGoal& proto_goal) {
    ActiveGoal goal;
    goal.drone_id = proto_goal.drone_id();
    goal.goal_id = proto_goal.goal_id();
    goal.revision = proto_goal.revision();
    goal.target = GeoPoint{
        .lat_deg = proto_goal.target().lat_deg(),
        .lon_deg = proto_goal.target().lon_deg(),
        .alt_m = proto_goal.target().alt_m(),
    };
    goal.speed_mps = proto_goal.speed_mps();
    goal.acceptance_radius_m = proto_goal.acceptance_radius_m();
    goal.deviation_radius_m = proto_goal.deviation_radius_m();
    goal.timeout_ms = proto_goal.timeout_ms();
    for (const auto& [key, value] : proto_goal.labels()) {
        goal.labels.emplace(key, value);
    }
    return goal;
}

[[nodiscard]] GoalStatus ToGoalStatus(swarmkit::v1::GoalStatus status) {
    using ProtoStatus = swarmkit::v1::GoalStatus;
    switch (status) {
        case ProtoStatus::GOAL_ACTIVE:
            return GoalStatus::kActive;
        case ProtoStatus::GOAL_REACHED:
            return GoalStatus::kReached;
        case ProtoStatus::GOAL_DEVIATING:
            return GoalStatus::kDeviating;
        case ProtoStatus::GOAL_TIMEOUT:
            return GoalStatus::kTimeout;
        case ProtoStatus::GOAL_CANCELLED:
            return GoalStatus::kCancelled;
        case ProtoStatus::GOAL_SUPERSEDED:
            return GoalStatus::kSuperseded;
        case ProtoStatus::GOAL_FAILED:
            return GoalStatus::kFailed;
        case ProtoStatus::GOAL_STATUS_UNSPECIFIED:
        default:
            return GoalStatus::kUnspecified;
    }
}

[[nodiscard]] AgentReportType ToAgentReportType(swarmkit::v1::AgentReportType type) {
    using ProtoType = swarmkit::v1::AgentReportType;
    switch (type) {
        case ProtoType::COMMAND_ACCEPTED:
            return AgentReportType::kCommandAccepted;
        case ProtoType::COMMAND_REJECTED:
            return AgentReportType::kCommandRejected;
        case ProtoType::COMMAND_ACKED:
            return AgentReportType::kCommandAcked;
        case ProtoType::COMMAND_FAILED:
            return AgentReportType::kCommandFailed;
        case ProtoType::GOAL_REPORT:
            return AgentReportType::kGoalReport;
        case ProtoType::TELEMETRY_STALE:
            return AgentReportType::kTelemetryStale;
        case ProtoType::HEARTBEAT_LOST:
            return AgentReportType::kHeartbeatLost;
        case ProtoType::HEALTH_CHANGED:
            return AgentReportType::kHealthChanged;
        case ProtoType::AUTHORITY_LOCKED:
            return AgentReportType::kAuthorityLocked;
        case ProtoType::AUTHORITY_REJECTED:
            return AgentReportType::kAuthorityRejected;
        case ProtoType::AUTHORITY_RELEASED:
            return AgentReportType::kAuthorityReleased;
        case ProtoType::TRAJECTORY_REPORT:
            return AgentReportType::kTrajectoryReport;
        case ProtoType::TIME_SYNC_REPORT:
            return AgentReportType::kTimeSyncReport;
        case ProtoType::AGENT_REPORT_TYPE_UNSPECIFIED:
        default:
            return AgentReportType::kUnspecified;
    }
}

[[nodiscard]] ReportSeverity ToReportSeverity(swarmkit::v1::ReportSeverity severity) {
    using ProtoSeverity = swarmkit::v1::ReportSeverity;
    switch (severity) {
        case ProtoSeverity::REPORT_WARNING:
            return ReportSeverity::kWarning;
        case ProtoSeverity::REPORT_ERROR:
            return ReportSeverity::kError;
        case ProtoSeverity::REPORT_CRITICAL:
            return ReportSeverity::kCritical;
        case ProtoSeverity::REPORT_INFO:
        case ProtoSeverity::REPORT_SEVERITY_UNSPECIFIED:
        default:
            return ReportSeverity::kInfo;
    }
}

[[nodiscard]] GoalReport ToGoalReport(const swarmkit::v1::GoalReport& proto_report) {
    return GoalReport{
        .drone_id = proto_report.drone_id(),
        .goal_id = proto_report.goal_id(),
        .revision = proto_report.revision(),
        .status = ToGoalStatus(proto_report.status()),
        .distance_to_goal_m = proto_report.distance_to_goal_m(),
        .deviation_m = proto_report.deviation_m(),
        .altitude_error_m = proto_report.altitude_error_m(),
        .acceptance_radius_m = proto_report.acceptance_radius_m(),
        .deviation_radius_m = proto_report.deviation_radius_m(),
        .elapsed_ms = proto_report.elapsed_ms(),
        .timeout_ms = proto_report.timeout_ms(),
        .message = proto_report.message(),
    };
}

[[nodiscard]] swarmkit::v1::TrajectoryPoint ToProtoTrajectoryPoint(const TrajectoryPoint& point) {
    swarmkit::v1::TrajectoryPoint proto;
    proto.set_time_offset_ms(point.time_offset_ms);
    proto.set_unix_time_ms(point.unix_time_ms);
    if (point.has_position) {
        *proto.mutable_position() = ToProtoGeoPoint(point.position);
    }
    if (point.has_local_position || point.use_local_position) {
        auto* local = proto.mutable_local_position();
        local->set_x_m(point.local_position.x_m);
        local->set_y_m(point.local_position.y_m);
        local->set_z_m(point.local_position.z_m);
    }
    proto.set_use_local_position(point.use_local_position);
    proto.set_vx_mps(point.vx_mps);
    proto.set_vy_mps(point.vy_mps);
    proto.set_vz_mps(point.vz_mps);
    proto.set_has_velocity(point.has_velocity);
    proto.set_yaw_deg(point.yaw_deg);
    proto.set_has_yaw(point.has_yaw);
    if (point.command.has_value()) {
        swarmkit::v1::CommandRequest request;
        commands::CommandEnvelope envelope;
        envelope.command = *point.command;
        BuildProtoCommand(envelope, request);
        *proto.mutable_command() = request.cmd();
    }
    return proto;
}

void PopulateProtoTrajectoryPlan(const TrajectoryPlan& plan, swarmkit::v1::TrajectoryPlan* proto) {
    if (proto == nullptr) {
        return;
    }
    proto->set_execution_id(plan.execution_id);
    proto->set_revision(plan.revision);
    proto->set_drone_id(plan.drone_id);
    proto->set_frame(plan.frame);
    for (const auto& point : plan.points) {
        *proto->add_points() = ToProtoTrajectoryPoint(point);
    }
    auto* validation = proto->mutable_validation();
    validation->set_min_battery_percent(plan.validation.min_battery_percent);
    if (plan.validation.geofence.has_value()) {
        auto* fence = validation->mutable_geofence();
        fence->set_min_lat_deg(plan.validation.geofence->min_lat_deg);
        fence->set_max_lat_deg(plan.validation.geofence->max_lat_deg);
        fence->set_min_lon_deg(plan.validation.geofence->min_lon_deg);
        fence->set_max_lon_deg(plan.validation.geofence->max_lon_deg);
        fence->set_min_alt_m(plan.validation.geofence->min_alt_m);
        fence->set_max_alt_m(plan.validation.geofence->max_alt_m);
    }
    validation->set_min_spacing_m(plan.validation.min_spacing_m);
    validation->set_require_gps(plan.validation.require_gps);
    validation->set_require_ekf_ok(plan.validation.require_ekf_ok);
    validation->set_max_horizontal_speed_mps(plan.validation.max_horizontal_speed_mps);
    validation->set_max_climb_speed_mps(plan.validation.max_climb_speed_mps);
    validation->set_max_descent_speed_mps(plan.validation.max_descent_speed_mps);
    validation->set_max_altitude_m(plan.validation.max_altitude_m);
    validation->set_tracking_tolerance_m(plan.validation.tracking_tolerance_m);
    for (const auto& [key, value] : plan.labels) {
        (*proto->mutable_labels())[key] = value;
    }
}

[[nodiscard]] GeoPoint ToGeoPoint(const swarmkit::v1::GeoPoint& point) {
    return {
        .lat_deg = point.lat_deg(),
        .lon_deg = point.lon_deg(),
        .alt_m = point.alt_m(),
    };
}

[[nodiscard]] std::optional<commands::Command> ToClientCommand(const swarmkit::v1::Command& proto) {
    switch (proto.kind_case()) {
        case swarmkit::v1::Command::kArm:
            return commands::FlightCmd{commands::CmdArm{}};
        case swarmkit::v1::Command::kForceArm:
            return commands::FlightCmd{commands::CmdForceArm{}};
        case swarmkit::v1::Command::kDisarm:
            return commands::FlightCmd{commands::CmdDisarm{}};
        case swarmkit::v1::Command::kLand:
            return commands::FlightCmd{commands::CmdLand{}};
        case swarmkit::v1::Command::kTakeoff:
            return commands::FlightCmd{commands::CmdTakeoff{proto.takeoff().alt_m()}};
        case swarmkit::v1::Command::kSetMode:
            return commands::FlightCmd{commands::CmdSetMode{
                .mode = proto.set_mode().mode(),
                .custom_mode = proto.set_mode().custom_mode(),
            }};
        case swarmkit::v1::Command::kForceDisarm:
            return commands::FlightCmd{commands::CmdForceDisarm{}};
        case swarmkit::v1::Command::kFlightTerminate:
            return commands::FlightCmd{commands::CmdFlightTerminate{}};
        case swarmkit::v1::Command::kSetWaypoint:
            return commands::NavCmd{commands::CmdSetWaypoint{
                .lat_deg = proto.set_waypoint().lat_deg(),
                .lon_deg = proto.set_waypoint().lon_deg(),
                .alt_m = proto.set_waypoint().alt_m(),
                .speed_mps = proto.set_waypoint().speed_mps(),
            }};
        case swarmkit::v1::Command::kReturnHome:
            return commands::NavCmd{commands::CmdReturnHome{}};
        case swarmkit::v1::Command::kHoldPosition:
            return commands::NavCmd{commands::CmdHoldPosition{}};
        case swarmkit::v1::Command::kSetSpeed:
            return commands::NavCmd{commands::CmdSetSpeed{proto.set_speed().ground_mps()}};
        case swarmkit::v1::Command::kGotoPosition:
            return commands::NavCmd{commands::CmdGoto{
                .lat_deg = proto.goto_position().lat_deg(),
                .lon_deg = proto.goto_position().lon_deg(),
                .alt_m = proto.goto_position().alt_m(),
                .speed_mps = proto.goto_position().speed_mps(),
                .yaw_deg = proto.goto_position().yaw_deg(),
                .use_yaw = proto.goto_position().use_yaw(),
            }};
        case swarmkit::v1::Command::kPause:
            return commands::NavCmd{commands::CmdPause{}};
        case swarmkit::v1::Command::kResume:
            return commands::NavCmd{commands::CmdResume{}};
        case swarmkit::v1::Command::kSetYaw:
            return commands::NavCmd{commands::CmdSetYaw{
                .yaw_deg = proto.set_yaw().yaw_deg(),
                .rate_deg_s = proto.set_yaw().rate_deg_s(),
                .relative = proto.set_yaw().relative(),
            }};
        case swarmkit::v1::Command::kVelocity:
            return commands::NavCmd{commands::CmdVelocity{
                .vx_mps = proto.velocity().vx_mps(),
                .vy_mps = proto.velocity().vy_mps(),
                .vz_mps = proto.velocity().vz_mps(),
                .duration_ms = proto.velocity().duration_ms(),
                .body_frame = proto.velocity().body_frame(),
            }};
        case swarmkit::v1::Command::kSetHome:
            return commands::NavCmd{commands::CmdSetHome{
                .use_current = proto.set_home().use_current(),
                .lat_deg = proto.set_home().lat_deg(),
                .lon_deg = proto.set_home().lon_deg(),
                .alt_m = proto.set_home().alt_m(),
            }};
        default:
            return std::nullopt;
    }
}

[[nodiscard]] TrajectoryPoint ToTrajectoryPoint(const swarmkit::v1::TrajectoryPoint& proto) {
    TrajectoryPoint point;
    point.time_offset_ms = proto.time_offset_ms();
    point.unix_time_ms = proto.unix_time_ms();
    point.has_position = proto.has_position();
    if (proto.has_position()) {
        point.position = ToGeoPoint(proto.position());
    }
    point.has_local_position = proto.has_local_position();
    if (proto.has_local_position()) {
        point.local_position = LocalPoint{
            .x_m = proto.local_position().x_m(),
            .y_m = proto.local_position().y_m(),
            .z_m = proto.local_position().z_m(),
        };
    }
    point.use_local_position = proto.use_local_position();
    point.vx_mps = proto.vx_mps();
    point.vy_mps = proto.vy_mps();
    point.vz_mps = proto.vz_mps();
    point.has_velocity = proto.has_velocity();
    point.yaw_deg = proto.yaw_deg();
    point.has_yaw = proto.has_yaw();
    if (proto.has_command()) {
        point.command = ToClientCommand(proto.command());
    }
    return point;
}

[[nodiscard]] TrajectoryPlan ToTrajectoryPlan(const swarmkit::v1::TrajectoryPlan& proto) {
    TrajectoryPlan plan;
    plan.execution_id = proto.execution_id();
    plan.revision = proto.revision();
    plan.drone_id = proto.drone_id();
    plan.frame = proto.frame();
    for (const auto& point : proto.points()) {
        plan.points.push_back(ToTrajectoryPoint(point));
    }
    if (proto.has_validation()) {
        const auto& validation = proto.validation();
        plan.validation.min_battery_percent = validation.min_battery_percent();
        if (validation.has_geofence()) {
            const auto& fence = validation.geofence();
            plan.validation.geofence = Geofence{
                .min_lat_deg = fence.min_lat_deg(),
                .max_lat_deg = fence.max_lat_deg(),
                .min_lon_deg = fence.min_lon_deg(),
                .max_lon_deg = fence.max_lon_deg(),
                .min_alt_m = fence.min_alt_m(),
                .max_alt_m = fence.max_alt_m(),
            };
        }
        plan.validation.min_spacing_m = validation.min_spacing_m();
        plan.validation.require_gps = validation.require_gps();
        plan.validation.require_ekf_ok = validation.require_ekf_ok();
        plan.validation.max_horizontal_speed_mps = validation.max_horizontal_speed_mps();
        plan.validation.max_climb_speed_mps = validation.max_climb_speed_mps();
        plan.validation.max_descent_speed_mps = validation.max_descent_speed_mps();
        plan.validation.max_altitude_m = validation.max_altitude_m();
        plan.validation.tracking_tolerance_m = validation.tracking_tolerance_m();
    }
    for (const auto& [key, value] : proto.labels()) {
        plan.labels.emplace(key, value);
    }
    return plan;
}

[[nodiscard]] ValidationSeverity ToValidationSeverity(swarmkit::v1::ValidationSeverity severity) {
    switch (severity) {
        case swarmkit::v1::VALIDATION_ERROR:
            return ValidationSeverity::kError;
        case swarmkit::v1::VALIDATION_WARNING:
            return ValidationSeverity::kWarning;
        case swarmkit::v1::VALIDATION_INFO:
        case swarmkit::v1::VALIDATION_SEVERITY_UNSPECIFIED:
        default:
            return ValidationSeverity::kInfo;
    }
}

[[nodiscard]] ValidateTrajectoryResult ToValidateTrajectoryResult(
    const swarmkit::v1::ValidateTrajectoryResult& proto) {
    ValidateTrajectoryResult result;
    result.ok = proto.ok();
    result.max_required_horizontal_speed_mps = proto.max_required_horizontal_speed_mps();
    result.max_required_climb_speed_mps = proto.max_required_climb_speed_mps();
    result.max_required_descent_speed_mps = proto.max_required_descent_speed_mps();
    result.first_failing_point_index = proto.first_failing_point_index();
    for (const auto& issue : proto.issues()) {
        result.issues.push_back(ValidationIssue{
            .severity = ToValidationSeverity(issue.severity()),
            .code = issue.code(),
            .message = issue.message(),
            .point_index = issue.point_index(),
        });
    }
    return result;
}

[[nodiscard]] ExecutionState ToExecutionState(swarmkit::v1::ExecutionState state) {
    switch (state) {
        case swarmkit::v1::EXECUTION_UPLOADED:
            return ExecutionState::kUploaded;
        case swarmkit::v1::EXECUTION_VALIDATED:
            return ExecutionState::kValidated;
        case swarmkit::v1::EXECUTION_READY:
            return ExecutionState::kReady;
        case swarmkit::v1::EXECUTION_STARTED:
            return ExecutionState::kStarted;
        case swarmkit::v1::EXECUTION_PAUSED:
            return ExecutionState::kPaused;
        case swarmkit::v1::EXECUTION_ABORTED:
            return ExecutionState::kAborted;
        case swarmkit::v1::EXECUTION_COMPLETED:
            return ExecutionState::kCompleted;
        case swarmkit::v1::EXECUTION_FAILED:
            return ExecutionState::kFailed;
        case swarmkit::v1::EXECUTION_STATE_UNSPECIFIED:
        default:
            return ExecutionState::kUnspecified;
    }
}

[[nodiscard]] ExecutionHandle ToExecutionHandle(const swarmkit::v1::ExecutionHandle& proto) {
    return {
        .execution_id = proto.execution_id(),
        .revision = proto.revision(),
        .drone_id = proto.drone_id(),
        .state = ToExecutionState(proto.state()),
        .uploaded_unix_ms = proto.uploaded_unix_ms(),
        .prepared_unix_ms = proto.prepared_unix_ms(),
        .start_unix_ms = proto.start_unix_ms(),
        .active_segment = proto.active_segment(),
        .last_report_sequence = proto.last_report_sequence(),
        .message = proto.message(),
    };
}

[[nodiscard]] TrajectoryReportStatus ToTrajectoryReportStatus(
    swarmkit::v1::TrajectoryReportStatus status) {
    switch (status) {
        case swarmkit::v1::TRAJECTORY_UPLOADED:
            return TrajectoryReportStatus::kUploaded;
        case swarmkit::v1::TRAJECTORY_VALIDATED:
            return TrajectoryReportStatus::kValidated;
        case swarmkit::v1::TRAJECTORY_READY:
            return TrajectoryReportStatus::kReady;
        case swarmkit::v1::TRAJECTORY_STARTED:
            return TrajectoryReportStatus::kStarted;
        case swarmkit::v1::TRAJECTORY_LATE:
            return TrajectoryReportStatus::kLate;
        case swarmkit::v1::TRAJECTORY_TRACKING:
            return TrajectoryReportStatus::kTracking;
        case swarmkit::v1::TRAJECTORY_DRIFTING:
            return TrajectoryReportStatus::kDrifting;
        case swarmkit::v1::TRAJECTORY_ABORTED:
            return TrajectoryReportStatus::kAborted;
        case swarmkit::v1::TRAJECTORY_COMPLETED:
            return TrajectoryReportStatus::kCompleted;
        case swarmkit::v1::TRAJECTORY_FAILED:
            return TrajectoryReportStatus::kFailed;
        case swarmkit::v1::TRAJECTORY_STATUS_UNSPECIFIED:
        default:
            return TrajectoryReportStatus::kUnspecified;
    }
}

[[nodiscard]] TrajectoryReport ToTrajectoryReport(const swarmkit::v1::TrajectoryReport& proto) {
    return {
        .drone_id = proto.drone_id(),
        .execution_id = proto.execution_id(),
        .revision = proto.revision(),
        .status = ToTrajectoryReportStatus(proto.status()),
        .active_segment = proto.active_segment(),
        .distance_to_target_m = proto.distance_to_target_m(),
        .drift_m = proto.drift_m(),
        .schedule_error_ms = proto.schedule_error_ms(),
        .message = proto.message(),
    };
}

[[nodiscard]] TimeSyncState ToTimeSyncState(const swarmkit::v1::TimeSyncState& proto) {
    return {
        .drone_id = proto.drone_id(),
        .agent_unix_time_ms = proto.agent_unix_time_ms(),
        .vehicle_unix_time_ms = proto.vehicle_unix_time_ms(),
        .clock_offset_ms = proto.clock_offset_ms(),
        .sync_quality_percent = proto.sync_quality_percent(),
        .synced = proto.synced(),
        .stale = proto.stale(),
        .source = proto.source(),
        .message = proto.message(),
    };
}

[[nodiscard]] AgentReport ToAgentReport(const swarmkit::v1::AgentReport& proto_report) {
    AgentReport report;
    report.drone_id = proto_report.drone_id();
    report.unix_time_ms = proto_report.unix_time_ms();
    report.sequence = proto_report.sequence();
    report.correlation_id = proto_report.correlation_id();
    report.type = ToAgentReportType(proto_report.type());
    report.severity = ToReportSeverity(proto_report.severity());
    report.message = proto_report.message();
    if (proto_report.has_goal()) {
        report.goal = ToGoalReport(proto_report.goal());
    }
    if (proto_report.has_trajectory()) {
        report.trajectory = ToTrajectoryReport(proto_report.trajectory());
    }
    if (proto_report.has_time_sync()) {
        report.time_sync = ToTimeSyncState(proto_report.time_sync());
    }
    return report;
}

[[nodiscard]] grpc::Status RunTelemetryStreamAttempt(ClientRuntime runtime,
                                                     StreamState& telemetry_stream,
                                                     const TelemetrySubscription& subscription,
                                                     const TelemetryHandler& on_frame,
                                                     std::string_view stream_id) {
    grpc::ClientContext* context = InstallStreamContext(telemetry_stream, stream_id);

    swarmkit::v1::TelemetryRequest request;
    request.set_drone_id(subscription.drone_id);
    request.set_rate_hz(subscription.rate_hertz);

    auto reader = runtime.stub.StreamTelemetry(context, request);
    EmitSubscriptionEvent(telemetry_stream, SubscriptionLifecycleState::kConnected,
                          "telemetry stream connected");
    swarmkit::v1::TelemetryFrame proto_frame;
    while (reader->Read(&proto_frame)) {
        if (IsStopRequested(telemetry_stream)) {
            break;
        }

        if (on_frame) {
            auto frame = ToCoreTelemetryFrame(proto_frame);
            static_cast<void>(EnqueueCallback(
                telemetry_stream, [on_frame, frame = std::move(frame)]() { on_frame(frame); }));
        }
    }

    const grpc::Status kFinalStatus = reader->Finish();
    ResetStreamContext(telemetry_stream);
    return kFinalStatus;
}

[[nodiscard]] grpc::Status RunReportStreamAttempt(ClientRuntime runtime, StreamState& report_stream,
                                                  const ReportSubscription& subscription,
                                                  const AgentReportHandler& on_report,
                                                  std::string_view stream_id) {
    grpc::ClientContext* context = InstallStreamContext(report_stream, stream_id);

    swarmkit::v1::ReportSubscription request;
    request.set_drone_id(subscription.drone_id);
    request.set_client_id(runtime.config.client_id);
    request.set_after_sequence(subscription.after_sequence);

    auto reader = runtime.stub.SubscribeReports(context, request);
    EmitSubscriptionEvent(report_stream, SubscriptionLifecycleState::kConnected,
                          "report stream connected");
    swarmkit::v1::AgentReport proto_report;
    while (reader->Read(&proto_report)) {
        if (IsStopRequested(report_stream)) {
            break;
        }

        if (on_report) {
            auto report = ToAgentReport(proto_report);
            static_cast<void>(EnqueueCallback(
                report_stream, [on_report, report = std::move(report)]() { on_report(report); }));
        }
    }

    const grpc::Status kFinalStatus = reader->Finish();
    ResetStreamContext(report_stream);
    return kFinalStatus;
}

[[nodiscard]] grpc::Status RunAuthorityStreamAttempt(ClientRuntime runtime,
                                                     StreamState& authority_stream,
                                                     const AuthoritySubscription& subscription,
                                                     const AuthorityEventHandler& on_event,
                                                     std::string_view stream_id) {
    grpc::ClientContext* context = InstallStreamContext(authority_stream, stream_id);

    swarmkit::v1::WatchAuthorityRequest request;
    request.set_drone_id(subscription.drone_id);
    request.set_client_id(runtime.config.client_id);
    request.set_priority(static_cast<std::int32_t>(subscription.priority));

    auto reader = runtime.stub.WatchAuthority(context, request);
    EmitSubscriptionEvent(authority_stream, SubscriptionLifecycleState::kConnected,
                          "authority watch connected");
    swarmkit::v1::AuthorityEvent proto_event;
    while (reader->Read(&proto_event)) {
        if (IsStopRequested(authority_stream)) {
            break;
        }

        if (on_event) {
            auto event = ToAuthorityEventInfo(proto_event);
            static_cast<void>(EnqueueCallback(
                authority_stream, [on_event, event = std::move(event)]() { on_event(event); }));
        }
    }

    const grpc::Status kFinalStatus = reader->Finish();
    ResetStreamContext(authority_stream);
    return kFinalStatus;
}

[[nodiscard]] RpcError MakeStreamError(const grpc::Status& status, std::string_view correlation_id,
                                       int attempt_number) {
    RpcError error;
    PopulateTransportError(&error, status, correlation_id, attempt_number);
    return error;
}

void RunTelemetryLoop(ClientRuntime runtime, StreamState& telemetry_stream,
                      const TelemetrySubscription& subscription, const TelemetryHandler& on_frame,
                      const TelemetryErrorHandler& on_error) {
    static_cast<void>(on_error);
    StreamRetryState retry_state = MakeStreamRetryState(runtime.config);

    while (!IsStopRequested(telemetry_stream)) {
        ++retry_state.attempt_number;
        const std::string kStreamId = MakeCorrelationId("telemetry");
        const grpc::Status kFinalStatus =
            RunTelemetryStreamAttempt(runtime, telemetry_stream, subscription, on_frame, kStreamId);

        if (IsStopRequested(telemetry_stream)) {
            break;
        }

        LogStreamFailure("telemetry", subscription.drone_id, kStreamId, kFinalStatus,
                         retry_state.attempt_number);
        if (kFinalStatus.ok()) {
            EmitSubscriptionEvent(telemetry_stream, SubscriptionLifecycleState::kStopped,
                                  "telemetry stream ended");
            break;
        }
        if (!ShouldRetryStream(runtime.config.stream_reconnect_policy, retry_state.attempt_number,
                               kFinalStatus)) {
            EmitSubscriptionEvent(
                telemetry_stream, SubscriptionLifecycleState::kFailed, kFinalStatus.error_message(),
                retry_state.attempt_number,
                MakeStreamError(kFinalStatus, kStreamId, retry_state.attempt_number));
            MaybeReportStreamFailure(telemetry_stream, kFinalStatus);
            break;
        }

        EmitSubscriptionEvent(telemetry_stream, SubscriptionLifecycleState::kReconnecting,
                              kFinalStatus.error_message(), retry_state.attempt_number,
                              MakeStreamError(kFinalStatus, kStreamId, retry_state.attempt_number));
        SleepBeforeNextRetry(runtime.config.stream_reconnect_policy, &retry_state.backoff_ms);
    }

    if (IsStopRequested(telemetry_stream)) {
        EmitSubscriptionEvent(telemetry_stream, SubscriptionLifecycleState::kStopped,
                              "telemetry stream stopped");
    }
    telemetry_stream.active.store(false, std::memory_order_relaxed);
    ShutdownCallbackQueue(telemetry_stream);
}

void RunAuthorityLoop(ClientRuntime runtime, StreamState& authority_stream,
                      const AuthoritySubscription& subscription,
                      const AuthorityEventHandler& on_event,
                      const TelemetryErrorHandler& on_error) {
    static_cast<void>(on_error);
    StreamRetryState retry_state = MakeStreamRetryState(runtime.config);

    while (!IsStopRequested(authority_stream)) {
        ++retry_state.attempt_number;
        const std::string kStreamId = MakeCorrelationId("authority");
        const grpc::Status kFinalStatus =
            RunAuthorityStreamAttempt(runtime, authority_stream, subscription, on_event, kStreamId);

        if (IsStopRequested(authority_stream)) {
            break;
        }

        LogStreamFailure("authority watch", subscription.drone_id, kStreamId, kFinalStatus,
                         retry_state.attempt_number);
        if (kFinalStatus.ok()) {
            EmitSubscriptionEvent(authority_stream, SubscriptionLifecycleState::kStopped,
                                  "authority watch ended");
            break;
        }
        if (!ShouldRetryStream(runtime.config.stream_reconnect_policy, retry_state.attempt_number,
                               kFinalStatus)) {
            EmitSubscriptionEvent(
                authority_stream, SubscriptionLifecycleState::kFailed, kFinalStatus.error_message(),
                retry_state.attempt_number,
                MakeStreamError(kFinalStatus, kStreamId, retry_state.attempt_number));
            MaybeReportStreamFailure(authority_stream, kFinalStatus);
            break;
        }

        EmitSubscriptionEvent(authority_stream, SubscriptionLifecycleState::kReconnecting,
                              kFinalStatus.error_message(), retry_state.attempt_number,
                              MakeStreamError(kFinalStatus, kStreamId, retry_state.attempt_number));
        SleepBeforeNextRetry(runtime.config.stream_reconnect_policy, &retry_state.backoff_ms);
    }

    if (IsStopRequested(authority_stream)) {
        EmitSubscriptionEvent(authority_stream, SubscriptionLifecycleState::kStopped,
                              "authority watch stopped");
    }
    authority_stream.active.store(false, std::memory_order_relaxed);
    ShutdownCallbackQueue(authority_stream);
}

void RunReportLoop(ClientRuntime runtime, StreamState& report_stream,
                   const ReportSubscription& subscription, const AgentReportHandler& on_report,
                   const TelemetryErrorHandler& on_error) {
    static_cast<void>(on_error);
    StreamRetryState retry_state = MakeStreamRetryState(runtime.config);

    while (!IsStopRequested(report_stream)) {
        ++retry_state.attempt_number;
        const std::string kStreamId = MakeCorrelationId("reports");
        const grpc::Status kFinalStatus =
            RunReportStreamAttempt(runtime, report_stream, subscription, on_report, kStreamId);

        if (IsStopRequested(report_stream)) {
            break;
        }

        LogStreamFailure("reports", subscription.drone_id, kStreamId, kFinalStatus,
                         retry_state.attempt_number);
        if (kFinalStatus.ok()) {
            EmitSubscriptionEvent(report_stream, SubscriptionLifecycleState::kStopped,
                                  "report stream ended");
            break;
        }
        if (!ShouldRetryStream(runtime.config.stream_reconnect_policy, retry_state.attempt_number,
                               kFinalStatus)) {
            EmitSubscriptionEvent(
                report_stream, SubscriptionLifecycleState::kFailed, kFinalStatus.error_message(),
                retry_state.attempt_number,
                MakeStreamError(kFinalStatus, kStreamId, retry_state.attempt_number));
            MaybeReportStreamFailure(report_stream, kFinalStatus);
            break;
        }

        EmitSubscriptionEvent(report_stream, SubscriptionLifecycleState::kReconnecting,
                              kFinalStatus.error_message(), retry_state.attempt_number,
                              MakeStreamError(kFinalStatus, kStreamId, retry_state.attempt_number));
        SleepBeforeNextRetry(runtime.config.stream_reconnect_policy, &retry_state.backoff_ms);
    }

    if (IsStopRequested(report_stream)) {
        EmitSubscriptionEvent(report_stream, SubscriptionLifecycleState::kStopped,
                              "report stream stopped");
    }
    report_stream.active.store(false, std::memory_order_relaxed);
    ShutdownCallbackQueue(report_stream);
}

core::TransportSecurityMode ClientSecurityConfig::EffectiveTransportSecurity() const {
    if (transport_security != core::TransportSecurityMode::kAuto) {
        return transport_security;
    }
    if (root_ca_cert_path.empty() && cert_chain_path.empty() && private_key_path.empty()) {
        return core::TransportSecurityMode::kInsecure;
    }
    if (!root_ca_cert_path.empty() && cert_chain_path.empty() && private_key_path.empty()) {
        return core::TransportSecurityMode::kTls;
    }
    return core::TransportSecurityMode::kMutualTls;
}

core::Result ClientSecurityConfig::Validate() const {
    const core::TransportSecurityMode mode = EffectiveTransportSecurity();
    if (mode == core::TransportSecurityMode::kInsecure) {
        return core::Result::Success();
    }
    if (!core::internal::FileExists(root_ca_cert_path)) {
        return core::Result::Rejected(
            "security.root_ca_cert_path must point to an existing file for TLS/mTLS");
    }
    if (mode == core::TransportSecurityMode::kMutualTls) {
        if (!core::internal::FileExists(cert_chain_path)) {
            return core::Result::Rejected(
                "security.cert_chain_path must point to an existing file for mTLS");
        }
        if (!core::internal::FileExists(private_key_path)) {
            return core::Result::Rejected(
                "security.private_key_path must point to an existing file for mTLS");
        }
    }
    return core::Result::Success();
}

core::Result ClientConfig::Validate() const {
    if (!LooksLikeAddress(address)) {
        return core::Result::Rejected("client address must be in host:port format");
    }
    if (client_id.empty()) {
        return core::Result::Rejected("client_id must not be empty");
    }
    if (deadline_ms < 0) {
        return core::Result::Rejected("deadline_ms must be >= 0");
    }
    if (!IsValidPriority(priority)) {
        return core::Result::Rejected("priority is not a supported CommandPriority");
    }
    if (retry_policy.max_attempts <= 0) {
        return core::Result::Rejected("retry_policy.max_attempts must be > 0");
    }
    if (retry_policy.initial_backoff_ms <= 0 || retry_policy.max_backoff_ms <= 0) {
        return core::Result::Rejected("retry_policy backoff values must be > 0");
    }
    if (retry_policy.initial_backoff_ms > retry_policy.max_backoff_ms) {
        return core::Result::Rejected(
            "retry_policy.initial_backoff_ms must be <= retry_policy.max_backoff_ms");
    }
    if (stream_reconnect_policy.initial_backoff_ms <= 0 ||
        stream_reconnect_policy.max_backoff_ms <= 0) {
        return core::Result::Rejected("stream_reconnect_policy backoff values must be > 0");
    }
    if (stream_reconnect_policy.initial_backoff_ms > stream_reconnect_policy.max_backoff_ms) {
        return core::Result::Rejected(
            "stream_reconnect_policy.initial_backoff_ms must be <= "
            "stream_reconnect_policy.max_backoff_ms");
    }
    if (stream_reconnect_policy.max_attempts < 0) {
        return core::Result::Rejected("stream_reconnect_policy.max_attempts must be >= 0");
    }
    return security.Validate();
}

void ClientConfig::ApplyEnvironment(std::string_view prefix) {
    const auto kApplyIntEnv = [&](std::string_view suffix, int* out) {
        const auto kValue = GetEnvValue(std::string(prefix) + std::string(suffix));
        if (!kValue.has_value()) {
            return;
        }
        const auto kParsed = ParseIntValue(*kValue, suffix);
        if (kParsed.has_value()) {
            *out = *kParsed;
        }
    };

    const auto kAddress = GetEnvValue(std::string(prefix) + std::string(kClientEnvAddress));
    if (kAddress.has_value()) {
        address = *kAddress;
    }

    const auto kClientId = GetEnvValue(std::string(prefix) + std::string(kClientEnvId));
    if (kClientId.has_value()) {
        client_id = *kClientId;
    }

    const auto kPriority = GetEnvValue(std::string(prefix) + std::string(kClientEnvPriority));
    if (kPriority.has_value()) {
        const auto kParsed = ParsePriorityValue(*kPriority, kClientEnvPriority);
        if (kParsed.has_value()) {
            priority = *kParsed;
        }
    }

    const auto kApplyBoolEnv = [&](std::string_view suffix, bool* out) {
        const auto kValue = GetEnvValue(std::string(prefix) + std::string(suffix));
        if (!kValue.has_value()) {
            return;
        }
        const auto kParsed = ParseBoolValue(*kValue, suffix);
        if (kParsed.has_value()) {
            *out = *kParsed;
        }
    };

    kApplyIntEnv(kClientEnvDeadlineMs, &deadline_ms);
    kApplyIntEnv(kClientEnvRetryMaxAttempts, &retry_policy.max_attempts);
    kApplyIntEnv(kClientEnvRetryInitialBackoffMs, &retry_policy.initial_backoff_ms);
    kApplyIntEnv(kClientEnvRetryMaxBackoffMs, &retry_policy.max_backoff_ms);
    kApplyBoolEnv(kClientEnvStreamReconnectEnabled, &stream_reconnect_policy.enabled);
    kApplyIntEnv(kClientEnvStreamReconnectInitialBackoffMs,
                 &stream_reconnect_policy.initial_backoff_ms);
    kApplyIntEnv(kClientEnvStreamReconnectMaxBackoffMs, &stream_reconnect_policy.max_backoff_ms);
    kApplyIntEnv(kClientEnvStreamReconnectMaxAttempts, &stream_reconnect_policy.max_attempts);

    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvRootCaCertPath));
        kValue.has_value()) {
        security.root_ca_cert_path = *kValue;
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvClientCertChainPath));
        kValue.has_value()) {
        security.cert_chain_path = *kValue;
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvClientPrivateKeyPath));
        kValue.has_value()) {
        security.private_key_path = *kValue;
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvServerAuthorityOverride));
        kValue.has_value()) {
        security.server_authority_override = *kValue;
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvTransportSecurity));
        kValue.has_value()) {
        const auto parsed = core::ParseTransportSecurityMode(*kValue);
        if (parsed.has_value()) {
            security.transport_security = *parsed;
        }
    }
}

Client::Client(ClientConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}

Client::~Client() {
    StopTelemetry();
    StopAuthorityWatch();
    StopReports();
}

AuthoritySession::~AuthoritySession() {
    Reset();
}

AuthoritySession::AuthoritySession(AuthoritySession&& other) noexcept
    : client_(std::exchange(other.client_, nullptr)), drone_id_(std::move(other.drone_id_)) {}

AuthoritySession& AuthoritySession::operator=(AuthoritySession&& other) noexcept {
    if (this == &other) {
        return *this;
    }

    Reset();
    client_ = std::exchange(other.client_, nullptr);
    drone_id_ = std::move(other.drone_id_);
    return *this;
}

ReleaseAuthorityResult AuthoritySession::Release() {
    const std::string correlation_id = MakeCorrelationId("unlock");
    if (client_ == nullptr) {
        ReleaseAuthorityResult out;
        out.ok = true;
        out.message = "authority session already inactive";
        out.correlation_id = correlation_id;
        PopulateSuccessError(&out.error, correlation_id, 0);
        return out;
    }

    ReleaseAuthorityResult result = client_->ReleaseAuthority(drone_id_);
    if (result.ok) {
        client_ = nullptr;
        drone_id_.clear();
    }
    return result;
}

void AuthoritySession::Reset() noexcept {
    if (client_ == nullptr) {
        return;
    }

    try {
        const ReleaseAuthorityResult result = client_->ReleaseAuthority(drone_id_);
        if (!result.ok) {
            core::Logger::WarnFmt(
                "AuthoritySession::Reset release failed for drone={} corr={} err={}", drone_id_,
                result.correlation_id, result.message);
        }
    } catch (const std::exception& exc) {
        core::Logger::WarnFmt("AuthoritySession::Reset failed for drone={}: {}", drone_id_,
                              exc.what());
    } catch (...) {
        core::Logger::WarnFmt("AuthoritySession::Reset failed for drone={}: unknown exception",
                              drone_id_);
    }
    client_ = nullptr;
    drone_id_.clear();
}

PingResult Client::Ping() const {
    PingResult out;
    const std::string kCorrelationId = MakeCorrelationId("ping");

    swarmkit::v1::PingRequest req;
    req.set_agent_id(impl_->config.client_id);

    swarmkit::v1::PingReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->Ping(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        out.ok = false;
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.error_message = out.error.user_message;
        return out;
    }

    out.ok = true;
    out.agent_id = rep.agent_id();
    out.version = rep.version();
    out.unix_time_ms = rep.unix_time_ms();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    PopulateSuccessError(&out.error, out.correlation_id, attempt_count);
    return out;
}

HealthStatus Client::GetHealth() const {
    HealthStatus out;
    const std::string kCorrelationId = MakeCorrelationId("health");

    swarmkit::v1::HealthRequest req;
    swarmkit::v1::HealthReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->GetHealth(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.ready = rep.ready();
    out.agent_id = rep.agent_id();
    out.version = rep.version();
    out.unix_time_ms = rep.unix_time_ms();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    out.backend_name = rep.backend_name();
    out.protocol = rep.protocol();
    out.last_heartbeat_unix_ms = rep.last_heartbeat_unix_ms();
    out.last_telemetry_unix_ms = rep.last_telemetry_unix_ms();
    out.armed = rep.armed();
    out.landed = rep.landed();
    out.mode = rep.mode();
    out.custom_mode = rep.custom_mode();
    out.failsafe = rep.failsafe();
    out.gps_ok = rep.gps_ok();
    out.gps_fix_type = rep.gps_fix_type();
    out.satellites_visible = rep.satellites_visible();
    out.gps_hdop = rep.gps_hdop();
    out.ekf_ok = rep.ekf_ok();
    out.has_relative_altitude = rep.has_relative_altitude();
    out.relative_alt_m = rep.relative_alt_m();
    if (rep.has_link_quality_percent()) {
        out.link_quality_percent = rep.link_quality_percent();
    }
    PopulateSuccessError(&out.error, out.correlation_id, attempt_count);
    return out;
}

RuntimeStats Client::GetRuntimeStats() const {
    RuntimeStats out;
    const std::string kCorrelationId = MakeCorrelationId("stats");

    swarmkit::v1::RuntimeStatsRequest req;
    swarmkit::v1::RuntimeStatsReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->GetRuntimeStats(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        return out;
    }

    out.ok = true;
    out.agent_id = rep.agent_id();
    out.unix_time_ms = rep.unix_time_ms();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    out.ping_requests_total = rep.ping_requests_total();
    out.health_requests_total = rep.health_requests_total();
    out.runtime_stats_requests_total = rep.runtime_stats_requests_total();
    out.command_requests_total = rep.command_requests_total();
    out.command_rejected_total = rep.command_rejected_total();
    out.command_failed_total = rep.command_failed_total();
    out.lock_requests_total = rep.lock_requests_total();
    out.watch_requests_total = rep.watch_requests_total();
    out.current_authority_watchers = rep.current_authority_watchers();
    out.total_telemetry_subscriptions = rep.total_telemetry_subscriptions();
    out.current_telemetry_streams = rep.current_telemetry_streams();
    out.telemetry_frames_sent_total = rep.telemetry_frames_sent_total();
    out.backend_failures_total = rep.backend_failures_total();
    out.ready = rep.ready();
    PopulateSuccessError(&out.error, out.correlation_id, attempt_count);
    return out;
}

BackendCapabilities Client::GetCapabilities() const {
    BackendCapabilities out;
    const std::string kCorrelationId = MakeCorrelationId("capabilities");

    swarmkit::v1::CapabilitiesRequest req;
    swarmkit::v1::CapabilitiesReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->GetCapabilities(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        return out;
    }

    out.ok = true;
    out.agent_id = rep.agent_id();
    out.unix_time_ms = rep.unix_time_ms();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    out.backend_name = rep.backend_name();
    out.protocol = rep.protocol();
    out.vehicle_class = rep.vehicle_class();
    out.supports_mission_upload = rep.supports_mission_upload();
    out.supports_payload_control = rep.supports_payload_control();
    out.supports_velocity_control = rep.supports_velocity_control();
    out.supports_flight_termination = rep.supports_flight_termination();
    out.supports_backend_commands = rep.supports_backend_commands();
    out.supports_time_sync = rep.supports_time_sync();
    out.supports_trajectory_upload = rep.supports_trajectory_upload();
    out.autopilot_type = rep.autopilot_type();
    out.supported_modes.assign(rep.supported_modes().begin(), rep.supported_modes().end());
    out.supported_commands.assign(rep.supported_commands().begin(), rep.supported_commands().end());
    out.supported_mission_items.assign(rep.supported_mission_items().begin(),
                                       rep.supported_mission_items().end());
    out.supported_payloads.assign(rep.supported_payloads().begin(), rep.supported_payloads().end());
    out.supported_telemetry_fields.assign(rep.supported_telemetry_fields().begin(),
                                          rep.supported_telemetry_fields().end());
    out.backend_command_names.assign(rep.backend_command_names().begin(),
                                     rep.backend_command_names().end());
    if (rep.has_max_horizontal_speed_mps()) {
        out.max_horizontal_speed_mps = rep.max_horizontal_speed_mps();
    }
    if (rep.has_max_climb_speed_mps()) {
        out.max_climb_speed_mps = rep.max_climb_speed_mps();
    }
    if (rep.has_max_descent_speed_mps()) {
        out.max_descent_speed_mps = rep.max_descent_speed_mps();
    }
    if (rep.has_max_altitude_m()) {
        out.max_altitude_m = rep.max_altitude_m();
    }
    PopulateSuccessError(&out.error, out.correlation_id, attempt_count);
    return out;
}

namespace {

enum class VerificationSource : std::uint8_t {
    kNone,
    kHealth,
    kTelemetry,
};

struct VerificationSpec {
    VerificationSource source{VerificationSource::kNone};
    std::string label;
    std::function<bool(const HealthStatus&)> health_predicate;
    std::function<bool(const core::TelemetryFrame&)> telemetry_predicate;
};

[[nodiscard]] CommandResult MakeVerifiedResult(const CommandResult& command_result,
                                               std::string_view label) {
    CommandResult out = command_result;
    out.ok = true;
    PopulateSuccessError(&out.error, command_result.correlation_id,
                         command_result.error.attempt_count);
    const std::string prefix = label.empty() ? "command" : std::string(label);
    out.message = command_result.message.empty()
                      ? prefix + " verified"
                      : command_result.message + "; " + prefix + " verified";
    return out;
}

[[nodiscard]] CommandResult MakeVerificationFailure(const CommandResult& command_result,
                                                    RpcStatusCode code, std::string message) {
    CommandResult out = command_result;
    out.ok = false;
    out.message = std::move(message);
    PopulateTypedError(&out.error, core::ErrorDomain::kCommand, code, out.message, out.message,
                       command_result.correlation_id, command_result.error.attempt_count);
    return out;
}

[[nodiscard]] VerificationSpec BuildVerificationSpec(const Command& command,
                                                     const CommandWaitOptions& options) {
    return std::visit(
        core::Overloaded{
            [&](const FlightCmd& flight) {
                return std::visit(
                    core::Overloaded{
                        [](const CmdArm&) {
                            return VerificationSpec{
                                .source = VerificationSource::kHealth,
                                .label = "arm",
                                .health_predicate =
                                    [](const HealthStatus& health) { return health.armed; },
                            };
                        },
                        [](const CmdForceArm&) {
                            return VerificationSpec{
                                .source = VerificationSource::kHealth,
                                .label = "force-arm",
                                .health_predicate =
                                    [](const HealthStatus& health) { return health.armed; },
                            };
                        },
                        [](const CmdDisarm&) {
                            return VerificationSpec{
                                .source = VerificationSource::kHealth,
                                .label = "disarm",
                                .health_predicate =
                                    [](const HealthStatus& health) { return !health.armed; },
                            };
                        },
                        [](const CmdForceDisarm&) {
                            return VerificationSpec{
                                .source = VerificationSource::kHealth,
                                .label = "force-disarm",
                                .health_predicate =
                                    [](const HealthStatus& health) { return !health.armed; },
                            };
                        },
                        [&](const CmdTakeoff& takeoff) {
                            return VerificationSpec{
                                .source = VerificationSource::kTelemetry,
                                .label = "takeoff",
                                .telemetry_predicate =
                                    [target_alt = static_cast<float>(takeoff.alt_m),
                                     tolerance = options.altitude_tolerance_m](
                                        const core::TelemetryFrame& frame) {
                                        return frame.HasRelativeAltitude() && frame.armed &&
                                               !frame.landed &&
                                               frame.rel_alt_m + tolerance >= target_alt;
                                    },
                            };
                        },
                        [](const CmdLand&) {
                            return VerificationSpec{
                                .source = VerificationSource::kHealth,
                                .label = "land",
                                .health_predicate =
                                    [](const HealthStatus& health) {
                                        return health.landed || !health.armed;
                                    },
                            };
                        },
                        [](const auto&) { return VerificationSpec{}; },
                    },
                    flight);
            },
            [&](const NavCmd& nav) {
                return std::visit(
                    core::Overloaded{
                        [&](const CmdSetWaypoint& waypoint) {
                            return VerificationSpec{
                                .source = VerificationSource::kTelemetry,
                                .label = "waypoint",
                                .telemetry_predicate =
                                    [waypoint, options](const core::TelemetryFrame& frame) {
                                        return frame.HasPosition() && frame.HasRelativeAltitude() &&
                                               DistanceMetres(frame.lat_deg, frame.lon_deg,
                                                              waypoint.lat_deg, waypoint.lon_deg) <=
                                                   options.position_radius_m &&
                                               std::abs(frame.rel_alt_m -
                                                        static_cast<float>(waypoint.alt_m)) <=
                                                   options.altitude_tolerance_m;
                                    },
                            };
                        },
                        [&](const CmdGoto& go_to) {
                            return VerificationSpec{
                                .source = VerificationSource::kTelemetry,
                                .label = "goto",
                                .telemetry_predicate =
                                    [go_to, options](const core::TelemetryFrame& frame) {
                                        return frame.HasPosition() && frame.HasRelativeAltitude() &&
                                               DistanceMetres(frame.lat_deg, frame.lon_deg,
                                                              go_to.lat_deg, go_to.lon_deg) <=
                                                   options.position_radius_m &&
                                               std::abs(frame.rel_alt_m -
                                                        static_cast<float>(go_to.alt_m)) <=
                                                   options.altitude_tolerance_m;
                                    },
                            };
                        },
                        [](const auto&) { return VerificationSpec{}; },
                    },
                    nav);
            },
            [](const auto&) { return VerificationSpec{}; },
        },
        command);
}

[[nodiscard]] CommandResult WaitForHealthVerification(const Client& client,
                                                      const CommandResult& command_result,
                                                      const CommandWaitOptions& options,
                                                      const VerificationSpec& spec) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, options.timeout_ms));
    const auto poll_interval = std::chrono::milliseconds(std::max(1, options.poll_interval_ms));
    HealthStatus last_health;
    do {
        last_health = client.GetHealth();
        if (last_health.ok && spec.health_predicate && spec.health_predicate(last_health)) {
            return MakeVerifiedResult(command_result, spec.label);
        }
        std::this_thread::sleep_for(poll_interval);
    } while (std::chrono::steady_clock::now() < deadline);

    std::string detail = spec.label + " verification timed out";
    if (!last_health.message.empty()) {
        detail += ": " + last_health.message;
    }
    return MakeVerificationFailure(command_result, RpcStatusCode::kDeadlineExceeded,
                                   std::move(detail));
}

[[nodiscard]] CommandResult WaitForTelemetryVerification(swarmkit::v1::AgentService::Stub& stub,
                                                         const std::string& drone_id,
                                                         const CommandResult& command_result,
                                                         const CommandWaitOptions& options,
                                                         const VerificationSpec& spec) {
    const std::string correlation_id = command_result.correlation_id.empty()
                                           ? MakeCorrelationId("verify")
                                           : command_result.correlation_id;
    grpc::ClientContext context;
    context.AddMetadata(std::string(kCorrelationMetadataKey), correlation_id);
    context.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::milliseconds(std::max(1, options.timeout_ms)));

    swarmkit::v1::TelemetryRequest request;
    request.set_drone_id(drone_id);
    request.set_rate_hz(std::max(1, options.telemetry_rate_hz));

    auto reader = stub.StreamTelemetry(&context, request);
    swarmkit::v1::TelemetryFrame proto_frame;
    while (reader->Read(&proto_frame)) {
        const core::TelemetryFrame frame = ToCoreTelemetryFrame(proto_frame);
        if (spec.telemetry_predicate && spec.telemetry_predicate(frame)) {
            context.TryCancel();
            static_cast<void>(reader->Finish());
            return MakeVerifiedResult(command_result, spec.label);
        }
    }

    const grpc::Status status = reader->Finish();
    if (!status.ok() && status.error_code() != grpc::StatusCode::DEADLINE_EXCEEDED &&
        !status.error_message().empty()) {
        return MakeVerificationFailure(
            command_result, ToRpcStatusCode(status),
            spec.label + " verification stream failed: " + status.error_message());
    }
    return MakeVerificationFailure(command_result, RpcStatusCode::kDeadlineExceeded,
                                   spec.label + " verification timed out");
}

[[nodiscard]] CommandEnvelope MakeClientCommandEnvelope(const ClientConfig& config,
                                                        std::string drone_id, Command command) {
    CommandEnvelope envelope;
    envelope.context.drone_id = std::move(drone_id);
    envelope.context.client_id = config.client_id;
    envelope.context.priority = config.priority;
    envelope.command = std::move(command);
    return envelope;
}

}  // namespace

CommandResult Client::SendCommand(const commands::CommandEnvelope& envelope) const {
    CommandResult out;

    swarmkit::v1::CommandRequest req;
    BuildProtoCommand(envelope, req);
    const std::string kCorrelationId =
        req.ctx().correlation_id().empty() ? MakeCorrelationId("cmd") : req.ctx().correlation_id();
    req.mutable_ctx()->set_correlation_id(kCorrelationId);

    swarmkit::v1::CommandReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->SendCommand(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = (rep.status() == swarmkit::v1::CommandReply::OK);
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count);
    return out;
}

CommandResult Client::SendCommandAndWait(const commands::CommandEnvelope& envelope,
                                         const CommandWaitOptions& options) const {
    CommandResult command_result = SendCommand(envelope);
    if (!command_result.ok) {
        return command_result;
    }

    const VerificationSpec spec = BuildVerificationSpec(envelope.command, options);
    switch (spec.source) {
        case VerificationSource::kNone:
            return command_result;
        case VerificationSource::kHealth:
            return WaitForHealthVerification(*this, command_result, options, spec);
        case VerificationSource::kTelemetry:
            return WaitForTelemetryVerification(*impl_->stub, envelope.context.drone_id,
                                                command_result, options, spec);
    }
    return command_result;
}

CommandResult Client::ArmAndWait(const std::string& drone_id,
                                 const CommandWaitOptions& options) const {
    return SendCommandAndWait(
        MakeClientCommandEnvelope(impl_->config, drone_id, FlightCmd{CmdArm{}}), options);
}

CommandResult Client::TakeoffAndWait(const std::string& drone_id, double alt_m,
                                     const CommandWaitOptions& options) const {
    return SendCommandAndWait(
        MakeClientCommandEnvelope(impl_->config, drone_id, FlightCmd{CmdTakeoff{.alt_m = alt_m}}),
        options);
}

CommandResult Client::GotoAndWait(const std::string& drone_id, double lat_deg, double lon_deg,
                                  double alt_m, float speed_mps,
                                  const CommandWaitOptions& options) const {
    return SendCommandAndWait(MakeClientCommandEnvelope(impl_->config, drone_id,
                                                        NavCmd{CmdGoto{
                                                            .lat_deg = lat_deg,
                                                            .lon_deg = lon_deg,
                                                            .alt_m = alt_m,
                                                            .speed_mps = speed_mps,
                                                        }}),
                              options);
}

CommandResult Client::LandAndWait(const std::string& drone_id,
                                  const CommandWaitOptions& options) const {
    return SendCommandAndWait(
        MakeClientCommandEnvelope(impl_->config, drone_id, FlightCmd{CmdLand{}}), options);
}

GoalResult Client::SetActiveGoal(const ActiveGoal& goal) const {
    GoalResult out;
    const std::string kCorrelationId = MakeCorrelationId("goal");

    swarmkit::v1::SetActiveGoalRequest req;
    auto* proto_ctx = req.mutable_ctx();
    proto_ctx->set_drone_id(goal.drone_id);
    proto_ctx->set_client_id(impl_->config.client_id);
    proto_ctx->set_priority(static_cast<std::int32_t>(impl_->config.priority));
    proto_ctx->set_correlation_id(kCorrelationId);
    PopulateProtoActiveGoal(goal, req.mutable_goal());

    swarmkit::v1::SetActiveGoalReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->SetActiveGoal(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count);
    if (rep.has_goal()) {
        out.goal = ToActiveGoal(rep.goal());
    }
    out.computed_timeout_ms = rep.computed_timeout_ms();
    return out;
}

CommandResult Client::CancelGoal(const std::string& drone_id, const std::string& goal_id) const {
    CommandResult out;
    const std::string kCorrelationId = MakeCorrelationId("cancel-goal");

    swarmkit::v1::CancelGoalRequest req;
    auto* proto_ctx = req.mutable_ctx();
    proto_ctx->set_drone_id(drone_id);
    proto_ctx->set_client_id(impl_->config.client_id);
    proto_ctx->set_priority(static_cast<std::int32_t>(impl_->config.priority));
    proto_ctx->set_correlation_id(kCorrelationId);
    req.set_goal_id(goal_id);

    swarmkit::v1::CancelGoalReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->CancelGoal(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count);
    return out;
}

ActiveGoalStatus Client::GetActiveGoal(const std::string& drone_id) const {
    ActiveGoalStatus out;
    const std::string kCorrelationId = MakeCorrelationId("get-goal");

    swarmkit::v1::GetActiveGoalRequest req;
    req.set_drone_id(drone_id);

    swarmkit::v1::GetActiveGoalReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->GetActiveGoal(context, req, &rep);
                             });

    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.has_goal = rep.goal_present();
    if (rep.has_goal()) {
        out.goal = ToActiveGoal(rep.goal());
    }
    out.status = ToGoalStatus(rep.status());
    out.computed_timeout_ms = rep.computed_timeout_ms();
    out.message = rep.message();
    PopulateSuccessError(&out.error, kCorrelationId, attempt_count);
    return out;
}

namespace {

[[nodiscard]] commands::CommandContext DefaultExecutionContext(const ClientConfig& config,
                                                               std::string drone_id) {
    commands::CommandContext context;
    context.drone_id = std::move(drone_id);
    context.client_id = config.client_id;
    context.priority = config.priority;
    return context;
}

void PopulateProtoCommandContext(const ClientConfig& config,
                                 const commands::CommandContext& context,
                                 std::string_view fallback_drone_id,
                                 std::string_view correlation_id,
                                 swarmkit::v1::CommandContext* proto_context) {
    if (proto_context == nullptr) {
        return;
    }
    proto_context->set_drone_id(context.drone_id.empty() ? std::string(fallback_drone_id)
                                                         : context.drone_id);
    proto_context->set_client_id(context.client_id.empty() ? config.client_id : context.client_id);
    proto_context->set_priority(static_cast<std::int32_t>(context.priority));
    proto_context->set_correlation_id(context.correlation_id.empty() ? std::string(correlation_id)
                                                                     : context.correlation_id);
    const auto epoch = std::chrono::system_clock::time_point{};
    if (context.deadline != epoch) {
        proto_context->set_deadline_unix_ms(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                context.deadline.time_since_epoch())
                                                .count());
    }
}

}  // namespace

ExecutionResult Client::UploadTrajectory(const TrajectoryPlan& plan) const {
    return UploadTrajectory(plan, DefaultExecutionContext(impl_->config, plan.drone_id));
}

ExecutionResult Client::UploadTrajectory(const TrajectoryPlan& plan,
                                         const commands::CommandContext& context) const {
    ExecutionResult out;
    const std::string kCorrelationId = MakeCorrelationId("trajectory-upload");
    swarmkit::v1::UploadTrajectoryRequest req;
    PopulateProtoCommandContext(impl_->config, context, plan.drone_id, kCorrelationId,
                                req.mutable_ctx());
    PopulateProtoTrajectoryPlan(plan, req.mutable_plan());

    swarmkit::v1::ExecutionReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->UploadTrajectory(context, req, &rep);
                             });
    out.correlation_id = kCorrelationId;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count);
    if (rep.has_handle()) {
        out.handle = ToExecutionHandle(rep.handle());
    }
    if (rep.has_validation()) {
        out.validation = ToValidateTrajectoryResult(rep.validation());
    }
    return out;
}

ExecutionResult Client::ValidateTrajectory(const TrajectoryPlan& plan) const {
    return ValidateTrajectory(plan, DefaultExecutionContext(impl_->config, plan.drone_id));
}

ExecutionResult Client::ValidateTrajectory(const TrajectoryPlan& plan,
                                           const commands::CommandContext& context) const {
    ExecutionResult out;
    const std::string kCorrelationId = MakeCorrelationId("trajectory-validate");
    swarmkit::v1::ValidateTrajectoryRequest req;
    PopulateProtoCommandContext(impl_->config, context, plan.drone_id, kCorrelationId,
                                req.mutable_ctx());
    PopulateProtoTrajectoryPlan(plan, req.mutable_plan());

    swarmkit::v1::ValidateTrajectoryReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->ValidateTrajectory(context, req, &rep);
                             });
    out.correlation_id = kCorrelationId;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kValidation);
    if (rep.has_validation()) {
        out.validation = ToValidateTrajectoryResult(rep.validation());
    }
    return out;
}

namespace {

template <typename Call>
[[nodiscard]] ExecutionResult InvokeExecutionMutation(const ClientConfig& config,
                                                      std::string_view prefix,
                                                      const std::string& drone_id,
                                                      const std::string& execution_id,
                                                      Call&& call) {
    ExecutionResult out;
    const std::string correlation_id = MakeCorrelationId(prefix);
    swarmkit::v1::ExecutionReply rep;
    int attempt_count = 0;
    const grpc::Status status = InvokeUnaryWithRetry(
        config, correlation_id, &attempt_count, [&](grpc::ClientContext* context) {
            return call(context, rep, std::string_view{correlation_id});
        });
    out.correlation_id = correlation_id;
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    static_cast<void>(drone_id);
    static_cast<void>(execution_id);
    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? correlation_id : rep.correlation_id();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count);
    if (rep.has_handle()) {
        out.handle = ToExecutionHandle(rep.handle());
    }
    if (rep.has_validation()) {
        out.validation = ToValidateTrajectoryResult(rep.validation());
    }
    return out;
}

}  // namespace

ExecutionResult Client::ClearTrajectory(const std::string& drone_id,
                                        const std::string& execution_id) const {
    return InvokeExecutionMutation(
        impl_->config, "trajectory-clear", drone_id, execution_id,
        [this, &drone_id, &execution_id](grpc::ClientContext* context,
                                         swarmkit::v1::ExecutionReply& rep,
                                         std::string_view correlation_id) {
            swarmkit::v1::ExecutionRequest req;
            PopulateProtoCommandContext(impl_->config,
                                        DefaultExecutionContext(impl_->config, drone_id), drone_id,
                                        correlation_id, req.mutable_ctx());
            req.set_execution_id(execution_id);
            return impl_->stub->ClearTrajectory(context, req, &rep);
        });
}

ExecutionResult Client::PrepareTrajectory(const std::string& drone_id,
                                          const std::string& execution_id) const {
    return PrepareTrajectory(drone_id, execution_id,
                             DefaultExecutionContext(impl_->config, drone_id));
}

ExecutionResult Client::PrepareTrajectory(const std::string& drone_id,
                                          const std::string& execution_id,
                                          const commands::CommandContext& command_context) const {
    return InvokeExecutionMutation(
        impl_->config, "trajectory-prepare", drone_id, execution_id,
        [this, &drone_id, &execution_id, &command_context](grpc::ClientContext* context,
                                                           swarmkit::v1::ExecutionReply& rep,
                                                           std::string_view correlation_id) {
            swarmkit::v1::ExecutionRequest req;
            PopulateProtoCommandContext(impl_->config, command_context, drone_id, correlation_id,
                                        req.mutable_ctx());
            req.set_execution_id(execution_id);
            return impl_->stub->PrepareTrajectory(context, req, &rep);
        });
}

ExecutionResult Client::StartExecutionAt(const std::string& drone_id,
                                         const std::string& execution_id,
                                         std::int64_t unix_time_ms) const {
    return StartExecutionAt(drone_id, execution_id, unix_time_ms,
                            DefaultExecutionContext(impl_->config, drone_id));
}

ExecutionResult Client::StartExecutionAt(const std::string& drone_id,
                                         const std::string& execution_id, std::int64_t unix_time_ms,
                                         const commands::CommandContext& command_context) const {
    return InvokeExecutionMutation(
        impl_->config, "execution-start", drone_id, execution_id,
        [this, &drone_id, &execution_id, unix_time_ms, &command_context](
            grpc::ClientContext* context, swarmkit::v1::ExecutionReply& rep,
            std::string_view correlation_id) {
            swarmkit::v1::StartExecutionAtRequest req;
            PopulateProtoCommandContext(impl_->config, command_context, drone_id, correlation_id,
                                        req.mutable_ctx());
            req.set_execution_id(execution_id);
            req.set_unix_time_ms(unix_time_ms);
            return impl_->stub->StartExecutionAt(context, req, &rep);
        });
}

ExecutionResult Client::PauseExecution(const std::string& drone_id,
                                       const std::string& execution_id) const {
    return InvokeExecutionMutation(
        impl_->config, "execution-pause", drone_id, execution_id,
        [this, &drone_id, &execution_id](grpc::ClientContext* context,
                                         swarmkit::v1::ExecutionReply& rep,
                                         std::string_view correlation_id) {
            swarmkit::v1::ExecutionRequest req;
            PopulateProtoCommandContext(impl_->config,
                                        DefaultExecutionContext(impl_->config, drone_id), drone_id,
                                        correlation_id, req.mutable_ctx());
            req.set_execution_id(execution_id);
            return impl_->stub->PauseExecution(context, req, &rep);
        });
}

ExecutionResult Client::ResumeExecution(const std::string& drone_id,
                                        const std::string& execution_id) const {
    return InvokeExecutionMutation(
        impl_->config, "execution-resume", drone_id, execution_id,
        [this, &drone_id, &execution_id](grpc::ClientContext* context,
                                         swarmkit::v1::ExecutionReply& rep,
                                         std::string_view correlation_id) {
            swarmkit::v1::ExecutionRequest req;
            PopulateProtoCommandContext(impl_->config,
                                        DefaultExecutionContext(impl_->config, drone_id), drone_id,
                                        correlation_id, req.mutable_ctx());
            req.set_execution_id(execution_id);
            return impl_->stub->ResumeExecution(context, req, &rep);
        });
}

ExecutionResult Client::AbortExecution(const std::string& drone_id,
                                       const std::string& execution_id) const {
    return InvokeExecutionMutation(
        impl_->config, "execution-abort", drone_id, execution_id,
        [this, &drone_id, &execution_id](grpc::ClientContext* context,
                                         swarmkit::v1::ExecutionReply& rep,
                                         std::string_view correlation_id) {
            swarmkit::v1::ExecutionRequest req;
            PopulateProtoCommandContext(impl_->config,
                                        DefaultExecutionContext(impl_->config, drone_id), drone_id,
                                        correlation_id, req.mutable_ctx());
            req.set_execution_id(execution_id);
            return impl_->stub->AbortExecution(context, req, &rep);
        });
}

TrajectoryStatus Client::GetExecution(const std::string& drone_id,
                                      const std::string& execution_id) const {
    TrajectoryStatus out;
    const std::string correlation_id = MakeCorrelationId("execution-get");
    swarmkit::v1::GetExecutionRequest req;
    req.set_drone_id(drone_id);
    req.set_execution_id(execution_id);
    swarmkit::v1::GetExecutionReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->GetExecution(context, req, &rep);
                             });
    if (!status.ok()) {
        PopulateTransportError(&out.error, status, correlation_id, attempt_count);
        out.message = out.error.user_message;
        return out;
    }
    out.found = rep.found();
    out.message = rep.message();
    if (rep.has_handle()) {
        out.handle = ToExecutionHandle(rep.handle());
    }
    if (rep.has_plan()) {
        out.plan = ToTrajectoryPlan(rep.plan());
    }
    PopulateSuccessError(&out.error, correlation_id, attempt_count);
    return out;
}

std::vector<ExecutionHandle> Client::ListExecutions(const std::string& drone_id) const {
    const std::string correlation_id = MakeCorrelationId("execution-list");
    swarmkit::v1::ListExecutionsRequest req;
    req.set_drone_id(drone_id);
    swarmkit::v1::ListExecutionsReply rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->ListExecutions(context, req, &rep);
                             });
    std::vector<ExecutionHandle> out;
    if (!status.ok()) {
        return out;
    }
    for (const auto& handle : rep.executions()) {
        out.push_back(ToExecutionHandle(handle));
    }
    return out;
}

TimeSyncState Client::GetTimeSyncState(const std::string& drone_id) const {
    const std::string correlation_id = MakeCorrelationId("time-sync");
    swarmkit::v1::TimeSyncRequest req;
    req.set_drone_id(drone_id);
    swarmkit::v1::TimeSyncState rep;
    int attempt_count = 0;
    const grpc::Status status =
        InvokeUnaryWithRetry(impl_->config, correlation_id, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->GetTimeSyncState(context, req, &rep);
                             });
    if (!status.ok()) {
        return {
            .drone_id = drone_id,
            .agent_unix_time_ms = 0,
            .vehicle_unix_time_ms = 0,
            .clock_offset_ms = 0,
            .sync_quality_percent = 0.0F,
            .synced = false,
            .stale = true,
            .source = "unavailable",
            .message = status.error_message(),
        };
    }
    return ToTimeSyncState(rep);
}

namespace {

[[nodiscard]] RpcError MakeSubscriptionStartError(SubscriptionKind kind, RpcStatusCode code,
                                                  std::string message) {
    RpcError error = core::SwarmError::Make(DomainForSubscriptionKind(kind), code,
                                            std::move(message), SeverityForCode(code),
                                            RetryabilityForCode(code), RemediationForCode(code));
    error.debug_message = error.user_message;
    return error;
}

[[nodiscard]] std::expected<std::uint64_t, RpcError> PrepareSubscriptionStart(
    const std::shared_ptr<StreamState>& stream, SubscriptionKind kind, std::string drone_id,
    TelemetryErrorHandler on_error, SubscriptionEventHandler on_event,
    SubscriptionOptions options) {
    if (options.backpressure.max_pending_callbacks == 0) {
        return std::unexpected(MakeSubscriptionStartError(
            kind, RpcStatusCode::kInvalidArgument,
            "subscription max_pending_callbacks must be greater than zero"));
    }

    if (!options.replace_existing &&
        (stream->active.load(std::memory_order_relaxed) || stream->worker.joinable() ||
         stream->callback_worker.joinable())) {
        return std::unexpected(MakeSubscriptionStartError(kind, RpcStatusCode::kAlreadyExists,
                                                          "subscription already active"));
    }

    CancelAndJoinStream(*stream);
    ResetCallbackQueue(*stream);

    stream->kind = kind;
    stream->drone_id = std::move(drone_id);
    stream->correlation_id = MakeCorrelationId(std::string(ToString(kind)));
    stream->options = options;
    stream->on_error = std::move(on_error);
    stream->on_event = std::move(on_event);
    stream->stop_requested.store(false, std::memory_order_relaxed);
    stream->active.store(true, std::memory_order_relaxed);
    const std::uint64_t generation =
        stream->generation.fetch_add(1, std::memory_order_relaxed) + 1U;

    try {
        stream->callback_worker = std::thread([stream] { RunCallbackDispatcher(*stream); });
    } catch (const std::system_error& exc) {
        stream->active.store(false, std::memory_order_relaxed);
        return std::unexpected(MakeSubscriptionStartError(
            kind, RpcStatusCode::kInternal,
            "failed to start subscription callback dispatcher: " + std::string(exc.what())));
    }

    EmitSubscriptionEvent(*stream, SubscriptionLifecycleState::kStarting,
                          std::string(ToString(kind)) + " subscription starting");

    return generation;
}

}  // namespace

std::expected<Subscription, RpcError> Client::StartTelemetry(TelemetrySubscription subscription,
                                                             TelemetryHandler on_frame,
                                                             TelemetryErrorHandler on_error,
                                                             SubscriptionEventHandler on_event,
                                                             SubscriptionOptions options) {
    if (subscription.drone_id.empty()) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, RpcStatusCode::kInvalidArgument,
            "telemetry subscription drone_id must not be empty"));
    }
    if (subscription.rate_hertz <= 0) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, RpcStatusCode::kInvalidArgument,
            "telemetry subscription rate_hertz must be greater than zero"));
    }
    if (!on_frame) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, RpcStatusCode::kInvalidArgument,
            "telemetry subscription frame callback must not be empty"));
    }

    auto generation = PrepareSubscriptionStart(impl_->telemetry, SubscriptionKind::kTelemetry,
                                               subscription.drone_id, std::move(on_error),
                                               std::move(on_event), options);
    if (!generation.has_value()) {
        return std::unexpected(std::move(generation.error()));
    }

    try {
        const auto stream = impl_->telemetry;
        stream->worker =
            std::thread([this, stream, subscription = std::move(subscription),
                         on_frame = std::move(on_frame), on_error = stream->on_error]() mutable {
                RunTelemetryLoop(ClientRuntime{.config = impl_->config, .stub = *impl_->stub},
                                 *stream, subscription, on_frame, on_error);
            });
    } catch (const std::system_error& exc) {
        CancelAndJoinStream(*impl_->telemetry);
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, RpcStatusCode::kInternal,
            "failed to start telemetry subscription worker: " + std::string(exc.what())));
    }

    auto state = std::make_shared<Subscription::State>();
    state->stream = impl_->telemetry;
    state->kind = SubscriptionKind::kTelemetry;
    state->generation = *generation;
    return Subscription(std::move(state));
}

void Client::StopTelemetry() {
    CancelAndJoinStream(*impl_->telemetry);
}

std::expected<Subscription, RpcError> Client::StartAuthorityWatch(
    AuthoritySubscription subscription, AuthorityEventHandler on_event,
    TelemetryErrorHandler on_error, SubscriptionEventHandler on_state,
    SubscriptionOptions options) {
    if (subscription.drone_id.empty()) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kAuthority, RpcStatusCode::kInvalidArgument,
            "authority subscription drone_id must not be empty"));
    }
    if (!on_event) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kAuthority, RpcStatusCode::kInvalidArgument,
            "authority subscription event callback must not be empty"));
    }

    auto generation = PrepareSubscriptionStart(impl_->authority, SubscriptionKind::kAuthority,
                                               subscription.drone_id, std::move(on_error),
                                               std::move(on_state), options);
    if (!generation.has_value()) {
        return std::unexpected(std::move(generation.error()));
    }

    try {
        const auto stream = impl_->authority;
        stream->worker =
            std::thread([this, stream, subscription = std::move(subscription),
                         on_event = std::move(on_event), on_error = stream->on_error]() mutable {
                RunAuthorityLoop(ClientRuntime{.config = impl_->config, .stub = *impl_->stub},
                                 *stream, subscription, on_event, on_error);
            });
    } catch (const std::system_error& exc) {
        CancelAndJoinStream(*impl_->authority);
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kAuthority, RpcStatusCode::kInternal,
            "failed to start authority subscription worker: " + std::string(exc.what())));
    }

    auto state = std::make_shared<Subscription::State>();
    state->stream = impl_->authority;
    state->kind = SubscriptionKind::kAuthority;
    state->generation = *generation;
    return Subscription(std::move(state));
}

void Client::StopAuthorityWatch() {
    CancelAndJoinStream(*impl_->authority);
}

std::expected<Subscription, RpcError> Client::StartReports(ReportSubscription subscription,
                                                           AgentReportHandler on_report,
                                                           TelemetryErrorHandler on_error,
                                                           SubscriptionEventHandler on_event,
                                                           SubscriptionOptions options) {
    if (subscription.drone_id.empty()) {
        return std::unexpected(
            MakeSubscriptionStartError(SubscriptionKind::kReports, RpcStatusCode::kInvalidArgument,
                                       "report subscription drone_id must not be empty"));
    }
    if (!on_report) {
        return std::unexpected(
            MakeSubscriptionStartError(SubscriptionKind::kReports, RpcStatusCode::kInvalidArgument,
                                       "report subscription callback must not be empty"));
    }

    auto generation = PrepareSubscriptionStart(impl_->reports, SubscriptionKind::kReports,
                                               subscription.drone_id, std::move(on_error),
                                               std::move(on_event), options);
    if (!generation.has_value()) {
        return std::unexpected(std::move(generation.error()));
    }

    try {
        const auto stream = impl_->reports;
        stream->worker =
            std::thread([this, stream, subscription = std::move(subscription),
                         on_report = std::move(on_report), on_error = stream->on_error]() mutable {
                RunReportLoop(ClientRuntime{.config = impl_->config, .stub = *impl_->stub}, *stream,
                              subscription, on_report, on_error);
            });
    } catch (const std::system_error& exc) {
        CancelAndJoinStream(*impl_->reports);
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kReports, RpcStatusCode::kInternal,
            "failed to start report subscription worker: " + std::string(exc.what())));
    }

    auto state = std::make_shared<Subscription::State>();
    state->stream = impl_->reports;
    state->kind = SubscriptionKind::kReports;
    state->generation = *generation;
    return Subscription(std::move(state));
}

void Client::StopReports() {
    CancelAndJoinStream(*impl_->reports);
}

CommandResult Client::LockAuthority(const std::string& drone_id, std::int64_t ttl_ms) const {
    CommandResult out;
    const std::string kCorrelationId = MakeCorrelationId("lock");

    swarmkit::v1::LockAuthorityRequest req;
    auto* proto_ctx = req.mutable_ctx();
    proto_ctx->set_drone_id(drone_id);
    proto_ctx->set_client_id(impl_->config.client_id);
    proto_ctx->set_priority(static_cast<std::int32_t>(impl_->config.priority));
    proto_ctx->set_correlation_id(kCorrelationId);
    req.set_ttl_ms(ttl_ms);

    swarmkit::v1::LockAuthorityReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->LockAuthority(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       out.correlation_id, attempt_count, core::ErrorDomain::kAuthority);
    return out;
}

std::expected<AuthoritySession, CommandResult> Client::AcquireAuthoritySession(
    const std::string& drone_id, std::int64_t ttl_ms) const {
    CommandResult result = LockAuthority(drone_id, ttl_ms);
    if (!result.ok) {
        return std::unexpected(std::move(result));
    }
    return AuthoritySession(this, drone_id);
}

ReleaseAuthorityResult Client::ReleaseAuthority(const std::string& drone_id) const {
    ReleaseAuthorityResult out;
    const std::string kCorrelationId = MakeCorrelationId("unlock");

    swarmkit::v1::ReleaseAuthorityRequest req;
    req.set_drone_id(drone_id);
    req.set_client_id(impl_->config.client_id);

    swarmkit::v1::ReleaseAuthorityReply rep;
    int attempt_count = 0;
    const grpc::Status kStatus =
        InvokeUnaryWithRetry(impl_->config, kCorrelationId, &attempt_count,
                             [this, &req, &rep](grpc::ClientContext* context) {
                                 return impl_->stub->ReleaseAuthority(context, req, &rep);
                             });

    out.correlation_id = kCorrelationId;
    if (!kStatus.ok()) {
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        core::Logger::WarnFmt(
            "Client::ReleaseAuthority failed: drone={} corr={} attempts={} err={}", drone_id,
            kCorrelationId, attempt_count, out.message);
        return out;
    }
    out.ok = true;
    out.message = "authority release acknowledged";
    PopulateSuccessError(&out.error, kCorrelationId, attempt_count);
    return out;
}

}  // namespace swarmkit::client
