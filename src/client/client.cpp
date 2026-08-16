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
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>

#include "env_utils.h"
#include "proto_execution.h"
#include "security_utils.h"
#include "sha256.h"
#include "swarmkit/client/telemetry_codec.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/overloaded.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::client {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)
using core::internal::MakeCorrelationId;

namespace {

constexpr std::string_view kCorrelationMetadataKey = "x-correlation-id";
constexpr double kEarthRadiusM = 6371000.0;
constexpr std::size_t kDefaultArtifactChunkBytes = std::size_t{64} * std::size_t{1024};

[[nodiscard]] std::int64_t NowUnixMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

[[nodiscard]] bool WriteBytes(std::ostream& output, std::span<const char> bytes) {
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return output.good();
}

[[nodiscard]] core::ErrorCode ToErrorCode(const grpc::Status& status) {
    if (status.ok()) {
        return core::ErrorCode::kOk;
    }

    switch (status.error_code()) {
        case grpc::StatusCode::INVALID_ARGUMENT:
            return core::ErrorCode::kInvalidArgument;
        case grpc::StatusCode::PERMISSION_DENIED:
            return core::ErrorCode::kPermissionDenied;
        case grpc::StatusCode::NOT_FOUND:
            return core::ErrorCode::kNotFound;
        case grpc::StatusCode::ALREADY_EXISTS:
            return core::ErrorCode::kAlreadyExists;
        case grpc::StatusCode::FAILED_PRECONDITION:
            return core::ErrorCode::kFailedPrecondition;
        case grpc::StatusCode::UNIMPLEMENTED:
            return core::ErrorCode::kUnsupported;
        case grpc::StatusCode::UNAVAILABLE:
            return core::ErrorCode::kUnavailable;
        case grpc::StatusCode::DEADLINE_EXCEEDED:
            return core::ErrorCode::kDeadlineExceeded;
        case grpc::StatusCode::CANCELLED:
            return core::ErrorCode::kCancelled;
        case grpc::StatusCode::INTERNAL:
            return core::ErrorCode::kInternal;
        default:
            return core::ErrorCode::kUnknown;
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

[[nodiscard]] core::ErrorCode ToErrorCode(swarmkit::v1::ErrorCode code) {
    using ProtoCode = swarmkit::v1::ErrorCode;
    switch (code) {
        case ProtoCode::ErrorCode_INT_MIN_SENTINEL_DO_NOT_USE_:
        case ProtoCode::ErrorCode_INT_MAX_SENTINEL_DO_NOT_USE_:
            return core::ErrorCode::kUnknown;
        case ProtoCode::ERROR_CODE_NONE:
            return core::ErrorCode::kOk;
        case ProtoCode::ERROR_CODE_INVALID_ARGUMENT:
            return core::ErrorCode::kInvalidArgument;
        case ProtoCode::ERROR_CODE_REJECTED:
            return core::ErrorCode::kRejected;
        case ProtoCode::ERROR_CODE_PERMISSION_DENIED:
            return core::ErrorCode::kPermissionDenied;
        case ProtoCode::ERROR_CODE_NOT_FOUND:
            return core::ErrorCode::kNotFound;
        case ProtoCode::ERROR_CODE_ALREADY_EXISTS:
            return core::ErrorCode::kAlreadyExists;
        case ProtoCode::ERROR_CODE_FAILED_PRECONDITION:
            return core::ErrorCode::kFailedPrecondition;
        case ProtoCode::ERROR_CODE_UNSUPPORTED:
            return core::ErrorCode::kUnsupported;
        case ProtoCode::ERROR_CODE_UNAVAILABLE:
            return core::ErrorCode::kUnavailable;
        case ProtoCode::ERROR_CODE_DEADLINE_EXCEEDED:
            return core::ErrorCode::kDeadlineExceeded;
        case ProtoCode::ERROR_CODE_CANCELLED:
            return core::ErrorCode::kCancelled;
        case ProtoCode::ERROR_CODE_INTERNAL:
            return core::ErrorCode::kInternal;
        case ProtoCode::ERROR_CODE_BACKEND_FAILURE:
            return core::ErrorCode::kBackendFailure;
        case ProtoCode::ERROR_CODE_UNSPECIFIED:
        case ProtoCode::ERROR_CODE_UNKNOWN:
            return core::ErrorCode::kUnknown;
    }
    return core::ErrorCode::kUnknown;
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

[[nodiscard]] core::ErrorSeverity SeverityForCode(core::ErrorCode code);
[[nodiscard]] core::ErrorRetryability RetryabilityForCode(core::ErrorCode code);
[[nodiscard]] std::string RemediationForCode(core::ErrorCode code);

void PopulateTransportError(core::SwarmError* error, const grpc::Status& status,
                            std::string_view correlation_id, int attempt_count) {
    if (error == nullptr) {
        return;
    }

    const core::ErrorCode code = ToErrorCode(status);
    *error = core::SwarmError::Make(core::ErrorDomain::kTransport, code, status.error_message(),
                                    SeverityForCode(code), RetryabilityForCode(code),
                                    RemediationForCode(code));
    error->debug_message = status.error_details();
    error->correlation_id = std::string(correlation_id);
    error->attempt_count = attempt_count;
    error->details["grpc_status_code"] = std::to_string(static_cast<int>(status.error_code()));
}

[[nodiscard]] core::ErrorSeverity SeverityForCode(core::ErrorCode code) {
    switch (code) {
        case core::ErrorCode::kOk:
            return core::ErrorSeverity::kInfo;
        case core::ErrorCode::kInvalidArgument:
        case core::ErrorCode::kRejected:
        case core::ErrorCode::kPermissionDenied:
        case core::ErrorCode::kNotFound:
        case core::ErrorCode::kAlreadyExists:
        case core::ErrorCode::kFailedPrecondition:
        case core::ErrorCode::kUnsupported:
            return core::ErrorSeverity::kWarning;
        case core::ErrorCode::kUnavailable:
        case core::ErrorCode::kDeadlineExceeded:
        case core::ErrorCode::kCancelled:
        case core::ErrorCode::kBackendFailure:
            return core::ErrorSeverity::kError;
        case core::ErrorCode::kInternal:
        case core::ErrorCode::kUnknown:
            return core::ErrorSeverity::kCritical;
    }
    return core::ErrorSeverity::kError;
}

[[nodiscard]] core::ErrorRetryability RetryabilityForCode(core::ErrorCode code) {
    switch (code) {
        case core::ErrorCode::kUnavailable:
        case core::ErrorCode::kDeadlineExceeded:
        case core::ErrorCode::kCancelled:
            return core::ErrorRetryability::kAfterBackoff;
        case core::ErrorCode::kInvalidArgument:
        case core::ErrorCode::kRejected:
        case core::ErrorCode::kPermissionDenied:
        case core::ErrorCode::kNotFound:
        case core::ErrorCode::kAlreadyExists:
        case core::ErrorCode::kFailedPrecondition:
        case core::ErrorCode::kUnsupported:
            return core::ErrorRetryability::kAfterRemediation;
        case core::ErrorCode::kInternal:
        case core::ErrorCode::kBackendFailure:
        case core::ErrorCode::kUnknown:
            return core::ErrorRetryability::kUnknown;
        case core::ErrorCode::kOk:
            return core::ErrorRetryability::kNever;
    }
    return core::ErrorRetryability::kUnknown;
}

[[nodiscard]] std::string RemediationForCode(core::ErrorCode code) {
    switch (code) {
        case core::ErrorCode::kInvalidArgument:
            return "Fix the request parameters before retrying";
        case core::ErrorCode::kRejected:
        case core::ErrorCode::kFailedPrecondition:
            return "Check authority, vehicle state, and command preconditions";
        case core::ErrorCode::kPermissionDenied:
            return "Check client identity, certificates, and allowed_client_ids";
        case core::ErrorCode::kNotFound:
            return "Verify the drone, goal, or command identifier";
        case core::ErrorCode::kAlreadyExists:
            return "Use a unique identifier or update the existing resource";
        case core::ErrorCode::kUnsupported:
            return "Check backend capabilities before sending this operation";
        case core::ErrorCode::kUnavailable:
            return "Check agent/backend availability and retry with backoff";
        case core::ErrorCode::kDeadlineExceeded:
            return "Increase the deadline or retry when the vehicle is responsive";
        case core::ErrorCode::kCancelled:
            return "Retry if the operation was not intentionally cancelled";
        case core::ErrorCode::kBackendFailure:
            return "Inspect backend/autopilot diagnostics before retrying";
        case core::ErrorCode::kInternal:
        case core::ErrorCode::kUnknown:
            return "Inspect logs with the correlation ID and report the failure";
        case core::ErrorCode::kOk:
            return {};
    }
    return {};
}

[[nodiscard]] core::ErrorDomain DomainForReply(core::ErrorCode code,
                                               core::ErrorDomain requested_domain) {
    if (code == core::ErrorCode::kOk) {
        return core::ErrorDomain::kNone;
    }
    if (code == core::ErrorCode::kBackendFailure) {
        return core::ErrorDomain::kBackend;
    }
    if (code == core::ErrorCode::kInvalidArgument) {
        return core::ErrorDomain::kValidation;
    }
    return requested_domain;
}

void PopulateSuccessError(core::SwarmError* error, std::string_view correlation_id,
                          int attempt_count) {
    if (error == nullptr) {
        return;
    }
    *error = core::SwarmError::Ok();
    error->correlation_id = std::string(correlation_id);
    error->attempt_count = attempt_count;
}

void PopulateTypedError(core::SwarmError* error, core::ErrorDomain domain, core::ErrorCode code,
                        std::string user_message, std::string debug_message,
                        std::string_view correlation_id, int attempt_count) {
    if (error == nullptr) {
        return;
    }
    if (code == core::ErrorCode::kOk) {
        PopulateSuccessError(error, correlation_id, attempt_count);
        return;
    }

    *error = core::SwarmError::Make(domain, code, std::move(user_message), SeverityForCode(code),
                                    RetryabilityForCode(code), RemediationForCode(code));
    error->debug_message = std::move(debug_message);
    error->correlation_id = std::string(correlation_id);
    error->attempt_count = attempt_count;
}

void PopulateReplyError(core::SwarmError* error, swarmkit::v1::ErrorCode proto_code,
                        std::string user_message, std::string debug_message,
                        std::string_view correlation_id, int attempt_count,
                        core::ErrorDomain domain = core::ErrorDomain::kCommand) {
    const core::ErrorCode code = ToErrorCode(proto_code);
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
        case ProtoKind::AuthorityEvent_Kind_RELEASED:
            return AuthorityEventKind::kReleased;
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
    if (envelope.context.execution_context.has_value()) {
        core::internal::PopulateExecutionContext(*envelope.context.execution_context,
                                                 proto_ctx->mutable_execution_context());
    }

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
    std::atomic<std::size_t> dropped_callbacks{0};

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
    std::unique_ptr<swarmkit::v1::DataService::Stub> data_stub;
    std::shared_ptr<StreamState> telemetry{std::make_shared<StreamState>()};
    std::shared_ptr<StreamState> authority{std::make_shared<StreamState>()};
    std::shared_ptr<StreamState> reports{std::make_shared<StreamState>()};
    std::shared_ptr<StreamState> messages{std::make_shared<StreamState>()};

    explicit Impl(ClientConfig cfg)
        : config(std::move(cfg)),
          channel(MakeChannel(config)),
          stub(swarmkit::v1::AgentService::NewStub(channel)),
          data_stub(swarmkit::v1::DataService::NewStub(channel)) {}
};

struct StreamRetryState {
    int attempt_number{0};
    int backoff_ms{1};
};

struct ClientRuntime {
    const ClientConfig& config;
    swarmkit::v1::AgentService::Stub& stub;
};

struct DataClientRuntime {
    const ClientConfig& config;
    swarmkit::v1::DataService::Stub& stub;
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
        case SubscriptionKind::kMessages:
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
        case SubscriptionKind::kMessages:
            return "messages";
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

    core::SwarmError error = core::SwarmError::Make(
        DomainForSubscriptionKind(stream_state.kind), core::ErrorCode::kInternal, detail,
        core::ErrorSeverity::kError, core::ErrorRetryability::kAfterRemediation,
        "Catch exceptions inside subscription callbacks");
    error.correlation_id = stream_state.correlation_id;

    SafeNotifySubscriptionEvent(
        stream_state.on_event,
        SubscriptionEvent{
            .kind = stream_state.kind,
            .state = SubscriptionLifecycleState::kCallbackError,
            .drone_id = stream_state.drone_id,
            .correlation_id = stream_state.correlation_id,
            .error = error,
            .message = detail,
            .dropped_callbacks = stream_state.dropped_callbacks.load(std::memory_order_relaxed),
        });
    SafeNotifyStreamError(stream_state.on_error, detail);
}

void ResetCallbackQueue(StreamState& stream_state) {
    std::lock_guard<std::mutex> lock(stream_state.callback_mutex);
    stream_state.callback_queue.clear();
    stream_state.callback_shutdown = false;
    stream_state.dropped_callbacks.store(0, std::memory_order_relaxed);
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
                           std::string message, int attempt_number = 0,
                           core::SwarmError error = {}) {
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
        .dropped_callbacks = stream_state.dropped_callbacks.load(std::memory_order_relaxed),
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

[[nodiscard]] core::ClockDomain ToCoreClockDomain(swarmkit::v1::TelemetryClockDomain domain) {
    switch (domain) {
        case swarmkit::v1::TELEMETRY_CLOCK_DOMAIN_UNIX_EPOCH:
            return core::ClockDomain::kUnixEpoch;
        case swarmkit::v1::TELEMETRY_CLOCK_DOMAIN_VEHICLE_BOOT:
            return core::ClockDomain::kVehicleBoot;
        case swarmkit::v1::TELEMETRY_CLOCK_DOMAIN_AGENT_MONOTONIC:
            return core::ClockDomain::kAgentMonotonic;
        case swarmkit::v1::TELEMETRY_CLOCK_DOMAIN_SDK_MONOTONIC:
            return core::ClockDomain::kSdkMonotonic;
        case swarmkit::v1::TELEMETRY_CLOCK_DOMAIN_UNKNOWN:
        default:
            return core::ClockDomain::kUnknown;
    }
}

[[nodiscard]] core::ClockSynchronization ToCoreClockSynchronization(
    swarmkit::v1::TelemetryClockSynchronization synchronization) {
    switch (synchronization) {
        case swarmkit::v1::TELEMETRY_CLOCK_UNSYNCHRONIZED:
            return core::ClockSynchronization::kUnsynchronized;
        case swarmkit::v1::TELEMETRY_CLOCK_ESTIMATED:
            return core::ClockSynchronization::kEstimated;
        case swarmkit::v1::TELEMETRY_CLOCK_SYNCHRONIZED:
            return core::ClockSynchronization::kSynchronized;
        case swarmkit::v1::TELEMETRY_CLOCK_SYNCHRONIZATION_UNKNOWN:
        default:
            return core::ClockSynchronization::kUnknown;
    }
}

[[nodiscard]] core::CapabilitySupport ToCoreCapabilitySupport(
    swarmkit::v1::CapabilitySupport support) {
    switch (support) {
        case swarmkit::v1::CAPABILITY_UNSUPPORTED:
            return core::CapabilitySupport::kUnsupported;
        case swarmkit::v1::CAPABILITY_SUPPORTED:
            return core::CapabilitySupport::kSupported;
        case swarmkit::v1::CAPABILITY_SUPPORT_UNKNOWN:
        default:
            return core::CapabilitySupport::kUnknown;
    }
}

[[nodiscard]] core::MotionLimitSemantics ToCoreMotionLimitSemantics(
    swarmkit::v1::MotionLimitSemantics semantics) {
    switch (semantics) {
        case swarmkit::v1::MOTION_LIMIT_CONFIGURED_COMMAND:
            return core::MotionLimitSemantics::kConfiguredCommandLimit;
        case swarmkit::v1::MOTION_LIMIT_PLATFORM_CAPABILITY_ASSUMPTION:
            return core::MotionLimitSemantics::kPlatformCapabilityAssumption;
        case swarmkit::v1::MOTION_LIMIT_OBSERVED:
            return core::MotionLimitSemantics::kObservedLimit;
        case swarmkit::v1::MOTION_LIMIT_VALIDATED_BOUND:
            return core::MotionLimitSemantics::kValidatedBound;
        case swarmkit::v1::MOTION_LIMIT_SEMANTICS_UNKNOWN:
        default:
            return core::MotionLimitSemantics::kUnknown;
    }
}

[[nodiscard]] core::MotionLimit ToCoreMotionLimit(const swarmkit::v1::MotionLimit& value) {
    return {
        .value = value.value(),
        .semantics = ToCoreMotionLimitSemantics(value.semantics()),
        .source = value.source(),
        .profile_id = value.profile_id(),
        .profile_version = value.profile_version(),
    };
}

[[nodiscard]] core::TimestampEvidence ToCoreTimestampEvidence(
    const swarmkit::v1::TelemetryTimestampEvidence& value) {
    core::TimestampEvidence out;
    if (value.has_timestamp_ms()) {
        out.timestamp_ms = value.timestamp_ms();
    }
    out.clock_domain = ToCoreClockDomain(value.clock_domain());
    out.synchronization = ToCoreClockSynchronization(value.synchronization());
    if (value.has_clock_uncertainty_ms()) {
        out.clock_uncertainty_ms = value.clock_uncertainty_ms();
    }
    return out;
}

[[nodiscard]] core::MeasurementProvenance ToCoreMeasurementProvenance(
    const swarmkit::v1::TelemetryMeasurementProvenance& value) {
    core::MeasurementProvenance out;
    out.updated = value.updated();
    out.generation = value.generation();
    if (value.has_source_time()) {
        out.source_time = ToCoreTimestampEvidence(value.source_time());
    }
    if (value.has_agent_receive_unix_time_ms()) {
        out.agent_receive_unix_time_ms = value.agent_receive_unix_time_ms();
    }
    if (value.has_agent_receive_monotonic_time_ns()) {
        out.agent_receive_monotonic_time_ns = value.agent_receive_monotonic_time_ns();
    }
    out.source = value.source();
    return out;
}

[[nodiscard]] core::UncertaintySemantics ToCoreUncertaintySemantics(
    swarmkit::v1::TelemetryUncertaintySemantics semantics) {
    switch (semantics) {
        case swarmkit::v1::TELEMETRY_UNCERTAINTY_STANDARD_DEVIATION:
            return core::UncertaintySemantics::kStandardDeviation;
        case swarmkit::v1::TELEMETRY_UNCERTAINTY_CONFIDENCE_BOUND:
            return core::UncertaintySemantics::kConfidenceBound;
        case swarmkit::v1::TELEMETRY_UNCERTAINTY_EMPIRICALLY_CALIBRATED_BOUND:
            return core::UncertaintySemantics::kEmpiricallyCalibratedBound;
        case swarmkit::v1::TELEMETRY_UNCERTAINTY_DETERMINISTIC_HARD_BOUND:
            return core::UncertaintySemantics::kDeterministicHardBound;
        case swarmkit::v1::TELEMETRY_UNCERTAINTY_BACKEND_SPECIFIC:
            return core::UncertaintySemantics::kBackendSpecific;
        case swarmkit::v1::TELEMETRY_UNCERTAINTY_UNKNOWN:
        default:
            return core::UncertaintySemantics::kUnknown;
    }
}

[[nodiscard]] core::UncertaintyDescriptor ToCoreUncertaintyDescriptor(
    const swarmkit::v1::TelemetryUncertaintyDescriptor& value) {
    core::UncertaintyDescriptor out;
    out.semantics = ToCoreUncertaintySemantics(value.semantics());
    if (value.has_confidence_level()) {
        out.confidence_level = value.confidence_level();
    }
    out.calibration_profile_id = value.calibration_profile_id();
    out.calibration_version = value.calibration_version();
    out.source = value.source();
    out.measurement_generation = value.measurement_generation();
    return out;
}

[[nodiscard]] core::UncertaintyEstimate ToCoreUncertaintyEstimate(
    const swarmkit::v1::TelemetryUncertaintyEstimate& value) {
    core::UncertaintyEstimate out{.value = value.value()};
    if (value.has_uncertainty()) {
        out.descriptor = ToCoreUncertaintyDescriptor(value.uncertainty());
    }
    return out;
}

template <typename ProtoCovariance>
[[nodiscard]] std::optional<std::array<float, 9>> ToCoreCovariance(
    const ProtoCovariance& covariance) {
    if (covariance.values_size() != 9) {
        return std::nullopt;
    }
    std::array<float, 9> out{};
    for (int index = 0; index < covariance.values_size(); ++index) {
        out[static_cast<std::size_t>(index)] = covariance.values(index);
    }
    return out;
}

TelemetryDelivery DecodeTelemetryFrame(const swarmkit::v1::TelemetryFrame& proto_frame,
                                       std::optional<std::int64_t> consumer_receive_unix_time_ms) {
    TelemetryDelivery delivery;
    delivery.transport_stream_id = proto_frame.transport_stream_id();
    delivery.sdk_receive_unix_time_ms = consumer_receive_unix_time_ms;
    core::TelemetryFrame& frame = delivery.frame;
    frame.drone_id = proto_frame.drone_id();
    frame.agent_session_id = proto_frame.agent_session_id();
    frame.telemetry_sequence = proto_frame.telemetry_sequence();
    frame.agent_receive_unix_time_ms = proto_frame.agent_receive_unix_time_ms();
    frame.agent_receive_monotonic_time_ns = proto_frame.agent_receive_monotonic_time_ns();
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
        if (accuracy.has_horizontal_position()) {
            frame.accuracy.horizontal_position =
                ToCoreUncertaintyEstimate(accuracy.horizontal_position());
        }
        if (accuracy.has_vertical_position()) {
            frame.accuracy.vertical_position =
                ToCoreUncertaintyEstimate(accuracy.vertical_position());
        }
        if (accuracy.has_horizontal_velocity()) {
            frame.accuracy.horizontal_velocity =
                ToCoreUncertaintyEstimate(accuracy.horizontal_velocity());
        }
        if (accuracy.has_vertical_velocity()) {
            frame.accuracy.vertical_velocity =
                ToCoreUncertaintyEstimate(accuracy.vertical_velocity());
        }
        if (accuracy.has_speed()) {
            frame.accuracy.speed = ToCoreUncertaintyEstimate(accuracy.speed());
        }
        if (accuracy.has_heading_deg()) {
            frame.accuracy.heading_deg = accuracy.heading_deg();
        }
        if (accuracy.has_attitude_deg()) {
            frame.accuracy.attitude_deg = accuracy.attitude_deg();
        }
        if (accuracy.has_position_covariance()) {
            frame.accuracy.position_covariance = ToCoreCovariance(accuracy.position_covariance());
        }
        if (accuracy.has_velocity_covariance()) {
            frame.accuracy.velocity_covariance = ToCoreCovariance(accuracy.velocity_covariance());
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
    if (proto_frame.has_execution_handle()) {
        frame.execution_handle =
            core::internal::ToCoreExecutionHandle(proto_frame.execution_handle());
    }
    if (proto_frame.has_provenance()) {
        const auto& provenance = proto_frame.provenance();
        if (provenance.has_position()) {
            frame.provenance.position = ToCoreMeasurementProvenance(provenance.position());
        }
        if (provenance.has_velocity()) {
            frame.provenance.velocity = ToCoreMeasurementProvenance(provenance.velocity());
        }
        if (provenance.has_accuracy()) {
            frame.provenance.accuracy = ToCoreMeasurementProvenance(provenance.accuracy());
        }
        if (provenance.has_estimator()) {
            frame.provenance.estimator = ToCoreMeasurementProvenance(provenance.estimator());
        }
        if (provenance.has_vehicle_state()) {
            frame.provenance.vehicle_state =
                ToCoreMeasurementProvenance(provenance.vehicle_state());
        }
    }
    return delivery;
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
    return kEarthRadiusM * 2.0 * std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));
}

[[nodiscard]] AuthorityEventInfo ToAuthorityEventInfo(
    const swarmkit::v1::AuthorityEvent& proto_event) {
    return AuthorityEventInfo{
        .kind = ToAuthorityEventKind(proto_event.kind()),
        .drone_id = proto_event.drone_id(),
        .holder_client_id = proto_event.holder_client_id(),
        .holder_priority = static_cast<CommandPriority>(proto_event.holder_priority()),
        .correlation_id = proto_event.correlation_id(),
        .affected_client_id = proto_event.affected_client_id(),
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

[[nodiscard]] swarmkit::v1::DataMessage ToProtoDataMessage(const DataMessage& message) {
    swarmkit::v1::DataMessage proto;
    proto.set_message_id(message.message_id);
    proto.set_sequence(message.sequence);
    proto.set_source_id(message.source_id);
    proto.set_target_id(message.target_id);
    proto.set_topic(message.topic);
    proto.set_unix_time_ms(message.unix_time_ms);
    proto.set_ttl_ms(message.ttl_ms);
    for (const auto& [key, value] : message.labels) {
        (*proto.mutable_labels())[key] = value;
    }
    proto.set_payload(message.payload);
    return proto;
}

[[nodiscard]] DataMessage ToDataMessage(const swarmkit::v1::DataMessage& proto) {
    DataMessage message;
    message.message_id = proto.message_id();
    message.sequence = proto.sequence();
    message.source_id = proto.source_id();
    message.target_id = proto.target_id();
    message.topic = proto.topic();
    message.unix_time_ms = proto.unix_time_ms();
    message.ttl_ms = proto.ttl_ms();
    for (const auto& [key, value] : proto.labels()) {
        message.labels.emplace(key, value);
    }
    message.payload = proto.payload();
    return message;
}

[[nodiscard]] DataPeerState ToDataPeerState(swarmkit::v1::DataPeerState state) {
    switch (state) {
        case swarmkit::v1::DATA_PEER_STATE_READY:
            return DataPeerState::kReady;
        case swarmkit::v1::DATA_PEER_STATE_UNREACHABLE:
            return DataPeerState::kUnreachable;
        case swarmkit::v1::DATA_PEER_STATE_MISCONFIGURED:
            return DataPeerState::kMisconfigured;
        case swarmkit::v1::DATA_PEER_STATE_UNKNOWN:
        default:
            return DataPeerState::kUnknown;
    }
}

[[nodiscard]] DataPeerStatus ToDataPeerStatus(const swarmkit::v1::DataPeerStatus& proto) {
    DataPeerStatus status;
    status.drone_id = proto.drone_id();
    status.address = proto.address();
    status.transport_security = proto.transport_security();
    status.state = ToDataPeerState(proto.state());
    status.last_checked_unix_ms = proto.last_checked_unix_ms();
    status.last_success_unix_ms = proto.last_success_unix_ms();
    status.last_failure_unix_ms = proto.last_failure_unix_ms();
    status.round_trip_ms = proto.round_trip_ms();
    status.message = proto.message();
    return status;
}

[[nodiscard]] swarmkit::v1::DataPeerConfig ToProtoDataPeerConfig(const DataPeerConfig& peer) {
    swarmkit::v1::DataPeerConfig proto;
    proto.set_drone_id(peer.drone_id);
    proto.set_address(peer.address);
    proto.set_transport_security(peer.transport_security);
    proto.set_root_ca_cert_path(peer.root_ca_cert_path);
    proto.set_cert_chain_path(peer.cert_chain_path);
    proto.set_private_key_path(peer.private_key_path);
    proto.set_server_authority_override(peer.server_authority_override);
    return proto;
}

[[nodiscard]] swarmkit::v1::ArtifactDescriptor ToProtoArtifactDescriptor(
    const ArtifactDescriptor& descriptor) {
    swarmkit::v1::ArtifactDescriptor proto;
    proto.set_artifact_id(descriptor.artifact_id);
    proto.set_source_id(descriptor.source_id);
    proto.set_target_id(descriptor.target_id);
    proto.set_content_type(descriptor.content_type);
    proto.set_filename(descriptor.filename);
    proto.set_size_bytes(descriptor.size_bytes);
    proto.set_created_unix_ms(descriptor.created_unix_ms);
    proto.set_ttl_ms(descriptor.ttl_ms);
    proto.set_sha256_hex(descriptor.sha256_hex);
    for (const auto& [key, value] : descriptor.labels) {
        (*proto.mutable_labels())[key] = value;
    }
    return proto;
}

[[nodiscard]] ArtifactDescriptor ToArtifactDescriptor(
    const swarmkit::v1::ArtifactDescriptor& proto) {
    ArtifactDescriptor descriptor;
    descriptor.artifact_id = proto.artifact_id();
    descriptor.source_id = proto.source_id();
    descriptor.target_id = proto.target_id();
    descriptor.content_type = proto.content_type();
    descriptor.filename = proto.filename();
    descriptor.size_bytes = proto.size_bytes();
    descriptor.created_unix_ms = proto.created_unix_ms();
    descriptor.ttl_ms = proto.ttl_ms();
    descriptor.sha256_hex = proto.sha256_hex();
    for (const auto& [key, value] : proto.labels()) {
        descriptor.labels.emplace(key, value);
    }
    return descriptor;
}

[[nodiscard]] ArtifactTransferState ToArtifactTransferState(
    swarmkit::v1::ArtifactTransferState state) {
    switch (state) {
        case swarmkit::v1::ARTIFACT_TRANSFER_STATE_QUEUED:
            return ArtifactTransferState::kQueued;
        case swarmkit::v1::ARTIFACT_TRANSFER_STATE_RUNNING:
            return ArtifactTransferState::kRunning;
        case swarmkit::v1::ARTIFACT_TRANSFER_STATE_COMPLETED:
            return ArtifactTransferState::kCompleted;
        case swarmkit::v1::ARTIFACT_TRANSFER_STATE_FAILED:
            return ArtifactTransferState::kFailed;
        case swarmkit::v1::ARTIFACT_TRANSFER_STATE_CANCELLED:
            return ArtifactTransferState::kCancelled;
        case swarmkit::v1::ARTIFACT_TRANSFER_STATE_UNKNOWN:
        default:
            return ArtifactTransferState::kUnknown;
    }
}

[[nodiscard]] ArtifactTransferStatus ToArtifactTransferStatus(
    const swarmkit::v1::ArtifactTransferStatus& proto) {
    ArtifactTransferStatus status;
    status.transfer_id = proto.transfer_id();
    status.state = ToArtifactTransferState(proto.state());
    status.descriptor = ToArtifactDescriptor(proto.artifact());
    status.bytes_total = proto.bytes_total();
    status.bytes_transferred = proto.bytes_transferred();
    status.started_unix_ms = proto.started_unix_ms();
    status.updated_unix_ms = proto.updated_unix_ms();
    status.completed_unix_ms = proto.completed_unix_ms();
    status.message = proto.message();
    PopulateReplyError(&status.error, proto.error_code(), proto.message(), proto.message(),
                       status.transfer_id, 1, core::ErrorDomain::kInternal);
    return status;
}

void PopulateDataPeerListResult(DataPeerListResult* out, const swarmkit::v1::DataPeerListReply& rep,
                                std::string_view fallback_correlation_id, int attempt_count) {
    if (out == nullptr) {
        return;
    }
    out->ok = rep.ok();
    if (!rep.message().empty()) {
        out->message = rep.message();
    } else if (rep.ok()) {
        out->message = "data peers listed";
    } else {
        out->message = "data peer operation failed";
    }
    out->correlation_id =
        rep.correlation_id().empty() ? std::string(fallback_correlation_id) : rep.correlation_id();
    out->peers.reserve(static_cast<std::size_t>(rep.peers_size()));
    for (const auto& peer : rep.peers()) {
        out->peers.push_back(ToDataPeerStatus(peer));
    }
    PopulateReplyError(&out->error, rep.error_code(), out->message, rep.message(),
                       out->correlation_id, attempt_count, core::ErrorDomain::kTransport);
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

[[nodiscard]] ReadinessCheckSeverity ToReadinessCheckSeverity(
    swarmkit::v1::ReadinessCheckSeverity severity) {
    switch (severity) {
        case swarmkit::v1::READINESS_WARNING:
            return ReadinessCheckSeverity::kWarning;
        case swarmkit::v1::READINESS_ERROR:
            return ReadinessCheckSeverity::kError;
        case swarmkit::v1::READINESS_INFO:
        default:
            return ReadinessCheckSeverity::kInfo;
    }
}

[[nodiscard]] GoalReport ToGoalReport(const swarmkit::v1::GoalReport& proto_report) {
    GoalReport report{
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
    return report;
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
    report.agent_session_id = proto_report.agent_session_id();
    switch (proto_report.execution_binding_case()) {
        case swarmkit::v1::AgentReport::kExecutionContext:
            report.execution_binding =
                core::internal::ToCoreExecutionContext(proto_report.execution_context());
            break;
        case swarmkit::v1::AgentReport::kExecutionHandle:
            report.execution_binding =
                core::internal::ToCoreExecutionHandle(proto_report.execution_handle());
            break;
        case swarmkit::v1::AgentReport::EXECUTION_BINDING_NOT_SET:
            report.execution_binding = std::monostate{};
            break;
    }
    if (proto_report.has_goal()) {
        report.goal = ToGoalReport(proto_report.goal());
    }
    return report;
}

[[nodiscard]] TelemetryStreamEventKind ToTelemetryStreamEventKind(
    swarmkit::v1::TelemetryStreamEventKind kind) {
    switch (kind) {
        case swarmkit::v1::TELEMETRY_REPLAY_STARTED:
            return TelemetryStreamEventKind::kReplayStarted;
        case swarmkit::v1::TELEMETRY_REPLAY_COMPLETED:
            return TelemetryStreamEventKind::kReplayCompleted;
        case swarmkit::v1::TELEMETRY_LIVE_BOUNDARY:
            return TelemetryStreamEventKind::kLiveBoundary;
        case swarmkit::v1::TELEMETRY_HISTORY_EVICTED:
            return TelemetryStreamEventKind::kHistoryEvicted;
        case swarmkit::v1::TELEMETRY_SESSION_MISMATCH:
            return TelemetryStreamEventKind::kSessionMismatch;
        case swarmkit::v1::TELEMETRY_CURSOR_AHEAD:
            return TelemetryStreamEventKind::kCursorAhead;
        case swarmkit::v1::TELEMETRY_STREAM_STARTED:
        case swarmkit::v1::TELEMETRY_STREAM_EVENT_UNSPECIFIED:
        default:
            return TelemetryStreamEventKind::kStarted;
    }
}

[[nodiscard]] TelemetryStreamStatus ToTelemetryStreamStatus(
    const swarmkit::v1::TelemetryStreamEvent& event) {
    return {
        .kind = ToTelemetryStreamEventKind(event.kind()),
        .drone_id = event.drone_id(),
        .agent_session_id = event.agent_session_id(),
        .transport_stream_id = event.transport_stream_id(),
        .expected_agent_session_id = event.expected_agent_session_id(),
        .requested_after_sequence = event.requested_after_sequence(),
        .oldest_available_sequence = event.oldest_available_sequence(),
        .latest_available_sequence = event.latest_available_sequence(),
        .replay_first_sequence = event.replay_first_sequence(),
        .replay_last_sequence = event.replay_last_sequence(),
        .live_from_sequence = event.live_from_sequence(),
        .detail = event.detail(),
    };
}

[[nodiscard]] core::BackendDispatchState ToCoreDispatchState(
    swarmkit::v1::BackendDispatchState state) {
    switch (state) {
        case swarmkit::v1::BACKEND_DISPATCH_REJECTED:
            return core::BackendDispatchState::kRejected;
        case swarmkit::v1::BACKEND_DISPATCH_FAILED:
            return core::BackendDispatchState::kFailed;
        case swarmkit::v1::BACKEND_DISPATCH_ACCEPTED:
            return core::BackendDispatchState::kAccepted;
        case swarmkit::v1::BACKEND_DISPATCH_STATE_UNSPECIFIED:
        default:
            return core::BackendDispatchState::kUnknown;
    }
}

[[nodiscard]] core::BackendCommandOutcome ToCoreBackendOutcome(
    const swarmkit::v1::BackendCommandOutcome& proto) {
    core::BackendCommandOutcome outcome;
    outcome.dispatch_state = ToCoreDispatchState(proto.dispatch_state());
    outcome.dispatch_monotonic_time_ns = proto.dispatch_monotonic_time_ns();
    outcome.completion_monotonic_time_ns = proto.completion_monotonic_time_ns();
    switch (outcome.dispatch_state) {
        case core::BackendDispatchState::kUnknown:
            outcome.result = core::Result::Failed("backend outcome is unknown");
            break;
        case core::BackendDispatchState::kAccepted:
            outcome.result = core::Result::Success(proto.message());
            break;
        case core::BackendDispatchState::kRejected:
            outcome.result = core::Result::Rejected(proto.message());
            break;
        case core::BackendDispatchState::kFailed:
            outcome.result = core::Result::Failed(proto.message());
            break;
    }
    outcome.protocol_responses.reserve(static_cast<std::size_t>(proto.protocol_responses_size()));
    for (const auto& response : proto.protocol_responses()) {
        core::BackendProtocolResponse converted{
            .protocol = response.protocol(),
            .command_name = response.command_name(),
            .native_command_id = std::nullopt,
            .response_expected = response.response_expected(),
            .response_received = response.response_received(),
            .response_timed_out = response.response_timed_out(),
            .result_code = std::nullopt,
            .result_name = response.result_name(),
            .status_text = response.status_text(),
        };
        if (response.has_native_command_id()) {
            converted.native_command_id = response.native_command_id();
        }
        if (response.has_result_code()) {
            converted.result_code = response.result_code();
        }
        outcome.protocol_responses.push_back(std::move(converted));
    }
    return outcome;
}

struct TelemetryCursorState {
    TelemetrySequenceTracker sequence;
    bool replayed{false};
};

[[nodiscard]] grpc::Status RunTelemetryStreamAttempt(
    ClientRuntime runtime, StreamState& telemetry_stream, const TelemetrySubscription& subscription,
    TelemetryCursorState* cursor, const TelemetryObservationHandler& on_observation,
    std::string_view stream_id) {
    grpc::ClientContext* context = InstallStreamContext(telemetry_stream, stream_id);

    swarmkit::v1::TelemetryRequest request;
    request.set_drone_id(subscription.drone_id);
    request.set_rate_hz(subscription.rate_hertz);
    request.set_after_sequence(cursor->sequence.LastAcceptedSequence());
    request.set_expected_agent_session_id(cursor->sequence.AgentSessionId());

    auto reader = runtime.stub.StreamTelemetry(context, request);
    EmitSubscriptionEvent(telemetry_stream, SubscriptionLifecycleState::kConnected,
                          "telemetry stream connected");
    swarmkit::v1::TelemetryStreamItem item;
    while (reader->Read(&item)) {
        if (IsStopRequested(telemetry_stream)) {
            break;
        }

        if (item.has_event()) {
            TelemetryStreamStatus status = ToTelemetryStreamStatus(item.event());
            if (status.kind == TelemetryStreamEventKind::kSessionMismatch) {
                cursor->sequence.AdoptSession(status.agent_session_id, 0, true);
            } else if (status.kind == TelemetryStreamEventKind::kCursorAhead) {
                cursor->sequence.AdoptSession(status.agent_session_id,
                                              status.latest_available_sequence);
            } else if (cursor->sequence.AgentSessionId().empty() &&
                       !status.agent_session_id.empty()) {
                cursor->sequence.AdoptSession(status.agent_session_id);
            }
            if (status.kind == TelemetryStreamEventKind::kReplayStarted) {
                cursor->replayed = true;
            } else if (status.kind == TelemetryStreamEventKind::kLiveBoundary) {
                cursor->replayed = false;
            }
            if (on_observation) {
                TelemetryObservation observation{std::move(status)};
                static_cast<void>(EnqueueCallback(
                    telemetry_stream, [on_observation, observation = std::move(observation)]() {
                        on_observation(observation);
                    }));
            }
            continue;
        }
        if (!item.has_frame()) {
            continue;
        }

        TelemetryDelivery delivery = DecodeTelemetryFrame(item.frame(), NowUnixMs());
        TelemetryFrameObservation frame_observation =
            cursor->sequence.Observe(std::move(delivery), cursor->replayed);

        if (on_observation) {
            TelemetryObservation observation{std::move(frame_observation)};
            static_cast<void>(EnqueueCallback(
                telemetry_stream, [on_observation, observation = std::move(observation)]() {
                    on_observation(observation);
                }));
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

[[nodiscard]] grpc::Status RunMessageStreamAttempt(DataClientRuntime runtime,
                                                   StreamState& message_stream,
                                                   const MessageSubscription& subscription,
                                                   const DataMessageHandler& on_message,
                                                   std::string_view stream_id) {
    grpc::ClientContext* context = InstallStreamContext(message_stream, stream_id);

    swarmkit::v1::MessageSubscription request;
    request.set_subscriber_id(subscription.subscriber_id.empty() ? runtime.config.client_id
                                                                 : subscription.subscriber_id);
    request.set_target_id(subscription.target_id);
    request.set_after_sequence(subscription.after_sequence);
    for (const auto& topic : subscription.topics) {
        request.add_topics(topic);
    }

    auto reader = runtime.stub.SubscribeMessages(context, request);
    EmitSubscriptionEvent(message_stream, SubscriptionLifecycleState::kConnected,
                          "message stream connected");
    swarmkit::v1::DataMessage proto_message;
    while (reader->Read(&proto_message)) {
        if (IsStopRequested(message_stream)) {
            break;
        }
        if (on_message) {
            auto message = ToDataMessage(proto_message);
            static_cast<void>(EnqueueCallback(
                message_stream,
                [on_message, message = std::move(message)]() { on_message(message); }));
        }
    }

    const grpc::Status final_status = reader->Finish();
    ResetStreamContext(message_stream);
    return final_status;
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

[[nodiscard]] core::SwarmError MakeStreamError(const grpc::Status& status,
                                               std::string_view correlation_id,
                                               int attempt_number) {
    core::SwarmError error;
    PopulateTransportError(&error, status, correlation_id, attempt_number);
    return error;
}

void RunTelemetryLoop(ClientRuntime runtime, StreamState& telemetry_stream,
                      const TelemetrySubscription& subscription,
                      const TelemetryObservationHandler& on_observation,
                      const TelemetryErrorHandler& on_error) {
    static_cast<void>(on_error);
    StreamRetryState retry_state = MakeStreamRetryState(runtime.config);
    TelemetryCursorState cursor;
    cursor.sequence.AdoptSession(subscription.expected_agent_session_id,
                                 subscription.after_sequence);

    while (!IsStopRequested(telemetry_stream)) {
        ++retry_state.attempt_number;
        const std::string kStreamId = MakeCorrelationId("telemetry");
        const grpc::Status kFinalStatus = RunTelemetryStreamAttempt(
            runtime, telemetry_stream, subscription, &cursor, on_observation, kStreamId);

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

void RunMessageLoop(DataClientRuntime runtime, StreamState& message_stream,
                    const MessageSubscription& subscription, const DataMessageHandler& on_message,
                    const TelemetryErrorHandler& on_error) {
    static_cast<void>(on_error);
    StreamRetryState retry_state = MakeStreamRetryState(runtime.config);

    while (!IsStopRequested(message_stream)) {
        ++retry_state.attempt_number;
        const std::string stream_id = MakeCorrelationId("messages");
        const grpc::Status final_status =
            RunMessageStreamAttempt(runtime, message_stream, subscription, on_message, stream_id);

        if (IsStopRequested(message_stream)) {
            break;
        }

        LogStreamFailure("messages", subscription.target_id, stream_id, final_status,
                         retry_state.attempt_number);
        if (final_status.ok()) {
            EmitSubscriptionEvent(message_stream, SubscriptionLifecycleState::kStopped,
                                  "message stream ended");
            break;
        }
        if (!ShouldRetryStream(runtime.config.stream_reconnect_policy, retry_state.attempt_number,
                               final_status)) {
            EmitSubscriptionEvent(
                message_stream, SubscriptionLifecycleState::kFailed, final_status.error_message(),
                retry_state.attempt_number,
                MakeStreamError(final_status, stream_id, retry_state.attempt_number));
            MaybeReportStreamFailure(message_stream, final_status);
            break;
        }
        SleepBeforeNextRetry(runtime.config.stream_reconnect_policy, &retry_state.backoff_ms);
    }

    if (IsStopRequested(message_stream)) {
        EmitSubscriptionEvent(message_stream, SubscriptionLifecycleState::kStopped,
                              "message stream stopped");
    }
    message_stream.active.store(false, std::memory_order_relaxed);
    ShutdownCallbackQueue(message_stream);
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

Client::Client(ClientConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}

Client::~Client() {
    StopTelemetry();
    StopAuthorityWatch();
    StopReports();
    StopMessages();
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
    out.agent_session_id = rep.agent_session_id();
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
        out.correlation_id = kCorrelationId;
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.ready = rep.ready();
    out.agent_id = rep.agent_id();
    out.version = rep.version();
    out.agent_session_id = rep.agent_session_id();
    out.unix_time_ms = rep.unix_time_ms();
    out.message = rep.message();
    out.correlation_id = rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    out.backend_name = rep.backend_name();
    out.protocol = rep.protocol();
    out.last_heartbeat_unix_ms = rep.last_heartbeat_unix_ms();
    out.last_telemetry_unix_ms = rep.last_telemetry_unix_ms();
    if (rep.has_armed()) {
        out.armed = rep.armed();
    }
    if (rep.has_landed()) {
        out.landed = rep.landed();
    }
    out.mode = rep.mode();
    out.custom_mode = rep.custom_mode();
    if (rep.has_failsafe()) {
        out.failsafe = rep.failsafe();
    }
    if (rep.has_gps_ok()) {
        out.gps_ok = rep.gps_ok();
    }
    out.gps_fix_type = rep.gps_fix_type();
    out.satellites_visible = rep.satellites_visible();
    out.gps_hdop = rep.gps_hdop();
    if (rep.has_ekf_ok()) {
        out.ekf_ok = rep.ekf_ok();
    }
    out.has_relative_altitude = rep.has_relative_altitude();
    out.relative_alt_m = rep.relative_alt_m();
    out.autonomous_ready = rep.autonomous_ready();
    for (const auto& check : rep.readiness_checks()) {
        out.readiness_checks.push_back(ReadinessCheck{
            .name = check.name(),
            .ok = check.ok(),
            .detail = check.detail(),
            .severity = ToReadinessCheckSeverity(check.severity()),
        });
    }
    out.arming_blockers.assign(rep.arming_blockers().begin(), rep.arming_blockers().end());
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
    out.data_messages_published_total = rep.data_messages_published_total();
    out.data_messages_rejected_total = rep.data_messages_rejected_total();
    out.current_message_subscribers = rep.current_message_subscribers();
    out.artifact_uploads_total = rep.artifact_uploads_total();
    out.artifact_downloads_total = rep.artifact_downloads_total();
    out.artifact_bytes_received_total = rep.artifact_bytes_received_total();
    out.artifact_bytes_sent_total = rep.artifact_bytes_sent_total();
    out.artifact_failures_total = rep.artifact_failures_total();
    out.ready = rep.ready();
    PopulateSuccessError(&out.error, out.correlation_id, attempt_count);
    return out;
}

CapabilitiesResult Client::GetCapabilities() const {
    CapabilitiesResult out;
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
    out.agent_session_id = rep.agent_session_id();
    auto& backend = out.backend;
    backend.backend_name = rep.backend_name();
    backend.protocol = rep.protocol();
    backend.vehicle_class = rep.vehicle_class();
    backend.supports_payload_control = rep.supports_payload_control();
    backend.supports_velocity_control = rep.supports_velocity_control();
    backend.supports_flight_termination = rep.supports_flight_termination();
    backend.supports_backend_commands = rep.supports_backend_commands();
    backend.autopilot_type = rep.autopilot_type();
    backend.supported_modes.assign(rep.supported_modes().begin(), rep.supported_modes().end());
    backend.supported_commands.assign(rep.supported_commands().begin(),
                                      rep.supported_commands().end());
    backend.supported_payloads.assign(rep.supported_payloads().begin(),
                                      rep.supported_payloads().end());
    backend.backend_command_names.assign(rep.backend_command_names().begin(),
                                         rep.backend_command_names().end());
    if (rep.has_max_horizontal_speed()) {
        backend.max_horizontal_speed = ToCoreMotionLimit(rep.max_horizontal_speed());
    }
    if (rep.has_max_climb_speed()) {
        backend.max_climb_speed = ToCoreMotionLimit(rep.max_climb_speed());
    }
    if (rep.has_max_descent_speed()) {
        backend.max_descent_speed = ToCoreMotionLimit(rep.max_descent_speed());
    }
    if (rep.has_max_altitude()) {
        backend.max_altitude = ToCoreMotionLimit(rep.max_altitude());
    }
    if (rep.has_telemetry_evidence()) {
        const auto& evidence = rep.telemetry_evidence();
        backend.evidence.source_timestamp = ToCoreCapabilitySupport(evidence.source_timestamp());
        backend.evidence.source_clock_domains.reserve(evidence.source_clock_domains_size());
        for (const auto domain : evidence.source_clock_domains()) {
            backend.evidence.source_clock_domains.push_back(
                ToCoreClockDomain(static_cast<swarmkit::v1::TelemetryClockDomain>(domain)));
        }
        backend.evidence.position_estimate = ToCoreCapabilitySupport(evidence.position_estimate());
        backend.evidence.horizontal_position_uncertainty =
            ToCoreCapabilitySupport(evidence.horizontal_position_uncertainty());
        backend.evidence.vertical_position_uncertainty =
            ToCoreCapabilitySupport(evidence.vertical_position_uncertainty());
        backend.evidence.horizontal_velocity =
            ToCoreCapabilitySupport(evidence.horizontal_velocity());
        backend.evidence.vertical_velocity = ToCoreCapabilitySupport(evidence.vertical_velocity());
        backend.evidence.horizontal_velocity_uncertainty =
            ToCoreCapabilitySupport(evidence.horizontal_velocity_uncertainty());
        backend.evidence.vertical_velocity_uncertainty =
            ToCoreCapabilitySupport(evidence.vertical_velocity_uncertainty());
        backend.evidence.speed_uncertainty = ToCoreCapabilitySupport(evidence.speed_uncertainty());
        backend.evidence.uncertainty_semantics =
            ToCoreCapabilitySupport(evidence.uncertainty_semantics());
        backend.evidence.estimator_health = ToCoreCapabilitySupport(evidence.estimator_health());
        backend.evidence.failsafe_state = ToCoreCapabilitySupport(evidence.failsafe_state());
        backend.evidence.active_goal_lineage =
            ToCoreCapabilitySupport(evidence.active_goal_lineage());
        backend.evidence.telemetry_sequence =
            ToCoreCapabilitySupport(evidence.telemetry_sequence());
        backend.evidence.telemetry_replay = ToCoreCapabilitySupport(evidence.telemetry_replay());
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
    std::string label{};
    std::function<bool(const HealthStatus&)> health_predicate{};
    std::function<bool(const core::TelemetryFrame&)> telemetry_predicate{};
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
                                                    core::ErrorCode code, std::string message) {
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
                                    [](const HealthStatus& health) { return health.armed == true; },
                            };
                        },
                        [](const CmdForceArm&) {
                            return VerificationSpec{
                                .source = VerificationSource::kHealth,
                                .label = "force-arm",
                                .health_predicate =
                                    [](const HealthStatus& health) { return health.armed == true; },
                            };
                        },
                        [](const CmdDisarm&) {
                            return VerificationSpec{
                                .source = VerificationSource::kHealth,
                                .label = "disarm",
                                .health_predicate =
                                    [](const HealthStatus& health) {
                                        return health.armed == false;
                                    },
                            };
                        },
                        [](const CmdForceDisarm&) {
                            return VerificationSpec{
                                .source = VerificationSource::kHealth,
                                .label = "force-disarm",
                                .health_predicate =
                                    [](const HealthStatus& health) {
                                        return health.armed == false;
                                    },
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
                                        return health.landed == true || health.armed == false;
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
    return MakeVerificationFailure(command_result, core::ErrorCode::kDeadlineExceeded,
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
    swarmkit::v1::TelemetryStreamItem item;
    while (reader->Read(&item)) {
        if (!item.has_frame()) {
            continue;
        }
        const core::TelemetryFrame frame = DecodeTelemetryFrame(item.frame(), NowUnixMs()).frame;
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
            command_result, ToErrorCode(status),
            spec.label + " verification stream failed: " + status.error_message());
    }
    return MakeVerificationFailure(command_result, core::ErrorCode::kDeadlineExceeded,
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
    if (rep.has_backend_outcome()) {
        out.backend_outcome = ToCoreBackendOutcome(rep.backend_outcome());
    }
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

GoalResult Client::SetActiveGoal(const ActiveGoalRequest& request) const {
    GoalResult out;
    const std::string kCorrelationId = MakeCorrelationId("goal");
    const ActiveGoal& goal = request.goal;

    swarmkit::v1::SetActiveGoalRequest req;
    auto* proto_ctx = req.mutable_ctx();
    proto_ctx->set_drone_id(goal.drone_id);
    proto_ctx->set_client_id(impl_->config.client_id);
    proto_ctx->set_priority(static_cast<std::int32_t>(impl_->config.priority));
    proto_ctx->set_correlation_id(kCorrelationId);
    if (request.execution_context.has_value()) {
        core::internal::PopulateExecutionContext(*request.execution_context,
                                                 proto_ctx->mutable_execution_context());
    }
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
    if (rep.has_execution_handle()) {
        out.execution_handle = core::internal::ToCoreExecutionHandle(rep.execution_handle());
    }
    if (rep.has_backend_outcome()) {
        out.backend_outcome = ToCoreBackendOutcome(rep.backend_outcome());
    }
    return out;
}

CancelGoalResult Client::CancelGoal(const core::ExecutionHandle& expected) const {
    CancelGoalResult out;
    const std::string kCorrelationId = MakeCorrelationId("cancel-goal");

    swarmkit::v1::CancelGoalRequest req;
    auto* proto_ctx = req.mutable_ctx();
    proto_ctx->set_drone_id(expected.drone_id);
    proto_ctx->set_client_id(impl_->config.client_id);
    proto_ctx->set_priority(static_cast<std::int32_t>(impl_->config.priority));
    proto_ctx->set_correlation_id(kCorrelationId);
    core::internal::PopulateExecutionHandle(expected, req.mutable_expected_execution_handle());

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
    if (rep.has_execution_handle()) {
        out.cancelled_execution = core::internal::ToCoreExecutionHandle(rep.execution_handle());
    }
    return out;
}

ActiveGoalResult Client::GetActiveGoal(const std::string& drone_id) const {
    ActiveGoalResult out;
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
        out.correlation_id = kCorrelationId;
        PopulateTransportError(&out.error, kStatus, kCorrelationId, attempt_count);
        out.message = out.error.user_message;
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    if (rep.has_active_goal()) {
        const auto& active = rep.active_goal();
        out.active_goal = ActiveGoalSnapshot{
            .goal = ToActiveGoal(active.goal()),
            .status = ToGoalStatus(active.status()),
            .computed_timeout_ms = active.computed_timeout_ms(),
            .execution_handle = core::internal::ToCoreExecutionHandle(active.execution_handle()),
        };
    }
    const std::string correlation_id =
        rep.correlation_id().empty() ? kCorrelationId : rep.correlation_id();
    out.correlation_id = correlation_id;
    PopulateReplyError(&out.error, rep.error_code(), rep.message(), rep.debug_message(),
                       correlation_id, attempt_count);
    return out;
}

namespace {

[[nodiscard]] core::SwarmError MakeSubscriptionStartError(SubscriptionKind kind,
                                                          core::ErrorCode code,
                                                          std::string message) {
    core::SwarmError error = core::SwarmError::Make(
        DomainForSubscriptionKind(kind), code, std::move(message), SeverityForCode(code),
        RetryabilityForCode(code), RemediationForCode(code));
    error.debug_message = error.user_message;
    return error;
}

[[nodiscard]] std::expected<std::uint64_t, core::SwarmError> PrepareSubscriptionStart(
    const std::shared_ptr<StreamState>& stream, SubscriptionKind kind, std::string drone_id,
    TelemetryErrorHandler on_error, SubscriptionEventHandler on_event,
    SubscriptionOptions options) {
    if (options.backpressure.max_pending_callbacks == 0) {
        return std::unexpected(MakeSubscriptionStartError(
            kind, core::ErrorCode::kInvalidArgument,
            "subscription max_pending_callbacks must be greater than zero"));
    }

    if (!options.replace_existing &&
        (stream->active.load(std::memory_order_relaxed) || stream->worker.joinable() ||
         stream->callback_worker.joinable())) {
        return std::unexpected(MakeSubscriptionStartError(kind, core::ErrorCode::kAlreadyExists,
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
            kind, core::ErrorCode::kInternal,
            "failed to start subscription callback dispatcher: " + std::string(exc.what())));
    }

    EmitSubscriptionEvent(*stream, SubscriptionLifecycleState::kStarting,
                          std::string(ToString(kind)) + " subscription starting");

    return generation;
}

}  // namespace

std::expected<Subscription, core::SwarmError> Client::StartTelemetry(
    TelemetrySubscription subscription, TelemetryObservationHandler on_observation,
    TelemetryErrorHandler on_error, SubscriptionEventHandler on_event,
    SubscriptionOptions options) {
    if (subscription.drone_id.empty()) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, core::ErrorCode::kInvalidArgument,
            "telemetry subscription drone_id must not be empty"));
    }
    if (subscription.rate_hertz <= 0) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, core::ErrorCode::kInvalidArgument,
            "telemetry subscription rate_hertz must be greater than zero"));
    }
    if (subscription.after_sequence > 0 && subscription.expected_agent_session_id.empty()) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, core::ErrorCode::kInvalidArgument,
            "expected_agent_session_id is required when after_sequence is non-zero"));
    }
    if (!on_observation) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, core::ErrorCode::kInvalidArgument,
            "telemetry observation callback must not be empty"));
    }

    auto generation = PrepareSubscriptionStart(impl_->telemetry, SubscriptionKind::kTelemetry,
                                               subscription.drone_id, std::move(on_error),
                                               std::move(on_event), options);
    if (!generation.has_value()) {
        return std::unexpected(std::move(generation.error()));
    }

    try {
        const auto stream = impl_->telemetry;
        stream->worker = std::thread([this, stream, subscription = std::move(subscription),
                                      on_observation = std::move(on_observation),
                                      on_error = stream->on_error]() mutable {
            RunTelemetryLoop(ClientRuntime{.config = impl_->config, .stub = *impl_->stub}, *stream,
                             subscription, on_observation, on_error);
        });
    } catch (const std::system_error& exc) {
        CancelAndJoinStream(*impl_->telemetry);
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kTelemetry, core::ErrorCode::kInternal,
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

std::expected<Subscription, core::SwarmError> Client::StartAuthorityWatch(
    AuthoritySubscription subscription, AuthorityEventHandler on_event,
    TelemetryErrorHandler on_error, SubscriptionEventHandler on_state,
    SubscriptionOptions options) {
    if (subscription.drone_id.empty()) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kAuthority, core::ErrorCode::kInvalidArgument,
            "authority subscription drone_id must not be empty"));
    }
    if (!on_event) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kAuthority, core::ErrorCode::kInvalidArgument,
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
            SubscriptionKind::kAuthority, core::ErrorCode::kInternal,
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

std::expected<Subscription, core::SwarmError> Client::StartReports(
    ReportSubscription subscription, AgentReportHandler on_report, TelemetryErrorHandler on_error,
    SubscriptionEventHandler on_event, SubscriptionOptions options) {
    if (subscription.drone_id.empty()) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kReports, core::ErrorCode::kInvalidArgument,
            "report subscription drone_id must not be empty"));
    }
    if (!on_report) {
        return std::unexpected(MakeSubscriptionStartError(
            SubscriptionKind::kReports, core::ErrorCode::kInvalidArgument,
            "report subscription callback must not be empty"));
    }

    auto generation =
        PrepareSubscriptionStart(impl_->reports, SubscriptionKind::kReports, subscription.drone_id,
                                 std::move(on_error), std::move(on_event), options);
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
            SubscriptionKind::kReports, core::ErrorCode::kInternal,
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

#include "client_data.inc"

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
