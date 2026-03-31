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
#include <cstdint>
#include <expected>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
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
constexpr std::string_view kCorrelationMetadataKey = "x-correlation-id";

[[nodiscard]] RpcStatusCode ToRpcStatusCode(const grpc::Status& status) {
    if (status.ok()) {
        return RpcStatusCode::kOk;
    }

    switch (status.error_code()) {
        case grpc::StatusCode::INVALID_ARGUMENT:
            return RpcStatusCode::kInvalidArgument;
        case grpc::StatusCode::FAILED_PRECONDITION:
            return RpcStatusCode::kRejected;
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

[[nodiscard]] std::shared_ptr<grpc::ChannelCredentials> MakeChannelCredentials(
    const ClientSecurityConfig& security) {
    grpc::SslCredentialsOptions options;
    static_cast<void>(
        core::internal::ReadTextFile(security.root_ca_cert_path, &options.pem_root_certs));
    static_cast<void>(
        core::internal::ReadTextFile(security.private_key_path, &options.pem_private_key));
    static_cast<void>(
        core::internal::ReadTextFile(security.cert_chain_path, &options.pem_cert_chain));
    return grpc::SslCredentials(options);
}

[[nodiscard]] std::shared_ptr<grpc::Channel> MakeChannel(const ClientConfig& config) {
    const auto kCredentials = MakeChannelCredentials(config.security);
    if (config.security.server_authority_override.empty()) {
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
        case ProtoCode::ERROR_CODE_BACKEND_FAILURE:
            return RpcStatusCode::kInternal;
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

void PopulateTransportError(RpcError* error, const grpc::Status& status,
                            std::string_view correlation_id, int attempt_count) {
    if (error == nullptr) {
        return;
    }

    error->code = ToRpcStatusCode(status);
    error->user_message = status.error_message();
    error->debug_message = status.error_details();
    error->correlation_id = std::string(correlation_id);
    error->attempt_count = attempt_count;
}

[[nodiscard]] agent::AuthorityEvent::Kind ToAuthorityEventKind(
    swarmkit::v1::AuthorityEvent::Kind kind) {
    using ProtoKind = swarmkit::v1::AuthorityEvent::Kind;
    switch (kind) {
        case ProtoKind::AuthorityEvent_Kind_AuthorityEvent_Kind_INT_MIN_SENTINEL_DO_NOT_USE_:
        case ProtoKind::AuthorityEvent_Kind_AuthorityEvent_Kind_INT_MAX_SENTINEL_DO_NOT_USE_:
            return agent::AuthorityEvent::Kind::kExpired;
        case ProtoKind::AuthorityEvent_Kind_GRANTED:
            return agent::AuthorityEvent::Kind::kGranted;
        case ProtoKind::AuthorityEvent_Kind_PREEMPTED:
            return agent::AuthorityEvent::Kind::kPreempted;
        case ProtoKind::AuthorityEvent_Kind_RESUMED:
            return agent::AuthorityEvent::Kind::kResumed;
        case ProtoKind::AuthorityEvent_Kind_EXPIRED:
        case ProtoKind::AuthorityEvent_Kind_KIND_UNSPECIFIED:
            return agent::AuthorityEvent::Kind::kExpired;
    }
    return agent::AuthorityEvent::Kind::kExpired;
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
                std::visit(core::Overloaded{
                               [&](const commands::CmdArm&) { proto_cmd->mutable_arm(); },
                               [&](const commands::CmdDisarm&) { proto_cmd->mutable_disarm(); },
                               [&](const commands::CmdTakeoff& takeoff) {
                                   proto_cmd->mutable_takeoff()->set_alt_m(takeoff.alt_m);
                               },
                               [&](const commands::CmdLand&) { proto_cmd->mutable_land(); },
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
                    },
                    nav);
            },

            [&](const commands::SwarmCmd& swarm) {
                std::visit(core::Overloaded{
                               [&](const commands::CmdSetRole& role) {
                                   proto_cmd->mutable_set_role()->set_role(role.role);
                               },
                               [&](const commands::CmdSetFormation& formation) {
                                   auto* proto_frm = proto_cmd->mutable_set_formation();
                                   proto_frm->set_formation_id(formation.formation_id);
                                   proto_frm->set_slot_index(formation.slot_index);
                               },
                               [&](const commands::CmdRunSequence& sequence) {
                                   auto* proto_seq = proto_cmd->mutable_run_sequence();
                                   proto_seq->set_sequence_id(sequence.sequence_id);
                                   proto_seq->set_sync_unix_ms(sequence.sync_unix_ms);
                               },
                           },
                           swarm);
            },

            /// @note PayloadCmd has no proto mapping yet; server will reject it.
            [&](const commands::PayloadCmd&) {},

        },
        envelope.command);
}

}  // namespace

struct StreamState {
    std::mutex mutex;
    std::unique_ptr<grpc::ClientContext> context;
    std::thread worker;
    std::atomic<bool> active{false};
    std::atomic<bool> stop_requested{false};
};

/// @brief Holds the gRPC channel, stub, and streaming state.
struct Client::Impl {
    ClientConfig config;
    std::shared_ptr<grpc::Channel> channel;
    std::unique_ptr<swarmkit::v1::AgentService::Stub> stub;
    StreamState telemetry;
    StreamState authority;

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

void CancelAndJoinStream(StreamState& stream_state) {
    if (!stream_state.active.load(std::memory_order_relaxed) && !stream_state.worker.joinable()) {
        return;
    }

    stream_state.stop_requested.store(true, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(stream_state.mutex);
        if (stream_state.context) {
            stream_state.context->TryCancel();
        }
    }

    if (stream_state.worker.joinable()) {
        stream_state.worker.join();
    }

    ResetStreamContext(stream_state);
    stream_state.active.store(false, std::memory_order_relaxed);
}

[[nodiscard]] bool ShouldRetryStream(const StreamReconnectPolicy& policy, int attempt_number,
                                     const grpc::Status& final_status) {
    if (!policy.enabled) {
        return false;
    }
    if (final_status.ok()) {
        return false;
    }

    const bool kUnlimitedAttempts = policy.max_attempts == kUnlimitedStreamReconnectAttempts;
    return kUnlimitedAttempts || attempt_number < policy.max_attempts;
}

void SleepBeforeNextRetry(const StreamReconnectPolicy& policy, int* backoff_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(*backoff_ms));
    *backoff_ms = std::min(policy.max_backoff_ms, *backoff_ms * 2);
}

void MaybeReportStreamFailure(const TelemetryErrorHandler& on_error, const grpc::Status& status) {
    if (on_error && !status.ok()) {
        on_error(status.error_message());
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
    frame.lat_deg = proto_frame.lat_deg();
    frame.lon_deg = proto_frame.lon_deg();
    frame.rel_alt_m = proto_frame.rel_alt_m();
    frame.battery_percent = proto_frame.battery_percent();
    frame.mode = proto_frame.mode();
    return frame;
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
    swarmkit::v1::TelemetryFrame proto_frame;
    while (reader->Read(&proto_frame)) {
        if (IsStopRequested(telemetry_stream)) {
            break;
        }

        if (on_frame) {
            on_frame(ToCoreTelemetryFrame(proto_frame));
        }
    }

    const grpc::Status kFinalStatus = reader->Finish();
    ResetStreamContext(telemetry_stream);
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
    swarmkit::v1::AuthorityEvent proto_event;
    while (reader->Read(&proto_event)) {
        if (IsStopRequested(authority_stream)) {
            break;
        }

        if (on_event) {
            on_event(ToAuthorityEventInfo(proto_event));
        }
    }

    const grpc::Status kFinalStatus = reader->Finish();
    ResetStreamContext(authority_stream);
    return kFinalStatus;
}

void RunTelemetryLoop(ClientRuntime runtime, StreamState& telemetry_stream,
                      const TelemetrySubscription& subscription, const TelemetryHandler& on_frame,
                      const TelemetryErrorHandler& on_error) {
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
        if (!ShouldRetryStream(runtime.config.stream_reconnect_policy, retry_state.attempt_number,
                               kFinalStatus)) {
            MaybeReportStreamFailure(on_error, kFinalStatus);
            break;
        }

        SleepBeforeNextRetry(runtime.config.stream_reconnect_policy, &retry_state.backoff_ms);
    }

    telemetry_stream.active.store(false, std::memory_order_relaxed);
}

void RunAuthorityLoop(ClientRuntime runtime, StreamState& authority_stream,
                      const AuthoritySubscription& subscription,
                      const AuthorityEventHandler& on_event,
                      const TelemetryErrorHandler& on_error) {
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
        if (!ShouldRetryStream(runtime.config.stream_reconnect_policy, retry_state.attempt_number,
                               kFinalStatus)) {
            MaybeReportStreamFailure(on_error, kFinalStatus);
            break;
        }

        SleepBeforeNextRetry(runtime.config.stream_reconnect_policy, &retry_state.backoff_ms);
    }

    authority_stream.active.store(false, std::memory_order_relaxed);
}

core::Result ClientSecurityConfig::Validate() const {
    if (!core::internal::FileExists(root_ca_cert_path)) {
        return core::Result::Rejected("security.root_ca_cert_path must point to an existing file");
    }
    if (!core::internal::FileExists(cert_chain_path)) {
        return core::Result::Rejected(
            "security.cert_chain_path must point to an existing file for mTLS");
    }
    if (!core::internal::FileExists(private_key_path)) {
        return core::Result::Rejected(
            "security.private_key_path must point to an existing file for mTLS");
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
}

Client::Client(ClientConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}

Client::~Client() {
    StopTelemetry();
    StopAuthorityWatch();
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

void AuthoritySession::Reset() {
    if (client_ == nullptr) {
        return;
    }

    client_->ReleaseAuthority(drone_id_);
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
    out.error.code = RpcStatusCode::kOk;
    out.error.correlation_id = out.correlation_id;
    out.error.attempt_count = attempt_count;
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
    out.error.code = RpcStatusCode::kOk;
    out.error.correlation_id = out.correlation_id;
    out.error.attempt_count = attempt_count;
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
    out.error.code = RpcStatusCode::kOk;
    out.error.correlation_id = out.correlation_id;
    out.error.attempt_count = attempt_count;
    return out;
}

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
    out.error.code = ToRpcStatusCode(rep.error_code());
    out.error.user_message = rep.message();
    out.error.debug_message = rep.debug_message();
    out.error.correlation_id = out.correlation_id;
    out.error.attempt_count = attempt_count;
    return out;
}

void Client::SubscribeTelemetry(TelemetrySubscription subscription, TelemetryHandler on_frame,
                                TelemetryErrorHandler on_error) {
    StopTelemetry();

    impl_->telemetry.stop_requested.store(false, std::memory_order_relaxed);
    impl_->telemetry.active.store(true, std::memory_order_relaxed);

    impl_->telemetry.worker =
        std::thread([this, subscription = std::move(subscription), on_frame = std::move(on_frame),
                     on_error = std::move(on_error)]() mutable {
            RunTelemetryLoop(ClientRuntime{impl_->config, *impl_->stub}, impl_->telemetry,
                             subscription, on_frame, on_error);
        });
}

void Client::StopTelemetry() {
    CancelAndJoinStream(impl_->telemetry);
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
    out.error.code = ToRpcStatusCode(rep.error_code());
    out.error.user_message = rep.message();
    out.error.debug_message = rep.debug_message();
    out.error.correlation_id = out.correlation_id;
    out.error.attempt_count = attempt_count;
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

void Client::ReleaseAuthority(const std::string& drone_id) const {
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

    if (!kStatus.ok()) {
        core::Logger::WarnFmt(
            "Client::ReleaseAuthority failed: drone={} corr={} attempts={} err={}", drone_id,
            kCorrelationId, attempt_count, kStatus.error_message());
    }
}

void Client::WatchAuthority(AuthoritySubscription subscription, AuthorityEventHandler on_event,
                            TelemetryErrorHandler on_error) {
    StopAuthorityWatch();

    impl_->authority.stop_requested.store(false, std::memory_order_relaxed);
    impl_->authority.active.store(true, std::memory_order_relaxed);

    impl_->authority.worker =
        std::thread([this, subscription = std::move(subscription), on_event = std::move(on_event),
                     on_error = std::move(on_error)]() mutable {
            RunAuthorityLoop(ClientRuntime{impl_->config, *impl_->stub}, impl_->authority,
                             subscription, on_event, on_error);
        });
}

void Client::StopAuthorityWatch() {
    CancelAndJoinStream(impl_->authority);
}

}  // namespace swarmkit::client
