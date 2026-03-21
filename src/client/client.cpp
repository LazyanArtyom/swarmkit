#include "swarmkit/client/client.h"

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>

#include "swarmkit/core/overloaded.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::client {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {

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
                        [&](const commands::CmdReturnHome&) {
                            proto_cmd->mutable_return_home();
                        },
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

/// @brief Holds the gRPC channel, stub, and telemetry stream state.
struct Client::Impl {
    ClientConfig config;
    std::shared_ptr<grpc::Channel> channel;
    std::unique_ptr<swarmkit::v1::AgentService::Stub> stub;

    /// Telemetry stream state.
    std::unique_ptr<grpc::ClientContext> telemetry_ctx;
    std::thread telemetry_thread;
    std::atomic<bool> telemetry_active{false};

    explicit Impl(ClientConfig cfg)
        : config(std::move(cfg)),
          channel(grpc::CreateChannel(config.address, grpc::InsecureChannelCredentials())),
          stub(swarmkit::v1::AgentService::NewStub(channel)) {}
};

Client::Client(ClientConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}

Client::~Client() {
    StopTelemetry();
}

PingResult Client::Ping() const {
    PingResult out;

    grpc::ClientContext ctx;
    if (impl_->config.deadline_ms > 0) {
        ctx.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::milliseconds(impl_->config.deadline_ms));
    }

    swarmkit::v1::PingRequest req;
    req.set_agent_id(impl_->config.client_id);

    swarmkit::v1::PingReply rep;
    const grpc::Status status = impl_->stub->Ping(&ctx, req, &rep);

    if (!status.ok()) {
        out.ok = false;
        out.error_message = status.error_message();
        return out;
    }

    out.ok = true;
    out.agent_id = rep.agent_id();
    out.version = rep.version();
    out.unix_time_ms = rep.unix_time_ms();
    return out;
}

CommandResult Client::SendCommand(const commands::CommandEnvelope& envelope) const {
    CommandResult out;

    grpc::ClientContext ctx;
    if (impl_->config.deadline_ms > 0) {
        ctx.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::milliseconds(impl_->config.deadline_ms));
    }

    swarmkit::v1::CommandRequest req;
    BuildProtoCommand(envelope, req);

    swarmkit::v1::CommandReply rep;
    const grpc::Status status = impl_->stub->SendCommand(&ctx, req, &rep);

    if (!status.ok()) {
        out.ok = false;
        out.message = status.error_message();
        return out;
    }

    out.ok = (rep.status() == swarmkit::v1::CommandReply::OK);
    out.message = rep.message();
    return out;
}

void Client::SubscribeTelemetry(TelemetrySubscription subscription, TelemetryHandler on_frame,
                                TelemetryErrorHandler on_error) {
    /// Stop any existing subscription first.
    StopTelemetry();

    impl_->telemetry_ctx = std::make_unique<grpc::ClientContext>();
    impl_->telemetry_active = true;

    swarmkit::v1::TelemetryRequest req;
    req.set_drone_id(subscription.drone_id);
    req.set_rate_hz(subscription.rate_hertz);

    auto reader = impl_->stub->StreamTelemetry(impl_->telemetry_ctx.get(), req);

    impl_->telemetry_thread =
        std::thread([this, reader = std::move(reader), on_frame = std::move(on_frame),
                     on_error = std::move(on_error)]() mutable {
            swarmkit::v1::TelemetryFrame proto_frame;

            while (reader->Read(&proto_frame)) {
                if (!impl_->telemetry_active.load(std::memory_order_relaxed)) {
                    break;
                }

                swarmkit::core::TelemetryFrame frame;
                frame.drone_id = proto_frame.drone_id();
                frame.unix_time_ms = proto_frame.unix_time_ms();
                frame.lat_deg = proto_frame.lat_deg();
                frame.lon_deg = proto_frame.lon_deg();
                frame.rel_alt_m = proto_frame.rel_alt_m();
                frame.battery_percent = proto_frame.battery_percent();
                frame.mode = proto_frame.mode();

                if (on_frame) {
                    on_frame(frame);
                }
            }

            /// Finish() must be called even after a cancelled context.
            const grpc::Status final_status = reader->Finish();

            /// Surface the error only when the stream ended unexpectedly
            /// (i.e. not as a result of our own StopTelemetry() call).
            if (!final_status.ok() && impl_->telemetry_active.load(std::memory_order_relaxed)) {
                if (on_error) {
                    on_error(final_status.error_message());
                }
            }

            impl_->telemetry_active.store(false, std::memory_order_relaxed);
        });
}

void Client::StopTelemetry() {
    if (!impl_->telemetry_active.load(std::memory_order_relaxed) &&
        !impl_->telemetry_thread.joinable()) {
        return;
    }

    impl_->telemetry_active.store(false, std::memory_order_relaxed);

    if (impl_->telemetry_ctx) {
        impl_->telemetry_ctx->TryCancel();
    }

    if (impl_->telemetry_thread.joinable()) {
        impl_->telemetry_thread.join();
    }

    impl_->telemetry_ctx.reset();
}

CommandResult Client::LockAuthority(const std::string& drone_id, std::int64_t ttl_ms) const {
    CommandResult out;

    grpc::ClientContext ctx;
    if (impl_->config.deadline_ms > 0) {
        ctx.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::milliseconds(impl_->config.deadline_ms));
    }

    swarmkit::v1::LockAuthorityRequest req;
    auto* proto_ctx = req.mutable_ctx();
    proto_ctx->set_drone_id(drone_id);
    proto_ctx->set_client_id(impl_->config.client_id);
    proto_ctx->set_priority(static_cast<std::int32_t>(impl_->config.priority));
    req.set_ttl_ms(ttl_ms);

    swarmkit::v1::LockAuthorityReply rep;
    const grpc::Status status = impl_->stub->LockAuthority(&ctx, req, &rep);

    if (!status.ok()) {
        out.ok = false;
        out.message = status.error_message();
        return out;
    }

    out.ok = rep.ok();
    out.message = rep.message();
    return out;
}

void Client::ReleaseAuthority(const std::string& drone_id) const {
    grpc::ClientContext ctx;
    if (impl_->config.deadline_ms > 0) {
        ctx.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::milliseconds(impl_->config.deadline_ms));
    }

    swarmkit::v1::ReleaseAuthorityRequest req;
    req.set_drone_id(drone_id);
    req.set_client_id(impl_->config.client_id);

    swarmkit::v1::ReleaseAuthorityReply rep;
    (void)impl_->stub->ReleaseAuthority(&ctx, req, &rep);
}

}  // namespace swarmkit::client
