#include "swarmkit/client/client.h"

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::client {

// ---------------------------------------------------------------------------
// Client::Impl — holds the gRPC channel, stub, and telemetry stream state.
// ---------------------------------------------------------------------------
struct Client::Impl {
    ClientConfig                                      config;
    std::shared_ptr<grpc::Channel>                    channel;
    std::unique_ptr<swarmkit::v1::AgentService::Stub> stub;

    // Telemetry stream state.
    std::unique_ptr<grpc::ClientContext> telemetry_ctx;
    std::thread                          telemetry_thread;
    std::atomic<bool>                    telemetry_active{false};

    explicit Impl(ClientConfig cfg)
        : config(std::move(cfg)),
          channel(grpc::CreateChannel(config.address,
                                      grpc::InsecureChannelCredentials())),
          stub(swarmkit::v1::AgentService::NewStub(channel)) {}
};

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------
Client::Client(ClientConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Client::~Client() {
    StopTelemetry();
}

// ---------------------------------------------------------------------------
// Ping
// ---------------------------------------------------------------------------
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
    const grpc::Status kStatus = impl_->stub->Ping(&ctx, req, &rep);

    if (!kStatus.ok()) {
        out.ok            = false;
        out.error_message = kStatus.error_message();
        return out;
    }

    out.ok           = true;
    out.agent_id     = rep.agent_id();
    out.version      = rep.version();
    out.unix_time_ms = rep.unix_time_ms();
    return out;
}

// ---------------------------------------------------------------------------
// SubscribeTelemetry
// ---------------------------------------------------------------------------
void Client::SubscribeTelemetry(TelemetrySubscription subscription,
                                TelemetryHandler      on_frame,
                                TelemetryErrorHandler on_error) {
    // Stop any existing subscription first.
    StopTelemetry();

    impl_->telemetry_ctx    = std::make_unique<grpc::ClientContext>();
    impl_->telemetry_active = true;

    swarmkit::v1::TelemetryRequest req;
    req.set_drone_id(subscription.drone_id);
    req.set_rate_hz(subscription.rate_hz);

    auto reader = impl_->stub->StreamTelemetry(impl_->telemetry_ctx.get(), req);

    impl_->telemetry_thread = std::thread(
        [this,
         reader   = std::move(reader),
         on_frame = std::move(on_frame),
         on_error = std::move(on_error)]() mutable {
            swarmkit::v1::TelemetryFrame proto_frame;

            while (reader->Read(&proto_frame)) {
                if (!impl_->telemetry_active.load(std::memory_order_relaxed)) {
                    break;
                }

                swarmkit::core::TelemetryFrame frame;
                frame.drone_id        = proto_frame.drone_id();
                frame.unix_time_ms    = proto_frame.unix_time_ms();
                frame.lat_deg         = proto_frame.lat_deg();
                frame.lon_deg         = proto_frame.lon_deg();
                frame.rel_alt_m       = proto_frame.rel_alt_m();
                frame.battery_percent = proto_frame.battery_percent();
                frame.mode            = proto_frame.mode();

                if (on_frame) {
                    on_frame(frame);
                }
            }

            // Finish() must be called even after a cancelled context.
            const grpc::Status kFinalStatus = reader->Finish();

            // Surface the error only when the stream ended unexpectedly
            // (i.e. not as a result of our own StopTelemetry() call).
            if (!kFinalStatus.ok() &&
                impl_->telemetry_active.load(std::memory_order_relaxed)) {
                if (on_error) {
                    on_error(kFinalStatus.error_message());
                }
            }

            impl_->telemetry_active.store(false, std::memory_order_relaxed);
        });
}

// ---------------------------------------------------------------------------
// StopTelemetry
// ---------------------------------------------------------------------------
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

}  // namespace swarmkit::client
