#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "swarmkit/core/telemetry.h"

namespace swarmkit::client {

static constexpr int kDefaultDeadlineMs = 5000;

// ---------------------------------------------------------------------------
// ClientConfig — connection parameters for the gRPC client.
// ---------------------------------------------------------------------------
struct ClientConfig {
    // Agent address in "host:port" format.
    std::string address{"127.0.0.1:50061"};

    // Logical identifier sent with outgoing requests.
    std::string client_id{"swarmkit-client"};

    // RPC deadline in milliseconds.  0 = no deadline.
    int deadline_ms{kDefaultDeadlineMs};
};

// ---------------------------------------------------------------------------
// PingResult — result of a Ping() call.
// ---------------------------------------------------------------------------
struct PingResult {
    bool         ok{false};
    std::string  agent_id;
    std::string  version;
    std::int64_t unix_time_ms{};
    std::string  error_message;
};

// ---------------------------------------------------------------------------
// TelemetrySubscription — parameters for a telemetry subscription.
//
// Using a dedicated struct (rather than individual arguments) keeps the
// SubscribeTelemetry() signature stable as new options are added in the
// future (e.g. compression, field filters, max_gap_ms).
// ---------------------------------------------------------------------------
struct TelemetrySubscription {
    // Drone identifier to subscribe to.
    std::string drone_id{"default"};

    // Requested frame rate in Hz.  The agent may clamp the value.
    int rate_hz{1};
};

// ---------------------------------------------------------------------------
// Callback types used by SubscribeTelemetry().
// Both are invoked from a background thread — they must be thread-safe and
// must return quickly without blocking.
// ---------------------------------------------------------------------------
using TelemetryHandler      = std::function<void(const swarmkit::core::TelemetryFrame&)>;
using TelemetryErrorHandler = std::function<void(const std::string&)>;

// ---------------------------------------------------------------------------
// Client — gRPC client for the SwarmKit agent.
//
// Typical usage:
//   ClientConfig cfg;
//   cfg.address = "192.168.1.100:50061";
//   Client client(cfg);
//
//   auto result = client.Ping();
//
//   client.SubscribeTelemetry({"uav-1", 5}, on_frame, on_error);
//   // ... do work ...
//   client.StopTelemetry();
//
// Thread safety:
//   Ping() is fully thread-safe.
//   SubscribeTelemetry() and StopTelemetry() must not be called concurrently
//   with each other.
// ---------------------------------------------------------------------------
class Client {
   public:
    explicit Client(ClientConfig config);
    ~Client();

    Client(const Client&)            = delete;
    Client& operator=(const Client&) = delete;

    // -----------------------------------------------------------------------
    // Ping — health-check RPC.  Blocks up to ClientConfig::deadline_ms ms.
    // Returns a PingResult describing the agent's identity and timestamp.
    // -----------------------------------------------------------------------
    [[nodiscard]] PingResult Ping() const;

    // -----------------------------------------------------------------------
    // SubscribeTelemetry — starts a background gRPC streaming subscription.
    //
    //   subscription — drone_id and rate_hz to request.
    //   on_frame     — called for every received TelemetryFrame.
    //   on_error     — called once when the stream ends due to an error;
    //                  not called on a clean StopTelemetry() cancellation.
    //
    // If a subscription is already active it is stopped first.
    // Returns immediately; frames arrive via on_frame until StopTelemetry()
    // is called or the server closes the connection.
    // -----------------------------------------------------------------------
    void SubscribeTelemetry(TelemetrySubscription   subscription,
                            TelemetryHandler        on_frame,
                            TelemetryErrorHandler   on_error = {});

    // -----------------------------------------------------------------------
    // StopTelemetry — cancels the active subscription (if any) and blocks
    // until the background thread exits.  Safe to call when idle.
    // -----------------------------------------------------------------------
    void StopTelemetry();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace swarmkit::client

