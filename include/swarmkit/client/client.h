#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "swarmkit/commands.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::client {

static constexpr int kDefaultDeadlineMs = 5000;

/// @brief Connection parameters for the gRPC client.
struct ClientConfig {
    /// Agent address in "host:port" format.
    std::string address{"127.0.0.1:50061"};

    /// Logical identifier sent with outgoing requests.
    std::string client_id{"swarmkit-client"};

    /// RPC deadline in milliseconds.  0 = no deadline.
    int deadline_ms{kDefaultDeadlineMs};

    /// Default command authority priority for LockAuthority calls.
    commands::CommandPriority priority{commands::CommandPriority::kOperator};
};

/// @brief Result of a Ping() call.
struct PingResult {
    bool         ok{false};
    std::string  agent_id;
    std::string  version;
    std::int64_t unix_time_ms{};
    std::string  error_message;
};

/// @brief Result of a SendCommand() call.
struct CommandResult {
    bool        ok{false};
    std::string message;  ///< Error description on failure, empty on success.
};

/**
 * @brief Parameters for a telemetry subscription.
 *
 * @details Using a dedicated struct (rather than individual arguments) keeps the
 * SubscribeTelemetry() signature stable as new options are added in the
 * future (e.g. compression, field filters, max_gap_ms).
 */
struct TelemetrySubscription {
    /// Drone identifier to subscribe to.
    std::string drone_id{"default"};

    /// Requested frame rate in Hz.  The agent may clamp the value.
    int rate_hertz{1};
};

/**
 * @brief Callback types used by SubscribeTelemetry().
 *
 * @details Both are invoked from a background thread — they must be thread-safe and
 * must return quickly without blocking.
 */
/// @{
using TelemetryHandler      = std::function<void(const swarmkit::core::TelemetryFrame&)>;
using TelemetryErrorHandler = std::function<void(const std::string&)>;
/// @}

/**
 * @brief gRPC client for the SwarmKit agent.
 *
 * @details Typical usage:
 * @code
 *   ClientConfig cfg;
 *   cfg.address = "192.168.1.100:50061";
 *   Client client(cfg);
 *
 *   auto result = client.Ping();
 *
 *   client.SubscribeTelemetry({"uav-1", 5}, on_frame, on_error);
 *   // ... do work ...
 *   client.StopTelemetry();
 * @endcode
 *
 * @par Thread safety
 *   Ping() is fully thread-safe.
 *   SubscribeTelemetry() and StopTelemetry() must not be called concurrently
 *   with each other.
 */
class Client {
   public:
    explicit Client(ClientConfig config);
    ~Client();

    Client(const Client&)            = delete;
    Client& operator=(const Client&) = delete;

    /**
     * @brief Health-check RPC.
     *
     * @details Blocks up to ClientConfig::deadline_ms ms.
     * @returns A PingResult describing the agent's identity and timestamp.
     */
    [[nodiscard]] PingResult Ping() const;

    /**
     * @brief Send a single command to the agent.
     *
     * @param envelope Command payload and routing context (drone_id, client_id,
     *                 priority, deadline, correlation_id).
     *
     * @details Blocks up to ClientConfig::deadline_ms ms.
     * The agent's CommandArbiter may reject the command if a higher-priority
     * client currently holds authority over the target drone.
     */
    [[nodiscard]] CommandResult SendCommand(
        const commands::CommandEnvelope& envelope) const;

    /**
     * @brief Acquire exclusive command authority for @p drone_id.
     *
     * @param drone_id Identifier of the drone to lock.
     * @param ttl_ms   Authority time-to-live in milliseconds.
     *                 0 = no automatic expiry; caller MUST call ReleaseAuthority().
     *                 >0 = authority expires after @p ttl_ms milliseconds.
     *
     * @details Uses ClientConfig::priority and ClientConfig::client_id.
     * @returns A failed CommandResult if the drone is not reachable or if a
     *          higher-priority client already holds authority.
     */
    [[nodiscard]] CommandResult LockAuthority(const std::string& drone_id,
                                              std::int64_t       ttl_ms = 0) const;

    /**
     * @brief Explicitly release authority for @p drone_id.
     *
     * @details No-op if this client does not currently hold authority.
     */
    void ReleaseAuthority(const std::string& drone_id) const;

    /**
     * @brief Start a background gRPC streaming telemetry subscription.
     *
     * @param subscription Drone identifier and rate_hertz to request.
     * @param on_frame     Called for every received TelemetryFrame.
     * @param on_error     Called once when the stream ends due to an error;
     *                     not called on a clean StopTelemetry() cancellation.
     *
     * @details If a subscription is already active it is stopped first.
     * Returns immediately; frames arrive via @p on_frame until StopTelemetry()
     * is called or the server closes the connection.
     */
    void SubscribeTelemetry(TelemetrySubscription   subscription,
                            TelemetryHandler        on_frame,
                            TelemetryErrorHandler   on_error = {});

    /**
     * @brief Cancel the active telemetry subscription (if any) and block
     *        until the background thread exits.
     *
     * @details Safe to call when idle.
     */
    void StopTelemetry();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace swarmkit::client
