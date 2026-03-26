// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include "swarmkit/agent/arbiter.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::client {

class Client;

inline constexpr int kDefaultDeadlineMs = 5000;
inline constexpr int kDefaultRetryMaxAttempts = 3;
inline constexpr int kDefaultRetryInitialBackoffMs = 200;
inline constexpr int kDefaultRetryMaxBackoffMs = 2000;
inline constexpr bool kDefaultStreamReconnectEnabled = true;
inline constexpr int kDefaultStreamReconnectInitialBackoffMs = 500;
inline constexpr int kDefaultStreamReconnectMaxBackoffMs = 5000;
inline constexpr int kUnlimitedStreamReconnectAttempts = 0;
inline constexpr int kDefaultTelemetryRateHertz = 1;

enum class RpcStatusCode : std::uint8_t {
    kOk,
    kInvalidArgument,
    kRejected,
    kUnavailable,
    kDeadlineExceeded,
    kCancelled,
    kInternal,
    kUnknown,
};

struct RpcError {
    RpcStatusCode code{RpcStatusCode::kOk};
    std::string user_message;
    std::string debug_message;
    std::string correlation_id;
    int attempt_count{0};
};

struct RetryPolicy {
    int max_attempts{kDefaultRetryMaxAttempts};
    int initial_backoff_ms{kDefaultRetryInitialBackoffMs};
    int max_backoff_ms{kDefaultRetryMaxBackoffMs};
};

struct StreamReconnectPolicy {
    bool enabled{kDefaultStreamReconnectEnabled};
    int initial_backoff_ms{kDefaultStreamReconnectInitialBackoffMs};
    int max_backoff_ms{kDefaultStreamReconnectMaxBackoffMs};
    int max_attempts{kUnlimitedStreamReconnectAttempts};  // 0 = unlimited.
};

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

    /// Retry policy for unary RPCs on transient transport failures.
    RetryPolicy retry_policy{};

    /// Auto-reconnect policy for telemetry and authority watch streams.
    StreamReconnectPolicy stream_reconnect_policy{};

    [[nodiscard]] core::Result Validate() const;
    void ApplyEnvironment(std::string_view prefix = "SWARMKIT_CLIENT_");
};

/// @brief Load client configuration from a YAML file.
///
/// Supports either a root-level client map or a `client:` section.
[[nodiscard]] std::expected<ClientConfig, core::Result> LoadClientConfigFromFile(
    const std::string& path);

/// @brief Result of a Ping() call.
struct PingResult {
    bool ok{false};
    std::string agent_id;
    std::string version;
    std::int64_t unix_time_ms{};
    std::string error_message;
    std::string correlation_id;
    RpcError error;
};

/// @brief Result of a SendCommand() call.
struct CommandResult {
    bool ok{false};
    std::string message;  ///< Error description on failure, empty on success.
    std::string correlation_id;
    RpcError error;
};

struct HealthStatus {
    bool ok{false};
    bool ready{false};
    std::string agent_id;
    std::string version;
    std::int64_t unix_time_ms{};
    std::string message;
    std::string correlation_id;
    RpcError error;
};

struct RuntimeStats {
    bool ok{false};
    std::string agent_id;
    std::int64_t unix_time_ms{};
    std::string correlation_id;
    std::uint64_t ping_requests_total{0};
    std::uint64_t health_requests_total{0};
    std::uint64_t runtime_stats_requests_total{0};
    std::uint64_t command_requests_total{0};
    std::uint64_t command_rejected_total{0};
    std::uint64_t command_failed_total{0};
    std::uint64_t lock_requests_total{0};
    std::uint64_t watch_requests_total{0};
    std::uint64_t current_authority_watchers{0};
    std::uint64_t total_telemetry_subscriptions{0};
    std::uint64_t current_telemetry_streams{0};
    std::uint64_t telemetry_frames_sent_total{0};
    std::uint64_t backend_failures_total{0};
    bool ready{false};
    RpcError error;
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
    int rate_hertz{kDefaultTelemetryRateHertz};
};

struct AuthoritySubscription {
    std::string drone_id{"default"};
    commands::CommandPriority priority{commands::CommandPriority::kOperator};
};

struct AuthorityEventInfo {
    agent::AuthorityEvent::Kind kind{agent::AuthorityEvent::Kind::kGranted};
    std::string drone_id;
    std::string holder_client_id;
    commands::CommandPriority holder_priority{commands::CommandPriority::kOperator};
    std::string correlation_id;
};

/**
 * @brief Callback types used by SubscribeTelemetry().
 *
 * @details Both are invoked from a background thread — they must be thread-safe and
 * must return quickly without blocking.
 */
/// @{
using TelemetryHandler = std::function<void(const swarmkit::core::TelemetryFrame&)>;
using TelemetryErrorHandler = std::function<void(const std::string&)>;
using AuthorityEventHandler = std::function<void(const AuthorityEventInfo&)>;
/// @}

/**
 * @brief RAII authority lease for a single drone.
 *
 * @details Created via Client::AcquireAuthoritySession(). Releasing the object
 * automatically calls ReleaseAuthority() if the lease is still active.
 */
class AuthoritySession {
   public:
    AuthoritySession() = default;
    ~AuthoritySession();

    AuthoritySession(const AuthoritySession&) = delete;
    AuthoritySession& operator=(const AuthoritySession&) = delete;
    AuthoritySession(AuthoritySession&& other) noexcept;
    AuthoritySession& operator=(AuthoritySession&& other) noexcept;

    [[nodiscard]] bool IsActive() const {
        return client_ != nullptr;
    }
    [[nodiscard]] const std::string& DroneId() const {
        return drone_id_;
    }
    void Reset();

   private:
    friend class Client;

    AuthoritySession(const Client* client, std::string drone_id)
        : client_(client), drone_id_(std::move(drone_id)) {}

    const Client* client_{nullptr};
    std::string drone_id_;
};

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
 *   Unary RPCs such as Ping(), SendCommand(), GetHealth(), and GetRuntimeStats()
 *   are thread-safe.
 *   Stream lifecycle pairs SubscribeTelemetry()/StopTelemetry() and
 *   WatchAuthority()/StopAuthorityWatch() must not be called concurrently with
 *   themselves, but may be used concurrently with unary RPCs.
 */
class Client {
   public:
    explicit Client(ClientConfig config);
    ~Client();

    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    /**
     * @brief Health-check RPC.
     *
     * @details Blocks up to ClientConfig::deadline_ms ms.
     * @returns A PingResult describing the agent's identity and timestamp.
     */
    [[nodiscard]] PingResult Ping() const;

    /// @brief Read current agent health/readiness state.
    [[nodiscard]] HealthStatus GetHealth() const;

    /// @brief Read current agent runtime counters for observability.
    [[nodiscard]] RuntimeStats GetRuntimeStats() const;

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
    [[nodiscard]] CommandResult SendCommand(const commands::CommandEnvelope& envelope) const;

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
                                              std::int64_t ttl_ms = 0) const;

    /**
     * @brief Acquire authority and return an RAII session that releases it.
     *
     * @returns An AuthoritySession on success, or the lock failure result on
     *          rejection/transport failure.
     */
    [[nodiscard]] std::expected<AuthoritySession, CommandResult> AcquireAuthoritySession(
        const std::string& drone_id, std::int64_t ttl_ms = 0) const;

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
    void SubscribeTelemetry(TelemetrySubscription subscription, TelemetryHandler on_frame,
                            TelemetryErrorHandler on_error = {});

    /**
     * @brief Cancel the active telemetry subscription (if any) and block
     *        until the background thread exits.
     *
     * @details Safe to call when idle.
     */
    void StopTelemetry();

    /**
     * @brief Start a background authority-watch stream for a single drone.
     *
     * @details If a watch is already active it is stopped first. Events may be
     * auto-reconnected according to ClientConfig::stream_reconnect_policy.
     */
    void WatchAuthority(AuthoritySubscription subscription, AuthorityEventHandler on_event,
                        TelemetryErrorHandler on_error = {});

    /// @brief Cancel the active authority watch stream (if any).
    void StopAuthorityWatch();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace swarmkit::client
