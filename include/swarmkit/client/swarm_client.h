// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <stop_token>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/result.h"

namespace swarmkit::client {

enum class SwarmAddressPreference : std::uint8_t {
    kPrimary,
    kPreferLocal,
};

struct SwarmDroneConfig {
    std::string drone_id;
    std::string address;
    std::string local_address;

    [[nodiscard]] core::Result Validate() const;
};

struct SwarmConfig {
    ClientConfig default_client_config{};
    std::vector<SwarmDroneConfig> drones;

    [[nodiscard]] core::Result Validate() const;
};

enum class SwarmPartialFailurePolicy : std::uint8_t {
    kAllOrAbort,
    kBestEffort,
    kQuorum,
    kLandFailed,
    kHoldFailed,
};

enum class SwarmExecutionSynchronization : std::uint8_t {
    kParallelFanout,
    kProtocolStartAt,
};

struct SwarmFanoutOptions {
    /// Maximum worker threads used for one swarm fanout. 0 uses the SDK default.
    std::size_t max_parallelism{};

    /// Optional caller-owned stop token. Cancellation stops dispatching work that has not started
    /// yet.
    std::stop_token cancellation;

    /// Request cancellation of queued work after the first failed task result.
    bool cancel_remaining_on_failure{false};
};

struct SwarmExecutionOptions {
    SwarmPartialFailurePolicy partial_failure_policy{SwarmPartialFailurePolicy::kAllOrAbort};
    SwarmExecutionSynchronization synchronization{SwarmExecutionSynchronization::kProtocolStartAt};
    SwarmFanoutOptions fanout{};
    std::size_t quorum{};
    std::chrono::milliseconds start_delay{200};
    std::chrono::milliseconds max_clock_offset{50};
    float min_time_sync_quality_percent{95.0F};
    float max_drift_m{2.0F};
    bool require_time_sync{true};
    bool verify{false};
    CommandWaitOptions wait_options{};
};

struct SwarmFormationAnchor {
    double lat_deg{};
    double lon_deg{};
    double alt_m{};
};

struct SwarmFormationSlot {
    std::string drone_id;
    double north_m{};
    double east_m{};
    double up_m{};
    float speed_mps{};
    float yaw_deg{};
    bool use_yaw{false};
    std::string role;
};

struct SwarmFormationPlan {
    std::string formation_id;
    SwarmFormationAnchor anchor;
    std::vector<SwarmFormationSlot> slots;
    float speed_mps{};
};

struct SwarmFormationAssignment {
    std::string formation_id;
    int slot_index{};
};

struct SwarmDroneReadiness {
    bool registered{false};
    bool uploaded{false};
    bool prepared{false};
    bool time_sync_ok{false};
    bool ready{false};
    std::string message;
    RpcError error;
    ExecutionHandle handle;
    TimeSyncState time_sync;
};

struct SwarmExecutionReport {
    bool ok{false};
    std::string message;
    RpcError error;
    std::size_t requested{};
    std::size_t succeeded{};
    std::size_t failed{};
    std::int64_t scheduled_start_unix_ms{};
    std::unordered_map<std::string, CommandResult> results;
    std::unordered_map<std::string, CommandResult> recovery_results;
    std::unordered_map<std::string, ExecutionResult> upload_results;
    std::unordered_map<std::string, ExecutionResult> prepare_results;
    std::unordered_map<std::string, ExecutionResult> start_results;
    std::unordered_map<std::string, ExecutionResult> abort_results;
    std::unordered_map<std::string, TimeSyncState> time_sync_states;
    std::unordered_map<std::string, SwarmDroneReadiness> readiness;
    std::unordered_map<std::string, commands::Command> planned_commands;
};

using SwarmSubscriptionResults = std::unordered_map<std::string, SubscriptionResult>;

/// @brief Load a swarm topology/configuration from a YAML file.
///
/// Expected shape:
/// - optional `client:` section for default per-agent ClientConfig values
/// - `swarm.drones:` or root-level `drones:` sequence for agent endpoints
[[nodiscard]] std::expected<SwarmConfig, core::Result> LoadSwarmConfigFromFile(
    const std::string& path);

/**
 * @brief Multi-agent client for swarm drone control.
 *
 * @details Manages a dynamic fleet of per-drone Client connections and provides:
 *   - Routing:   SendCommand routes by envelope.context.drone_id.
 *   - Broadcast: BroadcastCommand fans out to all drones with bounded parallelism.
 *   - Telemetry: per-drone or all-drones subscription; all frames carry
 *                a drone_id field so the caller can distinguish sources.
 *   - Health/stats: each drone connection exposes unary observability probes.
 *
 * All public methods are thread-safe.
 *
 * @par Typical usage (ground-control server side)
 * @code
 *   ClientConfig cfg;
 *   cfg.client_id  = "gcs-server";
 *   cfg.deadline_ms = 3000;
 *   cfg.security.root_ca_cert_path = "/etc/swarmkit/ca.pem";
 *   cfg.security.cert_chain_path = "/etc/swarmkit/clients/gcs.pem";
 *   cfg.security.private_key_path = "/etc/swarmkit/clients/gcs.key";
 *   SwarmClient swarm(cfg);
 *
 *   swarm.AddDrone("uav-1", "192.168.1.101:50061");
 *   swarm.AddDrone("uav-2", "192.168.1.102:50061");
 *   swarm.AddDrone("uav-3", "192.168.1.103:50061");
 *
 *   // Start telemetry from every drone at 5 Hz
 *   auto telemetry = swarm.StartAllTelemetry(5, [](const core::TelemetryFrame& frame) {
 *       // frame.drone_id identifies the source drone
 *   });
 *
 *   // Send a waypoint to one drone
 *   swarm.SendCommand({
 *       {.drone_id = "uav-1"},
 *       agent::NavCmd{agent::CmdSetWaypoint{40.18, 44.51, 30.0}},
 *   });
 *
 *   // Arm all drones simultaneously
 *   swarm.BroadcastCommand(agent::FlightCmd{agent::CmdArm{}}, {});
 * @endcode
 */
class SwarmClient {
   public:
    /**
     * @brief Construct a SwarmClient with a shared default configuration.
     *
     * @param default_config Applied to every drone connection.  The @c address
     *        field is overridden per-drone by AddDrone().
     */
    explicit SwarmClient(ClientConfig default_config = {});
    ~SwarmClient();

    SwarmClient(const SwarmClient&) = delete;
    SwarmClient& operator=(const SwarmClient&) = delete;

    /// @name Fleet management
    /// @{

    /**
     * @brief Register a drone and open a gRPC connection to its agent.
     *
     * @details If a drone with @p drone_id is already registered it is replaced:
     * its telemetry subscription is stopped and a new connection is opened
     * to @p address.
     */
    void AddDrone(const std::string& drone_id, const std::string& address);

    /**
     * @brief Unregister a drone and stop its active telemetry subscription.
     *
     * @details No-op if @p drone_id is not registered.
     */
    void RemoveDrone(const std::string& drone_id);

    /// @brief Return the number of currently registered drones.
    [[nodiscard]] std::size_t DroneCount() const;

    /**
     * @brief Replace the current drone registry with entries from a config.
     *
     * @param config Parsed swarm topology and default per-client settings.
     * @param address_preference Chooses whether local_address should override
     *                           address when present.
     */
    core::Result ApplyConfig(const SwarmConfig& config, SwarmAddressPreference address_preference =
                                                            SwarmAddressPreference::kPrimary);

    /// @}

    /// @name Commands
    /// @{

    /**
     * @brief Route a command to the drone identified by
     *        @c envelope.context.drone_id.
     *
     * @details Returns a failed CommandResult if the drone is not registered.
     * The agent's CommandArbiter may also reject the command if a
     * higher-priority client holds authority over the target drone.
     */
    [[nodiscard]] CommandResult SendCommand(const commands::CommandEnvelope& envelope) const;
    [[nodiscard]] CommandResult SendCommandAndWait(const commands::CommandEnvelope& envelope,
                                                   const CommandWaitOptions& options = {}) const;

    /// @brief Query agent health/readiness for one registered drone.
    [[nodiscard]] HealthStatus GetHealth(const std::string& drone_id) const;

    /// @brief Query runtime counters for one registered drone.
    [[nodiscard]] RuntimeStats GetRuntimeStats(const std::string& drone_id) const;

    /**
     * @brief Send the same command to every registered drone with bounded parallelism.
     *
     * @details @c context.drone_id is overwritten per-drone before dispatch.
     * RPCs execute concurrently using a bounded worker set; the call blocks
     * until every drone has responded or timed out.
     *
     * @returns Map of drone_id to CommandResult for each registered drone.
     */
    [[nodiscard]] std::unordered_map<std::string, CommandResult> BroadcastCommand(
        const commands::Command& command, const commands::CommandContext& context,
        const SwarmFanoutOptions& fanout_options = {}) const;
    [[nodiscard]] std::unordered_map<std::string, CommandResult> BroadcastCommandAndWait(
        const commands::Command& command, const commands::CommandContext& context,
        const CommandWaitOptions& options = {},
        const SwarmFanoutOptions& fanout_options = {}) const;

    /**
     * @brief Execute one concrete command across the fleet from a synchronized protocol start.
     *
     * @details This is the production swarm path for commands such as takeoff
     * and goto that should start together. In kProtocolStartAt mode the manager
     * uploads per-drone timed command plans, prepares them, validates time-sync
     * quality/readiness, then sends one shared StartExecutionAt timestamp.
     * Logical swarm commands are consumed by the manager and are not forwarded
     * to vehicle backends.
     */
    [[nodiscard]] SwarmExecutionReport ExecuteSynchronizedCommand(
        const commands::Command& command, const commands::CommandContext& context,
        const SwarmExecutionOptions& options = {}) const;

    /**
     * @brief Execute a planner-produced per-drone command plan.
     *
     * @details Higher-level path planners can translate their own route,
     * velocity, or mission decisions into concrete per-drone commands and use
     * this manager entry point for synchronized execution and failure policy.
     */
    [[nodiscard]] SwarmExecutionReport ExecutePlannedCommands(
        const std::unordered_map<std::string, commands::Command>& planned_commands,
        const commands::CommandContext& context, const SwarmExecutionOptions& options = {}) const;

    /**
     * @brief Translate a formation plan into per-drone goto commands and execute it.
     */
    [[nodiscard]] SwarmExecutionReport ApplyFormation(
        const SwarmFormationPlan& plan, const commands::CommandContext& context,
        const SwarmExecutionOptions& options = {}) const;

    [[nodiscard]] CommandResult AssignRole(const std::string& drone_id, std::string role) const;
    [[nodiscard]] CommandResult AssignFormationSlot(const std::string& drone_id,
                                                    std::string formation_id, int slot_index) const;
    [[nodiscard]] std::optional<std::string> GetDroneRole(const std::string& drone_id) const;
    [[nodiscard]] std::optional<SwarmFormationAssignment> GetFormationAssignment(
        const std::string& drone_id) const;

    /// @}

    /// @name Generic trajectory / execution coordination
    /// @{

    [[nodiscard]] ExecutionResult UploadTrajectory(const TrajectoryPlan& plan) const;
    [[nodiscard]] std::unordered_map<std::string, ExecutionResult> UploadTrajectories(
        const std::vector<TrajectoryPlan>& plans,
        const SwarmFanoutOptions& fanout_options = {}) const;
    [[nodiscard]] std::unordered_map<std::string, ExecutionResult> ValidateTrajectories(
        const std::vector<TrajectoryPlan>& plans,
        const SwarmFanoutOptions& fanout_options = {}) const;
    [[nodiscard]] std::unordered_map<std::string, ExecutionResult> PrepareAll(
        const std::string& execution_id, const SwarmFanoutOptions& fanout_options = {}) const;
    [[nodiscard]] std::unordered_map<std::string, ExecutionResult> StartAllAt(
        const std::string& execution_id, std::int64_t unix_time_ms,
        const SwarmFanoutOptions& fanout_options = {}) const;
    [[nodiscard]] std::unordered_map<std::string, ExecutionResult> AbortAll(
        const std::string& execution_id, const SwarmFanoutOptions& fanout_options = {}) const;
    [[nodiscard]] std::unordered_map<std::string, std::vector<ExecutionHandle>> ListAllExecutions(
        const SwarmFanoutOptions& fanout_options = {}) const;

    /// @}

    /// @name Authority lock / unlock
    /// @{

    /**
     * @brief Acquire exclusive command authority for a single drone.
     *
     * @param drone_id Identifier of the drone to lock.
     * @param ttl_ms   Authority time-to-live in milliseconds.
     *                 0 (default) = no expiry; caller MUST call UnlockDrone()
     *                 and inspect the returned ReleaseAuthorityResult.
     *
     * @returns A failed CommandResult if the drone is not registered or if a
     *          higher-priority client holds authority.
     */
    [[nodiscard]] CommandResult LockDrone(const std::string& drone_id,
                                          std::int64_t ttl_ms = 0) const;

    /// @brief Release command authority for @p drone_id and report cleanup status.
    [[nodiscard]] ReleaseAuthorityResult UnlockDrone(const std::string& drone_id) const;

    /**
     * @brief Acquire authority for every registered drone with bounded parallelism.
     *
     * @param ttl_ms Authority time-to-live in milliseconds.  0 = no expiry.
     * @returns Map of drone_id to CommandResult.
     */
    [[nodiscard]] std::unordered_map<std::string, CommandResult> LockAll(
        std::int64_t ttl_ms = 0, const SwarmFanoutOptions& fanout_options = {}) const;

    /// @brief Release authority for all registered drones with bounded parallelism.
    [[nodiscard]] std::unordered_map<std::string, ReleaseAuthorityResult> UnlockAll(
        const SwarmFanoutOptions& fanout_options = {}) const;

    /// @}

    /// @name Telemetry
    /// @{

    /**
     * @brief Start telemetry from a specific drone.
     *
     * @returns A RAII Subscription handle on success, or a typed error if the
     *          drone is not registered or the subscription cannot start.
     */
    [[nodiscard]] SubscriptionResult StartTelemetry(TelemetrySubscription subscription,
                                                    TelemetryHandler on_frame,
                                                    TelemetryErrorHandler on_error = {},
                                                    SubscriptionEventHandler on_event = {},
                                                    SubscriptionOptions options = {});

    /// @brief Stop the telemetry subscription for @p drone_id.  No-op if idle.
    void StopTelemetry(const std::string& drone_id);

    /**
     * @brief Start telemetry from all currently registered drones.
     *
     * @param rate_hertz Requested frame rate in Hz for each drone stream.
     * @param on_frame   Callback invoked for every received TelemetryFrame.
     * @param on_error   Callback invoked once per drone when a stream ends
     *                   due to an error.
     *
     * @details Starts one background stream per drone at @p rate_hertz and
     * returns a per-drone result map. Frames from all drones funnel into the
     * same @p on_frame callback; use @c TelemetryFrame::drone_id to distinguish
     * sources.
     *
     * @note Drones added after this call are not automatically subscribed.
     *       Call StartAllTelemetry() again or StartTelemetry() for the new
     *       drone explicitly.
     *
     * @note @p on_frame may be called concurrently from multiple drone
     *       threads.  Ensure the callback is thread-safe.
     */
    [[nodiscard]] SwarmSubscriptionResults StartAllTelemetry(
        int rate_hertz, const TelemetryHandler& on_frame,
        const TelemetryErrorHandler& on_error = {}, const SubscriptionEventHandler& on_event = {},
        SubscriptionOptions options = {});

    /// @brief Stop telemetry subscriptions for all registered drones.
    void StopAllTelemetry();

    /// @}

    /// @name Reports
    /// @{

    [[nodiscard]] SwarmSubscriptionResults StartAllReports(
        const AgentReportHandler& on_report, const TelemetryErrorHandler& on_error = {},
        std::uint64_t after_sequence = 0, const SubscriptionEventHandler& on_event = {},
        SubscriptionOptions options = {});
    void StopAllReports();

    /// @}

   private:
    struct Impl;
    [[nodiscard]] CommandResult HandleSwarmCommand(const commands::CommandEnvelope& envelope) const;
    std::unique_ptr<Impl> impl_;
};

}  // namespace swarmkit::client
