// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
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
 *   SwarmClient swarm(cfg);
 *
 *   swarm.AddDrone("uav-1", "192.168.1.101:50061");
 *   swarm.AddDrone("uav-2", "192.168.1.102:50061");
 *   swarm.AddDrone("uav-3", "192.168.1.103:50061");
 *
 *   // Subscribe to telemetry from every drone at 5 Hz
 *   swarm.SubscribeAllTelemetry(5, [](const core::TelemetryFrame& frame) {
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
    core::Result ApplyConfig(const SwarmConfig& config,
                             SwarmAddressPreference address_preference =
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
        const commands::Command& command, const commands::CommandContext& context) const;

    /// @}

    /// @name Authority lock / unlock
    /// @{

    /**
     * @brief Acquire exclusive command authority for a single drone.
     *
     * @param drone_id Identifier of the drone to lock.
     * @param ttl_ms   Authority time-to-live in milliseconds.
     *                 0 (default) = no expiry; caller MUST call UnlockDrone().
     *
     * @returns A failed CommandResult if the drone is not registered or if a
     *          higher-priority client holds authority.
     */
    [[nodiscard]] CommandResult LockDrone(const std::string& drone_id,
                                          std::int64_t ttl_ms = 0) const;

    /// @brief Release command authority for @p drone_id.  No-op if not locked.
    void UnlockDrone(const std::string& drone_id) const;

    /**
     * @brief Acquire authority for every registered drone with bounded parallelism.
     *
     * @param ttl_ms Authority time-to-live in milliseconds.  0 = no expiry.
     * @returns Map of drone_id to CommandResult.
     */
    [[nodiscard]] std::unordered_map<std::string, CommandResult> LockAll(
        std::int64_t ttl_ms = 0) const;

    /// @brief Release authority for all registered drones.
    void UnlockAll() const;

    /// @}

    /// @name Telemetry
    /// @{

    /**
     * @brief Subscribe to telemetry from a specific drone.
     *
     * @details Logs a warning and does nothing if the drone is not registered.
     * If a subscription is already active for that drone it is replaced.
     */
    void SubscribeTelemetry(TelemetrySubscription subscription, TelemetryHandler on_frame,
                            TelemetryErrorHandler on_error = {});

    /// @brief Stop the telemetry subscription for @p drone_id.  No-op if idle.
    void StopTelemetry(const std::string& drone_id);

    /**
     * @brief Subscribe to telemetry from all currently registered drones.
     *
     * @param rate_hertz Requested frame rate in Hz for each drone stream.
     * @param on_frame   Callback invoked for every received TelemetryFrame.
     * @param on_error   Callback invoked once per drone when a stream ends
     *                   due to an error.
     *
     * @details Starts one background stream per drone at @p rate_hertz.
     * Frames from all drones funnel into the same @p on_frame callback — use
     * @c TelemetryFrame::drone_id to distinguish sources.
     *
     * @note Drones added after this call are not automatically subscribed.
     *       Call SubscribeAllTelemetry() again or SubscribeTelemetry() for
     *       the new drone explicitly.
     *
     * @note @p on_frame may be called concurrently from multiple drone
     *       threads.  Ensure the callback is thread-safe.
     */
    void SubscribeAllTelemetry(int rate_hertz, const TelemetryHandler& on_frame,
                               const TelemetryErrorHandler& on_error = {});

    /// @brief Stop telemetry subscriptions for all registered drones.
    void StopAllTelemetry();

    /// @}

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace swarmkit::client
