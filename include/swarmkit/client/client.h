// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "swarmkit/client/command_verifier.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/security.h"
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

/// @brief Backward-compatible names for the unified SDK error model.
using RpcStatusCode = core::ErrorCode;
using RpcError = core::SwarmError;

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

struct ClientSecurityConfig {
    core::TransportSecurityMode transport_security{core::TransportSecurityMode::kAuto};
    std::string root_ca_cert_path;
    std::string cert_chain_path;
    std::string private_key_path;
    std::string server_authority_override;

    [[nodiscard]] core::TransportSecurityMode EffectiveTransportSecurity() const;
    [[nodiscard]] core::Result Validate() const;
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

    /// Transport security parameters.
    ClientSecurityConfig security{};

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

/// @brief Result of explicitly releasing command authority.
struct ReleaseAuthorityResult {
    bool ok{false};
    std::string message;
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
    std::string backend_name{"unknown"};
    std::string protocol{"unknown"};
    std::int64_t last_heartbeat_unix_ms{};
    std::int64_t last_telemetry_unix_ms{};
    bool armed{false};
    bool landed{false};
    std::string mode;
    int custom_mode{-1};
    bool failsafe{false};
    bool gps_ok{false};
    int gps_fix_type{};
    int satellites_visible{};
    float gps_hdop{};
    bool ekf_ok{true};
    bool has_relative_altitude{false};
    float relative_alt_m{};
    std::optional<float> link_quality_percent;
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

struct BackendCapabilities {
    bool ok{false};
    std::string agent_id;
    std::int64_t unix_time_ms{};
    std::string correlation_id;
    std::string backend_name{"unknown"};
    std::string protocol{"unknown"};
    std::string vehicle_class{"unknown"};
    bool supports_mission_upload{false};
    bool supports_payload_control{false};
    bool supports_velocity_control{false};
    bool supports_flight_termination{false};
    bool supports_backend_commands{false};
    bool supports_time_sync{false};
    bool supports_trajectory_upload{false};
    std::string autopilot_type{"unknown"};
    std::vector<std::string> supported_modes;
    std::vector<std::string> supported_commands;
    std::vector<std::string> supported_mission_items;
    std::vector<std::string> supported_payloads;
    std::vector<std::string> supported_telemetry_fields;
    std::vector<std::string> backend_command_names;
    std::vector<std::string> supported_payload_action_namespaces;
    std::vector<std::string> supported_payload_action_names;
    int payload_timing_precision_ms{};
    bool supports_payload_scheduling{false};
    std::optional<float> max_horizontal_speed_mps;
    std::optional<float> max_climb_speed_mps;
    std::optional<float> max_descent_speed_mps;
    std::optional<float> max_altitude_m;
    RpcError error;
};

/**
 * @brief Parameters for a telemetry subscription.
 *
 * @details Using a dedicated struct (rather than individual arguments) keeps the
 * StartTelemetry() signature stable as new options are added in the future
 * (e.g. compression, field filters, max_gap_ms).
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

/// @brief Client-facing authority stream event kind.
enum class AuthorityEventKind : std::uint8_t {
    kGranted,
    kPreempted,
    kResumed,
    kExpired,
};

struct AuthorityEventInfo {
    AuthorityEventKind kind{AuthorityEventKind::kGranted};
    std::string drone_id;
    std::string holder_client_id;
    commands::CommandPriority holder_priority{commands::CommandPriority::kOperator};
    std::string correlation_id;
};

struct GeoPoint {
    double lat_deg{};
    double lon_deg{};
    double alt_m{};
};

struct ActiveGoal {
    std::string drone_id{"default"};
    std::string goal_id;
    std::uint64_t revision{};
    GeoPoint target{};
    float speed_mps{};
    float acceptance_radius_m{2.0F};
    float deviation_radius_m{8.0F};
    std::int64_t timeout_ms{};  ///< 0 lets the agent compute from its vehicle profile.
    std::string role;
};

enum class GoalStatus : std::uint8_t {
    kUnspecified,
    kActive,
    kReached,
    kDeviating,
    kTimeout,
    kCancelled,
    kSuperseded,
    kFailed,
};

enum class ReportSeverity : std::uint8_t {
    kInfo,
    kWarning,
    kError,
    kCritical,
};

enum class AgentReportType : std::uint8_t {
    kUnspecified,
    kCommandAccepted,
    kCommandRejected,
    kCommandAcked,
    kCommandFailed,
    kGoalReport,
    kTelemetryStale,
    kHeartbeatLost,
    kHealthChanged,
    kAuthorityLocked,
    kAuthorityRejected,
    kAuthorityReleased,
    kTrajectoryReport,
    kTimeSyncReport,
};

struct GoalReport {
    std::string drone_id;
    std::string goal_id;
    std::uint64_t revision{};
    GoalStatus status{GoalStatus::kUnspecified};
    double distance_to_goal_m{};
    double deviation_m{};
    double altitude_error_m{};
    float acceptance_radius_m{};
    float deviation_radius_m{};
    std::int64_t elapsed_ms{};
    std::int64_t timeout_ms{};
    std::string message;
};

enum class TrajectoryReportStatus : std::uint8_t {
    kUnspecified,
    kUploaded,
    kValidated,
    kReady,
    kStarted,
    kLate,
    kTracking,
    kDrifting,
    kAborted,
    kCompleted,
    kFailed,
};

struct TrajectoryReport {
    std::string drone_id;
    std::string execution_id;
    std::uint64_t revision{};
    TrajectoryReportStatus status{TrajectoryReportStatus::kUnspecified};
    int active_segment{};
    double distance_to_target_m{};
    double drift_m{};
    std::int64_t schedule_error_ms{};
    std::string message;
};

struct TimeSyncState {
    std::string drone_id;
    std::int64_t agent_unix_time_ms{};
    std::int64_t vehicle_unix_time_ms{};
    std::int64_t clock_offset_ms{};
    float sync_quality_percent{};
    bool synced{false};
    bool stale{true};
    std::string source;
    std::string message;
};

struct AgentReport {
    std::string drone_id;
    std::int64_t unix_time_ms{};
    std::uint64_t sequence{};
    std::string correlation_id;
    AgentReportType type{AgentReportType::kUnspecified};
    ReportSeverity severity{ReportSeverity::kInfo};
    std::string message;
    std::optional<GoalReport> goal;
    std::optional<TrajectoryReport> trajectory;
    std::optional<TimeSyncState> time_sync;
};

struct PayloadAction {
    std::string action_namespace;
    std::string name;
    std::unordered_map<std::string, std::string> params;
};

struct TimedPayloadAction {
    std::int64_t time_offset_ms{};
    std::int64_t unix_time_ms{};
    PayloadAction action;
};

struct LocalPoint {
    double x_m{};
    double y_m{};
    double z_m{};
};

struct TrajectoryPoint {
    std::int64_t time_offset_ms{};
    std::int64_t unix_time_ms{};
    GeoPoint position;
    LocalPoint local_position;
    bool has_position{true};
    bool has_local_position{false};
    bool use_local_position{false};
    float vx_mps{};
    float vy_mps{};
    float vz_mps{};
    bool has_velocity{false};
    float yaw_deg{};
    bool has_yaw{false};
    std::vector<TimedPayloadAction> payload_actions;
    std::optional<commands::Command> command;
};

struct Geofence {
    double min_lat_deg{-90.0};
    double max_lat_deg{90.0};
    double min_lon_deg{-180.0};
    double max_lon_deg{180.0};
    float min_alt_m{};
    float max_alt_m{};
};

struct TrajectoryValidationPolicy {
    float min_battery_percent{};
    std::optional<Geofence> geofence;
    float min_spacing_m{};
    bool require_gps{false};
    bool require_ekf_ok{false};
    float max_horizontal_speed_mps{};
    float max_climb_speed_mps{};
    float max_descent_speed_mps{};
    float max_altitude_m{};
    float tracking_tolerance_m{2.0F};
};

struct TrajectoryPlan {
    std::string execution_id;
    std::uint64_t revision{};
    std::string drone_id{"default"};
    std::string frame{"global"};
    std::vector<TrajectoryPoint> points;
    std::vector<TimedPayloadAction> payload_timeline;
    TrajectoryValidationPolicy validation;
    std::unordered_map<std::string, std::string> labels;
};

enum class ValidationSeverity : std::uint8_t {
    kError,
    kWarning,
    kInfo,
};

struct ValidationIssue {
    ValidationSeverity severity{ValidationSeverity::kInfo};
    std::string code;
    std::string message;
    int point_index{-1};
};

struct ValidateTrajectoryResult {
    bool ok{false};
    std::vector<ValidationIssue> issues;
    float max_required_horizontal_speed_mps{};
    float max_required_climb_speed_mps{};
    float max_required_descent_speed_mps{};
    int first_failing_point_index{-1};
};

enum class ExecutionState : std::uint8_t {
    kUnspecified,
    kUploaded,
    kValidated,
    kReady,
    kStarted,
    kPaused,
    kAborted,
    kCompleted,
    kFailed,
};

struct ExecutionHandle {
    std::string execution_id;
    std::uint64_t revision{};
    std::string drone_id;
    ExecutionState state{ExecutionState::kUnspecified};
    std::int64_t uploaded_unix_ms{};
    std::int64_t prepared_unix_ms{};
    std::int64_t start_unix_ms{};
    int active_segment{};
    std::uint64_t last_report_sequence{};
    std::string message;
};

struct ExecutionResult {
    bool ok{false};
    std::string message;
    std::string correlation_id;
    RpcError error;
    ExecutionHandle handle;
    ValidateTrajectoryResult validation;
};

struct TrajectoryStatus {
    bool found{false};
    ExecutionHandle handle;
    TrajectoryPlan plan;
    std::string message;
    RpcError error;
};

struct GoalResult {
    bool ok{false};
    std::string message;
    std::string correlation_id;
    RpcError error;
    ActiveGoal goal;
    std::int64_t computed_timeout_ms{};
};

struct ActiveGoalStatus {
    bool has_goal{false};
    ActiveGoal goal;
    GoalStatus status{GoalStatus::kUnspecified};
    std::int64_t computed_timeout_ms{};
    std::string message;
    RpcError error;
};

struct ReportSubscription {
    std::string drone_id{"all"};
    std::uint64_t after_sequence{};
};

/**
 * @brief Callback types used by streaming subscriptions.
 *
 * @details Application callbacks are dispatched asynchronously by the
 * subscription dispatcher. They must be thread-safe and should return quickly.
 */
/// @{
using TelemetryHandler = std::function<void(const swarmkit::core::TelemetryFrame&)>;
using TelemetryErrorHandler = std::function<void(const std::string&)>;
using AuthorityEventHandler = std::function<void(const AuthorityEventInfo&)>;
using AgentReportHandler = std::function<void(const AgentReport&)>;
/// @}

enum class SubscriptionKind : std::uint8_t {
    kTelemetry,
    kAuthority,
    kReports,
};

enum class SubscriptionLifecycleState : std::uint8_t {
    kStarting,
    kConnected,
    kReconnecting,
    kStopped,
    kFailed,
    kCallbackError,
};

enum class StreamBackpressurePolicy : std::uint8_t {
    kBlock,
    kDropNewest,
    kDropOldest,
};

struct StreamBackpressureOptions {
    std::size_t max_pending_callbacks{1024};
    StreamBackpressurePolicy policy{StreamBackpressurePolicy::kDropOldest};
};

struct SubscriptionOptions {
    StreamBackpressureOptions backpressure{};
    bool replace_existing{true};
};

struct SubscriptionEvent {
    SubscriptionKind kind{SubscriptionKind::kTelemetry};
    SubscriptionLifecycleState state{SubscriptionLifecycleState::kStarting};
    std::string drone_id;
    std::string correlation_id;
    int attempt_number{};
    RpcError error;
    std::string message;
    std::size_t dropped_callbacks{};
};

using SubscriptionEventHandler = std::function<void(const SubscriptionEvent&)>;

class Subscription {
   public:
    Subscription() = default;
    ~Subscription();

    Subscription(const Subscription&) = delete;
    Subscription& operator=(const Subscription&) = delete;
    Subscription(Subscription&& other) noexcept;
    Subscription& operator=(Subscription&& other) noexcept;

    void Stop() noexcept;
    [[nodiscard]] bool IsActive() const noexcept;
    [[nodiscard]] SubscriptionKind Kind() const noexcept;
    [[nodiscard]] std::uint64_t Id() const noexcept;

   private:
    friend class Client;
    struct State;

    explicit Subscription(std::shared_ptr<State> state) : state_(std::move(state)) {}

    std::shared_ptr<State> state_;
};

using SubscriptionResult = std::expected<Subscription, RpcError>;

/**
 * @brief RAII authority lease for a single drone.
 *
 * @details Created via Client::AcquireAuthoritySession(). Releasing the object
 * automatically attempts a best-effort authority release if the lease is still active.
 * Call Release() when the application needs a checked release result.
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
    [[nodiscard]] ReleaseAuthorityResult Release();
    void Reset() noexcept;

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
 *   cfg.security.root_ca_cert_path = "/etc/swarmkit/ca.pem";
 *   cfg.security.cert_chain_path = "/etc/swarmkit/clients/gcs.pem";
 *   cfg.security.private_key_path = "/etc/swarmkit/clients/gcs.key";
 *   Client client(cfg);
 *
 *   auto result = client.Ping();
 *
 *   auto telemetry = client.StartTelemetry({"uav-1", 5}, on_frame, on_error);
 *   // ... do work ...
 *   if (telemetry) telemetry->Stop();
 * @endcode
 *
 * @par Thread safety
 *   Unary RPCs such as Ping(), SendCommand(), GetHealth(), and GetRuntimeStats()
 *   are thread-safe.
 *   Stream lifecycle pairs StartTelemetry()/StopTelemetry() and
 *   StartAuthorityWatch()/StopAuthorityWatch() must not be called concurrently with
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

    /// @brief Discover backend/autopilot features this agent can execute.
    [[nodiscard]] BackendCapabilities GetCapabilities() const;

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
    [[nodiscard]] CommandResult SendCommandAndWait(const commands::CommandEnvelope& envelope,
                                                   const CommandWaitOptions& options = {}) const;
    [[nodiscard]] CommandResult ArmAndWait(const std::string& drone_id,
                                           const CommandWaitOptions& options = {}) const;
    [[nodiscard]] CommandResult TakeoffAndWait(const std::string& drone_id, double alt_m,
                                               const CommandWaitOptions& options = {}) const;
    [[nodiscard]] CommandResult GotoAndWait(const std::string& drone_id, double lat_deg,
                                            double lon_deg, double alt_m, float speed_mps = 0.0F,
                                            const CommandWaitOptions& options = {}) const;
    [[nodiscard]] CommandResult LandAndWait(const std::string& drone_id,
                                            const CommandWaitOptions& options = {}) const;

    /// @brief Set or replace the agent-supervised active goal for a drone.
    [[nodiscard]] GoalResult SetActiveGoal(const ActiveGoal& goal) const;

    /// @brief Cancel the current active goal for a drone.
    [[nodiscard]] CommandResult CancelGoal(const std::string& drone_id,
                                           const std::string& goal_id = {}) const;

    /// @brief Read the current active goal state known by the agent.
    [[nodiscard]] ActiveGoalStatus GetActiveGoal(const std::string& drone_id) const;

    [[nodiscard]] ExecutionResult UploadTrajectory(const TrajectoryPlan& plan) const;
    [[nodiscard]] ExecutionResult UploadTrajectory(const TrajectoryPlan& plan,
                                                   const commands::CommandContext& context) const;
    [[nodiscard]] ExecutionResult ClearTrajectory(const std::string& drone_id,
                                                  const std::string& execution_id) const;
    [[nodiscard]] ExecutionResult ValidateTrajectory(const TrajectoryPlan& plan) const;
    [[nodiscard]] ExecutionResult ValidateTrajectory(const TrajectoryPlan& plan,
                                                     const commands::CommandContext& context) const;
    [[nodiscard]] ExecutionResult PrepareTrajectory(const std::string& drone_id,
                                                    const std::string& execution_id) const;
    [[nodiscard]] ExecutionResult PrepareTrajectory(const std::string& drone_id,
                                                    const std::string& execution_id,
                                                    const commands::CommandContext& context) const;
    [[nodiscard]] ExecutionResult StartExecutionAt(const std::string& drone_id,
                                                   const std::string& execution_id,
                                                   std::int64_t unix_time_ms) const;
    [[nodiscard]] ExecutionResult StartExecutionAt(const std::string& drone_id,
                                                   const std::string& execution_id,
                                                   std::int64_t unix_time_ms,
                                                   const commands::CommandContext& context) const;
    [[nodiscard]] ExecutionResult PauseExecution(const std::string& drone_id,
                                                 const std::string& execution_id) const;
    [[nodiscard]] ExecutionResult ResumeExecution(const std::string& drone_id,
                                                  const std::string& execution_id) const;
    [[nodiscard]] ExecutionResult AbortExecution(const std::string& drone_id,
                                                 const std::string& execution_id) const;
    [[nodiscard]] TrajectoryStatus GetExecution(const std::string& drone_id,
                                                const std::string& execution_id) const;
    [[nodiscard]] std::vector<ExecutionHandle> ListExecutions(
        const std::string& drone_id = "all") const;
    [[nodiscard]] TimeSyncState GetTimeSyncState(const std::string& drone_id = "default") const;

    /**
     * @brief Acquire exclusive command authority for @p drone_id.
     *
     * @param drone_id Identifier of the drone to lock.
     * @param ttl_ms   Authority time-to-live in milliseconds.
     *                 0 = no automatic expiry; caller MUST call ReleaseAuthority()
     *                 and inspect the returned ReleaseAuthorityResult.
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
     * @brief Explicitly release authority for @p drone_id and report cleanup status.
     *
     * @details No-op if this client does not currently hold authority.
     */
    [[nodiscard]] ReleaseAuthorityResult ReleaseAuthority(const std::string& drone_id) const;

    /**
     * @brief Start a background gRPC streaming telemetry subscription.
     *
     * @param subscription Drone identifier and rate_hertz to request.
     * @param on_frame     Called for every received TelemetryFrame.
     * @param on_error     Called once when the stream ends due to an error;
     *                     not called on a clean StopTelemetry() cancellation.
     *
     * @details If a subscription is already active it is stopped first.
     * StartTelemetry() returns a RAII Subscription handle and typed start errors;
     * callbacks are dispatched through a bounded queue so slow application code
     * does not block the gRPC reader unless configured to do so.
     */
    [[nodiscard]] SubscriptionResult StartTelemetry(TelemetrySubscription subscription,
                                                    TelemetryHandler on_frame,
                                                    TelemetryErrorHandler on_error = {},
                                                    SubscriptionEventHandler on_event = {},
                                                    SubscriptionOptions options = {});

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
    [[nodiscard]] SubscriptionResult StartAuthorityWatch(AuthoritySubscription subscription,
                                                         AuthorityEventHandler on_event,
                                                         TelemetryErrorHandler on_error = {},
                                                         SubscriptionEventHandler on_state = {},
                                                         SubscriptionOptions options = {});

    /// @brief Cancel the active authority watch stream (if any).
    void StopAuthorityWatch();

    /// @brief Start typed agent reports for goal/health/authority workflows.
    [[nodiscard]] SubscriptionResult StartReports(ReportSubscription subscription,
                                                  AgentReportHandler on_report,
                                                  TelemetryErrorHandler on_error = {},
                                                  SubscriptionEventHandler on_event = {},
                                                  SubscriptionOptions options = {});

    /// @brief Cancel the active report stream (if any).
    void StopReports();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace swarmkit::client
