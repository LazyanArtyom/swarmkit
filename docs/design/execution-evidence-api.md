# Canonical Execution-Evidence API

Status: approved greenfield contract, 2026-08-11.

SwarmKit has no deployed consumers. This document therefore defines the single canonical API; no
legacy source or wire interface is retained. Schema and source compatibility become release
requirements only after the project declares a stable public release.

## Boundary

SwarmKit provides algorithm-neutral physical-execution evidence. It owns command dispatch, Agent
session identity, goal and physical-attempt lineage, telemetry normalization, timing and uncertainty
provenance, health, capabilities, and bounded evidence infrastructure.

SwarmKit does not own graph nodes, rotor state, arrival certificates, logical commits, trace proofs,
or distributed rotor-state serialization. Those remain in the external TCRR controller.

## Identity

Controller intent is optional, but when supplied it is complete:

```cpp
struct ExecutionContext {
    std::string mission_id;
    uint64_t mission_revision;
    std::string model_hash;
    std::string operation_id;
    uint64_t operation_attempt_revision;
};
```

Mission and operation revisions are positive. `model_hash` is opaque to SwarmKit.

One accepted physical goal dispatch is identified by:

```cpp
struct ExecutionHandle {
    std::string agent_session_id;
    std::string drone_id;
    std::string goal_id;
    uint64_t goal_revision;
    std::string physical_attempt_id;
    uint64_t physical_attempt_revision;
    std::string client_id;
    std::string correlation_id;
    std::optional<ExecutionContext> context;
};
```

Correctness guards compare the complete handle. There is no goal-ID-only cancellation API.
`SetActiveGoal` accepts an `ActiveGoalRequest` containing the reusable physical goal definition and
an optional complete execution context. The context is request metadata, not a field on
`ActiveGoal`. The call returns the finalized handle, `GetActiveGoal` returns one snapshot containing
the goal/status/timeout/handle, and `CancelGoal` requires that exact handle.

An `AgentReport` has exactly one typed execution binding: none, a generic `ExecutionContext`, or an
exact `ExecutionHandle`. Physical-attempt reports use the handle; its nested context is
authoritative. Parallel context/handle fields are not exposed in the public SDK.

`agent_session_id` is generated once for one Agent process lifetime and changes after restart. It is
an opaque equality token, not a persisted or ordered numeric epoch. A physical attempt gets a new
opaque ID and a session-scoped positive revision on every accepted dispatch, including a retry of
the same goal revision.

Goal states are `ACTIVE`, `REACHED`, `CANCELLED`, `SUPERSEDED`, `FAILED`, and `TIMEOUT`.
`GOAL_REACHED` is only an execution-layer monitor result. It never commits TCRR state.

## Telemetry identity and delivery

The normalized producer identity is:

```text
(agent_session_id, drone_id, telemetry_sequence)
```

The Agent assigns the sequence exactly once at normalized ingress. It does not reset when SDK
subscribers disconnect. The active execution handle is captured at ingress, so later goal changes
cannot relabel an older normalized frame.

SDK transport data is deliberately outside the normalized frame:

```cpp
struct TelemetryDelivery {
    TelemetryFrame frame;
    std::string transport_stream_id;
    int64_t sdk_receive_unix_time_ms;
};
```

Two subscribers may receive the same producer frame through different transport streams. A
reconnect changes transport identity but does not reset producer sequence.

The canonical RPC returns `TelemetryStreamItem`, whose oneof is either a normalized frame or an
in-band `TelemetryStreamEvent`. A request supplies `after_sequence` and the
`expected_agent_session_id` associated with that cursor. Stream events identify stream start,
replay start/completion, the exact live boundary, evicted history, session mismatch, and a cursor
that is ahead of the current producer. The
per-drone ring is bounded by `telemetry.retention_frames_per_drone`; unavailable history is never
presented as continuous. A non-zero cursor without its producer session is invalid.

The SDK callback receives one `TelemetryObservation` variant. Frame observations classify the
producer sequence as first, next, gap, duplicate, reordered, or new session and expose any missing
range. Status observations preserve replay/live and history/session facts. Reconnect attempts use
the last producer sequence accepted by the gRPC reader. Callback queue drops are reported by the
subscription lifecycle counter and remain distinct from producer-sequence gaps.

## Timing and measurement provenance

Every normalized frame has Agent receive wall and monotonic timestamps. Position, velocity,
accuracy, estimator, and vehicle-state groups independently record:

- whether that group changed in this backend publication;
- its persistent sample generation;
- source timestamp, clock domain, synchronization state, and optional clock uncertainty;
- Agent receive wall and monotonic times for that measurement generation;
- the backend field/source name.

An absent source timestamp or clock uncertainty means unknown. Unknown is never represented as zero.
A heartbeat or attitude publication can produce a new frame without advancing position generation.
Cached measurements retain the provenance from the update that produced them.

## Uncertainty

Each optional position or velocity uncertainty is a value plus one descriptor:

```text
UNKNOWN
STANDARD_DEVIATION
CONFIDENCE_BOUND
EMPIRICALLY_CALIBRATED_BOUND
DETERMINISTIC_HARD_BOUND
BACKEND_SPECIFIC
```

Horizontal position, vertical position, horizontal velocity, vertical velocity, and generic speed
uncertainty are distinct optional estimates. Confidence level, calibration profile/version, source,
and measurement generation are retained when meaningful. Optional covariance matrices contain
exactly nine values.

MAVLink `GPS_RAW_INT` `h_acc`, `v_acc`, and `vel_acc` are retained as backend-specific estimates.
They are never labeled deterministic hard bounds. The generic `vel_acc` value is not fabricated into
independent horizontal and vertical components.

## Capabilities and motion limits

Evidence support is typed as `UNKNOWN`, `UNSUPPORTED`, or `SUPPORTED` for source timestamps,
position/velocity estimates, each uncertainty class, semantics, estimator health, failsafe state,
goal lineage, telemetry sequence, and replay. Typed support replaces free-form telemetry-field
lists.

The algorithm-neutral `core::BackendCapabilities` feature set is shared by Agent backends and SDK
consumers. `CapabilitiesResult` only adds RPC/Agent metadata and error state around that feature set;
it does not duplicate backend fields.

A motion limit is one cohesive value:

```cpp
struct MotionLimit {
    float value;
    MotionLimitSemantics semantics;
    std::string source;
    std::string profile_id;
    std::string profile_version;
};
```

Semantics distinguish configured command limits, platform assumptions, observed limits, validated
bounds, and unknown. Numeric values and provenance are never exposed as parallel structures. A
configured speed is not automatically a physical worst-case bound.

## Backend outcomes

`IDroneBackend::Execute` returns a typed `BackendCommandOutcome`. It distinguishes accepted,
rejected, and failed dispatch and carries zero or more native protocol responses. Each response
states whether a response was expected, received, or timed out and may retain a native command ID,
result code/name, and status text. MAVLink `COMMAND_ACK` details survive into SDK results and the
execution record. Setpoint messages truthfully state that no ACK exists. None of these outcomes
prove movement or arrival. SDK results expose the outcome as optional because Agent-side
validation or authority rejection means no backend dispatch occurred.

## Determinism and evidence recording

Wall time, monotonic time, and generated IDs are injectable. The execution recorder writes one
Agent-global order of versioned `ExecutionEventEnvelope` protobuf messages. It records session
start/clean completion, normalized telemetry, health/failsafe changes, command requests and typed
backend outcomes, exact goal lifecycle/attempt evidence, reports, and authority changes. The
session header includes run/scenario ID, optional seed, software/backend identity, configuration
hash, and calibration reference.

Every record uses deterministic protobuf serialization and is framed by a little-endian payload
length plus SHA-256 checksum. A clean session-completion event distinguishes orderly closure from
truncation. Storage is bounded. `invalidate_run` is the scientific policy: exhaustion or an I/O
failure invalidates readiness and blocks further commands. `rotate_oldest` is an explicit
operational data-loss policy. Report-only JSONL persistence does not exist.

The future scripted backend and fault decorator must preserve these canonical types and make every
realized fault explicit.

For a fixed normalized execution record and fixed higher-level configuration, TCRR must be able to
reproduce the same certificate decisions. Console logs are diagnostic and are not correctness
evidence.

## Higher-level controller contract

A controller may rely on exact handle equality, Agent-session mismatch detection, producer sequence
ordering, ingress-time execution binding, explicit measurement freshness, and typed but possibly
unknown timing and uncertainty evidence. It may not infer arrival from an ACK or `GOAL_REACHED`,
convert unknown uncertainty to zero, treat a configured command limit as a hard bound, or expect
SwarmKit to serialize shared rotor state.
