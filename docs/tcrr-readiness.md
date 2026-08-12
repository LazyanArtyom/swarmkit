# SwarmKit Physical-Execution Evidence Contract

Status: Phase 2 contract, 2026-08-12.

SwarmKit is the algorithm-neutral physical execution and evidence layer. TCRR is an external
consumer. Nothing in this contract evaluates a target region, certifies arrival, mutates rotor
state, commits a logical graph transition, or provides distributed rotor-state consensus.

## Guarantees

Within one Agent process lifetime, a normalized telemetry frame is identified by:

```text
(agent_session_id, drone_id, telemetry_sequence)
```

The Agent assigns the sequence once at normalized ingress, attaches Agent receive wall/monotonic
times, snapshots the exact active execution handle, and normalizes per-measurement provenance. A
subscriber, reconnect, replay, or recorder never renumbers or restamps that frame.

One accepted physical goal dispatch returns an `ExecutionHandle` containing Agent session, drone,
goal ID/revision, Agent-generated physical attempt ID/revision, client/correlation identity, and
optional complete controller execution context. Retry and replacement produce a new physical
attempt. Exact-handle equality is the cancellation and stale-evidence guard.

Timing uncertainty and measurement uncertainty are optional typed evidence. Absence means unknown,
not zero. Backend-specific accuracy remains backend-specific unless a calibration or authoritative
source supplies stronger semantics.

## Replay/live telemetry

`StreamTelemetry` accepts:

- `drone_id` and requested normalized producer rate;
- `after_sequence`;
- `expected_agent_session_id`.

A non-zero cursor without its producer session is rejected; sequence alone is never treated as a
safe cross-session identity.

It returns `TelemetryStreamItem`, an in-band variant of `TelemetryStreamEvent` and
`TelemetryFrame`. Every connection announces stream start, replay start/completion, and the exact
live boundary. It additionally announces session mismatch and unavailable/evicted history. The
per-drone ring is bounded by `telemetry.retention_frames_per_drone`. A cursor ahead of the current
producer is also explicit and is rebased to the announced latest available sequence.

The replay snapshot and live cursor are captured atomically under the normalized-ingress lock.
Frames produced while replay is being delivered remain available to the live reader, subject to the
same explicit bounded-retention policy. A slow consumer that falls behind receives another
history-evicted event.

The SDK exposes `TelemetryObservation`:

- `TelemetryStreamStatus` preserves protocol/session/range/boundary events;
- `TelemetryFrameObservation` classifies first, next, gap, duplicate, reordered, or new-session
  producer evidence and identifies a missing range;
- `replayed` distinguishes retained evidence from frames after the live boundary.

Reconnect starts from the last producer sequence accepted by the SDK reader. Callback dispatch has
its own bounded queue. `SubscriptionEvent::dropped_callbacks` describes local callback pressure;
it is not a producer gap and is never presented as one.

## Backend outcomes

`IDroneBackend::Execute` returns `BackendCommandOutcome`, not an unstructured success string. The
outcome contains dispatch state and timing plus protocol responses. A MAVLink response preserves
native command ID, ACK result code/name, timeout, and associated status text. A setpoint that has no
protocol ACK is labeled as such.

```text
protocol ACK != vehicle movement != physical arrival != TCRR commit
```

The same outcome is returned to SDK callers and recorded as evidence.
The SDK outcome is optional: absence means the Agent did not invoke the backend; it is never
silently converted into an accepted, rejected, or failed backend dispatch.

## Deterministic recorder

When `execution_recorder.file_path` is configured, SwarmKit writes the only durable correctness
record. Console/file logs remain diagnostics, and the report stream remains an in-memory
notification path.

Each record is:

```text
8-byte file magic "SWKEV2\r\n"
uint64 little-endian payload length
32-byte SHA-256 payload checksum
deterministically serialized ExecutionEventEnvelope protobuf
```

The envelope has a schema version, one Agent-global event sequence, Agent session, Agent wall and
monotonic record times, and one typed payload. Payloads cover session start/clean completion,
normalized telemetry, command request, backend outcome/ACK, exact goal lifecycle, Agent reports,
health/failsafe changes, and authority changes.

Session metadata records run/scenario ID, optional seed, SwarmKit version, backend/protocol,
canonical configuration hash, calibration profile/version, and experiment labels. The terminal
session-completed event is the clean-close marker; its absence indicates truncation or abnormal
termination. Checksums detect record corruption.

Recorder storage is bounded:

- `invalidate_run`: scientific fail-closed policy. Capacity or I/O failure invalidates Agent
  readiness and blocks further physical commands.
- `rotate_oldest`: explicit operational policy. Old segments may be deleted; this is declared
  evidence loss and is unsuitable for a run requiring a complete record.

Existing files are refused unless `overwrite_existing` is explicitly enabled.

## Capability discovery

Capability discovery is granular. It declares support for source timestamps, position and velocity,
XY/Z uncertainty, uncertainty semantics, estimator health, failsafe, active goal lineage,
normalized telemetry sequence, and telemetry replay. Motion limits include semantics and source;
a configured command speed is not silently promoted to a physical hard bound.

## Higher-level consumption sketch

```cpp
auto goal_result = client.SetActiveGoal({
    .goal = physical_goal,
    .execution_context = mission_operation,
});
if (!goal_result.ok || !goal_result.execution_handle) {
    return;
}
const auto expected = *goal_result.execution_handle;

auto stream = client.StartTelemetry(
    {.drone_id = expected.drone_id, .rate_hertz = 20},
    [expected](const swarmkit::client::TelemetryObservation& observation) {
        const auto* sample =
            std::get_if<swarmkit::client::TelemetryFrameObservation>(&observation);
        if (sample == nullptr || !sample->delivery.frame.execution_handle ||
            *sample->delivery.frame.execution_handle != expected) {
            return;  // status event, unbound evidence, or stale/superseded attempt
        }

        const auto& frame = sample->delivery.frame;
        // External TCRR now checks sequence/freshness, source and receive times,
        // estimator/failsafe validity, uncertainty semantics, and its own region policy.
    });
```

`GOAL_REACHED` remains an execution-layer experimental baseline. It is not a certified arrival and
cannot update rotor state. Reported covariance or accuracy is not a deterministic hard bound unless
its descriptor explicitly says so. SwarmKit does not provide shared rotor-state consensus.
