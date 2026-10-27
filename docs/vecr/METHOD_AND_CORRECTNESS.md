# VECR method and conditional correctness

## Contract

A directed graph step is a planner decision. Arrival at its target, route traversal, and observation of its assigned geographic strip are separate physical predicates. The current implementation uses a local metric straight segment and a strip whose full cross section is certified by each accepted capture adapter. It stores a sorted union of along-route intervals. A survey edge is fulfilled only after stop-style arrival and accepted survey intervals leave at most `maximum_missing_m`. A traversal edge additionally requires bounded movement evidence; the combined obligation requires both traversal and survey. An arrival-only edge requires only qualified arrival. No rotor or gossip state changes in SwarmKit. The agent rejects bent and multi-point routes because it currently dispatches a single GOTO to B.

Each request binds mission ID/revision, edge ID, unique attempt ID and drone ID. Captures with a different attempt ID are rejected. The request's route frame must be fixed; WGS84 helpers use a local equirectangular plane anchored at the supplied latitude/longitude. This approximation is suitable only for short local routes, away from poles and longitude wrap. A production geodetic adapter should enforce that domain.

## Evidence and guarantees

The sensor RPC accepts capture identity, mission/edge/attempt binding, a configured calibration ID, camera center, altitude above ground, field of view, deterministic hard pose bounds and quality assertions. It does not accept a caller-supplied covered interval. The agent requires a bounded vehicle position observation within 500 ms of the capture timestamp and checks that the claimed camera center is kinematically consistent with that position, the declared maximum speed and both hard bounds. It then computes a conservative interval with `BuildNadirStripEvidence` using the requested route and strip half-width. The core rejects invalid ranges, duplicates, stale or mismatched captures. This restricted calculation assumes a straight route, flat ground, rectangular nadir footprint aligned to the route, established image quality, and hard altitude/position bounds. The mTLS sensor allowlist authenticates one configured adapter; it does **not** prove that its camera pose, image quality, calibration, alignment or flat-ground assertions are true. A real calibrated producer has not been validated, so survey evidence in current tests remains simulated geometric evidence.

Stop-style arrival means two qualifying observations at least 200 ms apart, each with a known deterministic hard horizontal position bound, vertical position bound and horizontal speed bound, within the configured three-dimensional radius and maximum speed. A qualification is revoked on a later away, invalid or ambiguous observation; final survey evidence cannot use a position older than 1 s. A terminal fulfillment records that the obligation was satisfied at that time and remains terminal if the vehicle later departs. Fly-through arrival is not implemented. Backend command ACK and backend goal-reached messages do not satisfy this policy.

For deterministic traversal, both endpoint observations must have **hard** horizontal position error bounds and the request must have a valid physical maximum speed. The endpoint displacement must be at most `Vmax*dt+eps0+eps1`; an impossible jump cannot prove traversal. For distance to the route, `D0 = d(p̂0, route)+eps0`, `D1 = d(p̂1, route)+eps1`, and `dt=t1-t0`, the 1-Lipschitz property gives `d(p(t), route) <= min(D0+Vmax(t-t0), D1+Vmax(t1-t)) <= (D0+D1+Vmax*dt)/2`. If the last value is at most corridor width, the interval is conformant under the declared assumptions. Sample-only corridor checks are experimental baseline semantics, never a deterministic whole-interval guarantee. The implementation only credits forward projected progress and declines ambiguous nearest-route projection.

**Sound survey proposition.** If every accepted conservative footprint lies inside ground that was actually observed at the required quality, the union of accepted footprints lies inside actually observed ground. Thus a remaining required strip of at most the configured tolerance implies modeled survey fulfillment to that tolerance. The sensor inclusion assumption is substantive and unvalidated for a real camera.

The October 26 dissertation audit corrected along-route traversal credit: only
`[s_hat0 + eps0, s_hat1 - eps1]`, when nonempty, is guaranteed by two uncertain
endpoint observations on the straight route. Crediting the entire estimated
interval could report full traversal although an admissible actual path omitted
both ends. Overlapping endpoint bounds may describe a stationary vehicle and
cannot create positive traversed length. Two counterexample regressions reproduce
the prior failure and verify contraction. This may leave conservative boundary
gaps; they must not be filled merely because the vehicle arrived within a radius.
The same audit corrected the camera's along-route coordinate: a camera beyond
an endpoint must retain its signed coordinate when constructing its footprint.
Only the resulting interval is clipped to the route. Clamping the center itself
could move an entirely nonoverlapping footprint onto A or B. Eastbound and
northbound regression counterexamples cover both ends and partial overlap.
Frozen v5 measurements retain their original source revision. The separate
v5.1 protocol repeats the full campaign with both corrections.

**No skipped gap.** The interval union never fills an uncovered hole. Fulfillment checks the entire missing length, not maximum observed route coordinate. Later disjoint intervals remain available without advancing the contiguous prefix. An accepted interval remains stored after a separate navigation deviation, unless its original evidence is explicitly withdrawn. This may save repeated surveying; travel savings still include transit to the re-entry point.

## Recovery and state

The event processor proposes re-entry before the earliest missing interval with configured overlap. The agent dispatches recovery only for its simulator backend when explicitly enabled, with authority, battery, navigation bounds, backend health and autopilot mode checks. Re-entry itself requires a bounded stop-style dwell before the agent commands B. Geofence and live mission feasibility are not yet qualified; MAVLink recovery is disabled. Recovery is bounded by attempt count and cooldown. Terminal completion/interruption is idempotent. Unknown position produces `UNKNOWN` and does not create movement evidence; earlier capture intervals are retained. An unresponsive vehicle times out to an interrupted report when the agent remains running.

`ExecuteEdge` reserves a drone/attempt ID in an append-only journal and fsyncs it before backend dispatch. The journal stores terminal reports; a restarted agent returns a known terminal outcome without redispatching. An attempt with no durable terminal outcome returns `UNKNOWN` after restart, and retrying its ID is rejected. A changed request or client ID is rejected. Corrupt, unwritable, full, or concurrently locked journals block new edge dispatch. The journal path is configurable (`agent.vecr.attempt_journal_path` or `SWARMKIT_AGENT_VECR_ATTEMPT_JOURNAL_PATH`); otherwise an agent-specific file is created below `data.artifact_dir`. Deployments must place it on persistent storage and preserve it during upgrades and restore; deleting it loses duplicate protection for its IDs. This is fail-closed duplicate suppression and terminal retrieval while journal history remains intact, not exactly-once execution across process failure: a command can have acted physically while its final report remained unknown. The in-memory attempt store has a 4096-attempt limit; the journal has a 64 MiB limit and no compaction yet, so capacity exhaustion blocks new edge commands rather than silently forgetting attempt IDs.

The current core stores at most 4096 distinct capture IDs; interval unions are sorted and merged on insertion (`O(n log n)` per capture, `O(n)` storage). Projection is `O(m)` for `m` route segments. Sensor footprints beyond the strip interval model require a separate qualified adapter or polygon geometry. No 2-D area coverage claim follows from an interval unless full-strip coverage has been independently justified.

## Counterexamples

```text
Route A ───── verified ───── [ skipped 40..70 ] ───── verified ─── B
Camera A ──── captures ──── [ camera unavailable ] ─── captures ── B
```

The first case can arrive at B after a detour; the second can follow the route perfectly while the camera is unavailable. Neither is a verified survey. An off-corridor vehicle with a wide enough calibrated sensor can still provide valid strip coverage; navigation deviation and survey validity remain separate.

Waypoint checking, corridor tracking, interval unions, and recovery state machines are established techniques. The research claim to test is their **edge-obligation accounting contract with retained partial work and an algorithm-neutral onboard recovery interface**, against a strong coverage-aware baseline. No uniqueness claim is made here.

## Frozen initial experiment protocol (v1)

The first core matrix used eight coded cases S00, S01, S03, S04, S05, S06, S07 and S11 with seeds 1–5 and 1–100. The route is a 100 m straight strip, 4 m required half-width, 0.25 m truth raster, 1 m vehicle step and 2 m modeled along-track capture half-length. Faults used fixed 40–65 m camera/metadata outages and a 40–70 m detour. B0, B1, B2 and P received paired initial paths; B2 repeated the entire edge after an evidence gap and P traveled to its earliest missing prefix with 2 m overlap. `maximum_missing_m=0`. Initial recovery on deviation was gated off to isolate survey repair cost. The v1 files are retained and are not overwritten.

## Protocol amendment (v2)

After inspecting v1 results, v2 varies the fault start as `35 + seed % 11` metres while keeping a 25 m camera/metadata outage or 30 m detour. This reduces repeated identical distance outcomes but does not make seeds independent field trials. S01 still draws up to ±0.5 m jitter from the seeded generator. All other thresholds and the core matrix remain as above. V2 writes distinct raw files and manifest SHA-256 hashes. Further changes require v3. Both versions are deliberately simple geometric models, not flight dynamics or ArduPilot SITL.

## v5 recovery execution state

`DEVIATING` exposes bounded corridor departure. `deviation_hysteresis_m` sets
an inner reset threshold; unknown bounds never become a zero-error departure
proof. Three outside samples without definite re-entry is the default trigger.
Onboard recovery remains `RECOVERING` after command ACK, through navigation to
the selected target and bounded stationary dwell. Only successful continuation
from that physically verified point changes phase to `MOVING`. Active re-entry
blocks overlapping repairs; cooldown begins when re-entry completes. During
midflight departure the targeted point cannot lie ahead of current projected
progress. A wide camera's accepted off-corridor coverage remains credited.

`last_position_monotonic_ms` is absent before telemetry processing; agent status
exposes the last frame whose processing and immediate dispatch work have finished.
It provides an observation barrier for controlled simulation without fixed sleeps.
Reports emitted during processing can contain the preceding completed timestamp.
These simulator changes do not qualify live MAVLink recovery.

## v6 acquisition history and recovery extension

The current agent keeps up to 2,048 bounded position observations for capture
consistency at acquisition time. Requests declare `maximum_capture_age_ms` (0–10 s);
unknown motion between different timestamps requires a positive hard speed bound.
Late capture delivery does not move its acquisition timestamp. The agent rejects
captures after the edge deadline and cannot fulfill with stale endpoint evidence.
Historical v5/v5.1 artifacts retain their original software semantics and results.

The public repair policy now also includes departure-point, fixed-rewind and
`evidence-aware` recovery. The latter requires an explicit sensing envelope and
survey obligation. All policies can use a bounded stationary endpoint evidence
wait. These assumptions select a target; only accepted observations establish
coverage. See [the derivation and limits](RECOVERY_MODEL_V6.md) and the separately
versioned [v6 protocol](SIMULATION_PROTOCOL_V6.md). No new result is implied by the
existence of the policy or its restricted travel-minimum calculation.
