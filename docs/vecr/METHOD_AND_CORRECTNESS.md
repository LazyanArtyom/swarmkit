# VECR method and conditional correctness

## Contract

A directed graph step is a planner decision. Arrival at its target, route traversal, and observation of its assigned geographic strip are separate physical predicates. The current implementation uses a local metric straight segment and a strip whose full cross section is certified by each accepted capture adapter. It stores a sorted union of along-route intervals. A survey edge is fulfilled only after stop-style arrival and accepted survey intervals leave at most `maximum_missing_m`. A traversal edge additionally requires bounded movement evidence; the combined obligation requires both traversal and survey. An arrival-only edge requires only qualified arrival. No rotor or gossip state changes in SwarmKit. The agent rejects bent and multi-point routes because it currently dispatches a single GOTO to B.

Each request binds mission ID/revision, edge ID, unique attempt ID and drone ID. Captures with a different attempt ID are rejected. The request's route frame must be fixed; WGS84 helpers use a local equirectangular plane anchored at the supplied latitude/longitude. This approximation is suitable only for short local routes, away from poles and longitude wrap. A production geodetic adapter should enforce that domain.

## Evidence and guarantees

The sensor RPC accepts capture identity, mission/edge/attempt binding, a configured calibration ID, camera center, altitude above ground, field of view, deterministic hard pose bounds and quality assertions. It does not accept a caller-supplied covered interval. The agent requires a bounded vehicle position observation within 500 ms of the capture timestamp and checks that the claimed camera center is kinematically consistent with that position, the declared maximum speed and both hard bounds. It then computes a conservative interval with `BuildNadirStripEvidence` using the requested route and strip half-width. The core rejects invalid ranges, duplicates, stale or mismatched captures. This restricted calculation assumes a straight route, flat ground, rectangular nadir footprint aligned to the route, established image quality, and hard altitude/position bounds. The mTLS sensor allowlist authenticates one configured adapter; it does **not** prove that its camera pose, image quality, calibration, alignment or flat-ground assertions are true. A real calibrated producer has not been validated, so survey evidence in current tests remains simulated geometric evidence.

Stop-style arrival means two qualifying observations at least 200 ms apart, each with a known deterministic hard horizontal position bound, vertical position bound and horizontal speed bound, within the configured three-dimensional radius and maximum speed. A qualification is revoked on a later away, invalid or ambiguous observation; final survey evidence cannot use a position older than 1 s. A terminal fulfillment records that the obligation was satisfied at that time and remains terminal if the vehicle later departs. Fly-through arrival is not implemented. Backend command ACK and backend goal-reached messages do not satisfy this policy.

For deterministic traversal, both endpoint observations must have **hard** horizontal position error bounds and the request must have a valid physical maximum speed. The endpoint displacement must be at most `Vmax*dt+eps0+eps1`; an impossible jump cannot prove traversal. For distance to the route, `D0 = d(p̂0, route)+eps0`, `D1 = d(p̂1, route)+eps1`, and `dt=t1-t0`, the 1-Lipschitz property gives `d(p(t), route) <= min(D0+Vmax(t-t0), D1+Vmax(t1-t)) <= (D0+D1+Vmax*dt)/2`. If the last value is at most corridor width, the interval is conformant under the declared assumptions. Sample-only corridor checks are experimental baseline semantics, never a deterministic whole-interval guarantee. The implementation only credits forward projected progress and declines ambiguous nearest-route projection.

**Sound survey proposition.** If every accepted conservative footprint lies inside ground that was actually observed at the required quality, the union of accepted footprints lies inside actually observed ground. Thus a remaining required strip of at most the configured tolerance implies modeled survey fulfillment to that tolerance. The sensor inclusion assumption is substantive and unvalidated for a real camera.

**No skipped gap.** The interval union never fills an uncovered hole. Fulfillment checks the entire missing length, not maximum observed route coordinate. Later disjoint intervals remain available without advancing the contiguous prefix. An accepted interval remains stored after a separate navigation deviation, unless its original evidence is explicitly withdrawn. This may save repeated surveying; travel savings still include transit to the re-entry point.

## Recovery and state

The event processor proposes re-entry before the earliest missing interval with configured overlap. The agent dispatches recovery only for its simulator backend when explicitly enabled, with authority, battery, navigation bounds, backend health and autopilot mode checks. Re-entry itself requires a bounded stop-style dwell before the agent commands B. Geofence and live mission feasibility are not yet qualified; MAVLink recovery is disabled. Recovery is bounded by attempt count and cooldown. Terminal completion/interruption is idempotent. Unknown position produces `UNKNOWN` and does not create movement evidence; earlier capture intervals are retained. An unresponsive vehicle times out to an interrupted report when the agent remains running.

`ExecuteEdge` retries with the same drone and attempt ID return the recorded original dispatch and state without redispatching; a changed request is rejected. The agent keeps up to 4096 attempts in process memory and can query an earlier terminal attempt after a newer one starts. This ledger is not durable across an agent restart. A disconnected planner must treat a missing post-restart attempt as unknown, not as never executed. Exactly-once execution across process failure is not claimed.

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
