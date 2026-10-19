# VECR method and conditional correctness

## Contract

A directed graph step is a planner decision. Arrival at its target, route traversal, and observation of its assigned geographic strip are separate physical predicates. The current implementation uses a local metric polyline and a strip whose full cross section is certified by each accepted capture adapter. It stores a sorted union of along-route intervals. A survey edge is fulfilled only after stop-style arrival and accepted survey intervals leave at most `maximum_missing_m`. A traversal edge additionally requires bounded movement evidence; an arrival-only edge requires only qualified arrival. No rotor or gossip state changes in SwarmKit.

Each request binds mission ID/revision, edge ID, unique attempt ID and drone ID. Captures with a different attempt ID are rejected. The request's route frame must be fixed; WGS84 helpers use a local equirectangular plane anchored at the supplied latitude/longitude. This approximation is suitable only for short local routes, away from poles and longitude wrap. A production geodetic adapter should enforce that domain.

## Evidence and guarantees

Survey evidence is an explicit capture record containing capture identity, calibration ID, timestamp, success and a conservative full-strip interval. The core rejects missing calibration, failed/uncertain quality, invalid ranges and duplicates. The source adapter is responsible for proving the footprint actually covers the full required strip cross section. `BuildNadirStripEvidence` provides one restricted adapter calculation: a straight route, flat ground, rectangular nadir footprint aligned to the route, established image quality, and deterministic hard altitude/position bounds. It shrinks along and cross extents by the uncertainty bounds and rejects a footprint that cannot cover the full strip width. Its assertions still need a real calibrated producer; a camera ACK alone cannot establish them. In simulator trials, the camera is clearly modeled, so claims are about modeled geometric coverage only.

For deterministic traversal, both endpoint observations must have **hard** horizontal position error bounds and the request must have a valid physical maximum speed. For distance to the route, `D0 = d(p̂0, route)+eps0`, `D1 = d(p̂1, route)+eps1`, and `dt=t1-t0`, the 1-Lipschitz property gives `d(p(t), route) <= min(D0+Vmax(t-t0), D1+Vmax(t1-t)) <= (D0+D1+Vmax*dt)/2`. If the last value is at most corridor width, the interval is conformant under the declared assumptions. Sample-only corridor checks are experimental baseline semantics, never a deterministic whole-interval guarantee. The implementation only credits forward projected progress and declines ambiguous nearest-route projection.

**Sound survey proposition.** If every accepted conservative footprint lies inside ground that was actually observed at the required quality, the union of accepted footprints lies inside actually observed ground. Thus a remaining required strip of at most the configured tolerance implies modeled survey fulfillment to that tolerance. The sensor inclusion assumption is substantive and unvalidated for a real camera.

**No skipped gap.** The interval union never fills an uncovered hole. Fulfillment checks the entire missing length, not maximum observed route coordinate. Later disjoint intervals remain available without advancing the contiguous prefix. An accepted interval remains stored after a separate navigation deviation, unless its original evidence is explicitly withdrawn. This may save repeated surveying; travel savings still include transit to the re-entry point.

## Recovery and state

The event processor proposes re-entry before the earliest missing interval with configured overlap. It does not issue flight commands itself. An onboard integration must check authority, backend readiness/capability, geofence/mission feasibility and autopilot failsafe before dispatch. Recovery is bounded by attempt count and cooldown. Terminal completion/interruption is idempotent. Unknown position produces `UNKNOWN` and does not create movement evidence; earlier capture intervals are retained. An unresponsive vehicle cannot emit a fabricated terminal report.

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
