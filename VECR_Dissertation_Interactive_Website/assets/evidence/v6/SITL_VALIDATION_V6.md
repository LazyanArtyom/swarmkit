# SITL validation v6 — completed 28 October 2026

## Qualification outcome

All **24/24 final assignments** executed with zero infrastructure failures and
passed independent reconstruction. There were **12 fulfilled survey attempts**
(nominal, delayed metadata and multiple gaps, four each) and **12 intended negative
outcomes** (native-accuracy controls, rejected recovery and LAND, four each).
A passed qualification check does not mean that every mission fulfilled.

The native controls physically reached B with modeled camera coverage but reported
no fulfillment because their navigation/sensing evidence was not hard-bound
qualified. In the synthetic tier both policies completed all four multiple-gap
flights, with one real commanded re-entry per flight. GW averaged 55.490 m and
36.520 s; E averaged 60.626 m and 39.248 s. Two repeats per cell support only a
small integration qualification, not a population efficacy or reliability claim.

Independent checks reconstructed **1,058 accepted footprints** and **four sampled
re-entry dwells**, verified actual sampled stopped arrival, retained acquisition
truth, capture age/containment and synthetic pose consistency, and confirmed that
no recovery command followed the injected LAND override. Recovery refusals were
injected by the test backend and labeled as such; they are not ArduPilot failures.

## Software and reproducibility

SwarmKit: `df65c61f57fade8ba88f877329ffd31d0ecbe175`.
ArduPilot Copter-4.7.2: `fb773567896014da007bca58358704d4a0d8516f`.
Agent trial executable SHA-256:
`1f3be471f9fb3aa78db63329947cd9bea0154d0b19fa2d5f156e32666bc25409`.
ArduPilot executable SHA-256:
`d07af53d9a2a5fb0c980cbab41057ff4273195ffb32d16cfcc6b7e8e1ff68efd`.

The final manifest records a clean experimental source tree, immutable executable,
all 24 assignments, cross-language clock probe and four workers. Per-flight folders
include parameter files, process commands, raw MAVLink, flight logs, independent
SIM_STATE truth, acquisitions, evidence decisions, commands and agent state trace.
Host/Python environments and pinned submodule identities are archived. Reproduction
instructions are in `results/vecr/sitl_v6/README.md`. Historical research is unchanged.

## Failed attempts and corrections

* Pilot 01: 12 assignments, 11 infrastructure failures, one executed native control.
  TCP/auxiliary port collisions, missing simulated receiver input, one-shot failed
  takeoff and macOS Python/C++ monotonic clock mismatch were diagnosed and fixed.
  Arming checks stayed enabled; logs remain intact. Its `.tlog` files contain raw
  MAVLink packets without tlog framing; later runs correctly use `.raw`.
* Pilot 02: two nominal flights; both independently reconstructed. They exposed a
  backend speed-ordering defect: about 6.508 m/s peak despite a 2 m/s request.
* Pilot 03: twelve flights, all reconstructed, including actual re-entry and safety
  interruption; 394 accepted footprints and two sampled dwells. Before speed fix.
* Pilot 04: twelve flights after correction, all reconstructed; 525 accepted
  footprints and two sampled dwells. Nominal peaks about 2.06 m/s.
* Final: twenty-four flights with the same corrected protocol; all reconstructed.

SwarmKit now reapplies requested speed after position dispatch because ArduPilot's
GUIDED submode initialization can reset a pre-target speed setting. Regression
covers reposition ACK, fallback setpoint, and speed refusal after navigation was
already sent (explicit failure; motion may remain active). Full release and UBSan
suites pass. No failed record was removed or relabeled as a successful flight.

## Explicit limits

Native MAVLink accuracy remains backend-specific and production MAVLink recovery
remains disabled. Only the deliberately named SITL test adapter permits the
synthetic-bound recovery path. Synthetic navigation is perturbed SIM_STATE with a
laboratory envelope; the camera is ideal flat nadir geometry, not real imagery.
SIM_STATE lacks acquisition timestamps, so its assumed <=50 ms upstream transport
age cannot be proved from these records. Observed snapshot age and pose consistency
pass, but that does not certify the assumption. Independent dwell reconstruction
is sampled and does not establish continuous flight bounds. No actual calibrated
image quality, obstacle-aware recovery, physical aircraft or operational reliability
is validated. E's additional cost is reported; SITL does not establish novelty.

## Complete descriptive results

# ArduPilot SITL v6: actual qualification results

24/24 assignments returned records; all passed independent sampled-truth reconstruction.
Two flights per cell; no broad reliability confidence interval is inferred.
Native navigation accuracy remains backend-specific. Positive acceptance uses synthetic navigation and ideal camera observations.

| Observation tier / fault | Policy | Reported / 2 | Sampled truth / 2 | Mean travel (m) | Mean duration (s) |
|---|---|---:|---:|---:|---:|
| raw_mavlink / none | GW | 0 | 2 | 20.362 | 25.072 |
| raw_mavlink / none | E | 0 | 2 | 20.329 | 25.043 |
| synthetic_bounded / none | GW | 2 | 2 | 20.124 | 12.248 |
| synthetic_bounded / none | E | 2 | 2 | 20.090 | 12.061 |
| synthetic_bounded / delay | GW | 2 | 2 | 20.169 | 13.059 |
| synthetic_bounded / delay | E | 2 | 2 | 20.161 | 13.188 |
| synthetic_bounded / multi_gap | GW | 2 | 2 | 55.490 | 36.520 |
| synthetic_bounded / multi_gap | E | 2 | 2 | 60.626 | 39.248 |
| synthetic_bounded / command_rejected | GW | 0 | 0 | 20.196 | 15.032 |
| synthetic_bounded / command_rejected | E | 0 | 0 | 20.217 | 15.181 |
| synthetic_bounded / land_override | GW | 0 | 0 | 8.935 | 5.861 |
| synthetic_bounded / land_override | E | 0 | 0 | 8.940 | 5.854 |

Accepted captures checked: 1058; sampled re-entry dwells checked: 4.

Costs in failed/interrupting conditions describe actual execution; they are not efficiency gains.
The <=50 ms upstream SIM_STATE age assumption is unproven because source acquisition time is absent. Stop dwell is sampled, not continuous. Camera geometry is ideal, flat and route-aligned; no actual imagery is validated.
