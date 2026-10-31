# Planned ArduPilot SITL qualification — VECR v8

**Status: not executed. Every v8 SITL measurement is TBD.** The current digital
campaign uses an agent-backed geometric simulator. Retired SITL data used a
different endpoint policy and were removed from the current checkout; they are
recoverable from Git history and are not evidence for this protocol.

## Qualification boundary

Freeze a new source revision, v8-capable SITL runner, ArduPilot revision, binary
hashes, parameters, assignments and SHA-256 manifest before executing flights.
The retired v7 SITL runner and trial executable are absent from the current
checkout. Implement and audit a new harness for the current edge API, terminal
phases and no-return behavior at B. Assign a new versioned results directory.
Keep every assigned flight and unsuccessful pilot; never replace an
inconvenient result.

Run the complete SwarmKit agent and MAVLink backend with a real ArduPilot Copter
SITL process. Record raw MAVLink, backend command requests and ACKs, agent
status, observation provenance, camera reports, actual simulator trajectory,
capture truth, timing, failures, and exact mission/revision/edge/attempt/drone
identity. The independent oracle must not supply its truth to VECR.

## Minimum scenarios

1. Native backend accuracy without a declared hard bound: fail closed; do not
   treat a reported accuracy estimate as a deterministic bound.
2. Nominal edge: establish stopped arrival and separately score survey evidence.
3. Sustained departure **before B**: select `q=min(progress, first required
   unverified coordinate)`, command return, verify bounded stopped re-entry and
   sampled dwell, then resume toward B.
4. Delayed and missing camera metadata: wait only within the configured evidence
   horizon; allow `ARRIVED` with `survey_verified=false` and missing intervals.
5. Arrival at B with a residual gap: assert no post-arrival return `GOTO`.
6. Command refusal, low battery, motor failure, missing hard bounds, deadline,
   cancellation and safety override: record terminal reason, hold behavior and
   physical uncertainty separately from ACKs.

Check all accepted capture footprints against independent acquisition truth,
sampled arrival/re-entry dwell, source and assignment identity, command order and
absence of any accepted recovery command after qualified B arrival. Report all
outcomes and failed runs. A small SITL qualification tests integration with an
autopilot process; it does not establish field reliability or real camera
quality. Real focus, exposure, occlusion and footprint claims require calibrated
images and independent physical ground truth.
