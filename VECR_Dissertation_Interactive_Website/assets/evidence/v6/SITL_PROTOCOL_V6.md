# ArduPilot SITL qualification v6

Written before flights, 2026-10-27. Digital campaign and client-reuse execution
have finished; independent reconstruction is the gate before flight execution.
SITL uses native ArduPilot Copter-4.7.2 from its pinned release commit, compiled
on this host. Every flight owns its process, temporary parameters and loopback
MAVLink ports. There is no connection to physical hardware or an existing flight.

## Evidence tiers

1. **Native MAVLink navigation:** SwarmKit receives the actual autopilot telemetry
   and its declared accuracy semantics. GPS accuracy fields remain backend-specific;
   they are not converted into deterministic hard bounds. Expected result is
   unverified arrival/coverage and eventual timeout even if the vehicle reaches B.
2. **SITL with synthetic bounded navigation and ideal nadir sensing:** A test-only
   backend decorator identifies itself as `ardupilot-sitl`. Commands still pass
   through production MAVLink execution to the real ArduPilot SITL flight dynamics.
   The agent receives perturbed SIM_STATE-based navigation with an explicit lab
   envelope; an independent recorder retains raw SIM_STATE and all MAVLink traffic.
   This tests integration under synthetic observations, not native navigation
   accuracy certification. Production `mavlink` recovery remains disabled.

The timing envelope assumes SIM_STATE transport age at receipt <=50 ms, actual
horizontal speed <=10 m/s, and uses additional age since receipt plus 0.2 m
position margin. Its receipt-age monitor rejects snapshots older than 100 ms.
These are laboratory assumptions, not verified physical-sensor hard bounds.
Post-run checks compare claims and measurements with recorded simulator truth.
SIM_STATE has no acquisition timestamp; the one-way age assumption cannot be
fully established from this trace. Report this limitation even if every test passes.
A stronger deployment needs qualified acquisition timestamps and calibrated bounds.

Synthetic camera geometry is a flat strip with an ideal route-aligned nadir
camera independent of vehicle attitude. It produces geometric observations;
no photogrammetry, real image quality, actual gimbal or hardware result is implied.

## Assignments and gates

Pilot: native/nominal plus synthetic nominal, 2.5 s metadata delay, two temporary
camera gaps, injected backend recovery rejection and real LAND mode override;
GW and E, seed 60001 (12 flights). Final: same six conditions × two policies ×
two separate noise seeds 61001/61002 = 24 flights. This is a small integration
qualification, not a powered comparison or independent distribution of airframes.
Do not present confidence intervals from two repeated flights as broad reliability.

Each attempt begins airborne at 10 m, commands a 20 m edge at 2 m/s and has an
explicit 90 s deadline (25 s for native-accuracy negative controls). Re-entry and
arrival use the same production dwell/acceptance machinery. All failures remain
recorded. The isolated adapter rejects targets outside its laboratory box; safety
health and GUIDED mode are checked by the agent for re-entry and continuation.
LAND must interrupt. An injected recovery rejection is labeled a backend fault,
not an observed ArduPilot rejection. Camera gaps last a fixed 16 s from attempt
start and apply spatially, independent of which policy requests a repair.

Archive revisions, executable hashes, parameters, process command lines, bridge
messages, raw MAVLink, flight logs, independent truth, synthetic acquisitions,
capture decisions, agent trace and summary. Reconstruct actual stopped arrival,
qualified geometric coverage and accepted-footprint containment independently.
No unsuccessful pilot is replaced or silently discarded. Further runs following a
fix use a new directory; final configuration is frozen before final execution.

## Pre-final engineering amendment — 28 October 2026

Pilots revealed and corrected port/clock/bootstrap errors and a production speed
ordering defect. Final runs use explicit isolated auxiliary ports, simulated RC
input with arming checks enabled, a Python/C++ steady-clock probe, bounded takeoff
ACK retries and an immutable executable. SwarmKit reapplies requested speed after
position-target dispatch because ArduPilot initializes a new GUIDED submode and
can reset a speed set beforehand. A refused post-dispatch speed request is an
explicit failure with unresolved possible motion. No result is rewritten.

Pilot_01 retains its eleven infrastructure failures and one executed native control;
pilot_02 retains two nominal flights; pilot_03 retains twelve qualification flights
before the speed correction. Pilot_04 repeats the full twelve assignments after
that correction, using four isolated workers. The final campaign keeps the same
six conditions, two policies and two seeds (24 flights), also with four workers.
The analysis is an integration qualification, not a new scientific efficacy trial.
All final experimental source must be committed before dispatch. Simulator and
agent executable hashes, source revision and parameter files identify the run.
