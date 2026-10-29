# VECR defense brief

## Title and domain

**Verified Edge Coverage and Recovery: An Onboard Execution Contract for UAV Graph Edges**

The subject is physical execution and surveying by autonomous UAVs. Rotor-router and
gossip motivate the graph-level setting but are external applications. The measured
digital trials run a multirotor model; the archived v7 SITL qualification ran ArduPilot Copter. Revised-method v8 SITL is TBD.

## One-minute contribution statement

A graph planner can select a directed edge and a flight controller can report arrival
while a required survey strip remains unobserved. VECR turns the assigned edge into
an exact, versioned physical obligation. The SwarmKit agent accepts only qualified
bounded navigation and sensor evidence, retains disjoint verified intervals, detects
the earliest missing interval, and makes a bounded local repair attempt. A command
acknowledgement never constitutes arrival or survey fulfillment. The planner remains
free to choose any graph algorithm; SwarmKit supplies the same physical execution
contract.

## Exact conditional claim

Let `C_j` be each accepted conservative camera footprint and `A_j` the true ground
region acquired at the required quality. If `C_j ⊆ A_j` for every accepted capture,
then `⋃C_j ⊆ ⋃A_j`. Therefore accepted full-strip coverage implies actual coverage
to the declared allowance, provided the camera assertions, calibration, hard error
bounds, timestamps, identity and interval bookkeeping are valid. The accepted union
retains gaps and cannot credit them through movement or command acceptance.

During a sustained corridor departure *before B*, the recovery target is
`q=min(p,g)`, where `p` is projected progress and `g` the earliest required
unverified coordinate. Physical stopped re-entry qualification precedes
continuation. At B, the agent waits briefly for eligible delayed metadata,
then reports ARRIVED with separate coverage state and residual intervals;
it does not command a backward repair flight.
This rule does **not** guarantee convergence: capture, energy, commands, repair
budget or deadline can fail. Authentication of a sensor adapter does not establish
physical image truth.

## Evidence to show on slides

1. `contract.svg`: ACK, stopped arrival and surveyed coverage have distinct meanings.
2. `recovery.svg`: the current single production rule, retained intervals, safety
   invariant and no-convergence boundary. The old `recovery_v6.svg` was removed.
3. `comparison.svg` and `paired_comparisons.csv`: B2 has the same verifier, sensing,
   dynamics, safety and budget; only the repair target differs.
4. `completion_matrix` plus `per_attempt.csv`: all assigned outcomes, including
   failed executions and deliberately dishonest-sensor controls.

B2 is the **only measured recovery comparator** in this campaign. ArduPilot Mission
Rewind and other coverage or replanning systems are discussed as related work;
the experiment does not quantify superiority over those implementations.

The v7 digital and SITL measurements used a retired endpoint-repair policy and
are historical. The frozen v8 digital results are in
`results/vecr/v8/primary/analysis/RESULTS.md` with a source and binary manifest. The v8 pilots are
implementation gates, not confirmatory estimates. V8 dishonest-input
controls show why truthful camera metadata is an assumption. The earlier SITL
qualification used synthetic bounded sensing and does not validate the revised
endpoint contract.

## Failure slide: explain the aborts

Use the v8 per-attempt failure reasons and denominator from the frozen campaign.
Endpoint gaps are ARRIVED/INCOMPLETE results, not failed local corrections.
The distance comparison conditions on joint true-and-reported completion;
arrival and coverage counts must be displayed separately.

## Boundary statement

The defensible novelty is the implemented, algorithm-independent onboard edge
acceptance and recovery contract, its conditional invariant, and its measured
full-agent validation. VECR does not claim a new rotor-router theorem, globally
optimal repair route, unconditional completion, real image-quality validation,
field airworthiness, or deployed multi-UAV coordination. Real camera calibration,
terrain, field flight and distributed planner integration require separate work.
