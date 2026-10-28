# VECR defense brief

## Title and domain

**Verified Edge Coverage and Recovery: An Onboard Execution Contract for UAV Graph Edges**

The subject is physical execution and surveying by autonomous UAVs. Rotor-router and
gossip motivate the graph-level setting but are external applications. The measured
digital trials run a multirotor model; the SITL qualification runs ArduPilot Copter.

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

During a sustained corridor departure the recovery target is `q=min(p,g)`, where
`p` is projected progress and `g` the earliest missing coordinate. At a qualified
stopped endpoint, the agent waits for bounded in-flight metadata and then targets
`q=g` if a gap remains. Physical re-entry qualification precedes continuation.
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
   interruptions and deliberately dishonest-sensor controls.

B2 is the **only measured recovery comparator** in this campaign. ArduPilot Mission
Rewind and other coverage or replanning systems are discussed as related work;
the experiment does not quantify superiority over those implementations.

The 22-condition, 30-paired-seed digital campaign contains 1,320 attempts. On the
jointly successful gap conditions, VECR's mean distance reduction against B2 was
11.6–31.3%. Examples of paired VECR–B2 distance differences and **pointwise**
95% bootstrap intervals (metres): N03 `-29.920 [-29.979, -29.862]`, N04
`-13.955 [-14.008, -13.904]`, N07 `-24.933 [-24.973, -24.896]`. These
intervals characterize the selected simulator/seed distribution; they do not
measure uncertainty about real camera calibration or new environments.
All 22 paired completion differences were zero in the observed seeds; a paired
bootstrap interval of `[0,0]` here does not prove population-level equivalence.
For a 30/30 condition, the condition-wise 95% Wilson lower bound is about 88.6%.

Across 1,200 honest-sensor attempts there were zero false fulfillments. The 120
dishonest-sensor controls all produced false fulfillments. Present both facts:
the negative controls expose the assumption boundary. A separate 100-attempt
direct/routed SDK campaign found identical recorded metrics. Twelve ArduPilot
SITL qualification flights exercised the production MAVLink path; accepted
survey claims there used synthetic bounds and sensing.

## Failure slide: explain the aborts

Use the data-derived failure table in the dissertation reader. N11 terminated on
re-entry command refusal, N12 on vehicle failsafe, and N13 on the battery threshold.
N10, N14, N15, N16 and N21 ended at their configured deadlines after unsuccessful
or unavailable recovery. No primary assignment was excluded as an outlier or
infrastructure error. The distance comparison conditions on joint completion;
completion counts and failure reasons are displayed separately.

## Boundary statement

The defensible novelty is the implemented, algorithm-independent onboard edge
acceptance and recovery contract, its conditional invariant, and its measured
full-agent validation. VECR does not claim a new rotor-router theorem, globally
optimal repair route, unconditional completion, real image-quality validation,
field airworthiness, or deployed multi-UAV coordination. Real camera calibration,
terrain, field flight and distributed planner integration require separate work.
