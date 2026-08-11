# TCRR experimental protocol

Status: frozen protocol draft 1, 2026-08-11. Any change after confirmatory data collection starts must be versioned, justified, and reported as a protocol deviation. Pilot data may be used to validate tooling and choose nuisance parameters, but it must not enter confirmatory estimates.

## Purpose and scope

This protocol defines the empirical claims needed by the TCRR paper. It covers calibration of physical-position enclosures, same-log comparison of certificate rules, closed-loop execution, and adversarial stale/duplicate trials. It does not define the rotor-router algorithm or place certificate/commit policy inside SwarmKit.

The confirmatory implementation and configuration are identified by immutable source revision, scenario-set revision, SwarmKit protocol revision, TCRR configuration hash, calibration profile/version, vehicle profile/version, and recorder schema version. Every run records those identifiers and its random seed.

## Units and event definitions

The independent experimental unit is one complete seeded run. A run begins after initialization and preflight validation and ends when the scenario reaches its declared terminal condition. Transitions within a run are repeated, clustered observations; they are not treated as independent samples.

Definitions used in all result tables:

- **Transition opportunity:** one scenario-authorized occasion on which the controller has a committed source state and may select one legal outgoing transition. A retry is not a new opportunity unless the scenario explicitly abandons the old operation and selects a new logical transition.
- **Attempted transition:** an opportunity for which the Agent accepts a physical goal for dispatch and allocates a new physical-attempt handle. A backend ACK is an outcome of that attempt, not the definition of an attempt.
- **Eligible telemetry sample:** a newly updated position sample that satisfies the method's predeclared identity, producer-session, sequence, timing, validity, estimator, failsafe, and uncertainty requirements. Repeated cached position, duplicate transport delivery, a sample bound to another attempt, and an old Agent session are not new eligible samples.
- **Accepted certificate:** the method records that its complete acceptance predicate became true for one current physical attempt. This is a decision event; it is not itself a logical state mutation.
- **Commit:** the guarded logical-state transaction succeeds exactly once and durably changes the logical UAV node and source rotor state. A failed compare-and-set or rejected duplicate is not a commit.
- **False commit:** a successful logical commit for which ground truth is not inside the selected target region at the commit's effective time, irrespective of what estimated telemetry reported.
- **Invalid trace step:** a committed projected step that is not the unique legal rotor-router successor of the immediately preceding committed logical state. This includes wrong source, wrong target, wrong rotor successor, and a commit made from a stale source-state version.
- **Stale commit:** a successful commit whose evidence or handle belongs to a superseded operation, older physical attempt, older goal revision, or older Agent session.
- **Duplicate commit:** more than one successful logical commit for the same transition identity, or a second successful state effect from replay of already-consumed evidence.
- **Missed certificate:** the vehicle continuously occupies the target region and meets the method's truth-level speed condition for at least the configured hold duration, but the method does not accept before the attempt terminates. It is reported only where the scenario supplies sufficiently dense ground truth for that interval.
- **Timeout:** the current physical attempt reaches its predeclared execution deadline without a successful logical commit. Timeout is an availability outcome and never implies arrival.
- **Retry:** dispatch of another physical attempt for the same logical operation. Every retry receives a new Agent-generated physical-attempt ID and session-scoped attempt revision, even when goal ID, goal revision, operation ID, and operation-attempt revision are unchanged. Retry alone never changes logical rotor state.

One opportunity can contain zero, one, or several attempted transitions because backend rejection can precede acceptance and a policy can retry. The denominator for false-commit risk is transition opportunities. Attempt-level ACK, timeout, retry, and certificate outcomes use attempted transitions as their denominator and are labeled accordingly.

## Commit time and ground-truth label

The effective commit time, `t_commit`, is the monotonic timestamp captured inside the commit transaction immediately after its state/version guard has succeeded and immediately before publishing the successful commit record. The state mutation and record sequence are one transaction outcome. Wall time is descriptive only and is not used to order correctness events.

A commit is physically valid only if the ground-truth vehicle position at the same `t_commit` is a member of the complete target-node region, including horizontal and vertical constraints. If a speed-at-commit analysis is reported, truth velocity is evaluated at that same time.

Ground truth and execution records must share a validated clock mapping. The run records mapping method, offset, drift model, uncertainty, fit residuals, and calibration interval. Truth at `t_commit` is obtained by interpolation between the two bracketing truth samples only when both are within the configured maximum bracketing gap and the interpolation method was frozen before the run. Otherwise the opportunity is labeled `truth_unavailable`; it is never silently labeled safe. The default interpolation is linear Cartesian interpolation in the local experiment frame. The maximum permitted bracket is 50 ms for simulator/SITL truth and must be declared for each physical tracking system before confirmatory flight.

For a cylindrical target with center `q`, radius `r`, and half-height `h`, occupancy is:

```text
norm(p_truth_xy(t_commit) - q_xy) <= r
and
abs(p_truth_z(t_commit) - q_z) <= h
```

Boundary equality counts as occupied. Target geometry and coordinate transformation are fixed in the run manifest. Estimated flight telemetry must never be substituted for ground truth.

## Confirmatory outcomes

The primary outcome is the paired absolute difference in false-commit risk per transition opportunity between full TCRR and the best predeclared non-TCRR baseline. The baseline set is command ACK, one-sample fixed radius, radius plus dwell, SwarmKit `GOAL_REACHED`, and position-only TCRR. “Best” means the lowest false-commit risk on the separate tuning set; ties are broken by lower median completion time. The chosen baseline is frozen before confirmatory runs.

Primary reporting uses a two-sided 95% confidence interval. Point estimates retain the opportunity denominator, while uncertainty is run-clustered. The primary interval is a paired run bootstrap with 10,000 resamples of complete seed pairs. Seeds, not transitions, are resampled. A paired generalized linear mixed model with a run random intercept is a sensitivity analysis, not a replacement chosen after seeing results.

Secondary safety outcomes are invalid trace steps, stale commits, and duplicate commits. Secondary availability and performance outcomes are missed certificates, timeout and retry rates, end-to-end completion time, added certification latency, CPU time, peak memory, and network bytes. Latency is reported as median and IQR with a run-clustered interval. Multiplicity across confirmatory secondary method comparisons is controlled by Holm's procedure at family-wise alpha 0.05.

When zero corrupt commits are observed, the paper reports both the count and a one-sided 95% Clopper-Pearson upper bound using the stated opportunity denominator. Because transition outcomes are clustered, the run-clustered bootstrap/model result remains primary and the opportunity-level exact bound is explicitly labeled a sensitivity bound.

## Precision and sample-size rule

Simulation confirmatory collection uses at least 100 complete paired seeds per method and at least 3,000 opportunities in total per method. Collection then proceeds in fixed batches of 25 paired seeds, without inspecting method ranking, until either:

1. the 95% run-clustered interval for the paired primary risk difference has half-width at most 0.002; and
2. if full TCRR has zero false commits, the opportunity-level one-sided 95% upper bound is at most 0.001,

or a maximum of 500 paired seeds is reached. Failure to reach precision is reported; the interval is not suppressed. Pilot runs determine scenario duration and verify that the minimum opportunity count is feasible, but do not alter the outcome threshold after confirmatory collection starts.

SITL and physical-flight cohorts are reported separately and are not pooled with simulator runs. Each confirmatory SITL condition uses at least 30 paired seeded runs. Physical-flight sample size is declared in the flight-specific addendum after tracking precision and operational limits are measured; no physical data is collected as confirmatory data before that addendum is frozen.

## Paired seeds and method ordering

Offline comparisons apply every decision method to the same immutable normalized evidence log, target geometry, operation identity, and ground-truth stream. This is exact pairing.

Closed-loop comparisons use the same scenario seed and fault seed for each method, but a method can change later commands and therefore later physical trajectories. Each `(scenario, seed, method)` is an independent run. Method execution order within a seed block is randomized and recorded. Physical-flight order additionally uses balanced blocks to limit battery, weather, and thermal drift. A failed run is not silently replaced with a favorable new seed; any replacement seed is appended and both records remain in the accounting table.

## Offline and closed-loop analyses

The two analyses answer different questions and are never pooled:

- **Offline same-log analysis** replays one normalized SwarmKit evidence log into every certificate method. It isolates decision logic, stale/duplicate handling, and certification latency. It cannot claim that all methods would have produced the recorded trajectory.
- **Closed-loop end-to-end analysis** lets each method's decisions affect correction, retry, and subsequent commands. It measures realized false commits, trace validity, completion, and overhead for the whole policy. Pairing is by scenario seed, not by identical telemetry.

The paper's main safety comparison reports both. Any disagreement is interpreted as a closed-loop policy effect rather than resolved by selecting one analysis post hoc.

## Calibration and held-out validation

Uncertainty calibration data and confirmatory transition data are disjoint by run seed and flight session. Calibration profiles are fitted on the declared training set and frozen with a content hash. Held-out enclosure coverage is reported separately for hover, cruise, turn, and disturbed regimes, with sample count, run count, empirical coverage and a run-clustered 95% interval, plus 99th-percentile position error.

Calibration logs record airframe, autopilot and estimator versions, telemetry rate, accuracy-field source, coordinate transform, truth source, clock mapping, environmental regime, and profile/version. A backend-provided accuracy number with unknown statistical semantics remains backend-specific; calibration may produce a new empirical-bound descriptor but must not relabel the raw source as a deterministic hard bound.

## Exclusions, corruption, and incomplete runs

Exclusions are determined without reference to method outcome. Permitted confirmatory exclusions are:

- initialization failure before the first transition opportunity;
- missing or invalid ground-truth clock calibration detected by the pre-run check;
- recorder corruption that makes the execution order or commit-time truth unreconstructable;
- an operator safety abort caused by an external hazard unrelated to the evaluated method.

Backend failure, packet loss, estimator degradation, timeout, retry exhaustion, method-induced abort, and failure to finish are outcomes, not exclusions. A run that becomes incomplete after its first opportunity remains in intention-to-treat closed-loop summaries with all observable denominators and its terminal reason.

Every scheduled run receives exactly one disposition: complete, incomplete-observable, excluded-before-opportunity, or corrupt-unusable. Counts and reasons are published by method. A corrupt or truth-unavailable opportunity is excluded only from outcomes requiring its missing label; it remains in availability and data-integrity denominators. No log is repaired in place. Recovery creates a derived artifact with a new hash and an auditable transformation record.

## Adversarial trials

The stale/duplicate suite includes delayed samples, sequence gaps, duplicates, reordering, old goal revisions, superseded physical attempts, Agent restart/session mismatch, ACK without movement, estimator degradation, and target crossing. Each randomized trial records a seed. The confirmatory claim reports number of complete trials, opportunities, attempted stale/duplicate injections, successful corrupt commits, and the predeclared upper bound.

## Analysis reproducibility

The frozen analysis consumes only immutable run manifests, normalized execution records, truth records, and calibration profiles. It verifies hashes and schema versions before analysis. Derived tables include run-level rows so clustered calculations can be reproduced. Exploratory plots and parameter searches are clearly labeled and never overwrite confirmatory output.
