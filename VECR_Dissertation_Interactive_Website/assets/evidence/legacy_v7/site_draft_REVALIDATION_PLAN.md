# Required implementation and experiment revalidation

## Code / API changes (not performed in this web project)

- [ ] Remove the automatic endpoint-backtrack branch. Any qualified arrival at B must lead to a terminal outcome after no more than a bounded, passive evidence flush.
- [ ] Split navigation `ARRIVED` from evidence `VERIFIED`, `INCOMPLETE`, `PENDING_EVIDENCE`.
- [ ] Expose on-demand `getEdgeStatus(ExecutionId)` with coherent immutable attempt identity and intervals.
- [ ] Implement one logical terminal `onEdgeExecutionFinished` for arrival, failed correction, cancellation and unknown outcome, with durable idempotent retrieval after reconnect.
- [ ] Enforce max local recovery attempts, per-correction timeout and attempt deadline; release control authority after termination.
- [ ] Document evidence cutoff, late-capture policy and pending/confirmed distinction.

## Mandatory automated tests

- [ ] Mid-edge departure triggers `DEVIATING/RECOVERING` before B; repair target is evidence-selected and lies on authorized edge.
- [ ] Qualified re-entry dwell before resuming; ACK never advances the recovery state by itself.
- [ ] Gap at B produces `ARRIVED/INCOMPLETE` and **zero** subsequent `GOTO` commands from VECR.
- [ ] No gap at B produces `ARRIVED/VERIFIED`, without spurious repair.
- [ ] Rejected return, invalid bounds, exhausted K, deadline and battery intervention produce one terminal `FAILED/UNKNOWN` notification.
- [ ] Cancellation, duplicate IDs, restart/reconnect, late evidence and concurrent authority requests cannot duplicate terminal callbacks or dispatch unexpected commands.
- [ ] Snapshots are pull-only, consistent, and include pending/unverified intervals with units and reference frame.
- [ ] Sensor dishonest controls demonstrate the precise trust boundary; independent oracle never leaks to verifier.

## New experiments and calculations

1. Freeze a new source revision, binary hash, simulator parameters and seeds.
2. Run at least nominal, two sensing widths under in-flight deviation, multiple deviation locations, false-positive deviation, command refusal, motion/sensor failure, finite budget, cancellation, short deadline, and *arrival with known residual gap* cases.
3. Repeat paired seeds for baseline comparison with **in-flight-only** corrections; no policy may receive an extra endpoint-recovery maneuver.
4. Compute empirical arrival rate, sensor completeness, fraction ARRIVED/INCOMPLETE, residual interval length, safety failure counts, correction latency, flight distance/time, and API callback compliance. Do not condition all conclusions on success pairs.
5. Use a separate truth reconstruction and report honest and dishonest sensing separately.
6. Re-run native SITL and, when qualified, calibrated-camera field experiments with safe recovery procedures.
7. Recompute tables, confidence intervals, figures and references. Do not overwrite v7 archived measurements or silently relabel old `FULFILLED` as new `ARRIVED`.

## Existing numbers that may be useful without becoming new-policy claims

Historical N07–N09 traces and matched v7 distance differences are suitable as *preliminary in-flight evidence*. Nominal sensor qualification, old client reuse and old SITL investigations can guide regression test design, but **even these need confirmation under the changed binary**. All results relying on endpoint repair (notably N03/N04/N06/N17/N18) need new calculations. The global 1,320-trial completion matrix and percentage claims cannot be carried over unchanged.
