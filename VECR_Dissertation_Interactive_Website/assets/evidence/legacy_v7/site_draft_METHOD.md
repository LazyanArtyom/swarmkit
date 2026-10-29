# VECR revised method — normative research specification (2026-10-29)

## Status

This document describes the **agreed revised design**. The frozen v7 experimental binary bundled in the original website implemented an additional automatic post-arrival repair branch, which is expressly **not** part of this specification. New SDK implementation, independent validation and fresh numerical evaluation are required before claiming an experimentally completed revised method.

## Scientific problem and contribution

A graph planner specifies a directed physical edge A→B, while the real UAV's motion and sensor observations can violate required corridor or survey constraints. Reaching B is distinct from acquiring every required camera interval. VECR is an algorithm-independent execution supervisor that conservatively verifies physical evidence and performs **bounded in-flight** correction of sustained deviation; it reports any unresolved coverage without doing graph-level planning or post-arrival return.

## Execution interface

- `executeEdge(request) -> ExecutionId`: binds mission, revision, edge, attempt and UAV identifiers; physical route; sensing/traversal obligations; command authority and safety/retry/deadline limits.
- `getEdgeStatus(id) -> EdgeExecutionStatus`: pull-only snapshot of progress, phase, conservatively verified intervals, unverified/pending intervals, physical arrival qualification, recovery budget and active target.
- `onEdgeExecutionFinished(EdgeExecutionResult)`: exactly one **logical terminal notification** for `ARRIVED`, `FAILED`, `CANCELLED` or `UNKNOWN`; includes separate `coverage_state` (`VERIFIED`, `INCOMPLETE`, `PENDING_EVIDENCE`), interval data, evidence cutoff, identity, reason and last known vehicle state. No per-step callbacks are required.
- `cancelEdge(id)`: requests termination and command release. The agent must not silently treat uncertain cancellation as success.

These are desired public semantics. Existing v7 RPCs such as `GetEdgeStatus` are implementation starting points, but the revised callback and arrival/coverage separation must be developed and tested.

## Exactly one local recovery policy

During A→B, the supervisor monitors conservative navigation evidence and sensor coverage. A configured sustained, qualified corridor deviation triggers `DEVIATING`, then `RECOVERING`. For projected progress p and earliest required unverified coordinate g, use the heuristic `q = clip[0,L](min(p,g))` under authorized local recovery constraints. Command dispatch acknowledgement does **not** prove re-entry; a qualifying physical position and stopped dwell is required. If safe and within finite budget, resume toward B. A failed local return reports `FAILED` or `UNKNOWN` with reason; no infinite retry.

**On physical arrival at B, automatic recovery ends.** The agent may perform a short bounded, passive finalization of already acquired sensor evidence before reporting, but **must not dispatch `GOTO(g)` or any automatic flight back along the completed edge**. A terminal `ARRIVED/INCOMPLETE` is expected if coverage remains unverified. A subsequent visit is always a fresh, explicit higher-level mission assignment.

## Evidence and conditional claim

Let each accepted contracted camera interval be `I_j⁻` and its actual acquired region `A_j`. Under truthful calibration, valid deterministic navigation/altitude error bounds, correct timestamps and attempt identity, assume `I_j⁻ ⊆ A_j`. Then `⋃I_j⁻ ⊆ ⋃A_j`. Thus full accepted interval coverage implies actual coverage to the declared allowance, **under those assumptions**. Unverified does not automatically mean physically uncovered: pending sensor evidence must be reported separately. Movement, ACK and stopped arrival cannot fill a missing coverage interval.

The finite repair budget K and finite deadline T bound local correction commands and attempt duration when their expiry handlers execute correctly. This is a termination argument, not guaranteed task completion or field airworthiness.

## External algorithm boundary

Rotor-router is the primary motivating graph-edge workload, but VECR neither implements rotor states nor determines successor edges, UAV reassignment, mission completion, or policy for residual gaps. The caller can continue, retry, defer, or assign another UAV; its logic is intentionally independent of the VECR library. It must avoid conflicting flight commands while VECR holds execution authority. Emergency control overrides supersede recovery.

## Provenance qualification

Retrospective v7 conditions N07–N09 (180 policy attempts, 90 matched pairs) contain genuine in-flight corrections and are useful as preliminary historical evidence. N03, N04, N06, N17, N18 and selected failure-condition flight trajectories relied on the retired endpoint branch. The full v7 1,320-attempt results cannot validate the revised SDK. They remain archived in `legacy_v7/` for scientific audit, not as claims of current performance.
