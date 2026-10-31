# VECR v8 claim traceability

The active dissertation is `VECR_Dissertation_Interactive_Website/source/dissertation.md`. Its renderer reads the frozen v8 campaign rather than manually transcribed headline statistics. Each claim below has a source implementation, executable check and published evidence boundary.

| Dissertation claim | Runtime or SDK source | Verification and evidence |
|---|---|---|
| One qualified in-flight recovery target `q=min(progress, first required gap)`; B2 restarts at A only for research | `src/core/vecr.cpp` (`ConsiderRecovery`); compile-time research target, no public policy selector | `tests/test_vecr_agent_trial.cpp`; N07–N09 primary traces and paired comparisons |
| No automatic gap return after qualified B arrival | `src/core/vecr.cpp` (`ConsiderEndpointFinalization`); `src/agent/edge_execution_runtime.cpp` | Endpoint-gap integration regressions; independent validator asserts no accepted GOTO after first reconstructed B arrival; N03/N04/N06/N10/N15–N18/N21 |
| Physical stopped arrival differs from survey completion | `src/core/vecr.cpp` (`ObservePosition`, `ConsiderEndpointFinalization`); `include/swarmkit/edge/vecr.h`; protobuf edge status | Unit and integration tests; v8 `per_attempt.csv` separates `reported_arrived`, `reported_complete`, `truth_complete` |
| Qualified evidence retained as disjoint intervals, with hard-bound semantics | `src/core/vecr.cpp`, `include/swarmkit/edge/vecr.h`; capture validation in agent runtime | Evidence and delayed-capture regressions; independent capture/trajectory reconstruction; N01–N06 and N15–N21 |
| Recovery has finite attempt, duration and conservative travel limits; failure requests hold where supported | `src/core/vecr.cpp`; `src/agent/edge_execution_runtime.cpp` (`DispatchRecovery`, failure hold) | Budget, command refusal and safety regressions; N11–N14 failure rows and events |
| Caller can pull a consistent exact-attempt status and subscribe to terminal-only results | `src/agent/server.cpp` (`GetEdgeStatus`, `CancelEdge`); `src/client/client.cpp` (`GetEdgeStatus`, `StartEdgeResults`); `include/swarmkit/client/client.h` | Integration tests for concurrent polling, identity, callback deduplication within a live subscription, cancellation and reconciliation |
| SwarmKit owns an assigned edge, not a graph traversal algorithm | `include/swarmkit/edge/vecr.h`; client and agent API | Direct/routed v8 reuse campaign; neither rotor-router nor gossip is implemented |
| Conditional accepted-footprint soundness under honest hard bounds | `src/core/vecr.cpp` evidence acceptance; documented set-containment proof | `validate_digital.py` checks accepted footprint containment; zero false reported completions in 1,200 honest-sensor attempts; N19/N20 are negative controls |
| Conditional distance reduction versus fair full-restart B2 | `tests/vecr_digital_trial.cpp`; `scripts/vecr/run_digital.py` | 30 common seed pairs each in N07–N09; `paired_comparisons.csv`, `paired_distance.svg`, bootstrap intervals; other scenarios are fully reported without cost superiority claims |
| Revised-method SITL and real camera performance remain unverified | No v8 SITL or calibrated image campaign | Retired v7 SITL records remain in Git history; all revised-method SITL figures are TBD |

## Interface and evidence limits

The SDK deduplicates a terminal callback by execution identity **within one live subscription**. A caller should subscribe before dispatch and use journal-backed `GetEdgeStatus` to reconcile after reconnect. This is not an exactly-once network delivery theorem. A `GOAL_REACHED` or command ACK is not a physical stopped-arrival certificate. The adapter's asserted accuracy is used as a hard bound only when its semantics declare such a bound; no image-pixel quality validator was run.

The current reuse experiment compares direct `Client` and routed `SwarmClient` access to the *same* agent method. It supports transport-path parity for five cases, not independent algorithm or multi-UAV mission generality. The simulation uses straight local edges, idealized flat camera rectangles, synthetic bounded sensing and one vehicle per attempt. Dynamic obstacle avoidance, graph reassignment, calibrated camera imagery, revised-policy SITL and field safety remain outside the measured claim.
