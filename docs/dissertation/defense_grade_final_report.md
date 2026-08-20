# SwarmKit final scientific-correctness report

Date: 20 August 2026

Audited base commit: `be71251d25f7544b7e913e6f6f984f88b450e1fc`

Campaign host: Apple M3 Pro, Darwin 25.5.0, arm64, AppleClang 17.0.0, Release

## A. Implementation outcome

The remediation established one canonical `CERT_V3` path from runtime selection through
serialization, persisted replay, and independent verification. The common oracle now judges the
actual selected output against explicit contract predicates and physical truth at `t*`; it no
longer adds current simulator estimator health as an undeclared requirement.

The engine retains the exact clock model primitives used during selection: source and receive
times, offset estimate, base and effective uncertainty, drift bound, clock-update reference time,
model version, incarnation, domain, synchronization semantics, and deterministic-bound status.
The certificate binds those operands together with `g-`, `g+`, `Delta+`, propagation assumptions,
participants, membership revision, and the selected evidence identity.

The verifier independently reselects evidence and reconstructs all derived quantities from the
persisted store, fixed canonical contract, request context, session, membership, and clock state.
It does not call the state-acceptance engine.

## B. Core conformance checks

- Selection preserves `receive_time <= r*`, `g+ <= t*`, then maximum `g-`, maximum sequence, and
  canonical content-bound EvidenceId ordering.
- Deterministic clock use requires a present, valid model for the selected incarnation; no
  per-sample theta-zero fallback is used.
- Position propagation uses `hypot(Vhorizontal, Vvertical)` and reference-domain drift growth.
- Required GPS evidence is rejected when missing, incorrectly typed, or below the contract
  threshold.
- Replay request events store the canonical contract hash; a semantically changed fresh contract
  is rejected before replay verification.
- The canonical persisted-frontier regression includes causal evidence with `source < t*` but
  `receive > r*`; reload excludes it solely because it arrived after the frozen frontier.

## C. Build, tests, and sanitizers

The repository was rebuilt from a removed `build/` directory using the project presets.

| Configuration | Result |
| :--- | :--- |
| Debug CTest | 2/2 targets passed |
| Debug Catch2 | 163 test cases, 1,523 assertions, 0 failures |
| Release CTest | 2/2 targets passed |
| Release Catch2 | 163 test cases, 1,523 assertions, 0 failures |
| macOS UBSan CTest | 2/2 targets passed; no sanitizer finding |
| ASan / TSan | Not executed; no macOS preset is exposed by this repository |

The socket integration tests require loopback binds. Their first sandbox-restricted execution was
denied by the environment; the same binaries passed when rerun with loopback permission.

Important new regressions cover oracle health/mission/GPS/provenance semantics, missing and
incarnation-mismatched clock models, reference-domain drift, uncertainty monotonicity, actual clock
version binding, deterministic EvidenceId ordering, 3D propagation, comprehensive replay-event
round trips, canonical contract-hash binding, persisted `r*`, and rehashed certificate semantics.

## D. Frozen final protocol

| Parameter | Value |
| :--- | :--- |
| Primary N | 3 UAVs |
| Replicate clusters | 50 |
| Scenarios | 9 |
| Scored requests | 100 per replicate/scenario; 45,000 per method |
| Base seed | 42 |
| Bootstrap | 10,000 draws; seed 123456789; replicate ID is the cluster |
| `Umax` | 3.0 m |
| Maximum age | 300 ms |
| Maximum clock uncertainty | 10 ms |
| Maximum horizontal / vertical speed | 10.0 / 3.0 m/s |
| Propagation speed | 10.4403065089 m/s |
| High-speed scenario | (8.5, 1.0, 0.0) m/s; 200 ms configured delay |
| Restart | E1 captured step 29; restart/invalidation step 30; E1 injection step 31; E2 clock step 33 |
| Estimator degradation | From scored step 20 |

The exact protocol is recorded in `final_experiment_protocol.md` and was frozen before the final
campaign.

## E. Final Table II

| Method | Requests | Accepted | False Valid (FV) [95% CI] | Availability [95% CI] | Unsafe per Req (UAR) [95% CI] |
| :--- | ---: | ---: | ---: | ---: | ---: |
| `B0` | 45,000 | 44,514 | 20.591% [20.570%, 20.616%] | 98.920% [98.871%, 98.967%] | 20.369% [20.349%, 20.391%] |
| `B1` | 45,000 | 39,125 | 23.387% [23.349%, 23.426%] | 86.944% [86.798%, 87.084%] | 20.333% [20.333%, 20.333%] |
| `P` | 45,000 | 22,981 | 0.000% [0.000%, 0.000%] | 51.069% [50.891%, 51.236%] | 0.000% [0.000%, 0.000%] |

The paired `P-B1` FV difference was -23.387 percentage points, with 95% replicate-cluster
bootstrap CI [-23.426, -23.349] percentage points. No p-value was computed.

## F. Statistical analysis

The independent cluster was `replicate_id`, not an individual request. Each of 10,000 bootstrap
draws sampled 50 replicate IDs with replacement and included every scenario/method count for each
sampled cluster; paired `P` and `B1` calculations used the same sampled IDs. Main FV, availability,
and UAR estimates are ratios of summed counts. The analysis seed was 123456789.

Across the 50 replicate clusters, FV mean / median / standard deviation were 20.5913% / 20.5618% /
0.0844% for `B0`, 23.3874% / 23.3716% / 0.1432% for `B1`, and 0% / 0% / 0% for `P`.
Availability mean / median / standard deviation were 98.9200% / 99.0000% / 0.1739%, 86.9444% /
87.0000% / 0.5299%, and 51.0689% / 51.0000% / 0.6313%, respectively. The paired replicate-level
`P-B1` FV difference had mean -23.3874, median -23.3716, standard deviation 0.1432, and range
[-23.8592, -23.1353] percentage points. Full Q1/Q3 and range data are saved in
`replicate_distribution_summary.json`.

## G. Scenario activation and results

All intended mechanisms activated in the final run: 15,000 delayed frames in each delay-bearing
scenario; 7,438 reorder events with 4,941 within-UAV inversions; 6,750 dropped frames; 15,000
clock-fault events with maximum effective rho 15 ms; 12,000 degraded-estimator events; high-speed
truth speed 8.558621 m/s with maximum delayed age 203 ms and maximum propagated-motion term
2.119382 m; 50 restarts, 50 obsolete E1 injections, 50 clock invalidations, and 50 E2 clock
re-establishments; and 5,000 frame mismatches. In the restart scenario, `B0` and `B1` each selected
150 obsolete-epoch field records while `P` selected 0.

Each cell below is `FV / availability / UAR` for 5,000 requests.

| Scenario | `B0` | `B1` | `P` |
| :--- | ---: | ---: | ---: |
| `normal` | 0.000% / 100.000% / 0.000% | 0.000% / 100.000% / 0.000% | 0.000% / 99.000% / 0.000% |
| `network_delay` | 0.000% / 95.000% / 0.000% | 0.000% / 0.000% / 0.000% | 0.000% / 0.000% / 0.000% |
| `network_reorder` | 0.000% / 99.100% / 0.000% | 0.000% / 99.100% / 0.000% | 0.000% / 99.000% / 0.000% |
| `packet_loss` | 0.326% / 98.180% / 0.320% | 0.000% / 86.400% / 0.000% | 0.000% / 48.620% / 0.000% |
| `clock_offset_drift` | 0.000% / 100.000% / 0.000% | 0.000% / 99.000% / 0.000% | 0.000% / 0.000% / 0.000% |
| `estimator_degradation` | 80.000% / 100.000% / 80.000% | 80.000% / 100.000% / 80.000% | 0.000% / 20.000% / 0.000% |
| `high_speed_delayed_motion` | 0.000% / 98.000% / 0.000% | 0.000% / 98.000% / 0.000% | 0.000% / 98.000% / 0.000% |
| `agent_restart_delayed_packets` | 3.000% / 100.000% / 3.000% | 3.000% / 100.000% / 3.000% | 0.000% / 95.000% / 0.000% |
| `frame_mismatch` | 100.000% / 100.000% / 100.000% | 100.000% / 100.000% / 100.000% | 0.000% / 0.000% / 0.000% |

## H. Proposed false-valid diagnostics

0 observed in the final campaign.

Because no `P` false-valid event occurred, there is no per-event cause classification to report.

## I. Deterministic containment

The campaign evaluated 68,943 accepted deterministic position enclosures and observed 0
containment failures, for an observed rate of 0.000%. The truth-error/enclosure-radius ratio had
maximum 0.807200, p50 0.364502, p95 0.797248, and p99 0.805391.

This finite conformance result is consistent with the deterministic enclosure derivation under
the configured assumptions. It is not an empirical proof of a universal theorem or zero failure
probability.

## J. Persisted replay

All 22,981 accepted proposed decisions were persisted to JSON-lines form, reloaded into fresh
evidence/session/clock/membership/request state, and independently checked after certificate
deserialization. The result was 22,981 agreements, 0 disagreements, and agreement rate 100%.

The fixed canonical contract is supplied separately and bound by canonical hash. Accordingly, the
claim is limited to reconstruction of evidence, session, clock, membership, request-frontier state,
and certificates from disk and verification against that fixed contract.

## K. Mutation verification

The campaign exercised 44 mutation classes for every accepted proposed certificate. All semantic
mutations except the outer-hash case recomputed the SHA-256 outer consistency hash before verifier
evaluation. The verifier rejected 44/44 classes and 1,011,164/1,011,164 instantiated
mutated/inconsistent cases; no mutated case was accepted.

The classes cover outer hash; evaluation/frontier/contract/certificate semantics; participant and
membership state; accepted agents; evidence identity, content, timestamps, incarnation,
provenance, frame, uncertainty, health, mission, and GPS; every bound clock operand and interval;
propagation model/version/speeds; and aggregate certificate summaries.

## L. Release performance

Operation: `RequestSnapshot + BuildCertificate + SerializeCertificate`. Clock:
`std::chrono::steady_clock`. Each N used 100 warmups and 1,000 saved timed samples.

| N | Count | Mean ± stddev (µs) | p50 (µs) | p95 (µs) | p99 (µs) | Min–max (µs) | Median certificate bytes |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 3 | 1,000 | 139.091 ± 1.498 | 138.916 | 140.625 | 145.209 | 135.833–160.125 | 2,563 |
| 5 | 1,000 | 221.337 ± 2.865 | 220.833 | 225.458 | 233.125 | 217.291–264.167 | 4,040 |
| 10 | 1,000 | 406.746 ± 9.137 | 402.625 | 426.000 | 431.667 | 398.708–465.792 | 7,744 |

Latency increased approximately linearly across the tested N=3–10 range. Host performance does
not establish target embedded or real-time performance.

## M. Reproducibility artifacts

The final campaign generated:

- `table_ii_results.json` and `table_ii_results.csv`
- `table_iii_results.json`
- `scalability_results.json` and `scalability_latency_samples.csv`
- `per_scenario_results.json`
- `replicate_results.csv` and `replicate_distribution_summary.json`
- `bootstrap_results.json`
- `mutation_results.json`
- `replay_results.json`
- `experiment_manifest.json`

These files live under `results/dissertation/`. The manifest records git commit and dirty state,
semantic/schema versions, canonical contract hash, seed derivations, frozen thresholds, build
metadata, hardware, and campaign timestamp.

## N. Limitations

- `SimBackend` is the primary theorem-conformance environment.
- Finite experiment results do not establish a universal failure rate.
- Deterministic guarantees depend on valid hard upstream uncertainty, speed, and clock bounds.
- macOS ARM64 performance does not establish target embedded real-time performance.
- Unsigned SHA-256 consistency hashes do not establish adversarial authenticity.
- Component-wise spatial enclosure does not automatically prove arbitrary joint cross-UAV
  correlation properties.

## O. Verdict

**DEFENSE-GRADE IMPLEMENTATION READY FOR DISSERTATION RESULTS**

Every final acceptance gate in the frozen campaign passed: clean Debug and Release tests, macOS
UBSan, scenario activation, full 50-cluster results, persisted replay, rehashed semantic mutation
verification, replicate-cluster inference, end-to-end Release benchmarking, raw artifact export,
and documentation reconciliation. The verdict is scoped by the limitations above.
