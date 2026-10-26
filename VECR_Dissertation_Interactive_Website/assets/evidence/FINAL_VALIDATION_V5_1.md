# VECR v5.1 — corrected-code replication and dissertation reader

Completed 2026-10-26. This is **agent-backed digital simulation**, with the
production SDK → local mTLS AgentService → agent runtime → SimBackend path.
No ArduPilot SITL or hardware experiment was executed. All earlier archives and
v1/v2/v3/v4/v4.2/v5 results remain at their original paths and revisions.

## Corrections and experimental identity

The dissertation audit reproduced two production defects before fixing them:

1. **Traversal endpoint uncertainty.** Crediting an entire estimated segment
   could report coverage outside the guaranteed physical interval. Endpoint
   bounds now contract credited progress; overlapping bounds cannot manufacture
   forward motion. Two counterexample regressions cover both cases.
2. **Off-segment camera projection.** Clamping a camera center onto A or B could
   invent footprint overlap. The adapter now uses signed axial and perpendicular
   coordinates, then clips the footprint. Regressions cover both ends, eastbound
   and northbound routes, and legitimate partial overlap.

The API, runtime ownership and general-purpose control/communication boundaries
are unchanged. The fixes alter the implementation of existing physical evidence
predicates. No coordination algorithm was introduced.

The [v5.1 amendment](SIMULATION_PROTOCOL_V5_1.md) was written before execution.
The full [v5 matrix and inference](SIMULATION_PROTOCOL_V5.md) were retained:
same assignments, seeds, dynamics, sensors, policies, deadlines and exclusions.
Source was committed before confirmatory execution:

| Item | Identity |
|---|---|
| Experimental source | `7da3db33e70114503dfc7c9a45671da2b9335541` |
| Final analysis source | `ee3e321a5cfa4015cbcf0ecddc810545e2e1abe0` |
| Release executable SHA-256 | `6d607d7369ef1faa05ee919da935f9710ff2ab10a2cdf1895a8d1e884264c901` |
| Assignment manifest SHA-256 | `68644d690b86c610b87d9c27502870064f3ab73dfa8b6165736229e437d17f01` |
| Raw per-attempt CSV SHA-256 | `f1b9eb01ec69750c36b808f6f4a910dfe05736b1c424ad662ab1fda92c887ed1` |

The later analysis commit fixes only the version-specific validation-document
link and adds a comparison utility. It does not alter statistical inference.
The first generated report and its analysis manifest are retained in
`gates_v5_1/initial_analysis_reference`. Final manifests additionally hash the
runner, validator, analysis script, base protocol and amendment. The manifest's
untracked website entry records presentation work outside the executable source.

## Exact executions and gates

All paths below are relative to `results/vecr/agent_campaign_v5/`.

| Gate | Executed evidence | Result |
|---|---|---|
| Defect reproduction | `gates_v5_1/endpoint_bounds_before.txt`, `camera_endpoints_before.txt` | Previous code failed the counterexamples |
| Corrected endpoint regressions | 3 test cases, 20 assertions; `endpoints_after.txt` | Pass |
| Full release suite | 2/2 CTest targets; `release_ctest.txt` | Pass |
| Full UBSan suite | 2/2 CTest targets; `ubsan_ctest.txt` | Pass |
| Installed SDK | Install and external consumer configure/build/link; `sdk_install.txt`, `sdk_consumer.txt` | Pass |
| Statistical regressions | 2 tests including cross-condition seed clustering; `analysis_regressions.txt` | Pass |
| Engineering pilot | 39 conditions × one paired seed 10004 × two policies = 78 attempts | 78 results, 0 infrastructure errors |
| Pilot reconstruction | 6,764 delivered captures and 69 physical re-entry dwells; `pilot_validation.txt` | Pass |
| Primary campaign | 21 conditions × 100 paired seeds × two policies = 4,200 attempts | Complete |
| Sensitivity campaign | 18 conditions × 30 paired seeds × two policies = 1,080 attempts | Complete |
| Final infrastructure | All 5,280 assigned attempts retained in `corrected_endpoints_v5_1/` | 0 infrastructure errors |
| Independent science reconstruction | 5,280 attempts, 453,524 delivered captures, 4,246 completed re-entry dwells | Pass |
| Honest-sensor containment | 256,395 accepted captures checked against actual acquisitions | 0 uncontained footprints |
| Separate serial benchmark | 80 processes after science, same executable, one worker; `benchmark_v5_1/` | 80 results, 0 infrastructure errors |
| Benchmark reconstruction | 6,531 delivered captures, 80 re-entry dwells | Pass |
| Scientific SHA-256 inventory | 47,559 entries, `corrected_endpoints_v5_1/SHA256SUMS` | Verified |
| Benchmark SHA-256 inventory | 727 entries, `benchmark_v5_1/SHA256SUMS` | Verified |
| Original v5 | Full independent reconstruction rerun; original files preserved | Pass |

`gates_v5_1/final_analysis_validation.txt` records the reconstruction before and
after final scientific hashing. `benchmark_validation.txt` separately verifies
the benchmark hashes. These checks reconstruct geometry, acquisition, union,
distance, stopped arrival and physical re-entry from private logs; they do not
accept the agent's report as ground truth.

The 78 engineering attempts are not pooled with the 5,280 confirmatory attempts.
The 80 host measurements are also separate. Earlier pilots, interrupted studies,
their failures and superseded analyses remain preserved and disclosed by the
[v5 report](FINAL_VALIDATION_V5.md) and [analysis notes](V5_ANALYSIS_NOTES.md).

## Measured findings

The independent oracle counted 4,880 honest-sensor attempts with **zero false
fulfillment** under either method. This is a conditional observation, not a
universal safety guarantee. All **400 intentionally dishonest-sensor controls
falsely fulfilled**, demonstrating the physical sensor trust boundary.

For M03–M05, each method achieved true and reported completion in 100/100 seeds.
The following differences are VECR minus coverage-aware full-restart B2; negative
distance/time is lower cost. Each interval uses 10,000 paired bootstrap resamples.

| Condition | B2 mean distance | VECR mean distance | Paired difference, 95% CI | Reduction |
|---|---:|---:|---:|---:|
| M03 departure before B | 79.561 m | 57.764 m | −21.798 [−21.821, −21.773] m | 27.4% |
| M04 wide visible strip | 79.561 m | 57.418 m | −22.143 [−22.185, −22.101] m | 27.8% |
| M05 hidden strip gap | 79.561 m | 59.440 m | −20.121 [−20.153, −20.092] m | 25.3% |

Corresponding time differences were −9.943 [−9.961, −9.925] s,
−10.113 [−10.147, −10.080] s, and −9.248 [−9.264, −9.232] s.
These conditional cost comparisons accompany all-assigned availability and
cost tables; failed or uncertain attempts are not removed from those tables.

Across the 19 honest primary conditions (1,900 attempts per policy), both
methods actually surveyed in 1,300 attempts. True survey **plus stopped B** was
800 for B2 and 1,094 for VECR. Reported fulfillment was 700 and 794 respectively.
Recall was 87.50% versus 72.58%, with 100 versus 300 unaccepted true completions.
The denominators differ because VECR returned to B in more persistent-invalid-
evidence cases while B2 was still repairing at the common deadline. This is an
availability limitation, explicitly shown rather than hidden by the cost result.

M20 reported 0/100 B2 versus 94/100 VECR: paired acceptance difference
0.94 [0.89, 0.98]. There are no matched-success cost pairs for this condition.
All 540 sensitivity attempts per policy physically completed; 510/540 reported
fulfillment. Every miss occurred at the 1 m horizontal-bound condition X07.
The final phase inventory is 2,914 FULFILLED and 2,366 INTERRUPTED, including
the deliberately dishonest controls in the former count.

Whole-seed-cluster resampling across the fixed 19-condition honest mixture gives
distance difference −11.307 [−11.430, −11.157] m, time −3.715
[−3.780, −3.636] s, and reported acceptance +0.04947 [0.04684, 0.05158].
These aggregate effects concern this specified mixture, not a real-flight
prevalence distribution. Per-condition Wilson intervals and all raw counts are
in the tables. No multiple-comparison significance claim is made.

The new 80-process benchmark measured whole-process CPU median 0.091 s / p95
0.195 s; peak RSS median 22.312 MiB / p95 23.438 MiB; and per-trial capture-RPC
p95 latency median 117.354 µs / across-trial p95 155.531 µs. These include the
instrumented SDK, agent, backend and trace writer on the recorded macOS host;
they are not agent-only, embedded-hardware or hard real-time guarantees.

## Replication comparison

[`replication_comparison.csv`](../../results/vecr/agent_campaign_v5/gates_v5_1/replication_comparison.csv)
and its JSON companion compare every corresponding raw row and identify both
source CSV hashes. Across all 5,280 pairs, distance, elapsed time, actual survey,
true/stopped completion, reported completion, false positives/negatives,
recoveries, accepted captures and containment counts are exactly unchanged.
Reported coverage lengths differ in 606 rows, with maximum absolute difference
**2.842170943040401 × 10⁻¹⁴ m**: floating-point rounding scale.

The fixes therefore have no material effect on this matrix's scientific
outcomes. That does not establish the old defects were harmless: their dedicated
endpoint counterexamples previously failed. V5.1 is a fresh execution of the
corrected implementation, not reused or relabeled v5 data. The two campaigns
are not pooled as additional independent seeds.

## Dissertation reader

Open [the offline reader](../../VECR_Dissertation_Interactive_Website/index.htm).
Its 16 chapters now state implemented contracts, conditional correctness,
fairness, measured results, uncertainty, unsuccessful outcomes and scope. The
current edition includes seven scientific figures in SVG/PDF/PNG, five method
diagrams, all 39 conditions and 12 actual B2/VECR attempt replays. Local downloads
contain statistics, manifests, protocols and representative lossless logs.

The supplied 17-file website is archived byte-for-byte with a manifest. No
hypothetical animation is presented as measured evidence. The reader's separate
validation record checks copied-source SHA-256, local links, offline operation,
responsive layout, replay outcomes, negative controls, filters, search, figure
dialogs, keyboard behavior, themes and print media. Screenshots and
`SHA256SUMS` are stored with the reader. SITL has a prepared results chapter
with **TBD / not yet executed**, never invented numbers or a false pass status.

## Supported claims and remaining scope

Implemented and tested: bounded arrival, traversal and full-strip accounting;
sensor/attempt identity and explicit unavailable evidence; local recovery and
re-entry state consistency; shared B2/VECR verification and safety; the defined
21-condition primary and 18-condition sensitivity simulator matrix; independent
reconstruction and reproducible statistics.

Supported scientific claims are conditional full-strip soundness under truthful
bounded metadata, reduced modeled travel in the three equal-success midflight
conditions, and the measured availability/trust limitations. Reusability is an
API boundary, not an experimental proof for all planners or environments.

**ArduPilot SITL remains unexecuted.** Real-camera calibration and imagery,
complex terrain/occlusion/photogrammetry, realistic aerodynamics, obstacle and
geofence-qualified vehicle recovery, remote or simultaneous multi-UAV studies,
OS/power-loss recovery, embedded resource profiling and deployed cloud services
are outside this completed simulator contribution. They must not be described
as completed by changing presentation wording. No rotor-router or gossip
implementation or performance result is claimed.
