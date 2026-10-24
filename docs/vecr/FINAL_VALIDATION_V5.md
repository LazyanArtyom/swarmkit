# VECR v5 validation record

## Engineering gates before confirmatory execution

* Baseline audit completed before source changes; v4.2 2,400 rows and 107 artifact
  hashes independently revalidated. No historical result changed.
* Release CTest: 2/2 targets passed (full library/integration suite).
* UBSan CTest: 2/2 targets passed (same source runtime fixes).
* Focused new v5 tests: 5 cases, 14,819 assertions, including 3,000 randomized
  measured/actual footprint pairs and independent subcell-hole/rotation controls.
* SDK install and external C++ consumer compile/link: passed with the installed
  `swarmkit::client` target and the release dependency toolchain.
* Final engineering pilot: 234/234 attempts completed; 20,302 delivered captures
  and 207 physical re-entry dwells independently reconstructed and checked.
* Pilot 02 retained one startup infrastructure failure. Its generic error did not
  preserve the specific cause; later runs use isolated state and detailed errors.
  Pilots used work-in-progress source, not a clean source freeze; tracked patches
  are retained, but early untracked source snapshots are not complete. They are
  engineering evidence, excluded from confirmatory statistics.
* Initial sandbox-only integration invocation could not bind loopback sockets;
  reruns with authorized local sockets passed. The installed-consumer check first
  used the wrong exported target spelling and then passed with `swarmkit::client`.

## Completed experiment inventory

The v5 digital-simulation campaign is complete. No SITL or hardware trial was run.
All scientific attempts used SDK → local mTLS AgentService → production agent
runtime → SimBackend. B2 and VECR use the same verifier, observation generator,
dynamics, fault schedules and safety limits; their recovery targets differ.

| Execution | Scenarios | Paired seeds per scenario | Attempts | Infrastructure errors |
|---|---|---:|---:|---:|
| Primary | M00–M20 | 100, seeds 1–100 | 4,200 | 0 |
| Sensitivity | X00–X17 | 30, seeds 1–30 | 1,080 | 0 |
| Separate serial host benchmark | M00/M03/M05/M08 | 10, seeds 20001–20010 | 80 | 0 |

The science campaign used four workers; the benchmark ran afterward with one
worker and the same executable. The [scenario matrix](../../results/vecr/agent_campaign_v5/final_v5/tables/scenario_matrix.csv)
contains every frozen condition. The [assignment manifest](../../results/vecr/agent_campaign_v5/final_v5/manifest.json)
contains every seed and policy. All assigned attempts remain in the results.

The four engineering pilots assigned 744 attempts: 743 returned trial results
and one startup infrastructure error. All are preserved and excluded from
confirmatory estimates. No unsuccessful scientific attempt was replaced.

## Validation gates

| Gate | Result | Evidence |
|---|---|---|
| Audit before modification and v4.2 availability inventory | Pass; all 330 missed true completions enumerated, historical attribution limits retained | [Audit](V5_PREIMPLEMENTATION_AUDIT.md) |
| Release regressions | Pass, 2/2 CTest targets | [Log](../../results/vecr/agent_campaign_v5/gates/release_ctest.txt) |
| Undefined-behavior sanitizer regressions | Pass, 2/2 CTest targets | [Log](../../results/vecr/agent_campaign_v5/gates/ubsan_ctest.txt) |
| New state-machine and independent sensor/oracle regressions | Pass, 5 cases / 14,819 assertions | [Log](../../results/vecr/agent_campaign_v5/gates/model_regressions.txt) |
| Installed SDK and external consumer | Pass, compile and link against installed SDK | [Install](../../results/vecr/agent_campaign_v5/gates/sdk_install.txt), [consumer](../../results/vecr/agent_campaign_v5/gates/sdk_consumer.txt) |
| Analysis regression tests | Pass, 2 tests, including seed-cluster resampling and undefined precision | [Log](../../results/vecr/agent_campaign_v5/gates/analysis_regressions.txt) |
| Complete paired assignments | Pass, 5,280/5,280; no duplicate, missing or infrastructure-error rows | [Analysis manifest](../../results/vecr/agent_campaign_v5/final_v5/analysis_manifest.json) |
| Independent truth reconstruction | Pass, 453,524 delivered captures and 4,246 physical re-entry dwells | [Analysis manifest](../../results/vecr/agent_campaign_v5/final_v5/analysis_manifest.json) |
| Honest-sensor soundness | Pass, 4,880 attempts, zero false fulfillment; all 256,395 accepted footprints contained in actual corresponding coverage | [Per-scenario results](../../results/vecr/agent_campaign_v5/final_v5/tables/per_scenario.csv) |
| Positive and negative midflight interventions | Pass; real departure/re-entry in M03–M05; unsuccessful sustained-wind and motor-failure recovery retained | [Traces](../../results/vecr/agent_campaign_v5/final_v5/trials), [trajectory figure](../../results/vecr/agent_campaign_v5/final_v5/figures/03_positive_negative_trajectories.png) |
| Sensor truth separation | Pass, measured-telemetry-only estimator, private acquisitions/poses, independent Python reconstruction | [Models](../../tests/vecr_v5_models.h), [validator](../../scripts/vecr/validate_agent_campaign_v5.py) |
| Raster sensitivity | Pass at 0.1/0.25/0.5/1 m; no completion-classification changes observed in final trials | [Table](../../results/vecr/agent_campaign_v5/final_v5/tables/raster_sensitivity.csv) |
| Separate serial benchmark | Pass, 80 attempts / 6,531 delivered captures / 80 re-entry dwells | [Log](../../results/vecr/agent_campaign_v5/gates/benchmark_validation.txt) |
| Scientific artifact inventory | Pass, 47,560 SHA-256 entries; benchmark has 727 entries | [Science hashes](../../results/vecr/agent_campaign_v5/final_v5/SHA256SUMS), [benchmark hashes](../../results/vecr/agent_campaign_v5/benchmark_v5/SHA256SUMS) |
| Raw data unchanged during analysis correction | Pass, 47,523 files compared, zero changed | [Record](../../results/vecr/agent_campaign_v5/gates/raw_immutability.json) |
| Frozen prior research | Pass; v4.2 2,400 rows / 107 hashes valid; no prior result/archive paths changed | [Validator log](../../results/vecr/agent_campaign_v5/gates/frozen_v4_2_validation.txt), [preservation record](../../results/vecr/agent_campaign_v5/gates/historical_preservation.json) |

The regression suites, including the new tests, cover recovery ACK versus physical re-entry,
overlapping repair prevention, hysteresis, unavailable bounds, cooldown and
safety override, plus independently computed geometry and subcell holes. An
acceptance gate passing means the specified behavior was tested; it does not
mean every experimental mission succeeded.

## Measured comparison

Each midflight row below contains 100 paired seeds with **both policies truly
complete and reporting fulfillment in every pair**. Distance includes recovery.

| Scenario | B2 mean distance | VECR mean distance | VECR − B2, paired 95% CI | Reduction |
|---|---:|---:|---:|---:|
| M03 departure before B | 79.561 m | 57.764 m | −21.798 m [−21.821, −21.773] | 27.4% |
| M04 departure, strip remains visible | 79.561 m | 57.418 m | −22.143 m [−22.185, −22.101] | 27.8% |
| M05 departure with hidden strip | 79.561 m | 59.440 m | −20.121 m [−20.153, −20.092] | 25.3% |

Corresponding simulated time differences are −9.943 s [−9.961, −9.925],
−10.113 s [−10.147, −10.080] and −9.248 s [−9.264, −9.232]. This supports
a conditional recovery-cost advantage in these specified synthetic conditions.
It does not establish an energy or fleet-wide saving.

For the 19 honest primary conditions (1,900 attempts per policy):

| Policy | True survey | True survey + stopped B | Reported fulfillment | False fulfillment | True complete but unaccepted | Recall |
|---|---:|---:|---:|---:|---:|---:|
| B2 | 1,300 | 800 | 700 | 0 | 100 | 87.50% |
| VECR | 1,300 | 1,094 | 794 | 0 | 300 | 72.58% |

Recall has a different denominator for each policy. In M08/M09, VECR returned
to B within the deadline while B2 was still repairing; missing bounded evidence
prevented either from certifying completion. VECR therefore had more true
completions and more accepted completions, but lower conditional recall.
Neither method solves persistent missing evidence by moving again.

The 18 sensitivity conditions add 540 attempts per arm: 540 true completions,
510 reported completions and zero false fulfillment per arm. X07, the 1 m
position-bound condition, accounts for all 30 unaccepted true completions per
arm. The soundness rule was not relaxed to increase acceptance.

M20 sparse capture produced 94/100 accepted completions for VECR versus 0/100
for B2; paired difference +0.94 [0.89, 0.98]. There are no equally successful
pairs in that condition, so its matched-success cost comparison is undefined.
Sustained wind, command refusal, failed motor, exhausted repair budget and low
battery produced genuine unsuccessful outcomes. All 2,366 final interrupted
attempts remain in the raw dataset; transient UNKNOWN states are logged.

The two dishonest-sensor controls M14/M15 produced **400/400 false fulfillment
reports across both policies**. This intentionally failed physical soundness
demonstrates the trusted sensor boundary, not a positive validation result.
There is no verification-accuracy advantage over B2: they share the verifier.

Intervals use 10,000 paired bootstrap resamples, RNG seed 20261024. Per-scenario
binomial intervals use Wilson 95%. Aggregate paired estimates resample whole
seed clusters across conditions, because seed IDs recur. The fixed-mixture
primary honest distance difference is −11.307 m [−11.430, −11.157], and reported
acceptance differs by +4.947 percentage points [4.684, 5.158]. These combine
successful and unsuccessful work and are descriptive of this matrix only.
All-assigned metrics, p50/p95, confusion counts, conditional effects and every
unsuccessful attempt are in the [tables](../../results/vecr/agent_campaign_v5/final_v5/tables).

## Host measurements

The separate 80-process benchmark measured CPU median 0.091 s (p95 0.205 s)
and peak RSS median 22.266 MiB (p95 23.438 MiB). The median of per-trial
capture-RPC p95 latencies was 138.605 µs; its across-trial p95 was 162.863 µs.
Raw report/capture rates are in [isolated_host_performance.csv](../../results/vecr/agent_campaign_v5/final_v5/tables/isolated_host_performance.csv).

CPU/RSS include the in-process SDK, agent, backend and trace writer; RPC timing
includes mTLS transport. These are host-dependent, instrumented-process results,
not an agent-only profiler, embedded benchmark or real-time guarantee.

## Reproducibility and provenance

* Experimental source: `7896f200ec7fa41bd4f9ca1f293616dbd3896d36`.
* Final analysis source: `62a33636297de75d6abc72b549e209f0dc8e8919`.
* Release executable SHA-256: `61ae7bfe6306fe1673ceffd8039c8ad727eb19b83820c08500178dce70fa8491`.
* Assignment manifest SHA-256: `03a79283d975992e2d24091fa543a637de1adf8adf44383704942ea9680228e3`.
* Raw per-attempt CSV SHA-256: `3cb9f0e3680addd953115b3c7a7b6243eb9471d3d4dfe19921769aa0498fb3b2`.
* Final SHA256SUMS file SHA-256: `416321482573bde4c40c5195fc1671f4e8f6ef2fdbb0c8d18ea387c0b0378689`.

The [results directory](../../results/vecr/agent_campaign_v5/README.md) provides
reproduction commands, full traces, environment/build settings, seven figures
in SVG/PDF/PNG, raw rows and hashed artifacts. The final figures were visually
checked. Source and binary identity are separate from the publication commit.
[Analysis corrections](V5_ANALYSIS_NOTES.md) document the time-unit label and
aggregate uncertainty fixes. The superseded first analysis is retained; no raw
trial changed. Reproduction must use a new output directory.

## Supported functionality and remaining scope

Implemented and tested in the simulator: attainable departure detection with
hysteresis; explicit DEVIATING; recovery ACK distinct from physical completion;
bounded stationary re-entry before continuation; overlapping repair prevention;
cooldown; noisy bounded camera estimation; independent exact-strip reconstruction;
fair paired B2/VECR recovery; explicit rejected evidence and failed repairs.
Existing general control, telemetry and peer/server APIs remain available.

The evidence is a straight, flat, one-dimensional full-strip model under
rectangular nadir footprints. It does not combine partial-width images into a
2D mosaic. Angle controls use rotated ground rectangles, not pitch/roll
photogrammetry. Seeds model narrow synthetic variation, not populations of real
aircraft, terrain or weather. The nominal 90 s deadline is evaluated on sampled
telemetry; recorded timeout termination is 90.2 s, not a continuous-time bound.

Still incomplete: live camera calibration and image quality; safe recovery around
obstacles/geofences; realistic aerodynamics; independent MAVLink/ArduPilot SITL;
real hardware; remote replay, peer transport and simultaneous multi-UAV campaigns;
OS power-loss recovery; agent-only resource isolation; energy/radio measurements.
The new 21-case v5 matrix does not complete unsupported v4 S13/S14/S17.
V4.2 trace omissions prevent complete retrospective rejection-cause attribution;
v5 records those reasons prospectively. These limits remain explicit and are
not prerequisites silently claimed as passed. No rotor-router or gossip was added.
