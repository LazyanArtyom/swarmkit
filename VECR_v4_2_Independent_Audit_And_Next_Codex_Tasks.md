# SwarmKit VECR v4.2 — Independent Review and Next Codex Tasks

**Review date:** 22 October 2026. **Supplied archive:** `swarmkit copy(1).zip`. **Archive local main/origin-main ref:** `5063dfe492da1cf68790fb579837d7b07ac96178`. This is a source-and-data audit, not an independent rerun of the macOS release build or complete tests.

## 1. Independently verified evidence

- Final manifest SHA-256: `bd49772a668736fbd1d30373366b050641c4b714a1eda1a643e6e55f92e1dac8`; matches the actual byte content.
- All **107 of 107** manifest-listed final artifacts exist and their SHA-256 hashes match. Manifest-listed committed source file hashes also match the supplied snapshot for available files.
- Raw `per_attempt.csv`: **2,400 rows**, representing precisely 12 scenarios × 100 seeds × 2 arms; **2,400 distinct** `(scenario, seed, policy)` keys; no infrastructure-error records.
- Confirmed: 1,200 assignments/arm, 900 true edge completions/arm, 740 B2 and 730 VECR reported `FULFILLED`, zero observed false `FULFILLED` in the paired honest-sensor campaign. Mean distances **42.330423 m** (B2), **36.362895 m** (VECR), delta **−5.967528 m / −14.097%** of B2. Mean simulated times **28.909167 s** vs **27.252000 s**.
- Recomputed localized S03–S05 aggregate: 300/300 fulfilled per arm; B2 **60.814 m**, VECR **47.715 m**, delta **−13.099 m (−21.54%)**; mean time delta **−6.55 s**. This is the cleanest comparison for a cost advantage conditional on equal success.
- Trust-boundary negative control S11 (dishonest sensor) explicitly exhibits a false completion and must remain in dissertation discussion.
- The 39-case/1,818-assertion focused suite and macOS/UBSan gates are documented by Codex but **not independently executed** in this review.

## 2. What the current campaign actually validates

The paired study uses the real SDK → mTLS AgentService → production edge runtime → SimBackend invocation path; B2 and VECR share the verifier, camera and movement models, and differ in recovery re-entry policy. It supports reproducible **post-arrival gap-repair cost** conclusions under a one-dimensional, fixed-width, simulated survey strip. It does **not** validate real-camera imagery, real terrain coverage, real autopilot recovery, or broad UAV-swarm performance. Zero observed false completions for B2 and VECR does not establish superiority of VECR in verification accuracy; it also does not prove zero failure probability.

## 3. Material limitations and threats to scientific validity

**A. Midflight local deviation correction is not evaluated.** The experimental request sets `sustained_deviation_samples = 1000` (`tests/vecr_agent_trial.cpp`) while the protocol caps typical experiments at `maximum_ticks = 650`. The documentation says the purpose is to isolate post-arrival repair. Thus the results cannot justify a claim of experimentally validated *early deviation-triggered re-entry*—one of VECR's important intended functions.

**B. The camera metadata is constructed using exact simulator truth.** `CameraMessage(...)` in `tests/vecr_agent_trial.cpp` fills camera center and FOV directly from `SimulationTruthFrame`; many samples assert zero horizontal/altitude hard-bound errors. The private `VecrTruthOracle` scores against truth separately, but the agent-facing camera evidence is not an independently noisy or calibrated sensor estimate. This makes the main 0-false-completion finding conditional on ideal, honest evidence. The S11 dishonest-camera result proves the trust limitation matters.

**C. Availability is a genuine problem.** Of the 900 true completions per arm, only **740 B2** and **730 VECR** were accepted (true-completion reporting sensitivity 82.2% vs 81.1%). For S08, 100 physically true completions per arm yield only 18 vs 15 accepted. For S09, 100 true completions per arm yield only 22 vs 15 accepted. VECR loses seven reported completions to B2 in S09, with a paired 95% CI of −12 to −2 percentage points. Report false-negative/unknown/incomplete causes explicitly; do not optimize away protective uncertainty checks without evidence.

**D. All-scenario mean mixes successful, failed, and unaccepted work.** Scenarios S02, S16, S19 have zero true completions in both methods; S08/S09 have unequal reported completion. The 14.097% aggregate travel reduction is a fixed-mixture, descriptive statistic, **not** a generalized fleet energy saving or superior cost per successful certified mission. Include both overall results and matched-success S03–S05 effects with appropriate conditioning.

**E. Limited distributions and spatial modeling.** Each edge is 20 m. Gap center varies only ~1 m, gap length ~0.6 m, and wind strength ~±4% around preset levels. CIs quantify this narrow artificial variability, not uncertainty across actual wind, terrains, payloads, airframes or missions. The oracle is a one-dimensional 0.25 m raster (cell-center coverage); it is not exact two-dimensional terrain coverage or a photographic-quality measurement. Repeat sensitivity at more edge lengths, footprint widths, raster resolutions and uncertainties.

**F. Post-raw code correction and provenance must remain visible.** Raw experimental source is `5adb687`, analysis revision is `60f79e8`, and the final repository snapshot is `5063dfe`. This is appropriate when documented; frozen results must not be silently regenerated against a different source version.

**G. Missing scope.** S10 fast-fly-through is inconclusive; S13 (remote replay), S14 (peer transport), S17 (simultaneous multi-UAV) are unsupported; S15 exercises service restart rather than OS power loss. There are no CPU/RSS, network throughput, energy or independent MAVLink SITL measurements. Real-camera calibration and physical flights are out of scope of this simulation phase.

## 4. Next Codex brief: simulator-only v5 scientific gap closure

Do **not** rewrite SwarmKit, the frozen v1/v2/v3/v4.2 artifacts, or start ArduPilot SITL yet. Keep the existing canonical SDK/API and make a new protocol, code commit, and versioned outputs; call the proposed new study **v5**. First write a pre-registered protocol and validation criteria, then execute the tests. Never cherry-pick favorable seeds or remove unsuccessful trials.

### Priority 1 — Midflight deviation and recovery

1. Add an agent-backed study with *realistically attainable* sustained-deviation thresholds (not 1000 samples), explicitly configured sampling rate, bounded position/velocity errors, hysteresis and recovery cooldown.
2. Build scenarios for short wind gust, sustained crosswind, route departure and re-entry before B, localized deviation when the survey strip nevertheless remains visible, and deviation that does create a hidden strip gap. Verify `DEVIATING → RECOVERING → MOVING`, navigation-command targets, and actual physical motion from an independent trajectory oracle. Do not confuse off-corridor motion with automatically missing camera coverage.
3. Compare VECR and B2 under matched movement/sensor/fault conditions, with actual before-B recovery, safe-re-entry feasibility, failure/timeout, distance, elapsed time, and useful accepted coverage. Preserve scenarios where VECR is slower or less available.

### Priority 2 — Imperfect observation model

1. Derive agent-facing capture pose, attitude/FOV and quality from a **simulated sensor estimator** with configurable noise, delay, bias, calibration and explicit uncertainty semantics; keep the oracle dependent on private actual simulator poses and acquisitions. Never compute the claimed camera center directly from the hidden truth in the new main experiment.
2. Include bounded-valid-error and invalid/unbounded-error cases; missing/stale captures, false frame successes, camera-angle offsets, and inconsistent calibration. Test that accepted conservative intervals remain subsets of oracle-covered regions when assumptions hold. Include a few explicit counterexamples when they do not.
3. Evaluate raster sensitivity and clearly label any 1D full-strip abstraction. Avoid claims of validated image quality or full physical 2D coverage.

### Priority 3 — Availability and failure analysis

1. Inspect S08/S09 traces to explain all 160 and 170 missed true completions across B2 and VECR: classify `UNKNOWN`, timeout, invalid position evidence, travel/repair timing and sensor record rejection. Fix demonstrable software bugs, but do not relax soundness predicates merely to raise acceptance.
2. Record per-attempt: actual true completion, returned phase and reason, false positive, false negative, pending gap geometry, command dispatch/ack/rejection, begin/end of deviation, re-entry selection and completion, simulator distance/time, accepted/missing footprints and capture status.
3. Report precision/false completion, recall/availability, conditional cost for equally successful pairs, completion time, p50/p95, failures, and paired seed-based confidence intervals; keep all assigned attempts in the primary denominator. Publish stress/sensitivity matrices for route length, camera footprint, sampling period, uncertainty budget, gap position/length, simulated wind and recovery attempts. Freeze sources, configs, seeds, host/software versions, raw outputs, trace excerpts, report and SHA-256 manifest.

### Priority 4 — Lightweight performance

Add an isolated, repeatable simulator-mode benchmark for agent CPU time, peak RSS, report/capture rates and verification latency using the same build configuration. Treat performance as host-dependent; do not infer Raspberry Pi or real-aircraft results from a macOS run.

### Exit criteria

- All introduced scenarios run through actual SDK → mTLS AgentService → runtime → SimBackend.
- Both baseline and VECR use identical sensor generation, dynamics, sampling, fault schedules and safety limits, except for the explicitly varied recovery strategy; document any policy-dependent differences in realized faults.
- Hidden truth never leaks into the new verifier-facing sensor input.
- At least one reproducible positive and one negative **midflight** recovery outcome with true trajectory proof, and soundness/availability measured across repeated trials.
- The v4.2 paper-ready narrow conclusions remain frozen and correctly referenced; no fabricated statistics or broad validation claims.
- All source/product regression gates pass where dependencies are available. Final report clearly separates supported claims from future SITL, networking and real-sensor research.

## 5. Dissertation language that is supported *now*

> In an agent-backed, mTLS-connected software simulation using 20 m directed graph-edge survey tasks, 100 paired seeds per scenario and a one-dimensional simulated camera-footprint oracle, localized gap repair using VECR reduced mean modeled flight distance by approximately **21.5%** relative to coverage-aware full-edge restart across scenarios S03–S05, with **300/300 true and reported completions per arm**. Both methods produced **zero observed false fulfillment reports** in the honest synthetic-sensor comparative campaign; this does not establish zero risk or a verification-accuracy advantage. In estimator-related scenarios, VECR sometimes had lower reporting availability. The experiments isolate post-arrival repair and do not demonstrate early midflight correction or SITL/physical-flight performance.
