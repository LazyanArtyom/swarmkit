# 01 / Research overview {#overview}

## Abstract

We developed and evaluated **Verified Edge Coverage and Recovery (VECR)** as a reusable, algorithm-independent method in the SwarmKit agent. VECR assigns a physical obligation to a directed graph edge, checks bounded navigation and qualified sensing evidence, preserves verified work, identifies unverified intervals, and executes bounded local recovery. Command acceptance, arrival, route traversal and survey fulfillment have distinct meanings. The external application retains ownership of graph planning and distributed coordination.

The completed **v5.1 study**, repeated after endpoint corrections, used the production SDK → local mTLS AgentService → agent runtime → SimBackend path. A noisy sensor estimator produced verifier-facing camera metadata; private simulated acquisitions and motion supplied independent ground truth. We executed **4,200 primary attempts** across 21 conditions and **1,080 sensitivity attempts** across 18 conditions, with both policies assigned to every seed. A separate serial benchmark executed 80 attempts. Every final assignment completed without an infrastructure error; unsuccessful missions remained in the analysis. The preceding v5 campaign is preserved separately and is not pooled with this replication.

Against coverage-aware full-edge restart B2, targeted VECR recovery reduced modeled distance by **25.3–27.8%** in three midflight conditions with 100/100 true and reported completions per policy per condition. Across 4,880 honest-sensor attempts, neither policy falsely reported fulfillment. All 400 deliberate dishonest-sensor controls falsely fulfilled, exposing the trusted sensor boundary. Persistent unavailable or overly uncertain evidence reduced acceptance, even when the actual survey was complete. These findings establish conditional utility and limitations within a flat, one-dimensional full-strip model. ArduPilot SITL has not been executed for this study.

## Research scope and evidence levels

| Evidence level | Status | What it establishes |
|---|---|---|
| Geometric simulation v1/v2 | Frozen historical evidence | Simplified interval/raster behavior; no production-agent execution |
| Agent-backed v3/v4.2 | Frozen historical evidence | Integration and earlier post-arrival repair findings |
| Agent-backed v5 | Frozen completed evidence | Noisy bounded sensing, actual midflight recovery in SimBackend, fair paired comparison |
| Agent-backed v5.1 | Completed corrected-code replication | Same full matrix after traversal and camera endpoint fixes; current results in this reader |
| ArduPilot SITL | Not yet executed | Reserved results structure in Chapter 15; no measurements |
| Real aircraft and camera imagery | Outside this simulator claim | No physical-flight or image-quality result is asserted |

This reader documents a completed simulator research contribution, not an awarded degree or airworthiness certification. The wider multi-user mission-engineering context motivates the work. A deployed SwarmOps cloud service, simultaneous swarm performance and radio capacity are outside the evidence established here.

# 02 / The problem and core contribution {#innovation}

## Four facts that must remain separate

A planner can select A→B, a backend can accept its command, and a vehicle can eventually arrive at B while leaving an intermediate strip unobserved. Conversely, a wide camera can observe the strip during an off-corridor excursion. Navigation deviation is therefore neither necessary nor sufficient evidence of a survey gap. Image acquisition can also fail on an otherwise correct path.

{{diagram:contract}}

**The implemented contribution is an edge-obligation execution contract:** the agent binds evidence to the exact mission, revision, edge and attempt; maintains the union of qualified coverage; retains disjoint completed intervals; exposes what remains missing; and recovers locally while respecting authority, evidence and safety constraints. An application consumes factual outcomes without embedding its planner in SwarmKit.

## Why the contribution is useful

1. **Coverage accounting survives a deviation.** Correctly accepted later images remain credited even when a middle interval is missing.
2. **Recovery is informed by missing work.** VECR chooses a target before the earliest unverified interval, with overlap and a midflight progress cap.
3. **Arrival cannot substitute for sensing.** A survey obligation additionally needs accepted full-strip evidence.
4. **Uncertainty has an explicit outcome.** Missing bounds or invalid observations cannot silently become healthy, exact evidence.
5. **Physical execution remains local.** The agent performs detection, verification and recovery; the server is not in the correction loop.

Waypoint checks, interval unions, robot recovery and authenticated messaging are established techniques. The defensible research claim is their implemented edge-level contract and its measured behavior against the specified coverage-aware baseline. The study does not establish worldwide priority for every constituent technique or superiority over all coverage planners.

## Relationship to graph coordination

Graph-theoretic traversal concerns vertices, edges and legal abstract transitions. The rotor-router literature studies such discrete models [R1]. The physical execution layer adds evidence for an assigned geographical obligation; it does not change rotor order, implement gossip or prove an arbitrary planner correct after reassignment. **No rotor-router or gossip algorithm was implemented or run in this campaign.** Algorithm independence is demonstrated by the API boundary, not by a multi-planner experimental comparison.

# 03 / System architecture {#architecture}

## Where the work runs

{{diagram:architecture}}

SwarmKit is a C++23 runtime and SDK with typed control, telemetry, authority, reporting and peer/server communication interfaces. The production agent owns the edge supervisor, sensor qualification, recovery dispatch and attempt journal. The simulator replaces the vehicle backend; it does not replace the agent's verification or recovery state machine. The benchmark process includes the SDK, service, runtime, backend and trace writer.

The sensor estimator accepts measured telemetry plus calibration and simulated sensor status. It has no simulator-truth input. A private oracle reads actual poses and actual acquired footprints, including failed acquisitions. A separately written Python validator reconstructs strip intervals from rectangle half-space inequalities rather than calling the production interval code.

## Existing general-purpose interfaces

Control commands, normalized telemetry, health/capability discovery, application peer messages, artifacts and server communication remain available. VECR operates through existing command authority and backend boundaries. Presence of these interfaces is not a result about remote network throughput or simultaneous multi-UAV behavior; those were not measured by v5.

An external mission client can visualize reports or decide subsequent assignments. This local reader is an explanation and evidence explorer, not a flight-control client or deployed multi-tenant service.

# 04 / Model, evidence and fulfillment {#model}

## Directed edge and identity

The agent accepts a straight two-point route A→B of length L in a fixed local metric frame. A request carries mission ID, revision, edge ID, attempt ID and drone ID. The route has a movement corridor and a required survey strip. The sensing adapter assumes flat terrain, rectangular nadir optics and route-aligned camera axes. Bent or multi-point routes are rejected because dispatch currently follows a single GOTO to B.

{{equation:identity}}

An obligation selects arrival, bounded traversal, survey, or traversal-and-survey. The primary v5.1 comparison uses **survey**. Survey completion needs qualified stopped arrival and accepted sensor evidence; it does not require every flight segment to have stayed inside the navigation corridor.

## Conservative sensing intervals

For measured altitude ĥ, vertical hard bound εh, horizontal hard bound εp, and calibrated along/cross FOV θa/θc, the adapter contracts the footprint using h− = ĥ − εh. A capture is usable only when acquisition, quality, alignment and flat-ground conditions are established and the remaining width covers the entire required strip.

{{equation:footprint}}

Here ŝ is the signed along-route position, d is perpendicular cross-track distance, and w is required strip half-width. The footprint interval is clipped to [0,L]; the camera center is never shifted onto an endpoint. The audit reproduced a false-coverage case caused by the previous clamped-center calculation, and v5.1 fixes it. The runtime checks identity, freshness, allowed sensor client, calibration and kinematic consistency against bounded telemetry. A camera command ACK supplies none of these facts. The default horizontal bound is 0.3 m; ideal zero-error camera centers are not used in the main campaign.

## Coverage union and missing work

{{equation:coverage}}

V is the accepted interval union and M its complement. Coverage is the sum of union lengths, not the furthest observed coordinate. Later disjoint intervals stay stored. The core uses a **10⁻⁶ m geometric tolerance** for merging and endpoint comparisons, distinct from allowed missing length. The mathematical statement uses ideal sets; implementation precision is finite and sub-tolerance holes can be merged. This tolerance is not physical sensor accuracy.

## Stopped arrival and survey fulfillment

{{equation:arrival}}

The radius check is three-dimensional: horizontal distance plus its hard bound and vertical offset plus its hard bound must fit inside the configured radius. Horizontal speed plus its bound must satisfy the stopping threshold. Qualifying observations span at least 200 ms. A later away, invalid or ambiguous observation revokes pending arrival qualification. An ACK or backend goal-reached report does not establish this predicate. Sampled dwell is not a proof of continuously stationary motion between arbitrary telemetry samples.

## Bounded traversal is separate

For endpoint route-distance bounds D0,D1, physical speed bound Vmax and sample interval Δt, distance-to-route continuity gives a sufficient condition for corridor conformance.

{{equation:traversal}}

The code also rejects kinematically impossible endpoint displacement. Guaranteed forward progress excludes uncertain endpoint positions: only [ŝ0+ε0, ŝ1−ε1] is credited when nonempty. The website audit exposed and corrected earlier crediting of the entire estimated endpoint interval. Counterexample regressions demonstrate the defect and fix. V5.1 repeated the full survey-policy campaign after both corrections; original v5 observations remain at their recorded source revision.

# 05 / Conditional correctness {#proofs}

## Assumptions

The conclusions require fixed edge/frame/attempt identity; truthful acquisition and quality assertions; calibrated geometry; valid deterministic error bounds; accepted timestamps; and correct interval bookkeeping. A reported GNSS accuracy estimate is not a hard bound unless its declared semantics justify that interpretation. Authentication establishes the sender's identity, not the truth of its physical assertions.

## Proposition 1 — Conditional survey soundness

Let F⁻j be the accepted full-strip region of capture j and Fj its actual region acquired at required quality. Assume F⁻j ⊆ Fj for every accepted capture. If the accepted union covers the required strip Re, then Re lies inside the actual observed union.

{{equation:soundness}}

**Proof.** Inclusion is preserved by union. Re ⊆ ∪F⁻j and ∪F⁻j ⊆ ∪Fj imply Re ⊆ ∪Fj. Stopped arrival is checked separately. A nonzero missing-length allowance limits the conclusion accordingly. Floating-point comparison tolerances qualify correspondence between the ideal sets and code. This is an elementary conditional argument, not a claim to have discovered a new set-theoretic theorem.

The substantive obligation is establishing conservative input evidence. Honest trials checked this assumption against each corresponding actual acquisition. M14/M15 violate it and show false completion. Their results demonstrate why the premise matters.

## Proposition 2 — Gaps are not replaced by a progress maximum

The supervisor stores sorted, merged intervals and recomputes missing work over the whole route. A disjoint capture beyond a resolvable hole cannot remove it. Earlier valid evidence remains available after a navigation deviation. Interval-union and agent regressions exercise this property. Numerical holes below the merging tolerance are subject to the precision limitation stated above.

## Proposition 3 — ACK cannot complete recovery

Accepted re-entry dispatch sets recovery in progress. Further departure samples cannot issue an overlapping repair. The runtime requires bounded stationary re-entry and an accepted continuation command before it calls `RecoveryReentryCompleted`. Until then, the supervisor cannot return to moving merely because a command was acknowledged. Safety or invalid evidence can interrupt this sequence. Independent trajectories reconstructed 4,246 completed re-entry dwells.

## Mission-level consequence

If an external application obtains a sound fulfilled result for every required edge, taking the union gives coverage of the union of required strips, subject to the same assumptions and tolerances. This does not prove every edge will be assigned, recovery always succeeds, a destroyed UAV's evidence survives, or an arbitrary planner retains its invariants after reassignment. Persistent wind, motor failure and unavailable evidence provide concrete unsuccessful outcomes.

# 06 / Implementation and reliability {#implementation}

## Canonical public contract

These are implemented interfaces in `proto/swarmkit/v1/swarmkit.proto`.

| Operation/type | Role |
|---|---|
| `ExecuteEdge` / `EdgeOperation` | Submit a typed obligation with identity, route, limits and repair policy |
| `GetEdgeStatus` / `EdgeReport` | Retrieve phase, reason, physical predicates, missing intervals and repair count |
| `SubmitEdgeCapture` | Submit qualified sensor metadata; the agent derives the interval |
| `CancelEdge` | Interrupt the software attempt through the agent |
| `EDGE_REPAIR_FULL_EDGE_RESTART` | Coverage-aware B2 target at A |
| `EDGE_REPAIR_TARGETED_GAP` | VECR target before the earliest gap, with overlap/progress cap |

{{diagram:states}}

The phases are **MOVING, DEVIATING, RECOVERING, FULFILLED, INTERRUPTED and UNKNOWN**. Earlier reader labels such as EDGE_VERIFIED and TRACKING are not the canonical report enum.

## Reliability corrections

| Defect or ambiguity | Implemented behavior | Evidence |
|---|---|---|
| Arrival inferred from ACK or an endpoint hit | Bounded 3D stopped arrival, dwell, freshness and revocation | Core and agent regressions |
| Multi-point route dispatched as one endpoint command | Agent rejects unsupported route shape | Route acceptance regressions |
| Camera ACK or unbounded pose counted as survey | Qualified metadata, calibration/freshness/identity checks | Sensor and runtime regressions |
| Recovery ACK changed phase before physical return | RECOVERING persists through re-entry and continuation | v5 regressions and reconstructed dwells |
| Repeated departure overwrote active recovery | Active repair guard; completion-based cooldown | v5 regressions |
| Absent bounds became exact departure evidence | Unknown bounds reset the count; spatial hysteresis | v5 regressions |
| Future unseen tail targeted ahead of current progress | Midflight target capped at projected progress | Target-selection regression and commands |
| Estimated endpoints overstated guaranteed traversal | Interval contracted by endpoint uncertainty | October 26 before/after counterexamples |
| Off-segment camera center shifted its footprint onto A or B | Signed axial center; clip only the resulting footprint | Eastbound/northbound, both endpoints and partial-overlap regressions |

The three-sample departure trigger tracks failure of the bounded corridor-conformance test. An uncertainty envelope crossing the boundary does not itself prove the actual vehicle is outside. Midflight experimental gates additionally check the actual excursion from private truth.

## Safety, durability and resource limits

Recovery is enabled only for SimBackend and requires authority, suitable mode, battery, navigation bounds and health. Live MAVLink recovery remains disabled pending qualified feasibility/geofence behavior. A timeout marks the attempt interrupted; the report alone does not prove physical stop.

The attempt journal is fsynced before dispatch. A durable terminal outcome can be retrieved after restart; an ambiguous outcome remains UNKNOWN and cannot be redispatched with the same identity. This is duplicate suppression while history is retained, not exactly-once physical execution. Corrupt, full, locked or unwritable journals fail closed. Limits include 4,096 in-memory attempts, 4,096 capture identities per attempt and a 64 MiB journal without compaction. Persistent storage must be retained.

# 07 / Completed experimental design {#protocol}

## Fair B2 versus VECR

{{diagram:comparison}}

B2 already knows which survey evidence is accepted. It shares the verifier, sensor, dynamics, schedules, thresholds, safety, deadline and repair budget with VECR. B2 targets A; VECR targets before the earliest missing interval. Both retain valid earlier captures. The comparison estimates recovery target selection, not the benefit of adding verification to an uninformed baseline.

## Assignment and operating conditions

| Parameter | Frozen v5.1 setting (unchanged from v5) |
|---|---|
| Primary | M00–M20 × 100 paired seeds (1–100) × 2 policies = 4,200 attempts |
| Sensitivity | X00–X17 × 30 paired seeds (1–30) × 2 policies = 1,080 attempts |
| Order | Both methods on every seed, order balanced by seed |
| Default route / speed / acceleration | 40 m / 2 m/s / 2 m/s² |
| Strip and corridor | 4 m half-width |
| Telemetry / private truth | 100 ms / 20 ms integration |
| Captures / metadata delay | 0.5 s / 0.2 s default |
| Bounds | 0.3 m horizontal, 0.05001 m altitude, 0.03001 m/s speed |
| Departure / hysteresis / cooldown | 3 samples / 0.5 m / 1 s after completed re-entry |
| Dwell / recovery budget | 200 ms / 2 attempts |
| Deadline | 90 s; 80 m sensitivity condition uses 180 s |
| Actual optics | 0–2% wider than fixed conservative calibration |

Noise includes nonzero bias and bounded error. It depends on seed and absolute simulation tick, never policy. Spatial/time-scheduled faults can expose policies differently because their paths differ; this is recorded rather than forced to have identical post-treatment exposure. Sampled timeout cases terminate at 90.2 s.

## Independent truth and resolution

{{diagram:evidence}}

The primary oracle intersects each actual rectangle with both strip boundaries and unions the resulting full-strip intervals. It does not mosaic partial-width images. Independent Python reconstruction checks geometry, acquisition success, union, distance, stopped arrival, every accepted footprint and completed re-entry dwell.

All-cell and cell-center rasters at **0.1, 0.25, 0.5 and 1 m** are secondary sensitivity measures. No final trial changed completion classification across these resolutions. A regression with a 0.01 m hole demonstrates why cell-center sampling can nevertheless miss smaller holes. Exact union means interval geometry rather than raster sampling; numerical tolerances still apply.

## Statistical protocol

Per-scenario effects use paired seed differences, 10,000 bootstrap resamples and RNG 20261024. Precision, recall, acceptance and false-completion intervals use Wilson 95%. Undefined denominators stay undefined. Matched-success costs require both methods to truly complete and report fulfillment. All-assigned outcomes accompany them.

Seed IDs recur across conditions. Aggregate paired intervals resample **100 whole seed clusters**, retaining all 19 honest primary conditions. Pooled IID binomial intervals were removed in a documented correction. These intervals describe the chosen synthetic variation, not populations of real aircraft, terrain or weather. No multiple-comparison significance claim or favorable-seed filtering was used.

# 08 / Measured results {#results}

## Equal-success midflight comparison

{{matched_table}}

Each row has **100 matched true-and-reported successes** per policy. Distance includes recovery and return to B. Corresponding time differences are −9.943 s [−9.961, −9.925], −10.113 s [−10.147, −10.080] and −9.248 s [−9.264, −9.232], with paired 95% bootstrap intervals.

{{figure:02_matched_success_distance}}

## Every primary and sensitivity condition

The explorer is generated from frozen CSV tables. **True completion** means actual full-strip survey plus stopped arrival at B. **Reported** means FULFILLED. Primary rows contain 100 attempts per policy; sensitivity rows contain 30. The two dishonest-sensor controls remain visible and labeled.

{{scenario_explorer}}

## Fixed-mixture aggregate

Across 19 honest primary conditions, mean VECR−B2 distance is **−11.307 m [−11.430, −11.157]**, and time is **−3.715 s [−3.780, −3.636]**. Reported acceptance differs by **+4.947 percentage points [4.684, 5.158]**. These combine successful and unsuccessful work with this exact weighting. They are not fleet energy savings or cost per certified mission.

# 09 / Replay the recorded experiment {#replay}

This replays **seed 1** from positive, negative and trust-boundary conditions. It reads logged states and commands; no hypothetical percentage sequence is used. Both policies share the time axis. Displayed 100 ms trace samples are distinct from the 20 ms truth used by the validator. Display values are rounded; hovering over missing intervals shows their exact recorded text, also preserved in the downloaded logs. The controls have no connection to a live vehicle.

{{trace_explorer}}

The accepted-coverage bar is the agent's union. The separate truth bar is independently observed full-strip coverage. They disagree in the dishonest-sensor case. A target marker records the last accepted GOTO; it does not prove arrival. Final phases remain the actual recorded outcome, including INTERRUPTED. Playback starts only when requested.

{{figure:03_positive_negative_trajectories}}

# 10 / Availability, failures and counterexamples {#availability}

## Honest primary outcomes

{{confusion_table}}

Both methods achieved 1,300 true surveys. B2 achieved 800 with stopped arrival, of which 700 were accepted. VECR achieved 1,094, of which 794 were accepted. Conditional recall is **87.50% for B2 and 72.58% for VECR**, despite VECR accepting more missions. The denominators differ: in GNSS/estimator-invalid conditions VECR returned to B while B2 was still repairing. Neither could certify persistent missing evidence.

{{figure:01_availability}}

## Unsuccessful outcomes

* **M02 sustained wind:** recovery actually began before B; neither policy completed the survey or reported fulfillment.
* **M08/M09 GNSS/estimator invalidity:** repeated movement did not restore bounded evidence in the fault region; both reported zero fulfillment.
* **M10 unbounded pose:** both physically completed; neither certified completion.
* **M11/M12 stale or wrong calibration:** sensor records were rejected; the deadline constrained recovery.
* **M16/M17 command refusal or motor failure:** actual negative interventions occurred; neither policy fulfilled.
* **M18/M19 exhausted budget or low battery:** the required unsuccessful outcomes occurred.
* **M20 sparse captures:** VECR accepted 94/100 versus B2 0/100; paired difference +0.94 [0.89, 0.98]. No equally successful pairs exist for conditional cost estimation.

Final phases were **2,914 FULFILLED and 2,366 INTERRUPTED**. FULFILLED includes the false control outcomes below. No final row was UNKNOWN, but transient UNKNOWN states are retained. Absence of final UNKNOWN does not imply uncertainty never occurred.

## Trusted-sensor counterexamples

M14 falsely asserted acquisition success; M15 concealed a 45° ground-footprint yaw offset. Together they produced **400 false fulfillment reports in 400 attempts** across both methods. This deliberately failed physical outcome demonstrates the trust boundary. Encryption and conservative geometry cannot establish a dishonest assertion's truth.

Across **4,880 honest-sensor attempts**, there were zero observed false fulfillments and zero uncontained accepted footprints among **256,395 accepted captures**. This is conditional model evidence, not zero-risk proof or a verification-accuracy advantage over B2. For 0/100 failures in one condition, the Wilson 95% upper bound is still about 3.7%; reused seeds across conditions cannot become thousands of unrelated physical trials.

# 11 / Sensitivity and robustness {#sensitivity}

Eighteen one-factor conditions vary route length, along-track footprint, capture period, horizontal uncertainty, wind, recovery count, cross-track footprint and gap location/length. Exact assignments are below. No favorable subset replaced the frozen matrix.

{{figure:04_sensitivity}}

All 540 sensitivity attempts per method truly completed. Each method accepted **510/540**, with **30 unaccepted true completions** and zero false fulfillment. All misses occurred in X07, the **1 m horizontal-bound** condition. A conservative footprint becomes unavailable when uncertainty consumes its usable width. The soundness rule was not relaxed to raise acceptance.

{{sensitivity_matrix}}

## Resolution and useful evidence

{{figure:05_raster_sensitivity}}

{{figure:06_truth_and_verified_coverage}}

Each credited interval needs a complete strip cross-section justified by an individual footprint. The model omits terrain, occlusion, exposure, blur and reconstruction quality. Angle controls use yaw-rotated ground rectangles, not calibrated pitch/roll photogrammetry. These assumptions limit generalization even when all numerical gates pass.

# 12 / Host performance {#performance}

After the science campaign, the same release executable ran **80 serial processes**: M00/M03/M05/M08 × 10 paired seeds (20001–20010) × two methods. Four-worker scientific timings were not repurposed as the isolated benchmark.

{{performance_table}}

{{figure:07_host_performance}}

CPU/RSS include SDK, service, runtime, backend and trace writer, including startup. Capture latency measures the mTLS submission RPC, not the pure core verifier. Its p95 summary is a distribution of each trial's p95, not a pooled per-capture quantile. Report/capture rates describe accelerated instrumented execution, not radio goodput.

Host: Apple M3 Pro, macOS 26.6.2, clang 17.0.0 and CMake 3.31.6. These results do not establish Raspberry Pi capacity, hard real-time deadlines, energy use or aircraft performance. Agent-only profiling was not performed.

# 13 / Reproducibility and audit trail {#reproducibility}

## Executed validation gates

{{gates}}

The original v5 pilots assigned 744 attempts: 743 returned results and one startup infrastructure error. All remain preserved and excluded from confirmatory inference. Its early generic error omitted the specific cause; the report does not invent one. Per-trial journal isolation and detailed errors were added before final execution. A separate v5.1 engineering check ran **78/78 attempts**, with seed 10004 in every condition, before the corrected confirmatory campaign; independent reconstruction checked 6,764 delivered captures and 69 re-entry dwells.

All **330 missed true completions in v4.2** were enumerated. Archived logs lack sufficient per-capture detail for full retrospective rejection attribution. V5 records measurements, acquisitions, metadata outcomes/reasons, physical trajectories and commands prospectively. V4.2's 300 equally successful localized-repair pairs and approximately 21.5% distance reduction remain frozen.

## Sources and artifact identity

{{provenance}}

Historical v5 analysis corrections changed a time-unit label to seconds and replaced inappropriate pooled IID uncertainty with seed-cluster intervals. That superseded analysis is retained. **47,523 original raw files were compared; zero changed.** No v5 trial was rerun, excluded or edited for those analysis corrections. V5.1 reuses the corrected inference rules and records new trials after the code fixes.

The October 26 audit reproduced both endpoint defects before fixing them. Its separate v5.1 campaign does not retroactively change v5 observations or executable identity. Comparing all 5,280 corresponding rows, **distance, elapsed time, true/reported completion, false-completion counts and repair counts are unchanged**. Final reported coverage lengths differ in 606 attempts by at most **2.85 × 10⁻¹⁴ m**, a floating-point rounding-scale change. These observations do not make the defects harmless: the independent endpoint counterexamples previously failed and now pass. The original reader is archived; this edition is the current narrative.

{{replication_table}}

{{downloads}}

## Reproduce in a new directory

For the current results, build experimental source **`7da3db33e70114503dfc7c9a45671da2b9335541`** with the recorded release environment and protocol amendment. Generate analysis using **`{{analysis_revision}}`**. That later analysis revision changes the report's version-specific validation link and adds the raw-row comparison utility; it does not change inference. The first report and its analysis manifest are preserved in `gates_v5_1/initial_analysis_reference`. Original v5 used experimental source `7896f20`, corrected analysis `62a3363` and publication `456381c`; its outputs remain separate. Historical directories must not be overwritten.

```bash
python scripts/vecr/run_agent_campaign_v5.py --mode final --output results/new-science --workers 4
python scripts/vecr/run_agent_campaign_v5.py --mode benchmark --output results/new-benchmark --workers 1
python scripts/vecr/analyze_agent_campaign_v5.py --output results/new-science --benchmark results/new-benchmark
python scripts/vecr/validate_agent_campaign_v5.py --output results/new-science
```

Reproduction needs the recorded build and Python requirements. Reading the site needs no server, installation or internet. It bundles tables, original figures and representative traces. The full per-attempt trace collection stays in the repository's frozen research directory, identified by SHA-256 inventory.

# 14 / Defense: claims and answers {#defense}

## Supported statements

**Method.** We implemented an onboard contract distinguishing qualified arrival, bounded traversal, accepted survey and local recovery, with exact attempt identity and explicit unavailable evidence.

**Correctness.** Conservative truthful footprints support conditional survey soundness. Geometry/state regressions exercise the code; independent reconstruction checks assumptions in the simulator.

**Measured utility.** In M03–M05, targeted recovery reduced travel by 25.3–27.8% against coverage-aware full restart, with 100/100 true and reported completions per method in each condition. Failed and unaccepted work is reported alongside this conditional result.

**Limits.** The study measures modeled repair cost and evidence availability. It does not establish physical-camera correctness, universal recovery, better verification accuracy than the shared verifier or fleet-wide efficiency.

{{defense_questions}}

## Defense demonstration

Start with the contract and explain why arrival leaves an intermediate sensing obligation unresolved. Show the onboard architecture. Replay M03 for both policies, M04 for off-corridor evidence retention, M17 for failed recovery and M14 for dishonest evidence. Present equal-success confidence intervals together with availability. Finish with provenance and the explicitly unexecuted SITL section.

The replay uses seed 1 consistently, not a seed selected for a favorable outcome. Numerical claims use the full paired populations, not illustrative individual runs.

# 15 / ArduPilot SITL validation {#sitl}

<div class="pending-panel"><strong>Not yet executed · measurement cells are TBD</strong><p>This is the prepared results structure for the next stage. No VECR ArduPilot trials, effects, pass status or confidence intervals are implied. Digital-simulation measurements are not copied into these cells.</p></div>

## Experimental record

ArduPilot SITL runs autopilot software against simulated dynamics [R3], adding autopilot mode/command/telemetry behavior beyond SimBackend. The stage needs independent truth, declared observation bounds and qualified recovery feasibility; live MAVLink recovery currently remains disabled.

| Record | SITL value |
|---|---|
| Executed trial count | TBD — not executed |
| ArduPilot version / commit | TBD |
| SwarmKit source / binary SHA-256 | TBD |
| Vehicle model, parameters and environment | TBD |
| Frozen assignments and paired seeds | TBD |
| Independent truth and sensor assumptions | TBD |
| Safety/geofence recovery qualification | TBD |
| Raw logs, manifest and SHA-256 | TBD |

## Baseline and VECR measurements

| Measurement | B2 | VECR | Paired difference / 95% CI |
|---|---|---|---|
| True survey + stopped B | TBD | TBD | TBD |
| Reported fulfillment | TBD | TBD | TBD |
| False fulfillment | TBD | TBD | TBD |
| Unaccepted true completion / recall | TBD | TBD | TBD |
| Matched-success distance | TBD | TBD | TBD |
| Matched-success completion time | TBD | TBD | TBD |
| All-assigned distance and time | TBD | TBD | TBD |
| Failed / refused recovery | TBD | TBD | TBD |
| CPU / peak RSS / RPC latency | TBD | TBD | TBD |

## Analysis structure

Assignment counts, realized faults, actual trajectories, accepted evidence, reasons and uncertainty have separate records. A positive effect is not a validation prerequisite. Failed experiments retain their outputs; a fix requires a new versioned run. Conclusions can be added only after measured evidence supports them. SITL cannot establish real-camera image quality or field airworthiness.

# 16 / Conclusion and references {#conclusion}

We developed VECR in SwarmKit and completed its defined agent-backed digital evaluation. The method connects graph-edge execution to navigation and sensing evidence, retains completed intervals and recovers locally from missing work. Its measured benefit is lower recovery travel in the tested equal-success conditions. Its measured weaknesses are evidence-driven nonacceptance and dependence on truthful sensor assertions.

The simulator study, statistics, sensitivity analysis, benchmark and independent reconstruction are complete. **ArduPilot SITL is the remaining validation stage in this presentation.** Hardware, real camera calibration, complex terrain, obstacle/geofence-aware recovery, remote/multi-UAV campaigns, OS power-loss experiments and cloud deployment lie outside the completed simulator claim. These boundaries remain relevant after SITL.

## Primary references and source-to-claim mapping

* **[R1]** Holroyd, Levine, Mészáros, Peres, Propp and Wilson. *Chip-Firing and Rotor-Routing on Directed Graphs*, 2008, revised 2013. [Primary paper](https://arxiv.org/abs/0801.3306). Background on abstract directed-graph processes; no VECR result is attributed to it.
* **[R2]** MAVLink. *Command Protocol*. [Official documentation](https://mavlink.io/en/services/command.html), accessed 26 October 2026. Command acknowledgement/result protocol; SwarmKit supplies the survey contract.
* **[R3]** ArduPilot. *SITL Simulator*. [Official documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html), accessed 26 October 2026. Defines the next evidence level; contains no result for this campaign.
* **[S1]** V5.1 experimental source `7da3db3`: `src/core/vecr.cpp`, `src/agent/edge_execution_runtime.cpp`, `proto/swarmkit/v1/swarmkit.proto`, `tests/vecr_v5_trial.cpp`, `tests/vecr_v5_models.h`. Implemented contracts and experiment architecture.
* **[S2]** `corrected_endpoints_v5_1` manifest, raw CSV/traces and SHA256SUMS. Current scientific numbers; original `final_v5` remains historical.
* **[S3]** Analysis source `{{analysis_revision}}`, retaining inference from `62a3363`: `analyze_agent_campaign_v5.py`, `validate_agent_campaign_v5.py`, `test_analysis_v5.py`. Reconstruction and uncertainty calculations.
* **[S4]** `FINAL_VALIDATION_V5_1.md`, both protocol documents, historical `V5_ANALYSIS_NOTES.md` and gate logs. Scope, replication and corrections.
* **[S5]** Endpoint before/after regressions and reader-validation records under this site's `validation/`.

## Notation

| Symbol | Meaning |
|---|---|
| A, B, L | Edge origin, destination and length |
| ŝ, d, w | Along-route estimate, cross-track distance, strip/corridor half-width |
| εp, εh, εv | Hard horizontal, altitude and speed bounds |
| V, M | Accepted coverage union and missing intervals |
| F⁻j, Fj | Conservative accepted and actual acquired footprints |
| rB, vB, τ | Arrival radius, speed threshold and sampled dwell |
| B2, P | Full-edge restart and targeted-gap VECR |
| TP / FP / FN | True reported / false reported / true completion unaccepted |

<p class="end-mark">Completed digital-simulation study · reproducible, source-linked and explicit about its limits.</p>
