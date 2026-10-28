# 01 / Research overview {#overview}

## Researcher and institutional context {#researcher}

{{author_profile}}

## Abstract

We developed and evaluated **Verified Edge Coverage and Recovery (VECR)** as a reusable, algorithm-independent method in the SwarmKit agent. VECR assigns a physical obligation to a directed graph edge, checks bounded navigation and qualified sensing evidence, preserves verified work, identifies unverified intervals, and executes bounded local recovery. Command acceptance, arrival, route traversal and survey fulfillment have distinct meanings. The external application retains ownership of graph planning and distributed coordination.

The completed **v6 primary campaign** executed **5,940 attempts: 22 conditions × 30 paired seeds × nine policies** through the production SDK → local mTLS AgentService → agent runtime → SimBackend path. A separate 200-attempt campaign compared direct and routed SDK clients. Independent reconstruction checked 516,751 delivered captures and 3,984 physical re-entry dwells. Historical v5.1 results remain available in the reader and are not pooled with v6.

The stronger comparison separates **coverage tracking, evidence waiting and target conservatism**. Under delayed metadata, waiting reduced travel by 6.227 m versus immediate gap repair (95% paired CI [−6.291, −6.162] m). The derived uncertainty-envelope target E and the simpler waiting gap rule GW were identical there. Under multiple gaps, E traveled **6.240 m farther** than GW (CI [+6.23995, +6.24020] m), with no extra completions. Gap-aware policies reported 30/30 completions where departure resumption and fixed rewind reported 0/30. These are conditional findings, not predetermined support for a preferred policy.

Across **5,400 honest-sensor attempts**, no policy falsely fulfilled. The **540 dishonest-sensor controls** are reported separately; authentication does not establish physical truth. The technical contribution is an implemented reusable edge-obligation contract, a restricted conditional recovery rule and a reproducible characterization of its advantages and costs. The data do **not** establish that E is the best recovery algorithm or that returning to a departure point is new. ArduPilot qualification is a separate evidence tier in Chapter 15; real camera quality remains unvalidated.

## Research scope and evidence levels

| Evidence level | Status | What it establishes |
|---|---|---|
| Geometric simulation v1/v2 | Frozen historical evidence | Simplified interval/raster behavior; no production-agent execution |
| Agent-backed v3/v4.2 | Frozen historical evidence | Integration and earlier post-arrival repair findings |
| Agent-backed v5 | Frozen completed evidence | Noisy bounded sensing, actual midflight recovery in SimBackend, fair paired comparison |
| Agent-backed v5.1 | Frozen corrected-code replication | Historical two-policy comparison; retained figures and replay |
| Agent-backed v6 | Completed primary campaign | Nine policies, 22 conditions, 5,940 trials; current comparison |
| Client reuse v6 | Completed, 200 trials | 100 matched direct/routed client pairs; no metric differences |
| ArduPilot SITL v6 | Separate qualification | Actual autopilot dynamics; native-accuracy controls and explicitly synthetic sensing, Chapter 15 |
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

## Literature comparison and originality boundary

A targeted primary-source literature review on **26–28 October 2026** found substantial overlap with earlier work. V6 adds the previously missing resume/rewind comparisons and mechanism ablations. Its modest conditional result and negative comparison limit the originality claim. The following sources constrain it.

| Earlier work | Established overlap | Distinction VECR must substantiate |
|---|---|---|
| [Zudaire et al., ICRA 2021](https://software.imdea.org/~cesar/papers/zudaire21assumption.pdf) [R4] | UAV assumption monitoring, incomplete photographic coverage and corrective actions | The particular edge-obligation contract and retained evidence semantics |
| [Arvanitakis et al., 2018](https://pure.coventry.ac.uk/ws/files/13602376/1729881417750785.pdf) [R5] | Guaranteed sensed regions under bounded pose uncertainty, accumulated over time | An additional technical result beyond conservative-set union |
| [ArduPilot mission rewind](https://ardupilot.org/copter/docs/common-mission-rewind.html) [R6] | Survey resumption behind an interruption point | Evidence-selected missing intervals versus a configured rewind distance |
| [Fields2Cover, 2023](https://arxiv.org/abs/2210.07838) [R7] | Modular coverage-planning library and comparative evaluation | The execution/acceptance contract and its measured added utility |
| [Bouman et al., 2023](https://arxiv.org/abs/2302.03164) [R8] | Planning with accumulated sensor coverage and execution constraints | The narrower assigned-edge problem and its applicability |

Local repair of processing faults also predates this work: [Nykolaychuk and Ortmeier, 2015](https://doi.org/10.1007/978-3-319-22876-1_31) [R9]. Correct monitoring of streams arriving with different delays is established by [Leucker et al., 2020](https://link.springer.com/article/10.1007/s11219-019-09493-y) [R10]. Neither local repair nor delayed-event handling is claimed as a first invention.

V6 now compares full restart, departure resumption, two fixed rewinds, retained-gap recovery, waiting and the derived envelope target under shared evidence and safety. The simple GW comparator is as successful as E and often cheaper. The defensible thesis is **“Evidence-conditioned execution and recovery of assigned survey edges: an onboard contract, conditional bounds and comparative validation.”** The specific reusable contract and measured mechanism tradeoffs are the contribution; global algorithmic superiority and worldwide priority are not established. This targeted review is not an exhaustive priority search. The [updated assessment](assets/evidence/v6/NOVELTY_AND_DEFENSE_ASSESSMENT_V6_2026-10-28.md) identifies exact overlaps and limits; the [October 26 assessment](assets/evidence/NOVELTY_AND_DEFENSE_ASSESSMENT_2026-10-26.md) remains a historical record.

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

## V6: uncertainty-aware re-entry and its conditional guarantee

{{v6_rule}}

Let g be the first missing coordinate, p current projected progress, a the hard re-entry radius, v the physical speed bound, T the maximum time to a qualifying capture, e the maximum declared capture position error and H the minimum uncertainty-free along half-footprint. All captures must genuinely qualify across the whole strip. The first estimated center is at most q + a + vT + e; its accepted left endpoint is at most q + a + vT + 2e − H. Therefore:

<div class="equation" role="group" aria-label="Conditional recovery target"><div>q* = min(p, g − a − vT − 2e + H), if q* ≥ 0</div><small>Largest on-edge target satisfying the sufficient first-capture start bound</small></div>

Substitution shows that the first accepted interval **begins no later than g**. It need not contain g. Consecutive accepted intervals cannot separate under the additional sufficient condition **vT + 4e ≤ 2H**. Endpoint coverage still needs actual accepted captures. When the target expression is negative, returning to A is an explicitly unguaranteed retry.

For current coordinate (x,y), straight transit followed by a sweep to B costs √((x−q)²+y²) + L−q. This decreases as feasible q increases, so q* minimizes that restricted cost. It does not optimize real acceleration, energy, obstacles, future failures or arbitrary routes. The measured E policy can—and does—travel farther than a simpler rule that succeeds without enforcing the sufficient worst-case constraint.

At a currently verified stationary B, a bounded wait lets delayed observations arrive. It never creates coverage or predicts a future capture. Freshness, safety, endpoint validity and the overall deadline remain active. A fixed wait wastes time when missing data will never arrive. See the [complete derivation](assets/evidence/v6/RECOVERY_MODEL_V6.md).

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

# 07 / Experimental protocols {#protocol}

## Current v6 frozen design

Nine policies use the same acquisition-time history, conservative verifier, sensing model, simulated dynamics, safety, retry budget and common random streams. Only target selection and bounded endpoint waiting differ. The defaults are a 40 m edge, commanded speed 2 m/s, physical speed bound 5 m/s, 0.3 m position bound, 0.5 s capture interval, maximum capture age 3 s, at most two repairs and a 90 s deadline. Stress conditions vary these explicitly in the manifest.

| Policy | Recovery target | Endpoint wait |
|---|---|---|
| B2 | Restart at A | None |
| D | Recorded departure point; B if no departure | None |
| R2 / R8 | 2 m / 8 m before departure point | None |
| P | First gap minus 2 m overlap | None |
| G | First gap, zero overlap | None |
| GW | Same as G | Up to 3 s |
| E0 | Conditional sensing-envelope q* | None |
| E | Same as E0 | Up to 3 s |

Twenty-two conditions include delayed and reordered evidence, one/multiple gaps, changing uncertainty, departure, visible/hidden deviations, persistent camera loss, refusal, motor failure, low battery, no repairs, expired evidence, sparse captures, 20/80 m edges, dishonest controls and an insufficient deadline. Thirty paired seeds 30001–30030 yield 5,940 primary attempts. All assignments returned results. No unsuccessful mission was removed. The fixed stress mixture is not a deployment distribution.

Pointwise 95% confidence intervals use 10,000 paired bootstrap resamples (RNG 20261027); proportions use Wilson intervals. Cost is reported among **jointly true and reported successful pairs**, with the denominator. These intervals describe the seeded simulator, not its real-world fidelity, and are not corrected for multiple comparisons. The conditional bound, protocol and raw data are downloadable in Chapter 13.

## Historical v5.1 design — separate study

The remainder of this protocol subsection describes the preserved two-policy v5.1 experiment. Its samples, conditions and benchmark are not pooled with v6.

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

## Current v6 results and strongest comparator

**E has no completion advantage over G/GW in this matrix.** In delayed evidence, E equals GW, isolating waiting as the useful mechanism. With multiple missing captures, E travels farther than GW and waiting adds time. Gap-aware policies repair earlier missing work that departure and rewind heuristics can fail to revisit. P still improves on full restart in the specified midflight conditions, but that does not make it the best comparator.

{{v6_contrasts}}

{{v6_figures}}

All six coverage-aware policies (B2/P/G/GW/E0/E) truly and reportedly complete 360/600 honest attempts per policy. D/R2/R8 each truly complete 270/600 and report 210/600, leaving 60 false negatives. All nine have zero false fulfillment in the 5,400 honest attempts. The {{v6_false_controls}} false fulfillments in 540 deliberately dishonest controls expose the common trusted-sensor boundary. Shorter travel in an interrupted trial is not an efficiency win.

{{v6_matrix}}

## Historical v5.1 comparison — preserved evidence

The following figures and tables describe only the 5,280-attempt v5.1 study. These established benefits against B2; v6's stronger G/GW comparisons above narrow the interpretation.

## Equal-success midflight comparison

{{matched_table}}

Each row has **100 matched true-and-reported successes** per policy. Distance includes recovery and return to B. Corresponding time differences are −9.943 s [−9.961, −9.925], −10.113 s [−10.147, −10.080] and −9.248 s [−9.264, −9.232], with paired 95% bootstrap intervals.

{{figure:02_matched_success_distance}}

## Every primary and sensitivity condition

The explorer is generated from frozen CSV tables. **True completion** means actual full-strip survey plus stopped arrival at B. **Reported** means FULFILLED. Primary rows contain 100 attempts per policy; sensitivity rows contain 30. The two dishonest-sensor controls remain visible and labeled.

{{scenario_explorer}}

## Fixed-mixture aggregate

Across 19 honest primary conditions, mean VECR−B2 distance is **−11.307 m [−11.430, −11.157]**, and time is **−3.715 s [−3.780, −3.636]**. Reported acceptance differs by **+4.947 percentage points [4.684, 5.158]**. These combine successful and unsuccessful work with this exact weighting. They are not fleet energy savings or cost per certified mission.

# 09 / Historical v5.1 recorded replay {#replay}

This replays **seed 1** from positive, negative and trust-boundary conditions. It reads logged states and commands; no hypothetical percentage sequence is used. Both policies share the time axis. Displayed 100 ms trace samples are distinct from the 20 ms truth used by the validator. Display values are rounded; hovering over missing intervals shows their exact recorded text, also preserved in the downloaded logs. The controls have no connection to a live vehicle.

{{trace_explorer}}

The accepted-coverage bar is the agent's union. The separate truth bar is independently observed full-strip coverage. They disagree in the dishonest-sensor case. A target marker records the last accepted GOTO; it does not prove arrival. Final phases remain the actual recorded outcome, including INTERRUPTED. Playback starts only when requested.

{{figure:03_positive_negative_trajectories}}

# 10 / Historical v5.1 availability and failures {#availability}

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

# 11 / Historical v5.1 sensitivity {#sensitivity}

Eighteen one-factor conditions vary route length, along-track footprint, capture period, horizontal uncertainty, wind, recovery count, cross-track footprint and gap location/length. Exact assignments are below. No favorable subset replaced the frozen matrix.

{{figure:04_sensitivity}}

All 540 sensitivity attempts per method truly completed. Each method accepted **510/540**, with **30 unaccepted true completions** and zero false fulfillment. All misses occurred in X07, the **1 m horizontal-bound** condition. A conservative footprint becomes unavailable when uncertainty consumes its usable width. The soundness rule was not relaxed to raise acceptance.

{{sensitivity_matrix}}

## Resolution and useful evidence

{{figure:05_raster_sensitivity}}

{{figure:06_truth_and_verified_coverage}}

Each credited interval needs a complete strip cross-section justified by an individual footprint. The model omits terrain, occlusion, exposure, blur and reconstruction quality. Angle controls use yaw-rotated ground rectangles, not calibrated pitch/roll photogrammetry. These assumptions limit generalization even when all numerical gates pass.

# 12 / Historical v5.1 host performance {#performance}

After the science campaign, the same release executable ran **80 serial processes**: M00/M03/M05/M08 × 10 paired seeds (20001–20010) × two methods. Four-worker scientific timings were not repurposed as the isolated benchmark.

{{performance_table}}

{{figure:07_host_performance}}

CPU/RSS include SDK, service, runtime, backend and trace writer, including startup. Capture latency measures the mTLS submission RPC, not the pure core verifier. Its p95 summary is a distribution of each trial's p95, not a pooled per-capture quantile. Report/capture rates describe accelerated instrumented execution, not radio goodput.

Host: Apple M3 Pro, macOS 26.6.2, clang 17.0.0 and CMake 3.31.6. These results do not establish Raspberry Pi capacity, hard real-time deadlines, energy use or aircraft performance. Agent-only profiling was not performed.

# 13 / Reproducibility and audit trail {#reproducibility}

## Current v6 validation and reuse

The primary independent validator reconstructed **516,751 delivered captures and 3,984 physical re-entry dwells**. The separate 200-attempt client study has **zero differences across 100 matched direct Client / routed SwarmClient pairs** for phase, true/reported completion, false fulfillment, distance, duration, repairs and reported coverage. This demonstrates reuse of the same agent through two public API paths; it is not evidence for two independent planners. An installed SDK consumer also compiled.

V6 regression gates cover delayed acquisition consistency, expired/mismatched evidence, missing physical speed bounds, endpoint wait safety, distinct target rules and invalid envelopes. Native MAVLink recovery remains disabled; a separately identified SITL adapter exercises recovery under declared synthetic observations. Release and UBSan suite logs, configuration manifests, executable identities and SHA-256 inventories accompany the records.

Pilots are retained: 40 initial trials, an old-binary infrastructure failure affecting 396 assignments, 396 corrected full-pilot trials and 198 final preflight trials. A complete 3,960-attempt run used an accidental 20-seed default; it remains a protocol-deviation collection and is not pooled with the corrected 5,940-attempt primary campaign. The correction was made from the assignment count before inspecting its comparative statistics.

{{v6_downloads}}

## Historical v5.1 validation gates



{{gates}}

The original v5 pilots assigned 744 attempts: 743 returned results and one startup infrastructure error. All remain preserved and excluded from confirmatory inference. Its early generic error omitted the specific cause; the report does not invent one. Per-trial journal isolation and detailed errors were added before final execution. A separate v5.1 engineering check ran **78/78 attempts**, with seed 10004 in every condition, before the corrected confirmatory campaign; independent reconstruction checked 6,764 delivered captures and 69 re-entry dwells.

All **330 missed true completions in v4.2** were enumerated. Archived logs lack sufficient per-capture detail for full retrospective rejection attribution. V5 records measurements, acquisitions, metadata outcomes/reasons, physical trajectories and commands prospectively. V4.2's 300 equally successful localized-repair pairs and approximately 21.5% distance reduction remain frozen.

## Historical v5.1 sources and artifact identity

{{provenance}}

Historical v5 analysis corrections changed a time-unit label to seconds and replaced inappropriate pooled IID uncertainty with seed-cluster intervals. That superseded analysis is retained. **47,523 original raw files were compared; zero changed.** No v5 trial was rerun, excluded or edited for those analysis corrections. V5.1 reuses the corrected inference rules and records new trials after the code fixes.

The October 26 audit reproduced both endpoint defects before fixing them. Its separate v5.1 campaign does not retroactively change v5 observations or executable identity. Comparing all 5,280 corresponding rows, **distance, elapsed time, true/reported completion, false-completion counts and repair counts are unchanged**. Final reported coverage lengths differ in 606 attempts by at most **2.85 × 10⁻¹⁴ m**, a floating-point rounding-scale change. These observations do not make the defects harmless: the independent endpoint counterexamples previously failed and now pass. The original reader is archived; this edition is the current narrative.

{{replication_table}}

{{downloads}}

## Reproduce the historical v5.1 campaign in a new directory

To reproduce historical v5.1 results, build experimental source **`7da3db33e70114503dfc7c9a45671da2b9335541`** with the recorded release environment and protocol amendment. Generate analysis using **`{{analysis_revision}}`**. That later analysis revision changes the report's version-specific validation link and adds the raw-row comparison utility; it does not change inference. The first report and its analysis manifest are preserved in `gates_v5_1/initial_analysis_reference`. Original v5 used experimental source `7896f20`, corrected analysis `62a3363` and publication `456381c`; its outputs remain separate. Historical directories must not be overwritten.

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

## Readiness for defense

The present evidence supports a **candidate applied software/systems contribution**. It does not by itself establish that VECR is sufficiently original for a dissertation defense. The strongest current statement is the implemented edge contract and the measured tradeoffs under its stated model.

V6 completed the missing controlled resume/rewind comparisons, added waiting and target ablations, derived a restricted conditional result and tested API reuse. The strongest simple comparator matches or improves on E. This supports a systems and empirical contribution with an explicit negative finding; it does not establish a superior new optimizer. SITL adds integration evidence, not originality. A broader claim would need additional technical advances and separately preregistered evidence. The supervisor and examining body assess dissertation sufficiency and the publication record.

## Defense demonstration

Start with the contract, exact evidence identity and conditional assumptions. Present the v6 N04 gap-repair comparison, N01 waiting benefit and E−GW cost disadvantage together. Use the historical M03 replay to illustrate actual local recovery, clearly labeled v5.1. Explain failed and dishonest-evidence cases. Finish with client reuse, provenance and the separately qualified SITL results.

The replay uses seed 1 consistently, not a seed selected for a favorable outcome. Numerical claims use the full paired populations, not illustrative individual runs.

# 15 / ArduPilot SITL qualification {#sitl}

## Evidence tiers and actual execution

ArduPilot Copter-4.7.2 runs native autopilot software against its simulated flight dynamics. Commands pass through SwarmKit's production MAVLink backend. Each flight owns its process, parameters and isolated localhost ports; no physical aircraft was connected. The final protocol assigns **24 flights: six conditions × two policies × two noise seeds**. This is a small integration qualification, not a powered effectiveness comparison.

**Native MAVLink** preserves backend-specific accuracy semantics. Those fields do not become deterministic hard bounds; unverified arrival and coverage must remain unfulfilled even when the vehicle reaches B. **Synthetic bounded observations** use an explicitly named test adapter, perturbed simulator position and an ideal flat nadir camera. Raw SIM_STATE is recorded independently. Production MAVLink recovery stays disabled.

{{v6_sitl}}

## Reliability findings and limitations

The retained first pilot exposed port collisions, a Python/C++ monotonic clock-domain mismatch and unsuccessful takeoff handling. Corrected pilots isolate auxiliary ports, check the clock epoch and retain command acknowledgements. Later nominal flights exposed a real backend ordering defect: ArduPilot resets speed when initializing a new GUIDED position submode. SwarmKit now reapplies the requested speed after dispatch; regressions cover acknowledged reposition, unacknowledged setpoint fallback and explicit failure after dispatch. Every unsuccessful pilot remains archived.

The observation envelope assumes SIM_STATE transport age at receipt ≤50 ms and actual horizontal speed ≤10 m/s. Snapshot age at receipt is monitored, but SIM_STATE carries **no acquisition timestamp**, so the upstream-age assumption is not independently proved. Reconstructed arrival and re-entry use sampled truth; they do not establish continuous-flight dwell. The camera is ideal, route-aligned and independent of vehicle attitude. No real image-quality, terrain, gimbal or airworthiness claim follows. Two flights per cell do not support a broad reliability interval.

Future calibrated image evidence and physical aircraft tests remain **TBD — not executed**. Their numbers will be added only after actual measurements. The [SITL protocol](assets/evidence/v6/SITL_PROTOCOL_V6.md) defines the evidence boundary.

# 16 / Conclusion and references {#conclusion}

We developed VECR in SwarmKit and completed its defined agent-backed digital evaluation. The method connects graph-edge execution to navigation and sensing evidence, retains completed intervals and recovers locally from missing work. Its measured benefit is lower recovery travel in the tested equal-success conditions. Its measured weaknesses are evidence-driven nonacceptance and dependence on truthful sensor assertions.

The completed v6 campaign extends the retained v5.1 study with stronger comparators, acquisition-time evidence handling, a conditional sensing-envelope rule and API reuse. Its primary finding is qualified: evidence-aware gap tracking and bounded waiting are useful in the declared conditions, while the conservative E target has no measured completion advantage over GW and often costs more travel. The complete records include negative outcomes, unsuccessful pilots and a corrected protocol deviation.

ArduPilot qualification is reported separately from SimBackend and from real aircraft. Native accuracy remains unqualified for hard-bound acceptance; the positive integration tier uses explicitly synthetic navigation and camera observations. Hardware, calibrated image quality, complex terrain, obstacle/geofence-aware recovery, remote/multi-UAV campaigns and operational fleet reliability remain outside the established evidence. A targeted literature comparison supports the narrow systems contribution; it does not certify global novelty or degree acceptance.

## Primary references and source-to-claim mapping

* **[R1]** Holroyd, Levine, Mészáros, Peres, Propp and Wilson. *Chip-Firing and Rotor-Routing on Directed Graphs*, 2008, revised 2013. [Primary paper](https://arxiv.org/abs/0801.3306). Background on abstract directed-graph processes; no VECR result is attributed to it.
* **[R2]** MAVLink. *Command Protocol*. [Official documentation](https://mavlink.io/en/services/command.html), accessed 26 October 2026. Command acknowledgement/result protocol; SwarmKit supplies the survey contract.
* **[R3]** ArduPilot. *SITL Simulator*. [Official documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html), accessed 26 October 2026. Defines the separate autopilot simulation evidence level; campaign measurements come from the archived traces.
* **[R4]** Zudaire, Gorostiaga, Sánchez, Schneider and Uchitel. ICRA 2021, pp. 6824–6830. [Publisher record, DOI 10.1109/ICRA48506.2021.9561671](https://ieeexplore.ieee.org/document/9561671/). Closely related UAV runtime-monitoring research.
* **[R5]** Arvanitakis, Tzes and Giannousakis. *International Journal of Advanced Robotic Systems*, 2018. [DOI 10.1177/1729881417750785](https://doi.org/10.1177/1729881417750785). Prior bounded-uncertainty geometry.
* **[R6]** ArduPilot. [Mission Rewind on Resume](https://ardupilot.org/copter/docs/common-mission-rewind.html), accessed 26 October 2026. Existing practical resume behavior.
* **[R7]** Mier, Valente and de Bruin. *Fields2Cover*, IEEE RA-L 8(4), 2166–2172, 2023. [Primary paper](https://arxiv.org/abs/2210.07838), DOI 10.1109/LRA.2023.3248439.
* **[R8]** Bouman et al. *Adaptive Coverage Path Planning for Efficient Exploration of Unknown Environments*, 2023. [Primary paper](https://arxiv.org/abs/2302.03164).
* **[S1]** V5.1 experimental source `7da3db3`: `src/core/vecr.cpp`, `src/agent/edge_execution_runtime.cpp`, `proto/swarmkit/v1/swarmkit.proto`, `tests/vecr_v5_trial.cpp`, `tests/vecr_v5_models.h`. Implemented contracts and experiment architecture.
* **[S2]** `corrected_endpoints_v5_1` manifest, raw CSV/traces and SHA256SUMS. Current scientific numbers; original `final_v5` remains historical.
* **[S3]** Analysis source `{{analysis_revision}}`, retaining inference from `62a3363`: `analyze_agent_campaign_v5.py`, `validate_agent_campaign_v5.py`, `test_analysis_v5.py`. Reconstruction and uncertainty calculations.
* **[S4]** `FINAL_VALIDATION_V5_1.md`, both protocol documents, historical `V5_ANALYSIS_NOTES.md` and gate logs. Scope, replication and corrections.
* **[S5]** Endpoint before/after regressions and reader-validation records under this site's `validation/`.

* **[R9]** Nykolaychuk and Ortmeier. *Coverage Path Re-planning for Processing Faults* (2015). [Publisher DOI](https://doi.org/10.1007/978-3-319-22876-1_31). Prior local repair of processing faults; abstract-level comparison, full chapter unavailable.
* **[R10]** Leucker, Sánchez, Scheffel, Schmitz and Schramm. *Runtime verification of real-time event streams under non-synchronized arrival* (2020). [Primary article](https://link.springer.com/article/10.1007/s11219-019-09493-y). Prior asynchronous stream-monitoring correctness; bounds the delayed-evidence novelty claim.

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
