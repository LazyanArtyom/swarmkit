# 01 / Research overview {#overview}

## Researcher and institutional context {#researcher}

{{author_profile}}

## Abstract

We developed and evaluated **Verified Edge Coverage and Recovery (VECR)** as a canonical, algorithm-independent onboard execution runtime in the SwarmKit UAV SDK. VECR assigns a physical obligation to a directed graph edge, continuously checks bounded navigation and qualified sensor evidence, preserves verified progress across deviations, detects unverified intervals, and executes bounded local recovery via earliest-gap targeting with bounded evidence waiting. Command acceptance, physical arrival, route traversal, and survey fulfillment have strictly separate semantics. High-level swarm coordination and graph planning remain external to the agent.

The primary digital campaign executed **1,320 attempts: 22 conditions N00–N21 × 30 paired seeds (70001–70030) × VECR and the B2 full-restart research baseline** through the production SDK → local mTLS AgentService → agent runtime → SimBackend path. A separate 100-attempt client-reuse campaign found identical recorded metrics between direct `Client` and routed `SwarmClient` architectures. An independent validator verified 108,790 delivered captures and 720 physical re-entry dwells. Software-in-the-loop (SITL) qualification tested 12 flights using native ArduPilot Copter 4.7.2 processes over production MAVLink interfaces.

On midflight gap conditions (N03, N04, N06, N07, N08, N09, N17, N18), VECR reduced flight distance by **11.6% to 31.3%** compared to the coverage-aware full-restart baseline B2, saving up to 29.92 m of flight travel per attempt with 30/30 joint completions. Under delayed evidence (N01), bounded waiting prevented spurious re-entry maneuvers. Across **1,200 honest-sensor attempts**, zero false fulfillments occurred. Deliberate dishonest-sensor controls (N19, N20; 120 trials) yielded 100% false fulfillments, cleanly exposing the sensor trust boundary.

## Research scope and evidence levels

| Evidence level | Status | What it establishes |
|---|---|---|
| Digital simulation (N00–N21) | Completed primary campaign | 1,320 attempts; canonical VECR earliest-gap recovery vs B2 full-restart baseline across 22 conditions |
| Client reuse campaign | Completed, 100 trials | 50 matched direct vs routed SDK client pairs; zero metric differences |
| ArduPilot SITL qualification | Separately qualified, 12 flights | Native Copter 4.7.2 flight dynamics; fail-closed behavior on raw MAVLink and MAVLink recovery under synthetic bounds |
| Real aircraft and hardware imagery | Outside simulator claim | No real physical-flight or image-photogrammetry claim is asserted |

This reader documents a completed onboard method, its conditional argument, implementation, and measured digital evidence. It also explains the dissertation's graph-theoretic and multi-user mission-engineering lineage. Those broader systems are identified by their own evidence status; a deployed cloud fleet service, multi-UAV radio capacity, field flight, and real image-quality validation are outside this campaign.

## Research questions and answers

1. **What does a graph transition mean physically?** The planner's selected edge becomes a versioned route and observation obligation. The agent returns a verified outcome only after qualified physical evidence satisfies the selected predicate.
2. **Can already observed work survive a deviation?** Yes, accepted intervals are retained as a union. Recovery addresses missing intervals, while the B2 comparator restarts from the edge origin. This is an implemented design and a measured difference in the specified gap conditions.
3. **When is a success report sound?** Only under declared sensing geometry, honest acquisition metadata, hard error bounds, correct identity, and the stated interval model. Dishonest controls show the limit directly.
4. **Can external clients reuse the runtime?** The two implemented SDK paths produced identical metrics in 50 matched pairs. This is evidence of API-path reuse, not a test of multiple graph algorithms or fleet-scale networking.

# 02 / Research lineage and graph foundations {#foundations}

## The graph-level research programme

The earlier work behind this dissertation studies self-organizing UAV swarms as directed graph processes. A vertex can be a waypoint or task location; a directed edge is a permissible transition. An Eulerian circuit is a property of an abstract walk on a suitable directed graph. It says nothing by itself about a camera exposure, a missed strip, GNSS uncertainty, or safe flight. This is the precise gap that motivated an onboard physical-execution contract.

In rotor-router dynamics, each vertex maintains a cyclic order of outgoing edges. A rotor update selects the next graph transition. Under the hypotheses of the established unicycle and Eulerian results, a deterministic rotor walk can traverse the graph's directed edges. The group's published [Euler-tour and unicycle study](https://doi.org/10.1088/1742-5468/2014/06/P06003) and [loop-reversibility study](https://doi.org/10.1088/1751-8113/48/28/285203) belong to that mathematical lineage. They are prior work, not VECR inventions. The group's [cloud-based UAV swarm model](https://doi.org/10.1139/dsa-2023-0039) addresses multi-agent graph movement, different speeds, arrival information, and reassignment after a crash.

The distinction matters in a real survey. A logical transition $e=(u,v)$ can be selected correctly, yet the vehicle can miss imagery between the physical endpoints. A repair flight may traverse a section twice even when the planner's abstract walk visits the directed edge once. Rotor-router's graph guarantee remains a graph guarantee; VECR evaluates the physical obligation attached to each attempted edge.

## Gossip and dissemination are separate concerns

Earlier [Knödel-graph gossip research](https://doi.org/10.1109/CSITechnol.2017.8312126) concerns information dissemination topology. A gossip algorithm can spread a verified edge result or missing-work report to peers. It does not determine whether the camera actually observed a strip. SwarmKit supplies generic message and peer APIs; it does not implement a rotor-router or gossip policy. A network partition can prevent remote recipients from learning an onboard result even while the local agent continues its verification and recovery.

| Layer | Question it answers | Evidence in this repository |
|---|---|---|
| External graph controller | Which directed edge should a vehicle attempt next? | Interface boundary; no rotor-router implementation in SwarmKit |
| VECR agent | Was that edge's physical traversal or survey obligation verified, and what remains? | Implemented runtime, digital campaign, qualified SITL |
| Peer communication | How can application messages and artifacts move between agents? | Implemented API surface; fleet-scale delivery not measured here |
| Mission workspace | How do people author, approve and inspect graph missions? | Earlier platform publications and prior reader context; not part of VECR trials |

## Earlier platform publications

The [multi-user UAV swarm simulation platform](https://doi.org/10.1134/S0361768823090086) and [mission-preparation study](https://doi.org/10.1134/S0361768824700397) establish a platform research line. The [2025 multi-user software-system paper](https://doi.org/10.1109/ICECET63943.2025.11472550) is another part of that lineage. The previously supplied dissertation draft also lists a 2026 simulation-driven monitoring paper and a SwarmKit architecture paper; their bibliographic details are retained as author-supplied records pending formal publisher verification. Their results are not counted in the current VECR digital campaign.

This dissertation's specific addition is the *physical edge acceptance layer*: the planner may change, and the communication algorithm may change, while the agent applies the same exact attempt identity, bounded evidence, gap accounting, and local recovery. The two SDK-path experiment tests reuse of this layer; it does not experimentally demonstrate a complete rotor-router deployment.

# 03 / The problem and core contribution {#innovation}

## Four facts that must remain separate

A planner can dispatch a straight edge A→B, a vehicle backend can accept the command, and a drone can eventually reach B while leaving an intermediate survey strip completely unobserved. Conversely, a wide-angle camera can observe the strip during an off-corridor excursion. Navigation deviation is therefore neither necessary nor sufficient evidence of a survey gap. Sensor acquisition can also fail on a perfectly flown corridor.

{{diagram:contract}}

**The core contribution is an onboard edge-obligation execution contract:** the agent binds evidence to the exact mission, revision, edge, and attempt identity; maintains the conservative union of qualified coverage; retains disjoint completed intervals; identifies unverified intervals; and executes bounded local recovery while respecting authority, evidence validity, and physical safety limits. External applications consume factual outcomes without embedding planners into SwarmKit.

## Refutation of subject-matter misconceptions

An early external review incorrectly categorized this work under continuum manipulators, visual-elastic observers, Cosserat kinostatics, and soft robotics. This is factually erroneous:
1. **Kinematics:** The measured work uses a multirotor UAV and a straight local 3D route. No continuum manipulators, tendon actuators, elastic rods, or Cosserat models exist in this codebase.
2. **State Estimation:** SwarmKit normalizes backend telemetry and requires declared error bounds. The agent does not itself implement an inertial-GNSS fusion estimator or compliance-based observer.
3. **Application Domain:** The research concerns airborne edge coverage, conservative footprint accounting, and local spatial recovery for autonomous UAV missions.

## Why the contribution is useful

1. **Coverage survives flight deviations.** Disjoint intervals observed before or after an excursion remain credited.
2. **Targeted local repair.** VECR targets the earliest missing interval with bounded dwell, rather than restarting the entire edge at A.
3. **Arrival cannot substitute for sensing.** A survey obligation requires verified strip coverage; endpoint arrival alone is insufficient.
4. **Explicit uncertainty bounds.** Sensor data lacking hard uncertainty bounds fail closed; they cannot silently default to healthy status.
5. **Onboard execution.** Detection, verification, and recovery execute entirely inside the onboard agent; no offboard server is in the inner control loop.

# 04 / System architecture {#architecture}

## Execution hierarchy

{{diagram:architecture}}

SwarmKit is implemented in C++23 with protobuf-defined RPC interfaces. The production agent owns the edge supervisor, sensor qualification, interval arithmetic, recovery dispatch, and persistent attempt journal. The benchmark and evaluation harness replaces the physical autopilot backend with SimBackend; it does not replace or mock the agent's verification or recovery state machine.

The sensor estimator receives noisy telemetry and fixed camera calibration. It has no access to simulator ground truth. A separate Python validator reads simulator ground truth (actual 20 ms poses and rectangular footprints) to independently reconstruct coverage using half-space geometry without calling agent libraries.

## External planner boundary

High-level multi-robot coordination algorithms (e.g. rotor-router, gossip protocols, vehicle routing) belong strictly to external application layers. The external application decides graph structure and edge assignments; SwarmKit executes edge obligations, qualifies evidence, and reports factual terminal outcomes (`FULFILLED`, `INTERRUPTED`, `UNKNOWN`).

# 05 / SwarmKit as a reusable execution platform {#swarmkit}

## Agent, SDK, backend and authority

SwarmKit is a C++23 SDK with a per-vehicle agent. An external client requests control through the SDK; the agent arbitrates command authority, checks health and capabilities, normalizes telemetry, issues a command through a replaceable backend, and publishes reports. The existing generic `SendCommand`, `StreamTelemetry`, `SetActiveGoal`, `SubscribeReports`, and authority RPCs remain alongside `ExecuteEdge`, `GetEdgeStatus`, `SubmitEdgeCapture`, and `CancelEdge`. The edge API adds a physical obligation without replacing general-purpose control.

The simulated backend and MAVLink backend expose different vehicle environments to the same agent. Digital trials use the production agent with `SimBackend`; the separate SITL qualification uses native ArduPilot Copter processes and MAVLink. A command reply indicates dispatch or acceptance only. Subsequent telemetry, sensor evidence and physical dwell determine the edge outcome. A generic `GOAL_REACHED` report is not interchangeable with `FULFILLED`.

An edge attempt is bound to mission, revision, edge, attempt and drone identity. This prevents a delayed capture or stale report from silently changing the result of a new mission revision. The contract reports missing intervals and terminal reasons to the caller; the caller decides the next graph action. The agent retains only the local recovery authority described by the edge request and safety limits.

## Communication and artifact plane

The protobuf `DataService` exposes `PublishMessage`, `SendMessageToDrone`, `SubscribeMessages`, peer discovery/configuration, and artifact upload/download/transfer. Applications can carry graph state, gossip messages, edge reports, or larger imagery and logs through these interfaces. The data service is separate from vehicle command-authority locks. The API supports both sending and receiving; no throughput, reliable-delivery, or partition-healing result follows from the existence of an RPC alone.

For a distributed mission, five events must be distinguished: local evidence acceptance; local edge outcome; message creation; remote receipt; and swarm-wide knowledge. VECR's simulator campaign checks the first two. Transport and coordination guarantees require a separate communications experiment and a specified protocol. In a disconnected interval the onboard agent can continue its local task, while remote SwarmOps views may be stale.

## A planner integration contract

An external controller chooses a directed edge and submits a versioned obligation. It waits for `FULFILLED`, `INTERRUPTED`, or `UNKNOWN` with the corresponding coverage and reason information. On `FULFILLED` it can advance its own graph state; on incomplete outcomes it can retry, reassign, or stop according to its own policy. This sequencing is important for rotor-router: graph transitions should not be treated as physically completed on `Command ACK`. SwarmKit does not mutate a rotor state or run a gossip schedule.

# 06 / SwarmOps and the human mission workflow {#swarmops}

## Earlier work and its evidence status

The preceding dissertation reader described a Qt/C++ SwarmOps desktop snapshot with map selection, graph editing, simulation, mission views, workspace services, and gateway interfaces. That snapshot is historical context supplied with the earlier reader; it is not the executable artifact of the present SwarmKit VECR campaign. Published multi-user platform and mission-preparation papers establish the research lineage, but the current 1,320-attempt dataset does not measure SwarmOps UI performance, collaborative editing, or cloud operations.

SwarmOps addresses the human-facing task: select a geographic area, author a directed graph, attach physical obligations, select a planner, approve a mission revision, and monitor execution. SwarmKit addresses the vehicle-facing task: arbitrate commands, execute the assigned edge, qualify observations, repair local gaps where permitted, and report the result. A live map may display planned route, observed track, verified surveyed intervals, and unresolved gaps as separate layers. The map is an inspection surface, not the source of the agent's truth decision.

## Mission lifecycle and versioning

An operator workflow runs from workspace access and map-based mission editing to graph validation, preflight simulation, approved dispatch, onboard execution, event inspection and artifact review. The planned graph and imagery obligation must refer to an immutable mission revision. A later edit to an edge geometry cannot silently reinterpret captures accepted for the earlier revision. Command authority on a vehicle also remains a separate concern from a platform user's editing role.

Geographical display coordinates and execution coordinates must be linked explicitly. A map screenshot is not a metric survey model. The current experiments use a local straight-edge frame and rectangular ground-footprint oracle; a field deployment would need checked georeferencing, terrain and camera calibration, and a defined transformation from map geometry to the agent's local frame.

## Intended cloud architecture, outside the measured result

The wider programme describes a multi-user mission workspace, authorization, versioned mission store, telemetry gateway, event log, artifact store, and post-mission analysis. These are architectural targets and earlier research context. The current VECR result does not claim a deployed multi-tenant SaaS service or bounded latency under hundreds of concurrent drones. Even if a cloud service is unavailable, the experiment's local verification/recovery loop runs in the agent. Remote monitoring would require later synchronization and conflict handling to present an accurate view.

# 07 / Model, evidence and fulfillment {#model}

## Directed edge and identity

The agent accepts a straight two-point route A→B of length $L$ in a local metric Cartesian frame. Every obligation carries a unique identity:

{{equation:identity}}

The obligation specifies arrival, bounded corridor traversal, survey, or traversal-and-survey. Primary evaluations focus on **survey**: requiring verified full-strip sensor coverage and stopped 3D arrival at endpoint B.

## Conservative sensing intervals

For measured altitude $\hat{h}$, vertical bound $\varepsilon_h$, horizontal bound $\varepsilon_p$, and calibrated along/cross fields of view $\theta_a, \theta_c$, the adapter contracts the footprint using $\hat{h}^- = \hat{h} - \varepsilon_h$:

{{equation:footprint}}

A capture is accepted only when camera status is valid, timestamp is fresh ($\le 3\text{ s}$), horizontal cross-track error $d$ plus strip half-width $w$ fits within the contracted cross-track footprint $c^-$, and $a^- > 0$. The resulting interval is clipped to $[0, L]$.

## Coverage union and missing intervals

{{equation:coverage}}

The agent computes the set union $V = \bigcup I_j^-$ and its complement $M = [0, L] \setminus V$. Total unverified length is the sum of interval lengths in $M$. Interval operations use a $10^{-6}\text{ m}$ geometric merging tolerance.

## Stopped arrival and survey fulfillment

{{equation:arrival}}

Arrival at B requires 3D distance within radius $r_B = 1.0\text{ m}$ accounting for position bounds, speed within $v_B = 0.5\text{ m/s}$ accounting for speed bounds, and qualifying samples separated by at least 200 ms. This is a **sampled dwell** under the stated telemetry assumptions, not a continuously observed trajectory. Survey fulfillment also requires remaining missing length $\le$ allowance ($0.0\text{ m}$ in primary campaigns).

## Bounded corridor traversal

{{equation:traversal}}

Corridor conformance verifies that distance to route plus uncertainty remains within corridor half-width. Guaranteed forward progress credits $[s_0 + \varepsilon_0, s_1 - \varepsilon_1]$ only when non-empty.

## Canonical recovery target

{{equation:recovery}}

When unverified gaps remain upon reaching B or following a sustained departure, VECR computes the re-entry target $q = \min(p, g)$ on departure (capped at projected progress $p$) and $q = g$ at endpoint B, targeting the start of the earliest unverified interval $g$. A bounded wait of up to 3.0 s at B allows in-flight metadata to arrive before triggering re-entry.

# 08 / Conditional correctness {#proofs}

## Assumptions

The formal guarantees hold under the following explicit conditions:
1. Exact identity matching across all telemetry and sensor captures.
2. Calibrated camera optics and truthful sensor metadata assertions.
3. Valid deterministic upper bounds on horizontal, vertical, and velocity estimation errors.
4. Correct interval set-union bookkeeping.

## Proposition 1 — Conditional survey soundness

{{equation:soundness}}

**Proof.** Set inclusion is preserved under arbitrary unions. If each accepted contracted footprint $I_j^-$ is contained within the true acquired ground footprint $F_j$, then $\bigcup I_j^- \subseteq \bigcup F_j$. If the accepted union covers the required strip $R_e$, the true ground footprints necessarily cover $R_e$. Stopped arrival is checked independently.

## Proposition 2 — Gaps cannot be overwritten by progress

The supervisor maintains sorted, non-overlapping intervals and recomputes missing work across the entire edge $[0, L]$. Captures acquired downstream cannot remove an unverified upstream gap. Valid earlier work is never discarded following a deviation.

## Proposition 3 — Command ACK cannot complete recovery

Dispatching a re-entry command sets phase `RECOVERING`. The runtime requires qualified stationary re-entry dwell at the target before transitioning back to `MOVING`. A backend command acknowledgement alone cannot complete recovery.

# 09 / Implementation and reliability {#implementation}

## Canonical public contract

Implemented in `proto/swarmkit/v1/swarmkit.proto` and C++ headers:
- `ExecuteEdge`: Dispatches an edge obligation with route, corridor, strip, and repair configuration.
- `GetEdgeStatus`: Returns current execution phase, missing intervals, and physical metrics.
- `SubmitEdgeCapture`: Submits camera acquisition metadata for conservative qualification.
- `CancelEdge`: Requests graceful cancellation of the active attempt.

## State semantics

{{diagram:states}}

The six canonical execution phases are:
- `MOVING`: Traversing the route while accumulating qualified sensor coverage.
- `DEVIATING`: Three consecutive out-of-corridor samples detected; monitoring excursion.
- `RECOVERING`: Re-entry command active; awaiting physical arrival and dwell at target.
- `FULFILLED`: Terminal success; all obligation predicates verified.
- `INTERRUPTED`: Terminal failure; abort triggered by safety, refusal, or deadline.
- `UNKNOWN`: Evidence missing, ambiguous, or unverifiable.

## Reliability and safety limits

Recovery is enabled only under valid vehicle health, sufficient battery margin, and verified navigation bounds. The implemented supervisor interrupts or withholds fulfillment when its configured refusal, motor-health, battery, evidence, budget, or deadline conditions are met. The attempt journal is fsynced before command dispatch. That design supports restart consistency under its stated storage assumptions; the simulator campaign is not a proof against every possible crash or storage fault.

# 10 / Experimental protocol {#protocol}

## Fair paired evaluation: VECR vs B2

{{diagram:comparison}}

VECR is evaluated against B2, a coverage-aware full-restart baseline. Both methods share:
- The exact same conservative sensor qualification and interval union engine.
- Identical vehicle kinematics, SimBackend dynamics, and control loops.
- Identical safety triggers, timeouts, recovery budgets (max 2 repairs), and random seeds.
- Identical preservation of earlier accepted sensor evidence.

The only difference is the recovery target: VECR targets the earliest unverified gap $g$ with bounded wait, while B2 returns to route origin A.

## Independent evidence and truth paths

{{diagram:evidence}}

Simulator ground truth (20 ms private state) is strictly isolated from the agent's sensor estimator. Telemetry noise includes pseudorandom walk and bounded biases. Python reconstruction runs independently of C++ libraries.

## 22-Condition experimental matrix (1,320 attempts)

The campaign evaluates 22 conditions N00–N21 across 30 common seeds (70001–70030), generating 1,320 primary attempts (660 paired comparisons):
- **Nominal & Evidence Timing:** N00 (nominal), N01 (2.5 s delay), N02 (reordered captures).
- **Geometric Gaps:** N03 (single metadata gap), N04 (multiple capture gaps).
- **Localization Noise:** N05 (changing uncertainty 0.3–0.9 m), N06 (uncertainty spike during gap).
- **Physical Deviations:** N07 (sustained wind departure), N08 (departure with wide FOV), N09 (departure with narrow FOV).
- **Fail-Closed Faults:** N10 (sensor loss), N11 (command rejected), N12 (motor failure), N13 (low battery), N14 (zero repair budget), N15 (stale captures >3 s), N16 (sparse captures).
- **Route-length sensitivity:** N17 (20 m route), N18 (80 m route).
- **Dishonest Controls:** N19 (fraudulent acquisition claim), N20 (concealed 45° camera yaw offset).
- **Resource Limits:** N21 (30 s short deadline).

Pointwise 95% confidence intervals are computed using 10,000 paired bootstrap resamples.

# 11 / Measured results {#results}

## Preregistered completion matrix

{{figure:01_completion_matrix}}

Across the tested honest conditions, VECR and B2 had the same recorded completion counts. On the jointly successful gap conditions, VECR traveled less. The finite matrix does not establish universal recovery parity or a performance advantage under every possible geometry and fault.

## Paired distance reduction on gap conditions

{{matched_table}}

In every condition with unobserved gaps, VECR achieves statistically significant travel distance reductions compared to B2:
- **N03 (single gap):** $-29.92\text{ m}$ ($-24.9\%$), reducing distance from 120.20 m to 90.28 m.
- **N04 (multiple gaps):** $-13.96\text{ m}$ ($-11.6\%$), reducing distance from 120.20 m to 106.24 m.
- **N07 (sustained departure):** $-24.93\text{ m}$ ($-31.3\%$), reducing distance from 79.58 m to 54.64 m.
- **N08 & N09 (excursions):** $-25.22\text{ m}$ and $-23.48\text{ m}$ travel reductions.
- **N17 & N18 (route scaling):** $-5.94\text{ m}$ on 20 m routes; $-29.92\text{ m}$ on 80 m routes.

{{figure:02_paired_distance}}

## Interactive condition explorer

{{scenario_explorer}}

# 12 / Recorded physical replay {#replay}

{{trace_explorer}}

The interactive replay tool renders recorded physical trajectories at 100 ms resolution alongside accepted and actual full-strip coverage. Key cases include:
- **N00:** Nominal flight with coverage accumulation.
- **N03:** Intermediate metadata gap triggering localized re-entry.
- **N04:** Multiple disjoint gaps resolved sequentially.
- **N07:** Wind gust causing physical departure followed by bounded re-entry.
- **N11:** Command rejection causing immediate fail-closed termination.
- **N19:** Dishonest camera metadata exposing the sensor trust boundary.

# 13 / Evidence availability and limits {#availability}

## Honest population vs dishonest controls

{{confusion_table}}

Across 1,200 honest-sensor attempts (20 conditions), **zero false fulfillments occurred**. Both VECR and B2 achieved 100% precision on honest trials.

In contrast, the 120 dishonest-sensor controls (N19, N20) resulted in **100% false fulfillments**. Because the camera metadata asserted valid acquisitions, the agent verified coverage despite actual physical gaps. This result empirically demonstrates that cryptographic authentication verifies sender identity, not physical ground truth.

## Fail-closed safety verification

In the tested seeds of persistent camera loss (N10), command refusal (N11), motor propulsion loss (N12), low battery (N13), zero budget (N14), expired evidence (N15), sparse captures (N16), and short deadlines (N21), VECR made no false completion report. The terminal phase and reason for each run are available in `per_attempt.csv`; this empirical observation is not a proof covering all failure conditions.

# 14 / Sensitivity and stress analysis {#sensitivity}

## Route-length sensitivity

Evaluations at 20 m (N17), 40 m, and 80 m (N18) show the observed recovery-distance difference at those three lengths. On the 80 m condition, VECR saved 29.92 m of flight per attempt relative to B2. Three tested lengths cannot establish scale invariance.

## Localization uncertainty escalation

In conditions N05 and N06, horizontal position uncertainty escalated dynamically from 0.3 m to 0.9 m. The conservative footprint contraction equation contracted usable footprint width accordingly, ensuring no uncontained footprints were accepted into the coverage union.

# 15 / SDK client reuse evaluation {#reuse}

{{reuse_table}}

To demonstrate API reusability, a separate campaign compared direct SDK `Client` execution against routed `SwarmClient` execution across 5 conditions × 10 paired seeds (100 attempts total).

The evaluation verified **zero metric differences**:
- Identical terminal phases (30 `FULFILLED`, 20 `INTERRUPTED`).
- Identical physical flight distance: mean $64.97\text{ m}$, $\Delta = 0.000\text{ m}$ [$0.000, 0.000$].
- Identical physical re-entry dwells: exactly 20 dwells observed in each configuration.

# 16 / ArduPilot SITL qualification {#sitl}

## Autopilot-in-the-loop validation

{{figure:03_sitl_traces}}

The software-in-the-loop (SITL) tier connects SwarmKit's production MAVLink backend to native ArduPilot Copter 4.7.2 processes with simulated vehicle dynamics and the autopilot's navigation loops.

{{sitl_table}}

Key findings from 12 qualification flights:
1. **Raw MAVLink nonacceptance:** Under uncalibrated raw MAVLink telemetry lacking explicit hard bounds, VECR correctly refused fulfillment (0/2 reported), failing closed.
2. **Synthetic bounded nominal & delays:** Under qualified bounds, flights achieved 2/2 completions with stopped dwell at B.
3. **Multi-gap recovery:** In the presence of unobserved gaps, VECR commanded physical re-entry dwells and completed the missing survey work.
4. **Command rejection & LAND overrides:** The recorded command-rejection and `LAND` override runs ended without a false fulfillment report. Their event timings and terminal reasons are in the SITL traces.

# 17 / Worked mission cases and claim boundaries {#cases}

## Case A — endpoint reached, middle unobserved

The drone departs A, reaches B and stops, but captures from a middle section are missing. A waypoint-only controller could call this flight complete. VECR's survey predicate remains false because the union of accepted intervals leaves a gap. It waits for bounded delayed metadata and, if none closes the gap, targets the earliest missing interval. N03 and N04 are measured instances of this mechanism; the replay exposes the route, target events, coverage and terminal outcome.

## Case B — route deviation with and without a survey gap

A crosswind excursion creates a geometric departure. If the calibrated footprint still contains the required strip and all data are valid, an off-corridor path need not imply a survey gap. Conversely, a narrow footprint may leave one. The supervisor evaluates traversal and survey predicates independently; a request requiring corridor traversal may still fail even if surveying happened. N07–N09 exercise the relationship between departure and sensing width in the digital model.

## Case C — safe flight, failed camera

A vehicle can remain near the route and arrive safely while the camera delivers no valid acquisition. The edge remains incomplete. N10, N15 and N16 test permanent loss, expired evidence and sparse capture geometry. Those cases illustrate why vehicle position alone cannot certify survey fulfillment.

## Case D — a coordinator and a disconnected server

An external graph algorithm dispatches a versioned edge. The onboard agent verifies and, where permitted, repairs it locally. If the server link drops, the agent's physical loop remains local; the coordinator may later receive a delayed result and must reconcile it against the same mission and attempt identity. This is an architectural example, **not** an executed multi-drone or network-partition experiment. No rotor-router or gossip policy was implemented for this campaign.

## Limits of the present evidence

The digital oracle uses a local straight edge, simplified kinematics, a flat rectangular full-strip footprint and declared hard sensor bounds. The 12 SITL flights test the production MAVLink path and selected physical behaviors, with synthetic bounds for accepted survey claims. They do not validate real camera calibration, image usability, changing terrain, lighting, occlusion, GNSS multipath, multi-agent contention, or field airworthiness. The 120 deliberately dishonest controls show a particularly sharp limitation: validly authenticated but false acquisition metadata can still yield a false fulfillment. Conditional soundness therefore cannot be advertised as unconditional truth detection.

Future field work needs calibrated images with a separate ground-truth survey, explicit terrain/footprint modelling, documented uncertainty semantics, more environmental disturbances, network and peer tests, and safety review for the intended aircraft. Those are uncompleted scientific extensions; they are not hidden inside the current numerical result.

# 18 / Literature and originality {#literature}

## Context and prior work boundaries

A targeted literature review clarifies VECR's boundaries relative to prior research:

| Prior system / publication | Prior capability | VECR distinction |
|---|---|---|
| [Zudaire et al., ICRA 2021](https://software.imdea.org/~cesar/papers/zudaire21assumption.pdf) | UAV assumption monitoring for survey coverage | Implemented onboard attempt identity and conservative interval algebra |
| [Arvanitakis et al., 2018](https://pure.coventry.ac.uk/ws/files/13602376/1729881417750785.pdf) | Bounded-uncertainty footprint sets | Runtime integration with stopped arrival and physical re-entry dwells |
| [ArduPilot Mission Rewind](https://ardupilot.org/copter/docs/common-mission-rewind.html) | Fixed-distance rewind on mission resume | Evidence-selected missing intervals instead of fixed distance heuristics |
| [Fields2Cover, 2023](https://arxiv.org/abs/2210.07838) | Coverage path planning library | Execution-layer runtime contract rather than offline geometric planning |
| [Nykolaychuk & Ortmeier, 2015](https://doi.org/10.1007/978-3-319-22876-1_31) | Replanning for processing faults | Localized onboard re-entry for straight directed edge segments |
| [Leucker et al., 2020](https://link.springer.com/article/10.1007/s11219-019-09493-y) | Out-of-order stream verification | Bounded endpoint waiting for asynchronous camera metadata arrival |

VECR does not claim worldwide priority for the concept of returning behind an interruption or set-union coverage. The defensible contribution is the concrete, implemented onboard execution, verification, and recovery contract for UAV edge obligations.

## Dissertation contribution in one statement

VECR is a reusable onboard method that turns a directed edge's physical obligation into an attempt-bound evidence state, accepts only conservatively qualified traversal or sensor intervals, preserves valid partial work, and executes bounded earliest-gap recovery before reporting a terminal outcome. Its conditional soundness follows from containment of every accepted footprint in the true acquired region; the digital and SITL campaigns test the implementation under stated sensor and simulator assumptions. This is the claim defended here. It does not require the recovery target to be a universally optimal path or the underlying rotor-router and gossip algorithms to be new.

## Bibliographic lineage retained from the earlier dissertation

The preceding reader cited [Poghosyan and Priezzhev (2014)](https://doi.org/10.1088/1742-5468/2014/06/P06003), [Papoyan and coauthors (2015)](https://doi.org/10.1088/1751-8113/48/28/285203), [Poghosyan and coauthors (2024)](https://doi.org/10.1139/dsa-2023-0039), the [2023 platform paper](https://doi.org/10.1134/S0361768823090086), the [2024 mission-preparation paper](https://doi.org/10.1134/S0361768824700397), and the [2025 multi-user software-system paper](https://doi.org/10.1109/ICECET63943.2025.11472550). These publications support the broader programme; the current numerical results arise from the versioned SwarmKit artifacts below. The prior reader also listed 2026 ICECET and CODASSCA items from an author-supplied publication record; their bibliographic status should be independently checked before a final university submission.

# 19 / Defense: questions and answers {#defense}

{{defense_questions}}

# 20 / Reproducibility and artifact inventory {#reproducibility}

## Verification gates

{{gates}}

## Provenance and checksums

{{provenance}}

## Downloadable evidence artifacts

{{downloads}}

## Campaign reproduction commands

To reproduce the primary digital campaign and client-reuse evaluation from source, choose **new empty output directories**. The command lines below use the current script names and modes. The separately recorded SITL qualification requires an installed ArduPilot SITL checkout and its own protocol configuration; see `docs/vecr/SITL_PROTOCOL.md` and `scripts/vecr/run_sitl.py --help` before running it.

```bash
# Build C++23 release targets
cmake --preset mac-release
cmake --build --preset mac-release --target swarmkit_tests swarmkit_vecr_trial swarmkit_vecr_baseline

# Execute CTest verification suite
ctest --preset mac-release

# Execute primary digital campaign (1,320 attempts)
.venv/bin/python scripts/vecr/run_digital.py --mode final --output /tmp/vecr-primary-reproduction --workers 4

# Validate primary digital campaign
.venv/bin/python scripts/vecr/validate_digital.py --output /tmp/vecr-primary-reproduction
.venv/bin/python scripts/vecr/analyze_digital.py --output /tmp/vecr-primary-reproduction

# Run client reuse campaign (100 attempts)
.venv/bin/python scripts/vecr/run_digital.py --mode reuse --output /tmp/vecr-reuse-reproduction --workers 4
.venv/bin/python scripts/vecr/validate_digital.py --output /tmp/vecr-reuse-reproduction

# Build offline dissertation reader website
.venv/bin/python VECR_Dissertation_Interactive_Website/tools/build_site.py
.venv/bin/python VECR_Dissertation_Interactive_Website/tools/validate_site.py --browser
```

<p class="end-mark">Completed digital simulation study · reproducible, source-linked and explicit about its limits.</p>
