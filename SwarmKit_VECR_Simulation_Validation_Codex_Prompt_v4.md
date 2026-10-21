# CODEX MASTER TASK — Finish SwarmKit VECR Simulator Validation (No SITL Yet)

You are working **inside my existing SwarmKit repository**. Implement, test, execute, and document the remaining work for a scientifically credible **simulator-only experimental validation** of **Verified Edge Coverage and Recovery (VECR)**. Work autonomously through the tasks; do not stop after drafting a plan or after running only smoke tests. Use the repository as the source of truth. If a detail differs from this prompt, inspect the current implementation, preserve the scientific intent, document the discrepancy, and use the simplest correct design.

## 0. Research objective and firm scope

The research question is **not** “can a drone reach waypoint B?” It is: **When a graph-based mission orders traversal of directed edge A→B, can the onboard system correctly distinguish arrival at B from valid traversal and verified geographical surveying; preserve verified partial coverage; identify unobserved sections; and safely recover them, or truthfully report incomplete/unknown outcomes?**

VECR is a reusable, **algorithm-independent** onboard execution method. An external graph/rotor-router coordinator can request an edge; SwarmKit verifies and reports its physical obligation. It does **not** implement rotor-router, Abelian theorems, Eulerian planning, gossip dissemination, or swarm reallocation. Do not implement those algorithms. For the experiment use a small, deterministic **graph-edge request generator**, not a fake full rotor-router implementation. All deviation detection and recovery stay **inside the SwarmKit agent**. No cloud/server-side correction loops.

**This task is simulator validation only. DO NOT run or claim ArduPilot SITL, HIL, real vehicle, or real camera experiments.** Do not silently enable live MAVLink recovery. SITL is the next project phase. Retain existing flight-controller and communication interfaces but focus engineering time on correct onboard simulated execution and trustworthy statistics.

The requested final outcome is **a tested implementation plus actual reproducible raw simulation measurements, summary tables, figures, and evidence-bounded dissertation prose**. Do not invent results or numbers to match expectations. Report negative, inconclusive, and unsupported cases openly.

## 1. Existing repository: audit first, do not reimplement what works

Check the current git HEAD and worktree. Read `AGENTS.md`, `docs/vecr/METHOD_AND_CORRECTNESS.md`, `docs/vecr/IMPLEMENTATION_PROGRESS.md`, `docs/vecr/CLEANUP_INVENTORY.md` and the newest tests and protocols. The previous audited baseline was `faea016` on `main`; the checkout may have advanced.

Relevant existing files from the previous ZIP:

- `include/edge/vecr.h`, `src/core/vecr.cpp`: VECR geometry, interval accounting, arrival/traversal/survey predicates.
- `src/agent/edge_execution_runtime.{h,cpp}`: active onboard VECR execution, telemetry, local recovery dispatch, reporting.
- `src/agent/edge_attempt_journal.{h,cpp}`: restart-safe attempt journal.
- `proto/v1/swarmkit.proto`: public edge API, typed reports, capture submission and communication RPCs.
- `tests/test_vecr.cpp`, `tests/test_edge_attempt_journal.cpp`: existing regressions.
- `scripts/vecr/run_agent_pilot_v3.py`, `results/vecr/agent_pilot_v3/`: two **agent-backed** integration examples, not a statistical comparison.
- `apps/vecr_experiment.cpp`, `scripts/vecr/run_core.py`, `results/vecr/raw/core_v2_100_seeds.csv`: old **geometric core-only** experiments. Do not cite their 19.5% distance saving as agent-backed or flight validation.
- `docs/vecr/archive/`: frozen older research. **Preserve** all verified archives and frozen v1/v2/v3 results.

The existing v3 pilot reached `FULFILLED` after a modeled camera-gap recovery and `INTERRUPTED` during a modeled motor failure. It exercised SDK → AgentService → EdgeExecutionRuntime → SimBackend. It did **not** compare VECR to B2 through the agent and involved only two attempts. Build on this actual path, not just the core/geometric experiment.

**Before modifications:** record HEAD, branch, tracked/untracked state, compiler/OS/dependency versions, CMake preset, current test pass/fail baseline, and archive hash status in a new implementation note. Do not modify old experiments or their hashes. No blanket deletes. This is a pre-release product: remove demonstrated redundancy only when safe, update all affected API layers, and preserve relevant product/safety tests.

## 2. P0 — Fix execution reliability and semantics before producing new measurements

Inspect whether the following weaknesses still exist. If already fixed, cite source/test evidence. Otherwise fix them and add reproducible regression tests:

### P0-A. Durable terminal result versus publication

- Examine terminal report persistence inside `EdgeExecutionRuntime::Publish` and the journal interface. A journal append/fsync failure must **not** be silently logged while the system publishes an apparently durable `FULFILLED` result that will disappear on restart.
- Define precise semantics for **physical terminal observation**, **durably committed terminal record**, and **subscriber delivery**. Choose an explicit fail-closed behavior for persistence errors. If you emit a transient outcome, label its non-durability unambiguously. Do not fake atomicity across the autopilot, journal, and network.
- Cover disk full/quota, read-only location, partial writes, fsync failure (through injectable filesystem fault provider), corrupt trailing bytes, process crash during terminal transition, duplicate terminal events, and recovery on restart. Verify exactly what survives.

### P0-B. Reconciliation of an UNKNOWN physical outcome

- The existing journal prevents redispatch of the **same attempt ID** after uncertain restart. Also prevent a **new ID for the same UAV** from bypassing unresolved physical navigation and issuing contradictory commands.
- Define a per-drone **reconciliation gate** after an uncertain attempt: inspect backend mode/active goal and valid telemetry, require an authorized resolution or safe takeover, and never assume that a command timed out means the vehicle physically stopped.
- Test an agent restart while the SimBackend vehicle is still moving, reconnect without clear outcome, submit a new attempt, stale/duplicate reports, conflicting mission revision/client/attempt identities, and recovery after explicit reconciliation.
- Do not promise end-to-end exactly-once physical execution unless the backend can establish it. Prefer truthful `UNKNOWN` and explicit reconciliation.

### P0-C. Cancellation/preemption semantics

- Separate **stop monitoring**, **request physical cancellation/hold**, and **confirmed backend cancellation/hold**. An `INTERRUPTED` software status must not imply the aircraft is stationary.
- If supported by the selected backend, request safe cancel/hold via the existing authority/safety interfaces. If not supported, report physical state as unresolved. Critical simulated battery/navigation/failsafe policies take priority over VECR recovery.
- Test cancellation while moving, mid-recovery, during backend command failure, after terminal status, and after client disconnect. Ensure threads, leases, and authority locks are released without deadlocks or conflicting dispatch.

### P0-D. Preserve completion soundness

- Keep arrival, bounded traversal, qualified survey, and combined obligations distinct. No command ACK is physical arrival. No nominal camera command ACK is surveyed terrain.
- Test sustained stop-style arrival, hard position/vertical/speed bound handling, stale/away samples, timestamp ordering, terminal latching with documented semantics, sensor evidence identity and calibration mismatch, duplicate capture, unknown/unbounded measurement claims, mission-revision changes, and unsupported route geometry.
- The previous project intentionally supports only straight two-point edge routes. **Do not silently navigate a polyline directly to its endpoint and declare it verified.** Either safely add actual polyline-following support with tests or reject S20 as an explicit supported-capability boundary. No false comparison numbers for an unsupported feature.

**P0 exit gate:** Full release build, SDK installation, full CTest, focused VECR tests and sanitizers available in the local toolchain pass; fault-injection tests support the documented reliability invariants. If there are failures, fix them before freezing an experimental campaign. Never disable or remove safety tests merely to get green CI.

## 3. Scientific protocol — define and freeze it before running the final campaign

Create `docs/vecr/SIMULATION_PROTOCOL_V4.md` and `results/vecr/agent_campaign_v4/config/` containing a machine-readable experiment specification. The protocol is a **pre-committed specification**, separate from the subsequent measured outcomes. It must specify:

- Scientific hypotheses H1–H4; definitions of edge fulfillment, truth and observed evidence; baseline semantics; primary and secondary endpoints; exclusion rules (ideally none except infrastructure-invalid runs); exact scenarios; seeds; simulator parameters; intervention limits; sensor model; hardware/software environment; stop/timeout policies; confidence interval methods and analysis plan.
- **H1:** VECR does not incorrectly certify incomplete required terrain under its accepted evidence assumptions; show cases that could falsify this.
- **H2:** Compared with **coverage-aware B2 full-edge restart**, VECR uses less *actual simulated travel/time/re-observation* to repair localized gaps in relevant conditions **without unacceptable loss of completion rate**. No assumption that H2 holds in every scenario.
- **H3:** Its local SDK/agent execution survives realistic restart, communication and fault conditions with truthful terminal semantics and no silent loss of pending obligations.
- **H4:** It can produce informative results to both a local graph-edge client and remote subscribers through SwarmKit's existing APIs, without putting deviation correction on the server or implementing gossip.
- Clearly separate **deterministic modeled guarantees**, **simulated statistical observations**, and **unvalidated real-world sensor and vehicle properties**.

**Crucial comparator question:** Is any claimed improvement from smarter *gap-aware recovery*, from verification evidence quality, from arrival checks, or from better telemetry? Avoid mixing these factors. Design B2 and VECR to use the **same evidence acceptance rules**, camera footprint model, arrival policy, safety gates, movement limits, retries, confidence bounds and data model. The primary contrast must isolate the **recovery strategy**.

## 4. P1 — Implement a genuinely FAIR AGENT-BACKED comparison

Build a reusable experiment harness that drives **both B2 and VECR through the actual production API path**:

`Graph-edge fixture/client → SwarmKit SDK → AgentService → EdgeExecutionRuntime/VECR monitoring and policy → SimBackend → simulated physical vehicle + camera adapter`.

Do **not** compare a B2 algorithm in a hand-coded geometric replay against VECR in the actual agent. Use the same process architecture, SDK, agent, backend, simulated physics, camera events, telemetry transport and command authority for both. Either factor a configurable local recovery policy into the same agent supervisor or implement clearly separated policies behind the identical agent execution interface. Do not introduce a duplicate second VECR state machine or copy-and-paste a parallel full agent.

### Required baseline definitions

- **B0 — arrival-only diagnostic:** destination completion based on ordinary valid navigation arrival; does **not** verify survey. Its false-survey-completion risk is a *diagnostic demonstration*, not a fair coverage-aware competitor.
- **B1 — sampled-corridor diagnostic:** sample-based corridor/arrival checking without full sensor survey evidence. Label as diagnostic; do not describe as equivalent sensing-capability baseline.
- **B2 — coverage-aware FULL-EDGE RESTART:** uses the **same qualified capture evidence and interval-union completeness predicate** as VECR. Upon an incomplete survey after arrival, navigate safely back to A (or the same configured edge start) and resurvey A→B; all transit, turnaround, repeated sensing and failed recovery consume real simulated distance/time and obey the same dynamic/safety limits. It may keep its original evidence, but its configured repair plan intentionally repeats the whole edge. No teleporting, idealized repair, infinite retries or magically working camera.
- **P — VECR targeted-gap recovery:** same verification predicate and resources as B2; selects the earliest still-unverified segment with configured overlap, safely returns toward a feasible re-entry point, resurveys missing intervals, and preserves accepted evidence. It must actually dispatch commands via the agent/backend.

**Primary experimental pairing = B2 versus P.** If needed, compute B0/B1 from the same independent held-out trace for illustrative classification, OR run them as separate actual-policy arms and explicitly label the difference. Do not mix an observational replay with interventional recovery results in a single performance table.

For added scientific rigor implement an optional **B3 / ablation**: a generic gap-aware geometric re-entry policy using the same coverage map and vehicle limits but without VECR's bounded interval verification. This tests whether VECR's advantage is merely a trivial “return to the gap” policy. If B3 equals or beats VECR, report it. The B2/P comparison is mandatory; B3 is strongly recommended for a mature dissertation claim, not a reason to hide incomplete v4 results.

### Identical trial conditions

For each `(scenario_id, seed, parameter_set)` start every compared policy from the same graph, drone initial physical state, camera configuration, planned edge, disturbances and **pre-generated exogenous random streams**. Use separated random seeds for wind, sensor noise, camera outages and communications to avoid accidental cross-coupling. The same randomized **environment** should be shared, but actions will change actual subsequent trajectories; do not force identical realized sensor samples when policies fly different paths. Log **requested faults and realized faults**.

Use a realistic *simulator model* within declared scope: finite acceleration/turning, speed limits, imperfect positional observations, discrete captures, realistic timing, battery drain (clearly labeled simulated), bounded controls, cooldowns, max retries, occasional unsuccessful recovery, and persistent faults when injected. No direct assignment of drone position to a target (“teleport”) and no unconditional success on a repair leg. Distinguish simulated battery/energy proxy from measured electrical energy.

Every policy must share the same navigation/sensing constraints, safety state, maximum attempts, failure triggers, termination horizon and simulator timebase. Audit for method-specific rescue logic and assert that none exists. A fair baseline may outperform VECR in some scenarios.

### Independent ground-truth oracle (mandatory)

The simulator privately knows **true simulated vehicle poses and actual simulated camera acquisition geometry and validity**. The VECR agent must receive only **noisy/limited telemetry and sensor adapter messages**. Truth must not be read by the verifier, high-level application, recovery policy, or sensor evidence acceptor. Use separate interfaces and, if practical, separate compilation units/processes. Add tests that fail if a truth-only field leaks into the agent input.

Compute ground-truth raster/polygon geographic coverage **independently** from the method's accepted-footprint implementation, with explicit resolution and tolerance; test the oracle on hand-constructed paths/captures, narrow gaps, and boundary cases. A separate independent oracle must also track actual travel/time, true arrival, executed commands, collision/airspace events if simulated, and fault onset. Declare what truth the simplified model can and cannot establish.

For S11 (false camera success), do not pretend SwarmKit can detect a compromised authenticated camera that lies about quality without independent evidence. Test two variants: (a) detectable bad/stale/contradictory metadata that the verifier should reject; (b) plausible yet dishonest sensor metadata that can cause a false completion. Report this trust boundary honestly, do not guarantee sensor authenticity implies physical truth.

## 5. P2 — Complete the controlled scenario matrix S00–S20

Produce runnable, versioned **agent-backed** scenario configurations for all 21 IDs below. Record **per-scenario applicability** to B2, P, and optional diagnostics. A case that tests a P0 invariant may be non-comparative; label it rather than force a fake B2 row. Each fault must have observable effects and meaningful parameters. At least one adversarial or negative outcome must be possible for each relevant class.

| ID | Required scenario and behavior to investigate |
|---|---|
| S00 | Nominal A→B with healthy position, camera and communication: no unnecessary recovery; true complete survey. |
| S01 | Transient GPS jitter but physical flight inside corridor: false alarms, false recovery, arrival conservatism. |
| S02 | Persistent lateral wind/drift: local corridor excursion and bounded re-entry; allow failed recovery. |
| S03 | UAV physically reaches B after a detour missing a middle ground strip: separate arrival from survey. |
| S04 | Camera stops capturing for a central segment while navigation is correct: preserve and repair missing survey. |
| S05 | Capture metadata delay/drop and missing observations: distinguish unverified evidence from true failure, measure unnecessary repeat coverage. |
| S06 | UAV flies outside corridor but wide field of view still truly observes the required strip: retain valid observations while supervising navigation. |
| S07 | High speed plus sparse telemetry allowing excursions between samples: compare sampled checker and bounded traversal, including indeterminate status. |
| S08 | GNSS invalid/stale for a span and subsequently recovers: no fictitious covered intervals from unknown location. |
| S09 | Estimator jumps or wrong route reference/origin: no skipped progress, test coordinate-frame rejection. |
| S10 | Brief B-radius passage at high speed / away-after-arrival: arrival dwell, speed and latching semantics. |
| S11 | Camera reports success despite unobserved ground: detectable bad metadata and undetectable dishonest-adapter variant as separate subcases. |
| S12 | Critical low battery/RTL or equivalent simulated safety condition mid-edge or mid-repair: safety has priority; incomplete result truthful. |
| S13 | Ground/server transport lost while agent flies: local recovery continues; remote event loss/replay evaluated separately. |
| S14 | Peer channel interrupted: verify local progress independent of peer messaging; no claim of guaranteed dissemination. |
| S15 | Actual agent process restart/power-loss simulation with nonterminal attempt: journal, UNKNOWN/reconciliation gate, no conflicting new dispatch. |
| S16 | UAV stops responding or physical movement ends after partial coverage: no false completion; report incomplete/unknown appropriately. |
| S17 | Multiple simultaneous simulated UAVs moving at different speeds on separate edges: identity isolation and throughput; no global equal-speed assumption. |
| S18 | Same edge under conflicting mission revisions or duplicate/stale attempt ID: no mixed evidence or improper redispatch. |
| S19 | Recovery unsafe, infeasible, mode-disallowed or authority rejected: agent does not force dispatch; report safe and truthful result. |
| S20 | Curved or near-self-intersecting polyline: either implement and verify correctly, **or** explicitly reject as unsupported, mark as capability-boundary test, and exclude from performance-comparison denominator. Never imply straight-edge implementation supports this. |

Parameter studies should include at least: different wind strengths, sensor-gap positions/lengths, telemetry frequencies, measured error-bound availability, vehicle speed/acceleration, corridor width, camera footprint/overlap, recovery limits, and simulated connection latency. Use limited preregistered grids, not arbitrary parameter tuning after seeing results.

**Do not claim “21 successful comparable scenarios” if S20 or fault/cancellation-only scenarios are not valid B2/P performance comparisons.** Show the scenario matrix with status `COMPARATIVE`, `DIAGNOSTIC`, `SAFETY`, `CAPABILITY_BOUNDARY`, or `UNSUPPORTED`, and summarize the counts honestly.

## 6. Experiment quantity and run plan

Provide two commands implemented by a new harness (example CLI; document actual flags):

```bash
python3 scripts/vecr/run_agent_campaign_v4.py \
  --config results/vecr/agent_campaign_v4/config/protocol.json \
  --mode smoke --seeds 1:3 \
  --output results/vecr/agent_campaign_v4/smoke

python3 scripts/vecr/run_agent_campaign_v4.py \
  --config results/vecr/agent_campaign_v4/config/protocol.json \
  --mode full --seeds 1:100 \
  --output results/vecr/agent_campaign_v4/final
```

Make this work in the actual supported build environment, rather than requiring an unavailable machine-specific binary at `build/mac-release/tests/swarmkit_tests`. Use configurable executable discovery/CMake presets. A research runner should not silently overwrite frozen results.

- **Smoke gate:** 3 seeds per relevant case, B2/P policies, strict correctness checks; inspect sample traces and injected-fault realizations. Smoke numbers are **not dissertation conclusions**.
- **Final target:** **100 paired seeds per eligible comparative scenario**, with both **B2** and **P** actually executed. If all 21 are eligible, that is `21 × 100 × 2 = 4,200` agent-backed attempts. If fewer are eligible, compute and display the exact count; do not invent placeholders. Add diagnostic-only arms separately.
- Use deterministic recorded seed lists and a separate seed-validation sensitivity run where practical. If the complete target cannot be executed, preserve partial raw data and label the study `INCOMPLETE`; never extrapolate final outcomes from smoke runs.
- Always capture both successes and failures. Do not drop timeouts, simulator crashes, agent exceptions, non-arrivals, zero-completion cases or interrupted trials from tables. Distinguish experiment infrastructure invalidity from a policy failure, document any allowed exclusions with reasons, and report counts in both cases.
- Where completion is rare, run paired sensitivity analyses rather than pretending averages conditional on success represent all attempted tasks.

## 7. Required measurements, definitions, statistics and diagrams

Use transparent **per-attempt raw records** as the basis of every number. Include stable identifiers `run_id`, `scenario_id`, `seed`, `policy`, `attempt_id`, `edge_id`, `mission_revision`, and hashed `config_id`, along with fault injection/realization logs, terminal status, ground truth, and resources consumed.

### Primary outcomes

1. **False survey completion:** `false_complete = (reported EDGE_FULFILLED) AND (independent truth says required observation incomplete beyond configured tolerance)`.
2. **False-completion incidence:** `false_completes / all eligible assigned attempts` (unconditional per-attempt metric).
3. **False-completion fraction among completed reports:** `false_completes / number of reported fulfilled attempts` (precision-related metric). If denominator is zero, report **undefined/NA**, not 0%.
4. **True completion, reported completion and verified availability:** separately report independent true success, reported success, and true-positive accepted completion rates. Report `UNKNOWN`, `INTERRUPTED`, and invalid-observation rates; a system that refuses all tasks must not look perfect.
5. **Uncovered ground truth:** missing raster cells / total required cells, missed length or area, gap count and maximum gap length; raster resolution and acceptable tolerance explicit. Report false-negative/incomplete decisions too.
6. **Actual simulated travel distance:** integrate the true positions along the executed physical trajectory, including outbound movement, turning, re-entry, transit, restart loops and safety actions. Do not calculate it only from ideal route segments.
7. **Actual simulated completion time** (mission-clock): elapsed from attempt accepted to terminal; report censoring/timeouts. Distinguish simulator time from benchmark wall-clock time.
8. **Recovery success/overhead:** requested/accepted/failed recoveries, extra distance, turnaround and re-entry cost, repeated surveying, battery proxy/estimated energy, timeout count, aborted safety events.

### Secondary outcomes

- Distance and time **per assigned task including failures**, not only conditional on successes; successful-only statistics may be shown separately and clearly labeled to avoid survivorship bias.
- Verification conservatism: genuinely surveyed but reported incomplete; false alarms; unnecessary repeat flights; progress latency; observed/true coverage agreement.
- State transition/event correctness: monotonic verified-set bookkeeping subject to explicit evidence revocation; journal and reconciliation integrity; duplicate/stale event handling.
- Communications: delivered/lost/replayed report count, local-to-client and agent-to-monitor latency, simulated network interruptions; label modeled network effects versus measured host transport and **do not assert high bandwidth from API names**.
- Product performance: process CPU %, peak RSS memory, events/s, agent thread count, simulator steps/s, SDK RPC latency and run wall time; label host hardware, debug/release build, execution mode.

### Scientific comparison

- Report `n` and uncertainty for each scenario and aggregated results; show numerators/denominators, not just percentages.
- Predefine paired difference `Δdistance = distance_P − distance_B2`, `Δtime`, `Δcompletion_rate`, and `Δfalse_completion`. For beneficial reduction, negative Δdistance is good **only when completion rates and safety are also comparable**.
- Use **paired bootstrap over independent seeds** (or another justified paired interval method), e.g. 10,000 fixed-seed bootstrap resamples, and Wilson 95% intervals for relevant binomial proportions. Give exact numerator/denominator. **Zero observed failures is not proof of zero failure probability.**
- Do not pool duplicated or correlated attempts as though independent. For multi-UAV cases identify trial/cluster unit. Report per-scenario distribution plus macro-summary. A pooled mean can conceal a poor-performing category.
- Provide median, p95, IQR/SD (with declared convention) and confidence intervals where appropriate; avoid interpreting a single pilot attempt as a probability.
- Explain if both B2 and VECR have the same zero observed false-completion rate: the evidence then supports comparable measured soundness, and any advantage pertains to recovery cost or another endpoint, not superior verification accuracy.
- Calculate absolute and relative distance savings from **new agent-backed numbers**. Never copy the old v2 `19.5%` saving as an agent-backed conclusion. If VECR loses in a scenario, show why and retain the finding.
- Include a limitation for **simulated sensing**: the oracle proves agreement with modeled camera acquisition and assumptions, not actual real-world photo quality or terrain visibility.

### Required figures and tables

Produce at least:

1. `scenario_inventory.csv`: 21 IDs, purpose, eligible baselines, configuration, actually injected fault and validation status.
2. `per_attempt.csv`: every executed trial and every defined metric (one row per policy/seed/scenario).
3. `fault_events.csv`, `execution_events.csv`: time-ordered injection, actual physical effects, reports and backend command events.
4. `methods_summary.csv` and `scenarios_summary.csv`: all denominators, rates, distribution summaries, CIs and resource cost.
5. `paired_comparisons.csv`: VECR−B2 paired differences by scenario and overall.
6. `figures/trajectory_good_bad.svg` with intended corridor, actual physical path, re-entry and camera coverage gap.
7. `figures/coverage_timeline.svg`: truth vs verified intervals through deviation and recovery.
8. `figures/distance_by_scenario.svg` and `figures/time_by_scenario.svg` with uncertainty.
9. `figures/false_completion_and_availability.svg`: correctly shows accuracy–availability tradeoff with valid denominators.
10. `figures/failure_modes.svg`: outcome counts for completed, incomplete, unknown, cancelled, safety-preempted and simulator infrastructure error.

Use SVG or PNG generated locally from raw data. The report must be reproducible and readable; do not embed invented hand-drawn numbers. Save selected full event traces for illustrative positive, negative, and boundary cases (not only successful ones).

## 8. Quality checks that catch bad experimental methodology

Add automated checks that make the campaign FAIL LOUDLY for:

- Ground truth leaked into VECR or policy inputs; synthetic camera always succeeding on recovery legs; vehicles teleporting or skipping acceleration/turns; failure injection scheduled but never physically taking effect.
- B2 using different camera quality, error bounds, arrival guard, retry budget, battery, vehicle model or safety policy than P.
- “Truth complete” derived from VECR's own reported interval union rather than the independent oracle.
- A B2 or P attempt missing an artifact, with different initial state/seed/configuration, or incorrectly excluded from paired statistics.
- A false complete silently dropped, a zero denominator recorded as 0%, an incomplete trial treated as a completed one, or a simulated energy proxy labeled as measured joules.
- Stale/foreign attempt or mission-revision metadata contaminating accepted evidence.
- Recovery and arrival events being fabricated from a status variable without actual simulator/backend transitions.
- Different behavior between smoke and final configuration without a recorded override.
- A fixed plot/report that contains hand-entered percentages; all tables/plots must be generated from raw data.

Include at least several **adversarial oracle self-tests**, where you intentionally insert a known gap and verify the truth classifier catches it independent of the VECR status. Include method-agnostic sensor edge cases and random-seed reproducibility checks.

## 9. Required final output layout and reproducibility

Use a fresh versioned destination, for example:

```text
results/vecr/agent_campaign_v4/
  config/
    protocol.json
    scenarios/
  smoke/
  final/
    raw/per_attempt.csv
    raw/execution_events.csv
    raw/fault_events.csv
    raw/traces/
    tables/scenario_inventory.csv
    tables/methods_summary.csv
    tables/scenarios_summary.csv
    tables/paired_comparisons.csv
    figures/*.svg
    reports/EXECUTIVE_RESULTS.md
    reports/SCIENTIFIC_RESULTS.md
    reports/LIMITATIONS_AND_NEGATIVE_RESULTS.md
    manifest.json
  README.md
```

Record git commit, dirty-tree status, source file hashes, protocol/config hashes, compiler, operating system, CPU architecture, simulator version, run commands, RNG stream seeds, sampled config, dependency lockfiles and build binary hashes. A measured dataset should be bound to **one clean commit or a complete source hash snapshot**; do not casually regenerate figures from changed logic without a new result version. Use SHA-256 for raw tables and figures; provide a validator script that recomputes all hashes and rechecks row counts and claimed baselines. Avoid output directory overwrite. Preserve v1/v2/v3 as frozen historical records.

Generate dissertation-ready prose in `SCIENTIFIC_RESULTS.md` with these headings: **Experimental goal**, **System under test**, **Baselines and fairness**, **Scenarios**, **Ground-truth design**, **Metrics**, **Results (actual values with uncertainty)**, **Tradeoffs/negative results**, **Threats to validity**, **Reproducibility** and **What is not demonstrated**. Use simple, defensible language. Do not label data from simulation as SITL or physical-flight measurements.

## 10. Product cleanup: focused, safe, and non-destructive

While implementing P0/P1/P2, keep the product clean:

- Prefer one canonical public `ExecuteEdge` request and typed result/report semantics; update protobuf/SDK/agent/CLI/examples/docs together for justified breaking changes, consistent with `AGENTS.md`.
- Reuse existing AgentService, SimBackend, telemetry, authority, ReportHub, DataService and journal. Avoid redundant engines, double state machines, unused wrappers and duplicate math implementations.
- Remove proven dead source/build rules or redundant newly written experiment helpers **only after** verifying no public product/test dependency. Document deletions and replacements. **Do not remove unrelated tests, the archives, or frozen result sets**.
- Do not weaken real-vehicle safety policy or turn on MAVLink recovery. Keep P0 changes backend-safe.
- Keep a clean release build and installable SDK. Document changes in `docs/vecr/IMPLEMENTATION_PROGRESS.md` and update `README.md` only for genuine new capabilities.

## 11. Execution order and explicit stage gates

Proceed in this order:

**Gate 1 — P0 reliability:** audit code, fix persistence/reconciliation/cancel issues; focused tests, full CTest and available sanitizer suite pass. Save a short explanation of the actual consistency guarantees and limits.

**Gate 2 — Agent-backed benchmark:** implement common B2/P agent policy harness, independent truth oracle and fault model. Demonstrate via hand-inspected traces that both arms execute actual backend commands; confirm simulated physical trajectories differ only as dictated by policy. No old core replay permitted for the primary results.

**Gate 3 — Scenario coverage:** implement S00–S20 with applicable outcomes, safety/capability diagnostics, and independent truth validation. Smoke run with 3 seeds; use results to debug correctness (not selectively optimize benchmarks).

**Gate 4 — Freeze protocol and full simulation:** freeze configs and source revision; run 100 paired seeds per eligible scenario (or explicitly document incomplete target). Capture all successes, interruptions and failures.

**Gate 5 — Analysis and research artifacts:** independently analyze raw rows; produce confidence intervals, tables, SVG figures, per-scenario discussion, failure/uncertainty analysis, manifests and dissertation-ready text. Recheck all output hashes and counts.

**Gate 6 — Final project verification:** release build; SDK install check; CTest; available sanitizer build; formatter/linter if established in repository; final `git status` and evidence integrity. Commit code/protocol/report changes with comprehensible messages; do not push unless the repository workflow/authorization explicitly permits it. Do not modify/replace frozen historical results.

At the end, summarize in chat:

- Changes made and exact affected files.
- P0 tests actually executed and results, including fault-injection cases.
- Exact number of full **agent-backed** B2/P attempts and which S00–S20 cases are comparative vs diagnostic/unsupported.
- A results table with counts/denominators, true completion, false-completion metrics, mean/median travel, completion time, recoveries, confidence intervals and VECR−B2 differences.
- Which scenarios favor VECR and which favor B2; any significant failures or measurement caveats.
- Path to frozen raw data, protocol, plots and validation manifest.
- Explicit statement: **“Only simulator evidence has been obtained. SITL, real camera calibration, and physical flights are not validated.”**
- Any unfinished tasks and why. Do not mark the work experimentally validated when major gates or final data are missing.

## 12. Acceptance criteria — don't game the benchmark

I will consider the simulator phase complete only when:

1. The P0 reliability issues are fixed or demonstrably nonexistent, with explicit regressions that exercise real failures.
2. A versioned B2/P comparison exists through the same **real SDK → agent → SimBackend path**, not core geometry alone.
3. The simulator uses **independent private ground truth**, never available to the verifier, with tested validity checks.
4. The 21 planned scenario IDs are implemented or honestly classified as unsupported/diagnostic; no fabricated comparable trials.
5. A preregistered repeated-seed campaign produces raw **agent-backed** measurements, including failed and unknown attempts, and correctly paired B2/P policies.
6. Final statistical analysis includes correct denominators, uncertainty intervals, fair recovery-cost metrics, availability and unsuccessful outcomes; reported effects are calculated from new data.
7. Every dissertation number is traceable from a report/table to raw data, source/config revision and a manifest hash.
8. The product still builds, installs, and passes its safety/transport/core tests; old archives and v1/v2/v3 data remain unchanged.
9. Research conclusions are bounded to **geometric/dynamic simulator and simulated camera assumptions**. The final report makes no SITL or real-world performance claim.

**Most important instruction:** Fairness and correctness come before impressive numbers. If VECR saves less distance than the old 19.5% geometric figure, or B2 performs equally well, or some policies fail, preserve that truth. The dissertation needs defensible evidence, not predetermined success.
