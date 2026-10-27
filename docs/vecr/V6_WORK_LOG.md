# VECR v6 research work log

Started 2026-10-27 from `27522fdd78ab46c7efc8aba95ab802358f137cca`.
The user authorized closest-policy comparisons, a formal recovery rule, a new
digital campaign, external-client reuse, and subsequent ArduPilot SITL validation.
This authorization supersedes the earlier instruction to defer SITL. Historical
protocols and results remain frozen. New work does not establish novelty by naming it.

## Audit before implementation

Read AGENTS.md, METHOD_AND_CORRECTNESS.md, SIMULATION_PROTOCOL_V5.md,
SIMULATION_PROTOCOL_V5_1.md, the v5.1 results and novelty assessment, core verifier,
agent runtime, SDK serialization, CLI, protobuf, v5 trial/runner and truth models.

* Current policies are full-edge restart and earliest-gap minus fixed overlap.
  Neither is compared against departure-point or fixed rewind in frozen v5.1.
* Capture consistency uses only the most recent bounded pose, with a 500 ms
  neighborhood. Delivery latency is therefore conflated with pose availability.
  An acquisition-time pose history can retain valid evidence without changing
  its geometric acceptance conditions. It must be bounded and age-limited.
* Re-entry ACK is already separated from physically verified stationary re-entry.
  Recovery, authority, safety checks and final evidence acceptance remain onboard.
* A derived target requires an explicit future sensing envelope. This is a
  conditional planning assumption, never evidence that a future capture succeeded.
* A wait at a verified endpoint can allow evidence to arrive. A departure must
  still trigger prompt recovery; waiting cannot suspend safety or deadlines.
* Live MAVLink recovery remains disabled. SITL requires a distinct documented
  qualification; ordinary MAVLink accuracy must not be relabeled a hard bound.

## Milestones

1. Common bounded pose-history verifier; departure/fixed-rewind/zero-overlap
   comparators; regressions and an engineering comparison pilot.
2. Formal restricted recovery model, derived robust target, endpoint evidence
   wait, and tests of soundness boundaries, timing, failures and resource limits.
3. Freeze source, executable, full assignments and protocol before a new digital
   campaign. Independently reconstruct every run; paired uncertainty and failures.
4. Direct and routed external client entry paths through the same execution API.
5. Genuine ArduPilot SITL tests with independent simulator truth; preserve
   unsuccessful qualification attempts and distinguish synthetic camera evidence.
6. Update literature comparison and dissertation reader to actual evidence;
   explicitly state equivalences, negative results and unsupported claims.

## Initial status at audit

Audit complete. Implementation and experiments pending at that checkpoint; subsequent progress is recorded below.

### Engineering progress

Core/RPC/SDK/CLI implement D, fixed rewind and conditional sensing-envelope targets.
Acquisition-time bounded pose history accepts delayed captures with a configured
age limit. Added a missing-speed-bound rejection for noncontemporaneous alignment.
Nearest-pose ties use the latest received sample at the same millisecond.

Pilot 01: 40/40 executed, independently reconstructed (2,560 delivered captures,
23 physical re-entry dwells). G used less travel than E in the tested repairs.
Added GW before freeze to isolate waiting and strengthen the comparison.

Pilot 02: retained infrastructure failure. The run was mistakenly started while
an unrelated new regression fixture failed compilation (namespace placement),
so the old executable rejected GW. All 396 assigned rows remain archived; they
are not confirmatory observations. Corrected the fixture namespace, rebuilt, and
started a new pilot version. This is an execution-process error, not a policy
failure. A future runner must check executable support before assigning a campaign.

The first sandboxed integration invocation could not bind localhost; the same
suite was run with authorized localhost access. The initial ArduPilot build found
an uninitialized littlefs submodule; dependency checkout continues. No SITL flight
has yet been run.

Pilot 03: 396/396 executed and independently reconstructed. Pilot 04: final
preflight, 198/198 executed and independently reconstructed. Release and UBSan
complete suites passed (2/2 targets each); four new v6 cases passed 70 assertions.
The new agent history fixture initially omitted required backend health timestamps;
those fixture fields were restored. Production acceptance was not relaxed.

The v6 executable now describes its supported policy set. The runner checks that
before creating assignments and refuses uncommitted source for confirmatory runs.
Added capture-delivery deadline and stale-endpoint checks. Captures cannot exploit
history to declare fulfillment after the attempt deadline or with a stale endpoint.

ArduPilot Copter-4.7.2 compiled natively after initializing its pinned littlefs and
lwip dependencies. Compilation is preparation, not SITL experimental evidence.

### Confirmatory assignment correction

The initial final runner exposed 3,960 assignments in its progress counter. An
implementation text replacement had changed the intended default 30 seeds to 20.
The manifest records those 20 exactly; the 22-condition/nine-policy protocol still
requires 30. No comparative final statistics were inspected for this decision.
Preserve `confirmatory/` as a protocol-deviation run and do not pool it with the
new full `confirmatory_v6_1/` campaign. Fix the default and require exactly 5,940
assignments and seeds 30001–30030 before final dispatch. Repeat the complete
planned design using the unchanged physics/policy binary. The overlapping seeds
are repeated observations, not additional independent samples.

### Completed digital validation and resumed SITL work — 28 October

Primary confirmatory_v6_1 completed all 5,940 assignments with no infrastructure
failures. Independent reconstruction checked 516,751 captures and 3,984 re-entry
dwells. The 200 client trials matched in all 100 direct/routed pairs. The observed
result does not favor the derived E target over the strong GW comparator: equal
completion, frequently greater travel. Full comparisons and paired intervals are
retained; the dissertation now describes that limitation.

SITL pilot_01: 12 assigned, one native-accuracy attempt executed, 11 infrastructure
failures. Diagnosed overlapping TCP/auxiliary simulator ports, missing simulated
receiver input, unsuccessful one-shot takeoff and Python monotonic versus libc++
steady-clock epoch mismatch on macOS. Kept all logs. Fixed isolated port blocks,
explicit simulated receiver input, bounded takeoff ACK retries and a cross-language
clock probe; arming checks remain enabled. MAVLink binary streams are now correctly
named `.raw` (pilot_01 `.tlog` contains raw packets without tlog timestamp headers).

SITL pilot_02: two nominal synthetic-bound flights, independently reconstructed.
Pilot_03: all 12 assignments executed and independently reconstructed, including
native accuracy refusal, delayed evidence, actual re-entry, backend recovery refusal
and LAND override. Reconstruction checked 394 accepted captures and two sampled
re-entry dwells. Neither run is the final qualification.

Nominal flights exposed a production defect: setting speed before GUIDED position
submode initialization allowed ArduPilot to reset it. Peak modeled speed was
6.508 m/s despite a 2 m/s request in pilot_02. Reapply requested speed after target
dispatch. Regression emulates the submode reset for both reposition ACK and fallback
setpoint, plus post-dispatch speed refusal (explicit FAILED, vehicle may still move).
V6 regression suite now passes 104 assertions in five cases. Pilot_04 uses the
corrected backend; each runner now freezes its executable before assignments.
