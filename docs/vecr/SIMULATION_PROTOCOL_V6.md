# VECR v6 digital preregistration

Written 2026-10-27 before v6 trials. Frozen historical results are preserved.
See RECOVERY_MODEL_V6.md and V6_WORK_LOG.md. No favorable effect is required.

## Design

Nine policies: B2 full restart, D departure point, R2/R8 fixed rewind at two
prespecified strengths, P existing VECR with 2 m overlap, G zero-overlap earliest
missing gap, GW zero-overlap gap repair with endpoint waiting, E0 the derived sensing-envelope target, E the same target with bounded
endpoint evidence wait. All receive identical measurements and qualified capture
inputs, retain disjoint evidence, and share the verifier, safety and retry limits.
The physical model uses bounded acceleration and actual unsuccessful commands.

The executable matrix in run_agent_campaign_v6.py is frozen with its revision and
binary digest before final execution. Primary conditions include nominal, delayed
and reordered delivery, one and multiple missing regions, changing uncertainty,
physical departure, persistent sensing failure, command rejection, motor failure,
low battery, exhausted attempts and dishonest-evidence controls. Short/long edges,
sparse sensing and wider uncertainty are included. Thirty paired confirmatory
seeds 30001–30030 per condition; independent pilot seeds 40001–40002. Each policy
uses the same random streams by seed/tick, never by policy. Order rotates by seed.
Changes after pilots must be recorded before freezing. No repeated tests on final
seeds to tune parameters. All assigned attempts and unsuccessful pilot versions
remain available. Infrastructure errors invalidate a complete-campaign gate.

No pooling with v5.1. Exact physical strip truth is independently reconstructed
from private simulator trajectories/acquisitions, without production geometry.
Primary results include true+reported completion, false fulfillment, false
rejection, distance, time and recoveries. Report per-condition paired differences
with 10,000 bootstrap resamples and fixed RNG 20261027. Cost among jointly true
and reported successful pairs is secondary and states its denominator. Whole-
seed resampling preserves the scenario block for any fixed-mixture summaries.
Do not turn a mixture average into fleet savings or rank failed policies by their
shorter interrupted travel. Report zero-event observations without claiming zero
population risk. Control dishonest inputs separately and retain their outcomes.

## Gates

1. Core and localhost SDK/agent/backend regression suites, including late evidence,
   expired evidence, distinct policies, maximal feasible target and wait safety.
2. Frozen source, binary, assignments, environment, protocol and configuration.
3. Every assigned raw trajectory, measurement, acquisition, capture decision,
   command, phase trace and result retained, plus independent reconstruction.
4. Tables, paired intervals, figures and raw data with SHA-256 verification.
5. Reuse through direct Client and routed SwarmClient with identical agent logic;
   report this as API-client reuse, not proof for all high-level planners.
6. Subsequent separately versioned ArduPilot SITL qualification. Real imagery
   remains unvalidated unless actual calibrated images are acquired.

## Engineering amendment before freeze

Pilot 01 (40 runs, five conditions, seed 40001) was independently reconstructed.
All eight policies passed nominal; simple G outperformed E0/E in travel in the
observed repair cases. Waiting avoided an unnecessary delayed-evidence repair.
Add GW (G plus the same endpoint wait) to isolate that mechanism and provide a
stronger comparator. No parameters or conditions were removed to favor E. The
final design is 22 conditions × 30 seeds × 9 policies = 5,940 attempts.
