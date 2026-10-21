# VECR agent campaign v4.1 — fault-realization correction

This addendum was frozen before the v4.1 full campaign. The base design,
hypotheses, B2/P pairing, outcome definitions, seeds, bootstrap plan and
limitations remain as specified in `SIMULATION_PROTOCOL_V4.md`. The complete
machine-readable specification is
`results/vecr/agent_campaign_v4/config/protocol_v4_1.json`.

The first v4.0 full run stopped at S16 seed 10, B2 after 2,018 completed
attempts. The scripted camera gap was defined by camera-center position. At
that seed's capture phase, images on either side of the gap overlapped enough
to cover the entire strip. The agent correctly finished without recovery, so
the intended motor-failure-on-repair intervention never happened. The runner
refused to count that trial. The partial output is retained as
`partial_aborted_s16_010/` with `INCOMPLETE` status; it is not mixed into v4.1.

V4.1 suppresses each scheduled capture whose *physical footprint intersects*
the scripted gap during the initial 12 simulated seconds for S04, S16 and S19.
S05 suppresses the corresponding metadata submission. Fault realization is
recorded only on an affected scheduled capture, and S16/S19 count as realized
only when the motor failure or recovery-command rejection actually occurs.
The vehicle, verifier, B2/P policy, attempt limit and oracle are unchanged.
This correction applies equally to B2 and P and was made before inspecting
any v4.1 outcomes. It creates a new source commit, config hash, binary hash
and final output directory. No v4.0 results are used as confirmatory evidence.

The analysis also records an `ALL_MACRO` P−B2 row. It averages the 12 fixed
scenario differences within each seed and bootstraps the 100 seed clusters
10,000 times. The relative distance saving divides the negative macro
distance difference by B2's mean assigned-task distance. These aggregate
costs include failed tasks and must be interpreted beside completion rates.

Before the v4.1 full run, all 100 preregistered seeds in S16 and S19 are
preflighted for both policies to verify that the intended physical recovery
fault is realized; those development checks are not scientific outcomes.
