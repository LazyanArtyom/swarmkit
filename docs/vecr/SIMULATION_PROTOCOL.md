# Frozen digital protocol — in-flight VECR, edition 8

## Selection and preregistration

The single production method permits bounded correction only after a sustained
departure during A-to-B flight. Qualified arrival at B terminates the attempt
after at most 3 s of evidence finalization. Residual survey gaps are reported
without an automatic backward command. This is a selected-method validation,
not a blinded discovery study. The v7 campaign used endpoint repair and is
historical; its measurements are not pooled with this edition.

This protocol is committed before the frozen campaign. All assignments and binary
hashes are recorded before execution. No failed attempt is replaced. A correction
requires a new separately named run and retention of the failed run.

## Assignment matrix

22 conditions × 30 paired seeds (70001–70030) × two executables = **1,320 attempts**.
The production method is labelled VECR; B2 is a non-installed full-edge-restart
research comparator on the same qualified *in-flight* departure. Both wait up
to 3 s at B and use the same evidence verifier, 3 s capture-age limit, two
recovery attempts, 30 s per recovery, 200 m conservative recovery-travel bound,
dynamics, observations and safety. Only the in-flight
re-entry target differs. Neither policy repairs an endpoint gap. No production
policy selector exists.

| ID | Condition |
|---|---|
| N00 | Nominal |
| N01 | Evidence delivery delayed 2.5 s |
| N02 | Reordered evidence |
| N03 | Missing metadata; endpoint gap is reported |
| N04 | Multiple capture gaps; endpoint gaps are reported |
| N05 | Changing localization uncertainty |
| N06 | Uncertainty increase during a gap; endpoint gap is reported |
| N07 | Sustained physical departure |
| N08 | Departure with wide sensing |
| N09 | Departure with narrow sensing |
| N10 | Persistent sensor failure |
| N11 | Recovery command rejected |
| N12 | Motor failure during qualified in-flight repair |
| N13 | Low battery |
| N14 | Zero recovery budget |
| N15 | Evidence arrives after 3 s validity horizon |
| N16 | Sparse small-footprint acquisition |
| N17 | Multiple gaps, 20 m route; no endpoint repair |
| N18 | Multiple gaps, 80 m route; no endpoint repair |
| N19 | Dishonest camera-frame control |
| N20 | Undeclared camera-angle control |
| N21 | Multiple gaps with short deadline |

Default route 40 m; half-footprint 2 m along and 7 m across; hard pose bound
0.3 m; acquisition period 0.5 s; delivery 0.2 s; command speed 2 m/s;
physical speed bound 5 m/s; deadline 90 s. The runner's complete assignments are
the authoritative machine-readable configuration, including scenario overrides.
No policy identifier enters noise/fault random-number generation. Rotate execution
order by seed. Simulated time is controlled; host timing is descriptive only.

## Independent validation and analysis

Reconstruct truth from actual acquired footprints and trajectories using independent
half-space geometry, not the production interval calculation. Check accepted-footprint
containment, physical sampled re-entry dwell, exact assignment membership, reported
arrival, reported obligation verification, actual completion, and failures. Assert
that no accepted recovery GOTO follows qualified arrival. Dishonest controls are
separate from honest-input soundness estimates. No oracle truth enters the agent
or sensor estimator.

Report all 44 condition/method rows. Per-condition completion uses Wilson 95%
intervals. Pair by seed. Bootstrap paired acceptance differences on all 30 pairs;
bootstrap distance/time differences only among jointly true-and-reported successful
pairs, always showing that denominator. Use 10,000 resamples, RNG seed 20261027.
Intervals are pointwise, not multiplicity-adjusted; no fleet reliability inference.
Never call a failed shorter flight an efficiency advantage.

Reuse campaign: five conditions N00/N01/N04/N07/N11 × ten seeds 72001–72010 ×
direct/routed SDK entry paths × VECR = **100 attempts**, 50 matched pairs.
This tests client entry paths, not multiple independently implemented graph planners.
Pilot: all conditions and both executables, two seeds 71001–71002 = 88 attempts.
All pilot outcomes remain separate from primary statistics.

## Reproduction

Build the release targets and commit experiment source, then run into a new v8 directory:

```
.venv/bin/python scripts/vecr/run_digital.py --mode final --output results/vecr/v8/primary
.venv/bin/python scripts/vecr/validate_digital.py --output results/vecr/v8/primary
.venv/bin/python scripts/vecr/analyze_digital.py --output results/vecr/v8/primary
```

Use a new output directory for repetition; never overwrite a completed campaign.
