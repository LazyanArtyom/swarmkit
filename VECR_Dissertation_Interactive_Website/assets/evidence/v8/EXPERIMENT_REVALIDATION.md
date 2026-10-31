# VECR v8 experiment revalidation

## Protocol boundary

The production method now corrects only a sustained, qualified deviation while traveling A→B. Qualified stopped arrival at B ends local execution after at most the configured passive evidence wait and reports the remaining intervals. There is no autonomous flight from B to a gap. The retired v7 endpoint-repair campaign and its SITL flights are absent from the current checkout; their pre-cleanup state remains in Git history. None of their numerical estimates enters the active v8 tables or figures.

The frozen primary source revision is `66603912556002d162dc42d910b598bfbbc480a2`. `results/vecr/v8/primary/manifest.json` records the source, executable hashes, assignments, 22 condition overrides, 30 common seeds (70001–70030), policy, host and protocol. VECR and the separately compiled non-installed B2 comparator share the agent verifier, dynamics, faults, evidence wait and safety limits; only their qualified in-flight return target differs. B2 returns to A. The campaign contains 1,320 attempts and no substituted failures.

## Validation gates and outcomes

| Gate | Executed evidence | Outcome |
|---|---|---|
| Build and CTest | Release build and 3 CTest targets, including the agent/SDK integration suite | Passed; current rerun logged in final change report |
| Pilot | Four 88-attempt pilots were executed during development; their directories were removed from the current checkout | Diagnostic only; excluded from primary estimates; pre-cleanup records remain in Git history |
| Primary assignment | `primary/manifest.json` and `per_attempt.csv` | 1,320/1,320 assigned attempts recorded |
| Independent oracle | `scripts/vecr/validate_digital.py`, `primary/validation.json` | Passed: 1,320 attempts, 54,850 delivered captures, 180 physical re-entry dwells; no accepted recovery GOTO after qualified B arrival |
| Statistical analysis | `scripts/vecr/analyze_digital.py`, `primary/analysis/` | 44 condition/policy rows, 22 paired contrasts; 10,000 paired bootstrap resamples and Wilson proportion intervals |
| Reuse | `reuse-qualified/`, `scripts/vecr/analyze_reuse.py` | Passed: 100 attempts, 50 direct/routed pairs, zero differences in ten audited fields |
| Unsuccessful infrastructure attempt | `reuse/` | 100/100 attempts failed because the sandbox denied local loopback bind; preserved, excluded from qualified parity statistics |
| Revised-method ArduPilot SITL | No v8 SITL campaign | **Not executed; TBD**. Archived v7 SITL is not v8 evidence |
| Real calibrated imagery and field flight | No campaign | **Not executed** |

## Measured results

Across the full 1,320-attempt campaign, **1,080** reached qualified B arrival and **540** reported the survey complete. Those totals include 120 deliberately dishonest-sensor controls. In the **1,200 honest-sensor attempts**, 960 reached qualified B arrival, 420 reported the survey complete, and independent truth found 540 complete. There were zero false *reported* completions in this honest subset; N03 and N15 account for 120 conservative false negatives across both policies. In the **120 dishonest controls** N19/N20, all 120 arrived and reported complete despite independently incomplete truth. These controls show why adapter identity and metadata checks cannot establish physical image truth.

**Denominator correction (30 October 2026):** the frozen `analysis/summary.json` has a full-campaign `attempts: 1320` field, while each `by_policy` block deliberately counts only its `honest_attempts: 600`. The previous reader and prose incorrectly displayed the honest-subset totals 960 and 420 over the full-campaign denominator 1,320. The corrected reader derives both full and honest-subset totals directly from the unchanged `per_attempt.csv`. The frozen raw records, per-condition estimates, paired comparisons, figure values and SHA-256 manifest were not altered.

Only N07–N09 contain jointly successful, recovery-triggered VECR/B2 distance pairs. All 30 pairs per case achieved true and reported completion under both policies. VECR−B2 distance (m), with pointwise 95% paired bootstrap intervals: N07 **−24.933 [−24.972, −24.894]**, N08 **−25.219 [−25.278, −25.162]**, N09 **−23.478 [−23.531, −23.424]**. These are conditional simulator flight-cost effects, not evidence of superior completion probability or universal recovery optimality. Endpoint-gap cases arrive with unresolved intervals and zero endpoint-repair commands. N11–N14 fail explicitly under their command, motor, energy or budget fault controls. All 44 rows and unsuccessful outcomes are retained in `primary/analysis/RESULTS.md` and `condition_summary.csv`.

## Figure and table provenance

| Active dissertation item | Generating records and method |
|---|---|
| Full completion matrix | `primary/per_attempt.csv` → `analyze_digital.py` → `primary/analysis/completion_matrix.svg` |
| Paired distance chart | Same raw attempts, common seeds and joint-success filter → `paired_distance.svg` |
| All-condition table | `primary/analysis/condition_summary.csv` copied by site builder |
| N07–N09 effect table | `primary/analysis/paired_comparisons.csv` copied by site builder |
| Recorded replay | Original `trace.csv.gz` and `events.csv.gz` for seed 70001 in N00/N03/N07/N11/N19 → `assets/experiment-data.js` |
| Reuse parity | `reuse-qualified/per_attempt.csv` → `analyze_reuse.py` → `reuse_results.json` |

Figures, condition tables, replay rows and the reader itself are regenerated by `VECR_Dissertation_Interactive_Website/tools/build_site.py` from the committed v8 records. SHA-256 inventories are in each frozen campaign and the website. The conceptual animations are explanatory drawings, not recorded trajectories. Wilson and bootstrap intervals are pointwise for this simulator design; they do not quantify real UAV fleet reliability. The raw results are the definitive record, including failure rows and failed infrastructure attempts.
