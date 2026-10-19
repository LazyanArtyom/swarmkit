# VECR core strip simulation — measured results

Protocol: `core-strip-v1`; seeds 1–5; 8 coded scenarios; 160 paired method trials.

| Method | Complete reports | False completions | FCR among reports | Truth complete | Mean distance | Unnecessary interventions |
|---|---:|---:|---:|---:|---:|---:|
| B0 | 40/40 | 15 | 37.5% | 25 | 100.0 m | 0 |
| B1 | 30/40 | 10 | 33.3% | 25 | 100.0 m | 0 |
| B2 | 40/40 | 0 | 0.0% | 40 | 200.0 m | 5 |
| P | 40/40 | 0 | 0.0% | 40 | 161.0 m | 5 |

FCR denominator is reported-complete attempts. An empty denominator is N/A. The truth evaluator rasterizes actual modeled captures at 0.25 m; VECR sees only capture metadata and observed position. B2 and P share camera evidence and have equal modeled false-completion performance in this matrix. S05 is a metadata loss case: the terrain was truly observed, but B2/P repeat it because that evidence was unavailable.

| Scenario | B0 FCR | B1 FCR | B2 FCR | P FCR | B2 distance | P distance |
|---|---:|---:|---:|---:|---:|---:|
| S00 | 0.0% | 0.0% | 0.0% | 0.0% | 100.0 m | 100.0 m |
| S01 | 0.0% | 0.0% | 0.0% | 0.0% | 100.0 m | 100.0 m |
| S03 | 100.0% | N/A | 0.0% | 0.0% | 300.0 m | 222.0 m |
| S04 | 100.0% | 100.0% | 0.0% | 0.0% | 300.0 m | 222.0 m |
| S05 | 0.0% | 0.0% | 0.0% | 0.0% | 300.0 m | 222.0 m |
| S06 | 0.0% | N/A | 0.0% | 0.0% | 100.0 m | 100.0 m |
| S07 | 0.0% | 0.0% | 0.0% | 0.0% | 100.0 m | 100.0 m |
| S11 | 100.0% | 100.0% | 0.0% | 0.0% | 300.0 m | 222.0 m |

These are deterministic structural demonstrations, not independent field flights or statistical evidence for a population of UAVs. Eight of the requested 21 scenario IDs are implemented. Recovery is a modeled route replay in this experiment, not a measured backend/autopilot navigation outcome. The sensor quality predicate is supplied by a simulated adapter; no real imagery quality or SITL independent truth has been validated.
