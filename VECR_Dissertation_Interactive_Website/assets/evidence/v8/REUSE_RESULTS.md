# Frozen v8 client-path reuse audit

Source revision: `8b0ad012140d1d1849990fe4aca9936c18497100`. Agent-backed attempts: **100**, paired direct/routed seeds: **50**.

Exact differences across 10 predeclared result fields: **0**.

| Case | Direct terminal | Routed terminal | Matched pairs |
|---|---|---|---:|
| N00 | {'ARRIVED': 10} | {'ARRIVED': 10} | 10 |
| N01 | {'ARRIVED': 10} | {'ARRIVED': 10} | 10 |
| N04 | {'ARRIVED': 10} | {'ARRIVED': 10} | 10 |
| N07 | {'ARRIVED': 10} | {'ARRIVED': 10} | 10 |
| N11 | {'FAILED': 10} | {'FAILED': 10} | 10 |

This tests two SDK transport entry paths to the same agent method. It does not test two independent graph algorithms or multi-UAV scheduling.
The earlier sandbox-blocked 100-attempt run is preserved separately under `results/vecr/v8/reuse/` as an unsuccessful infrastructure attempt.
