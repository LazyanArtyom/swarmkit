# VECR revised evidence status — 2026-10-29

The revised method has **not yet been implemented, rebuilt or rerun** within this website revision. Below are the only historical v7 distance comparisons shown in the active reader, selected because their recorded `midflight_repairs` field equals 1 in every v7 N07–N09 policy attempt. These are *historical observations*, not validation of the new endpoint-only reporting behavior.

| Condition | Matched successes | B2 mean distance | v7 VECR mean distance | VECR − B2, 95% paired CI |
|---|---:|---:|---:|---:|
| N07 sustained departure | 30/30 | 79.58 m | 54.64 m | −24.93 [−24.97, −24.90] m |
| N08 departure, wide FOV | 30/30 | 79.58 m | 54.36 m | −25.22 [−25.28, −25.16] m |
| N09 departure, narrow FOV | 30/30 | 79.58 m | 56.10 m | −23.48 [−23.53, −23.42] m |

The original 22-condition completion matrix and endpoint-gap distance comparisons are archived in `legacy_v7/` and deliberately omitted from the revised result argument. They used the retired after-B automatic return.

To claim new-policy effects, rerun an updated C++ agent/SDK and compute arrival/coverage-state rates, residual gap distributions, detection and physical re-entry latencies, distance overhead, safety/failure outcomes, callback exactly-once behavior, and absence of any correction command at B. Record every failed run and use independent ground truth. The historical 100-client and 12-SITL runs must be requalified for the new endpoint semantics.
