# Agent-backed VECR simulator results

## Experimental goal

Evaluate truthfulness and recovery cost for a directed A→B surveying obligation.

## System under test

SDK → TLS AgentService → production edge runtime → SimBackend; synthetic camera and private 0.25 m truth oracle.

## Baselines and fairness

B2 full-edge restart and P targeted-gap repair share the same verifier, camera, dynamic vehicle, safety bounds, timeout and retry limit. Only repair re-entry policy differs.

## Scenarios

12 comparative cases × 3 seeds × 2 policies = 72 actual agent-backed attempts. S13, S14 and S17 remain unsupported; other noncomparative cases are excluded from these statistics.

## Ground-truth design

Private simulator poses and actual camera acquisition feed a separate 0.25 m along-edge cell oracle. A cell counts only when the modeled camera footprint spans the full required strip width. This is not a two-dimensional terrain-image oracle. The agent receives normalized telemetry and adapter metadata only.

## Metrics

Distance integrates true 20 ms physical steps. Time is simulated mission-clock duration. True edge completion requires both independent full-strip survey and true physical arrival at B. False survey completion is FULFILLED with at least one required truth cell missing; false edge completion additionally includes absence of true arrival. All assigned trials stay in denominators.

## Results (actual values with uncertainty)

| Scenario | n pairs | B2 true edge/reported/false survey | P true edge/reported/false survey | B2 mean m | P mean m | Δ P−B2 m (95% paired CI) | Δ time s (95% paired CI) |
|---|---:|---|---|---:|---:|---|---|
| S00 | 3 | 3/3/0 | 3/3/0 | 20.040 | 20.040 | 0.000 [0.000, 0.000] | 0.000 [0.000, 0.000] |
| S01 | 3 | 3/3/0 | 3/3/0 | 20.040 | 20.040 | 0.000 [0.000, 0.000] | 0.000 [0.000, 0.000] |
| S02 | 3 | 0/0/0 | 0/0/0 | 47.384 | 47.384 | 0.000 [0.000, 0.000] | 0.000 [0.000, 0.000] |
| S03 | 3 | 3/3/0 | 3/3/0 | 62.022 | 45.623 | -16.399 [-16.400, -16.398] | -8.200 [-8.200, -8.200] |
| S04 | 3 | 3/3/0 | 3/3/0 | 60.198 | 49.132 | -11.066 [-12.398, -10.400] | -5.533 [-6.200, -5.200] |
| S05 | 3 | 3/3/0 | 3/3/0 | 60.198 | 49.132 | -11.066 [-12.398, -10.400] | -5.533 [-6.200, -5.200] |
| S06 | 3 | 3/3/0 | 3/3/0 | 23.848 | 23.848 | 0.000 [0.000, 0.000] | 0.000 [0.000, 0.000] |
| S07 | 3 | 3/3/0 | 3/3/0 | 60.500 | 55.380 | -5.120 [-5.120, -5.120] | -2.000 [-2.000, -2.000] |
| S08 | 3 | 3/0/0 | 3/0/0 | 60.200 | 45.800 | -14.400 [-14.400, -14.400] | 0.000 [0.000, 0.000] |
| S09 | 3 | 3/1/0 | 3/0/0 | 60.199 | 45.133 | -15.066 [-16.400, -14.398] | 9.067 [0.000, 27.200] |
| S16 | 3 | 0/0/0 | 0/0/0 | 20.040 | 20.040 | 0.000 [0.000, 0.000] | 0.000 [0.000, 0.000] |
| S19 | 3 | 0/0/0 | 0/0/0 | 20.040 | 20.040 | 0.000 [0.000, 0.000] | 0.000 [0.000, 0.000] |

Each true-edge/reported/false-survey entry is a count of assigned attempts, not a conditional success-only mean. Wilson intervals, independent truth-survey counts and false-edge-completion counts are in `tables/methods_summary.csv`; all paired effect intervals are in `tables/paired_comparisons.csv`.
Both arms use the same evidence acceptance rules. They recorded B2 0/36 and P 0/36 false survey completions in the comparative campaign. A zero-event scenario with 3 attempts still has a Wilson 95% upper bound of 56.150%; zero observed failures is not proof of zero risk or a verification-accuracy advantage for P.
Across the fixed 12-scenario set, the descriptive macro mean P−B2 distance is -6.093 m (95% seed-cluster bootstrap [-6.260, -5.927]); the corresponding relative travel saving is 14.206% of B2 mean assigned-task distance. This includes failed tasks and is not a recovery-efficiency claim where completion differs.

### Separate noncomparative diagnostics

| Scenario | Variant | Reported | True survey | False complete | Fault realized |
|---|---|---|---|---|---|
| S10 | fast_fly_through | FULFILLED | 1 | 0 | 0 |
| S11 | detectable_bad_metadata | FULFILLED | 1 | 0 | 1 |
| S11 | dishonest_camera | FULFILLED | 0 | 1 | 1 |
| S12 | safety_battery | INTERRUPTED | 0 | 0 | 1 |

These single diagnostic attempts are not probability estimates and are not pooled with the B2/P comparison.

## Tradeoffs/negative results

- S02: true edge completion B2 0/3, P 0/3; reported fulfillment B2 0/3, P 0/3. Shorter interrupted travel does not establish superior recovery.
- S08: true edge completion B2 3/3, P 3/3; reported fulfillment B2 0/3, P 0/3. Shorter interrupted travel does not establish superior recovery.
- S09: true edge completion B2 3/3, P 3/3; reported fulfillment B2 1/3, P 0/3. Shorter interrupted travel does not establish superior recovery.
- S16: true edge completion B2 0/3, P 0/3; reported fulfillment B2 0/3, P 0/3. Shorter interrupted travel does not establish superior recovery.
- S19: true edge completion B2 0/3, P 0/3; reported fulfillment B2 0/3, P 0/3. Shorter interrupted travel does not establish superior recovery.

Persistent faults can cause both policies to fail. A plausible dishonest camera can create false fulfillment, outside honest-adapter assumptions. The S08/S09 travel contrasts must be read beside their reported availability and should not be framed as pure efficiency benefits when completion differs.

## Threats to validity

The simulator has simplified horizontal acceleration, instant modeled camera acquisition, a one-dimensional full-strip coverage oracle and a terminal-sample true-arrival check; it has perfect base telemetry and no terrain occlusion. Location-tied interventions may have policy-dependent realized duration. Scenario seeds vary preregistered gap position/length and wind strength but do not sample a full field distribution. S15 recreates AgentService in one host process with a surviving simulator; an OS process power loss was not tested. No electrical energy or CPU/RSS benchmark is reported.

## Reproducibility

Trial source revision `e6253f9194cb2209cbfaf128adb772bcaae7cad8`; analysis revision `5adb6872f4b593cdd21ab2ce5c63063814a03b7f`; binary SHA-256 `888652952c521cb0860d0902510d4b46d422231843108226620b2669dd8fb8b6`; config SHA-256 `f72d23b81e15de7010f1ca5cb9f30bfe34377988559c27d0f1a193000c69d516`. See manifest for host, commands and artifact hashes.

## What is not demonstrated

Only simulator evidence has been obtained. SITL, real camera calibration, and physical flights are not validated. Remote transport-loss replay, peer interruption, OS process crash/power loss and simultaneous multi-UAV throughput are not validated by this campaign.
