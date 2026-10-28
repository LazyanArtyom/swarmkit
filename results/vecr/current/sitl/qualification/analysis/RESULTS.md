# ArduPilot SITL current: actual qualification results

12/12 assignments returned records; all passed independent sampled-truth reconstruction.
Two flights per cell; no broad reliability confidence interval is inferred.
Native navigation accuracy remains backend-specific. Positive acceptance uses synthetic navigation and ideal camera observations.

| Observation tier / fault | Policy | Reported / 2 | Sampled truth / 2 | Mean travel (m) | Mean duration (s) |
|---|---|---:|---:|---:|---:|
| raw_mavlink / none | VECR | 0 | 2 | 20.347 | 25.050 |
| synthetic_bounded / none | VECR | 2 | 2 | 20.109 | 12.143 |
| synthetic_bounded / delay | VECR | 2 | 2 | 20.165 | 13.018 |
| synthetic_bounded / multi_gap | VECR | 2 | 2 | 55.086 | 36.322 |
| synthetic_bounded / command_rejected | VECR | 0 | 0 | 20.201 | 15.100 |
| synthetic_bounded / land_override | VECR | 0 | 0 | 8.880 | 5.837 |

Accepted captures checked: 509; sampled re-entry dwells checked: 2.

Costs in failed/interrupting conditions describe actual execution; they are not efficiency gains.
The <=50 ms upstream SIM_STATE age assumption is unproven because source acquisition time is absent. Stop dwell is sampled, not continuous. Camera geometry is ideal, flat and route-aligned; no actual imagery is validated.
