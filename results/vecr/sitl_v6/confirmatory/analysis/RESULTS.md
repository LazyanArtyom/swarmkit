# ArduPilot SITL v6: actual qualification results

24/24 assignments returned records; all passed independent sampled-truth reconstruction.
Two flights per cell; no broad reliability confidence interval is inferred.
Native navigation accuracy remains backend-specific. Positive acceptance uses synthetic navigation and ideal camera observations.

| Observation tier / fault | Policy | Reported / 2 | Sampled truth / 2 | Mean travel (m) | Mean duration (s) |
|---|---|---:|---:|---:|---:|
| raw_mavlink / none | GW | 0 | 2 | 20.362 | 25.072 |
| raw_mavlink / none | E | 0 | 2 | 20.329 | 25.043 |
| synthetic_bounded / none | GW | 2 | 2 | 20.124 | 12.248 |
| synthetic_bounded / none | E | 2 | 2 | 20.090 | 12.061 |
| synthetic_bounded / delay | GW | 2 | 2 | 20.169 | 13.059 |
| synthetic_bounded / delay | E | 2 | 2 | 20.161 | 13.188 |
| synthetic_bounded / multi_gap | GW | 2 | 2 | 55.490 | 36.520 |
| synthetic_bounded / multi_gap | E | 2 | 2 | 60.626 | 39.248 |
| synthetic_bounded / command_rejected | GW | 0 | 0 | 20.196 | 15.032 |
| synthetic_bounded / command_rejected | E | 0 | 0 | 20.217 | 15.181 |
| synthetic_bounded / land_override | GW | 0 | 0 | 8.935 | 5.861 |
| synthetic_bounded / land_override | E | 0 | 0 | 8.940 | 5.854 |

Accepted captures checked: 1058; sampled re-entry dwells checked: 4.

Costs in failed/interrupting conditions describe actual execution; they are not efficiency gains.
The <=50 ms upstream SIM_STATE age assumption is unproven because source acquisition time is absent. Stop dwell is sampled, not continuous. Camera geometry is ideal, flat and route-aligned; no actual imagery is validated.
