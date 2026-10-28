# ArduPilot SITL qualification — canonical VECR, edition 7

Freeze source and binary before executing six conditions × two seeds
73001/73002 = 12 flights, all using the single production VECR method:

1. Native backend accuracy, nominal route: unfulfilled because hard bounds are absent.
2. Synthetic bounded navigation and ideal sensing, nominal route.
3. Synthetic observations, delayed metadata.
4. Synthetic observations, multiple gaps and physical re-entry.
5. Synthetic observations, recovery-command refusal: interrupted/unfulfilled.
6. Synthetic observations, LAND override: no subsequent GOTO, unfulfilled.

Use isolated native ArduPilot Copter processes and localhost ports, real production
MAVLink command handling, declared synthetic observation adapters and independent
SIM_STATE records. Arming checks remain enabled. Record exact ArduPilot commit,
binary and agent hashes, parameters, raw MAVLink, backend commands, truth,
observations, acquisitions, submitted captures, agent states and outcomes.

Check independent footprint containment, sampled stop/re-entry dwell and safety
intervention. Report all assignments and unsuccessful pilots. Two flights per
condition are integration qualification; do not infer a population confidence
interval or field reliability from them.

Positive sensing is an ideal flat, route-aligned camera and synthetic bounded
navigation. SIM_STATE lacks an acquisition timestamp: upstream age <=50 ms remains
an assumption, combined with monitored receipt age <=100 ms and physical horizontal
speed bound 10 m/s. Sampled dwell is not proof of continuous dwell. Native GNSS
accuracy estimates remain backend-specific; production MAVLink recovery stays
disabled. No calibrated image-quality or physical-flight claim follows.

Run `.venv/bin/python scripts/vecr/run_sitl.py --mode final --output
results/vecr/current/sitl/qualification` after committing experiment source.
Validate with `validate_sitl.py` and analyze with `analyze_sitl.py`. New executions
must use new directories. Optional pilots are diagnostic and remain separate.
