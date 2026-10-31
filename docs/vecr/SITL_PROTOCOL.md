# ArduPilot SITL qualification — historical v7 protocol

This document records the completed v7 qualification protocol. The current v8
method stops local correction at qualified B arrival, so these flights cannot
validate its revised terminal behavior. No v8 SITL figures are available; they
remain TBD until a new versioned campaign runs the revised binary and validator.

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

The archived run was written under `results/vecr/current/sitl/qualification`.
Do not reuse that output path or its statistics for v8. A future v8 protocol
must first verify that its SITL runner exercises the revised agent API, then
record a sustained departure before B, bounded stopped re-entry, B arrival
with an unresolved strip and no post-B return command, refusal and safety
failures, exact attempt identity, source/binary hashes and independent truth.
