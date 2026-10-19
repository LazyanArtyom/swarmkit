# VECR v3 agent-backed integration pilot

Two executed attempts use Client SDK → AgentService → EdgeExecutionRuntime → SimBackend in manual-clock mode. The private `SimBackendControl::Truth` channel drives the independent raster check and physical-distance measurement. The synthetic camera producer uses simulator truth to model calibrated pose; this does not validate a real camera. `reported_phase` encodes MOVING=0, RECOVERING=1, FULFILLED=2, INTERRUPTED=3, UNKNOWN=4.

The camera-gap attempt tests completed onboard re-entry and continuation. The motor-failure attempt records a failed recovery. These are one run per scenario, with no fair B2 comparator or uncertainty interval. They are not the planned 21-scenario campaign, ArduPilot SITL, or hardware evidence. The results support only the observed software integration behavior.
