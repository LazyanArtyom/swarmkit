# SwarmKit scientific context

SwarmKit is the onboard UAV execution and communication SDK for the implemented
Verified Edge Coverage and Recovery (VECR) method. An external graph algorithm
assigns a directed edge. The onboard agent distinguishes destination arrival,
bounded route traversal, and accepted survey evidence for the assigned strip;
it reports missing regions and may request bounded in-flight re-entry. It does not
implement rotor-router, gossip, mission planning, or cloud correction.

The active v8 evidence is the agent-backed geometric campaign in
[`results/vecr/v8`](../results/vecr/v8). Twelve historical v7 ArduPilot SITL
flights used the retired endpoint policy; revised-policy SITL and real camera
coverage remain untested. The [method](vecr/METHOD.md) states the conditional
assumptions, and [current validation](vecr/README.md) identifies open work.
