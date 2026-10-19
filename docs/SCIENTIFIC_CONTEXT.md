# SwarmKit scientific context

SwarmKit is the onboard UAV execution and communication SDK for the proposed
Verified Edge Coverage and Recovery (VECR) method. An external graph algorithm
assigns a directed edge. The onboard agent distinguishes destination arrival,
bounded route traversal, and accepted survey evidence for the assigned strip;
it reports missing regions and may request safe simulator re-entry. It does not
implement rotor-router, gossip, mission planning, or cloud correction.

The archived prior state-acceptance research and its measurements are separate
from VECR. Current VECR evidence is the modeled geometric campaign in
[`results/vecr`](../results/vecr); SITL and real camera coverage remain untested.
The [method and correctness notes](vecr/METHOD_AND_CORRECTNESS.md) state the
conditional assumptions, and [implementation progress](vecr/IMPLEMENTATION_PROGRESS.md)
lists the remaining validation work.
