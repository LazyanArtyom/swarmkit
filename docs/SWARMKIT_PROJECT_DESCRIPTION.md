# SwarmKit project description

SwarmKit is a C++23 UAV agent and SDK. It provides authority-controlled flight
commands, normalized telemetry with explicit validity and uncertainty meaning,
health/capability reports, typed goal and VECR edge events, peer messages,
and chunked artifact transport over configurable secure gRPC connections.
Vehicle-specific commands remain behind `IDroneBackend`; the built-in simulator
and MAVLink UDP backend are retained.

The VECR edge supervisor runs inside the agent. It accounts for arrival,
bounded route traversal, accepted strip-survey intervals, missing sections,
and bounded recovery in qualified simulation backends. A trusted sensor adapter must justify
conservative full-strip capture intervals. There is no validated real camera
adapter. Agent-backed digital simulation and separate ArduPilot SITL qualification
use explicit synthetic sensing; neither establishes physical image quality.
See [the method](vecr/METHOD.md) and [current validation](vecr/README.md).

External applications own graph traversal, rotor updates, gossip policy,
fleet scheduling, mission design, and multi-user cloud workflows.
