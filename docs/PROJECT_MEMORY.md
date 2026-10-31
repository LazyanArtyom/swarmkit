# SwarmKit project memory

Current product direction: onboard VECR, exact mission/edge/attempt identity,
explicit unknown evidence, bounded recovery through authority and backend
checks, and policy-neutral client/peer transport. `GOAL_REACHED` is a waypoint
fact; `EDGE_ARRIVED` is a physical endpoint result with separately reported
survey and traversal verification. In-flight deviation can trigger bounded
local recovery; arrival at B never triggers automatic backward repair. The
agent-backed simulator is a research model, and revised-policy ArduPilot SITL
and real camera validation remain pending.

Use [README](../README.md), [architecture](source/architecture.md),
[method](vecr/METHOD.md), [current validation](vecr/README.md), and
[traceability](vecr/TRACEABILITY.md) for the maintained details.
