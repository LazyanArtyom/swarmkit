# FAQ

## Is SwarmKit A Flight Controller?

No. SwarmKit is an SDK, gRPC agent, and tooling layer. It sends commands to a
backend that talks to a simulator, MAVLink autopilot, or custom vehicle system.

## Which Backend Should I Use First?

Use the simulator backend for local SDK and CLI development. Use the MAVLink
backend when testing with ArduPilot SITL or a companion-computer deployment that
provides MAVLink UDP traffic.

## Is Insecure Transport Safe For Production?

No. `--insecure` is for local development and isolated tests. Use TLS or mTLS
for real deployments, and configure `allowed_client_ids` when the agent should
limit command access to known client certificates.

## What Is The Default Agent Address?

The agent defaults to `0.0.0.0:50061`. The client defaults to
`127.0.0.1:50061`.

## How Do I Know Whether A Telemetry Field Is Real?

Check the validity helpers or flags. For example, call
`TelemetryFrame::HasPosition()` before using latitude and longitude, and
`TelemetryFrame::HasBattery()` before using battery percentage.

## How Do I Control Multiple Drones?

Use `SwarmClient` in C++ or the CLI `swarm` command with a YAML file containing
`swarm.drones`. Each drone entry maps a logical `drone_id` to an agent address.

## What Happens If Two Clients Send Commands?

The agent's `CommandArbiter` grants authority based on priority. A higher
priority client can preempt a lower priority holder. Authority can be released
explicitly or expire by TTL.

## Can I Add A Custom Backend?

Yes. Implement `swarmkit::agent::IDroneBackend`, register a creator in
`BackendRegistry`, and select it by backend name. The backend receives typed
command envelopes and produces normalized telemetry frames.

## Are Payload Commands Fully Implemented?

The public command model includes payload commands for cameras, video, gimbals,
ROI, servo, relay, and gripper operations. Actual behavior depends on backend
support. A backend with no payload hardware should reject payload commands as
unsupported.

## Why Are There Both Client And CLI APIs?

The CLI is an operator/developer tool built on the SDK. Application code should
use `Client` or `SwarmClient` directly when embedding SwarmKit into a larger C++
system.
