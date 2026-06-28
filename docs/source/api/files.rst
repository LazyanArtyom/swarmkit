Public Header Files
===================

Client headers
--------------

``include/swarmkit/client/client.h``
   Single-agent gRPC client, subscriptions, authority sessions, goals,
   reports, data messages, peers, artifacts, and client YAML configuration.

``include/swarmkit/client/swarm_client.h``
   Fleet-level client, swarm topology loading, bounded fanout, per-drone
   routing, all-drone telemetry, and all-drone reports.

Command headers
---------------

``include/swarmkit/commands.h``
   Aggregates the public command model.

``include/swarmkit/commands/flight.h``
   Fundamental vehicle commands: arm, disarm, takeoff, land, modes, and
   emergency actions.

``include/swarmkit/commands/nav.h``
   Navigation and guided-control commands: waypoint, goto, speed, yaw,
   velocity, pause/resume, home, and return-home.

``include/swarmkit/commands/payload.h``
   Payload commands for cameras, video, gimbals, ROI, servo, relay, and
   gripper integrations.

``include/swarmkit/commands/backend.h``
   Backend-specific extension command escape hatch.

Agent headers
-------------

``include/swarmkit/agent/server.h``
   Agent server configuration, security settings, vehicle profile, data-plane
   settings, and server entry point.

``include/swarmkit/agent/backend.h``
   Public drone backend interface, health model, and capabilities model.

``include/swarmkit/agent/backend_factory.h``
   Backend registry and built-in backend registration.

``include/swarmkit/agent/sim_backend.h``
   Built-in simulator backend factory.

``include/swarmkit/agent/mavlink_backend.h``
   Direct MAVLink UDP backend configuration and factory.

``include/swarmkit/agent/arbiter.h``
   Command authority arbiter and authority event queue.

Core headers
------------

``include/swarmkit/core/telemetry.h``
   Normalized telemetry frame, validity flags, GPS quality, estimator state,
   accuracy metadata, and home origin.

``include/swarmkit/core/result.h``
   Stable error domains/codes and result types used across the SDK.

``include/swarmkit/core/security.h``
   Transport security mode parsing and string conversion.

``include/swarmkit/core/logger.h``
   Thread-safe global logger facade and pluggable backend interface.
   :no-link:
