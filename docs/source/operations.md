# Operations and release runbook

This runbook covers deployment and incident handling for the SwarmKit Agent and SDK. It does not
certify a vehicle, authorize flight, or replace the flight-controller manufacturer's procedures.
An operator must keep an independent means to make the vehicle safe.

## Deployment prerequisites

Before a release is installed:

1. Require green Debug, Release, sanitizer, static-analysis, protocol-lint, package-consumer, and
   documentation jobs for the exact commit being deployed.
2. Record the SwarmKit version, source commit, Agent configuration, vehicle/autopilot version, and
   calibration profile in the deployment record.
3. Use mTLS with a deployment-specific CA and client allow-list. The repository certificates and
   `--insecure` mode are for local development only.
4. Store private keys outside the release archive with access restricted to the Agent service
   account. Do not place secrets in command-line arguments, logs, evidence labels, or source control.
5. Give each Agent a stable unique `agent_id`, each vehicle a stable `drone_id`, and each process a
   unique gRPC and MAVLink bind address.
6. Put artifacts, rotating operational logs, and execution evidence on monitored persistent
   storage. Select the recorder loss policy deliberately; scientific runs should use
   `invalidate_run`.

Deploy immutable SDK and tools archives into a versioned directory and switch a service-manager
symlink only after staging checks pass. Keep the previous version and configuration available for
rollback.

## Staging checks

Run these checks with the production client credentials and configuration before command authority
is granted:

```bash
swarmkit-cli --config /etc/swarmkit/client.yaml ping
swarmkit-cli --config /etc/swarmkit/client.yaml capabilities
swarmkit-cli --config /etc/swarmkit/client.yaml health
swarmkit-cli --config /etc/swarmkit/client.yaml preflight --drone drone-1
swarmkit-cli --config /etc/swarmkit/client.yaml stats
```

Treat an unknown armed, landed, failsafe, GPS, or estimator state as a failed readiness check. Do
not bypass this with `--allow-unsafe-bench-commands` outside a restrained, propeller-off bench test.
For a MAVLink deployment, also prove heartbeat freshness, telemetry freshness, target system and
component identity, mode mapping, and command-ACK behavior against the exact autopilot release.

## Starting and supervising the Agent

Run one Agent under the platform service manager; do not daemonize it inside a shell script:

```bash
/opt/swarmkit/current/bin/swarmkit-agent \
  --config /etc/swarmkit/agent.yaml \
  --log-sink both \
  --log-file /var/log/swarmkit/agent.log \
  --log-level info
```

The service manager should restart on unexpected failure with bounded backoff, preserve the exit
status, and send `SIGTERM` for normal shutdown. Repeated crashes, recorder failures, stale telemetry,
or an unexpected Agent session change require operator intervention rather than an unbounded restart
loop.

After every start, record the new `agent_session_id` from health/telemetry evidence. Consumers must
discard old-session cursors and evidence; a successful reconnect does not make an old session
current.

## Routine monitoring

Monitor at least:

- `health` and `swarm health` readiness failures and arming blockers;
- heartbeat and telemetry age;
- backend failures, failed commands, active streams, callback drops, and reconnect counters from
  `stats` or `swarm stats`;
- evidence-recorder capacity and write failures;
- log rotation, filesystem capacity, certificate expiry, and system clock health;
- Agent session changes and repeated authority preemption.

Alerts must retain the Agent id, drone id, session id, correlation id, event time, and error code.
Avoid recording credentials, private keys, raw certificate contents, or payload data that is not
needed for diagnosis.

## Controlled shutdown

1. Stop issuing commands and release command authority.
2. Stop telemetry/report consumers and wait for their owned subscription handles to finish.
3. Send `SIGTERM` to the Agent and wait for a clean process exit. Escalate to a forced kill only
   under the deployment's safety procedure.
4. Preserve the evidence file, Agent log, configuration snapshot, and software versions before
   moving or rotating the run directory.
5. Validate evidence with `swarmkit-evidence-inspect` before declaring a scientific run usable.

`StopTelemetry()` drains in-flight backend callbacks before returning. A process kill bypasses that
contract and can leave the last operational log or evidence record incomplete.

## Incident response

When health becomes unknown, stale, or failsafe-active, stop autonomous command issuance and defer
to the vehicle safety procedure. Preserve evidence before restarting. A restart creates a new Agent
session and must never be used to reinterpret pre-restart telemetry as current.

For command ambiguity, use the exact correlation id and execution handle to inspect command reports,
goal state, backend protocol responses, and recorded telemetry. An ACK proves protocol acceptance,
not physical arrival. Do not infer success from the absence of an error or from a reconnect.

For recorder loss, corruption, or an `invalidate_run` event, mark the scientific run invalid. Do not
fill gaps from console logs or silently continue the same run id.

## Upgrade and rollback

SwarmKit is pre-release and may make deliberate C++ and protobuf source changes. Upgrade Agent,
client, and external consumers as one tested release set unless a compatibility policy explicitly
says otherwise.

1. Drain command authority and shut down the old Agent cleanly.
2. Preserve the old binary, configuration, evidence, and logs.
3. Install the new version beside the old version and run the staging checks.
4. Start a new run id and accept the new Agent session explicitly.
5. Roll back by stopping the new Agent, restoring the previous version and matching configuration,
   and repeating staging checks. Never reuse the new process's session id or replay cursor.

## Release checklist

A release candidate is publishable only when:

- the production-readiness checklist has no undisposed release blockers;
- every supported-platform CI job is green for the release commit;
- the Conan export, relocatable archives and SHA-256 sidecars, external CMake consumer, and staged
  Agent ping pass;
- public headers, protobuf changes, examples, and operator documentation agree;
- version and release notes identify breaking changes and known limitations;
- no repository test key, local path, generated output, or temporary artifact is packaged;
- an operator has approved the security configuration, storage policy, rollback plan, and residual
  limitations.

Passing this checklist establishes a software release gate only. Vehicle-specific validation and
operational approval remain separate.
