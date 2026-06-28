# Architecture

SwarmKit is organized around a clean boundary between SDK clients, an agent
process, and pluggable vehicle backends.

```text
Application / CLI / test tool
        |
        | swarmkit::client::Client or SwarmClient
        v
    gRPC channel
        |
        v
swarmkit-agent process
  - AgentService RPC implementation
  - CommandArbiter
  - TelemetryManager
  - ReportHub
  - data message and artifact services
        |
        | swarmkit::agent::IDroneBackend
        v
Vehicle backend
  - simulator
  - direct MAVLink UDP
  - registered custom backend
```

## Main Modules

`include/swarmkit/core`
: Common result/error types, telemetry model, transport-security helpers, and
  the global logger facade.

`include/swarmkit/commands*.h`
: Public command model. Commands are protocol-neutral C++ variants.

`include/swarmkit/client`
: Single-agent `Client`, fleet-level `SwarmClient`, configuration loaders,
  subscription handles, goals, reports, messages, peers, and artifacts.

`include/swarmkit/agent`
: Public agent-side configuration, backend interface, backend registry,
  built-in backend factories, and command authority arbiter.

`src/agent`
: Agent server implementation, telemetry lifecycle management, report hub,
  readiness checks, runtime counters, and backend implementations.

`apps/agent` and `apps/cli`
: Command-line entry points and option parsing around the SDK libraries.

`proto`
: gRPC service definitions and generated protocol code.

## Command Flow

```text
Client::SendCommand(envelope)
        |
        v
AgentService::SendCommand RPC
        |
        v
CommandArbiter::CheckAndGrantDetailed()
        |
        v
command preconditions and readiness checks
        |
        v
IDroneBackend::Execute(envelope)
        |
        v
CommandResult and AgentReport events
```

The `CommandEnvelope` carries both the command payload and context. The context
includes `drone_id`, `client_id`, priority, deadline, and correlation ID.

## Telemetry Flow

```text
Backend telemetry producer thread
        |
        v
TelemetryManager cache and subscriber state
        |
        v
Agent StreamTelemetry RPC
        |
        v
Client subscription dispatcher
        |
        v
Application callback
```

The client dispatches callbacks through a bounded queue. Configure
`SubscriptionOptions::backpressure` when slow application callbacks should block,
drop newest callbacks, or drop oldest callbacks.

## Concurrency Model

- `Client` unary RPCs are thread-safe.
- Stream lifecycle methods should not be called concurrently with themselves.
- `SwarmClient` public methods are thread-safe and snapshot registered clients
  before fanout.
- `IDroneBackend` methods may be called from any thread.
- Backend telemetry callbacks are invoked from backend-owned threads and must
  return quickly.

## Transport Security

SwarmKit supports `auto`, `insecure`, `tls`, and `mtls` transport modes. Empty
security config resolves to insecure transport. Providing a root CA on the
client enables TLS; providing root CA, client certificate, and client key enables
mTLS. The agent can enforce certificate-bound client identities with
`allowed_client_ids`.

## Error Model

Public APIs return result structs and `swarmkit::core::SwarmError` details.
Errors include a domain, stable code, severity, retryability, user/debug
messages, optional backend details, remediation text, and correlation IDs.
