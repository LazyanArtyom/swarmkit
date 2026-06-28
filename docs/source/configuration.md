# Configuration

SwarmKit configuration can come from defaults, YAML files, environment
variables, and command-line overrides. CLI options generally override YAML.

## Agent YAML

The agent loader accepts either a root-level map or an `agent:` section.

```yaml
agent:
  agent_id: "agent-1"
  bind_addr: "0.0.0.0:50061"
  backend: "sim"
  default_authority_ttl_ms: 5000
  default_telemetry_rate_hz: 5
  min_telemetry_rate_hz: 1
  report_log_file: "/tmp/swarmkit-agent-1-reports.jsonl"
  vehicle_profile:
    profile_id: "generic-quad"
    cruise_speed_mps: 4.0
    climb_speed_mps: 1.5
    descent_speed_mps: 1.0
    max_altitude_m: 120.0
    min_battery_percent: 20.0
    battery_reserve_percent: 15.0
    tracking_tolerance_m: 2.0
    goal_margin_ms: 15000
    takeoff_timeout_margin_ms: 20000
    land_timeout_margin_ms: 30000
    max_goal_timeout_ms: 300000
    min_gps_fix_type: 3
    min_satellites_visible: 6
    max_gps_hdop: 2.5
  security:
    transport_security: "mtls"
    root_ca_cert_path: "certs/ca.pem"
    cert_chain_path: "certs/agent.pem"
    private_key_path: "certs/agent.key"
    allowed_client_ids:
      - "swarmkit-cli"
```

The agent also supports report persistence under `reports:` or
`report_persistence:`, safety options under `safety:`, and data-plane settings
under `data:` or `data_plane:`.

## MAVLink Backend YAML

The MAVLink backend is selected by setting `agent.backend` to `mavlink`.

```yaml
agent:
  backend: "mavlink"
  mavlink:
    drone_id: "drone-1"
    bind_addr: "0.0.0.0:14601"
    autopilot_profile: "ardupilot-copter"
    target_system: 1
    target_component: 1
    source_system: 245
    source_component: 191
    telemetry_rate_hz: 5
    peer_discovery_timeout_ms: 2000
    command_ack_timeout_ms: 2000
    set_guided_before_arm: true
    set_guided_before_takeoff: true
    guided_mode: 4
    allow_flight_termination: false
```

Supported autopilot profile values are `ardupilot-copter` and
`ardupilot-plane`.

## Client YAML

The client loader accepts either a root-level map or a `client:` section.

```yaml
client:
  address: "127.0.0.1:50061"
  client_id: "swarmkit-cli"
  deadline_ms: 5000
  priority: "supervisor"
  retry:
    max_attempts: 3
    initial_backoff_ms: 200
    max_backoff_ms: 2000
  stream_reconnect:
    enabled: true
    initial_backoff_ms: 500
    max_backoff_ms: 5000
    max_attempts: 0
  security:
    transport_security: "mtls"
    root_ca_cert_path: "certs/ca.pem"
    cert_chain_path: "certs/swarmkit-cli.pem"
    private_key_path: "certs/swarmkit-cli.key"
    server_authority_override: "localhost"
```

`stream_reconnect.max_attempts: 0` means unlimited reconnect attempts.

## Swarm YAML

Swarm config combines default client settings with a fleet topology.

```yaml
client:
  client_id: "test-server"
  deadline_ms: 3000
  priority: "override"

swarm:
  drones:
    - drone_id: "drone-1"
      address: "192.168.10.101:50061"
      local_address: "127.0.0.1:50061"
    - drone_id: "drone-2"
      address: "192.168.10.102:50061"
      local_address: "127.0.0.1:50062"
```

Use `--address-mode local` in the CLI or
`SwarmAddressPreference::kPreferLocal` in C++ when running agents on local test
ports.

## Agent Command-Line Options

Important `swarmkit-agent` options:

```text
--config PATH
--id AGENT_ID
--bind HOST:PORT
--backend sim|mavlink|custom-name
--mavlink-bind HOST:PORT
--mavlink-drone ID
--mavlink-autopilot ardupilot-copter|ardupilot-plane
--mavlink-target-system N
--mavlink-target-component N
--artifact-dir PATH
--allow-unsafe-bench-commands
--transport-security auto|insecure|tls|mtls
--insecure
--ca-cert PATH
--server-cert PATH
--server-key PATH
--log-sink stdout|file|both
--log-file PATH
--log-level trace|debug|info|warn|error|critical|off
```

## CLI Options

Important `swarmkit-cli` options:

```text
--config PATH
--client-id ID
--priority operator|supervisor|override|emergency
--transport-security auto|insecure|tls|mtls
--insecure
--ca-cert PATH
--client-cert PATH
--client-key PATH
--server-name NAME
--swarm-config PATH
--address-mode primary|local
--duration-ms MS
--log-level trace|debug|info|warn|error|critical|off
```

Commands include `ping`, `health`, `preflight`, `stats`, `capabilities`,
`telemetry`, `command`, `sequence`, `goal`, `reports`, `message`, `artifact`,
`lock`, `unlock`, `watch-authority`, and `swarm`.

## Environment Variables

Client environment variables use the `SWARMKIT_CLIENT_` prefix:

```text
SWARMKIT_CLIENT_ADDRESS
SWARMKIT_CLIENT_CLIENT_ID
SWARMKIT_CLIENT_DEADLINE_MS
SWARMKIT_CLIENT_PRIORITY
SWARMKIT_CLIENT_RETRY_MAX_ATTEMPTS
SWARMKIT_CLIENT_RETRY_INITIAL_BACKOFF_MS
SWARMKIT_CLIENT_RETRY_MAX_BACKOFF_MS
SWARMKIT_CLIENT_STREAM_RECONNECT_ENABLED
SWARMKIT_CLIENT_STREAM_RECONNECT_INITIAL_BACKOFF_MS
SWARMKIT_CLIENT_STREAM_RECONNECT_MAX_BACKOFF_MS
SWARMKIT_CLIENT_STREAM_RECONNECT_MAX_ATTEMPTS
SWARMKIT_CLIENT_ROOT_CA_CERT_PATH
SWARMKIT_CLIENT_CLIENT_CERT_CHAIN_PATH
SWARMKIT_CLIENT_CLIENT_PRIVATE_KEY_PATH
SWARMKIT_CLIENT_SERVER_AUTHORITY_OVERRIDE
SWARMKIT_CLIENT_TRANSPORT_SECURITY
```

Agent environment variables use the `SWARMKIT_AGENT_` prefix:

```text
SWARMKIT_AGENT_AGENT_ID
SWARMKIT_AGENT_BIND_ADDR
SWARMKIT_AGENT_DEFAULT_AUTHORITY_TTL_MS
SWARMKIT_AGENT_DEFAULT_TELEMETRY_RATE_HZ
SWARMKIT_AGENT_MIN_TELEMETRY_RATE_HZ
SWARMKIT_AGENT_ALLOW_UNSAFE_BENCH_COMMANDS
SWARMKIT_AGENT_REPORT_LOG_FILE
SWARMKIT_AGENT_REPORT_SEQUENCE_STATE_FILE
SWARMKIT_AGENT_ARTIFACT_DIR
SWARMKIT_AGENT_TRANSPORT_SECURITY
SWARMKIT_AGENT_ROOT_CA_CERT_PATH
SWARMKIT_AGENT_CERT_CHAIN_PATH
SWARMKIT_AGENT_PRIVATE_KEY_PATH
SWARMKIT_AGENT_ALLOWED_CLIENT_IDS
```
