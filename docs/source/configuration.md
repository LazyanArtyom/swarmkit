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
  telemetry:
    retention_frames_per_drone: 4096
  reports:
    backlog_size: 1000
  execution_recorder:
    file_path: "/tmp/run-001.swkevidence"
    run_id: "run-001"
    scenario_id: "nominal-sitl"
    random_seed: 104729
    max_segment_bytes: 268435456
    max_segments: 1
    loss_policy: "invalidate_run"
    flush_each_record: true
    fsync_each_record: false
    overwrite_existing: false
    calibration_profile_id: ""
    calibration_version: ""
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

`reports` is an in-memory notification/replay stream. Durable correctness evidence is written only
by `execution_recorder`; the old report-only JSONL path does not exist. `invalidate_run` is the
scientific fail-closed policy. `rotate_oldest` must be selected explicitly for operational logging
where bounded evidence loss is acceptable.

The built-in simulator accepts strict options through `agent.backend_options`.
Unknown keys and invalid values fail backend creation:

```yaml
agent:
  backend: "sim"
  backend_options:
    integration_step_ms: "20"
    initial_source_unix_time_ms: "1700000000000"
    home_lat_deg: "40.1811"
    home_lon_deg: "44.5136"
    home_alt_m: "0"
    initial_battery_percent: "95"
    max_horizontal_speed_mps: "10"
    max_climb_speed_mps: "5"
    max_descent_speed_mps: "3"
    max_altitude_m: "120"
    default_cruise_speed_mps: "4"
    battery_drain_percent_per_second: "0.002"
    stop_at_target: "true"
```

Manual simulator time and the truth observer are C++ experiment controls and are
not enabled by the production Agent YAML path.

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
--evidence-file PATH
--evidence-run-id ID
--evidence-scenario-id ID
--evidence-seed N
--evidence-overwrite
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
SWARMKIT_AGENT_TELEMETRY_RETENTION_FRAMES
SWARMKIT_AGENT_ALLOW_UNSAFE_BENCH_COMMANDS
SWARMKIT_AGENT_REPORT_BACKLOG_SIZE
SWARMKIT_AGENT_EVIDENCE_FILE
SWARMKIT_AGENT_EVIDENCE_RUN_ID
SWARMKIT_AGENT_EVIDENCE_SCENARIO_ID
SWARMKIT_AGENT_EVIDENCE_MAX_SEGMENT_BYTES
SWARMKIT_AGENT_EVIDENCE_MAX_SEGMENTS
SWARMKIT_AGENT_EVIDENCE_LOSS_POLICY
SWARMKIT_AGENT_EVIDENCE_RANDOM_SEED
SWARMKIT_AGENT_EVIDENCE_FLUSH_EACH_RECORD
SWARMKIT_AGENT_EVIDENCE_FSYNC_EACH_RECORD
SWARMKIT_AGENT_EVIDENCE_OVERWRITE
SWARMKIT_AGENT_EVIDENCE_CALIBRATION_PROFILE
SWARMKIT_AGENT_EVIDENCE_CALIBRATION_VERSION
SWARMKIT_AGENT_ARTIFACT_DIR
SWARMKIT_AGENT_TRANSPORT_SECURITY
SWARMKIT_AGENT_ROOT_CA_CERT_PATH
SWARMKIT_AGENT_CERT_CHAIN_PATH
SWARMKIT_AGENT_PRIVATE_KEY_PATH
SWARMKIT_AGENT_ALLOWED_CLIENT_IDS
```
