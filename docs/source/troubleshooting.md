# Troubleshooting

## `find_package(SwarmKit REQUIRED)` Fails

Make sure the SDK was installed and `CMAKE_PREFIX_PATH` points to the install
prefix:

```bash
cmake --install build/mac-release --component sdk --prefix /tmp/swarmkit-sdk
cmake -S . -B build -DCMAKE_PREFIX_PATH=/tmp/swarmkit-sdk
```

The package files are installed under `lib/cmake/SwarmKit`.

## Conan Cannot Resolve Dependencies

Run `conan profile detect` once, then repeat the install step with the correct
build type:

```bash
conan profile detect
conan install . -of build/conan -s build_type=Release -s compiler.cppstd=23 --build=missing
```

If the compiler is too old, use GCC 13+, Clang 17+, or current Xcode Command
Line Tools.

## Agent Address Is Rejected

Client and agent address fields must look like `host:port`. Examples:

```text
127.0.0.1:50061
0.0.0.0:50061
192.168.10.101:50061
```

## TLS Or mTLS Fails Locally

For local development, start both tools with `--insecure`:

```bash
swarmkit-agent --insecure --bind 127.0.0.1:50061
swarmkit-cli --insecure 127.0.0.1:50061 ping
```

For mTLS, verify that the agent has a server certificate and key, the client has
a client certificate and key, both use the same root CA, and the client's
`server_authority_override` matches the certificate name when needed.

## Commands Are Rejected

Common causes:

- Another client holds command authority with an equal or higher priority.
- The target drone ID is missing or not registered in `SwarmClient`.
- The backend does not support the command category.
- MAVLink/autopilot state rejects the command.
- Readiness checks fail for autonomous commands.

Use these probes:

```bash
swarmkit-cli --insecure 127.0.0.1:50061 health
swarmkit-cli --insecure 127.0.0.1:50061 capabilities
swarmkit-cli --insecure 127.0.0.1:50061 watch-authority --drone drone-1
```

## Telemetry Does Not Arrive

Check that the agent is running, the drone ID matches the backend configuration,
and the requested rate is at least the agent's minimum rate. For MAVLink, confirm
that SITL or the autopilot is sending traffic to the configured UDP bind address.

For local simulator testing:

```bash
swarmkit-agent --insecure --backend sim
swarmkit-cli --insecure telemetry --drone default --rate 1
```

## MAVLink SITL Does Not Respond

Verify the MAVLink settings:

- `mavlink.bind_addr` is the UDP endpoint SwarmKit should listen on.
- `target_system` and `target_component` match the autopilot.
- `autopilot_profile` is `ardupilot-copter` or `ardupilot-plane`.
- The agent log level is `debug` while diagnosing command ACK behavior.

Example:

```bash
swarmkit-agent --config testdata/agent_mavlink_drone1.yaml --log-level debug
```

## Documentation Build Fails

Install the Python requirements and Doxygen:

```bash
cd docs
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
make html
```

If Sphinx reports that Breathe cannot find XML, run `make doxygen` and confirm
that `docs/build/doxygen/xml/index.xml` exists.
