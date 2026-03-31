Copyright (c) 2026 Artyom Lazyan. All rights reserved.
Licensed under the SwarmKit Proprietary License. See `LICENSE.md`.

These certificates are development-only mTLS fixtures for local testing and example tools.

- `ca.pem` / `ca.key`: local development certificate authority
- `agent.pem` / `agent.key`: localhost server identity for `swarmkit-agent`
- `test-client.pem` / `test-client.key`: example low-priority client identity
- `test-server.pem` / `test-server.key`: example GCS/test-server identity
- `swarmkit-cli.pem` / `swarmkit-cli.key`: example CLI identity

They are intended only for local development, tests, and SDK examples.
Do not reuse them in production.
