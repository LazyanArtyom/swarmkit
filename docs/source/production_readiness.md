# Production-readiness milestone

Status: Phase 0 release candidate, audited 2026-08-16; hosted CI gates remain pending.

This checklist tracks the production-hardening gate from the generic runtime task. It is
deliberately separate from the dissertation runtime phases: common-time snapshots, state
contracts, contract-aware scheduling, and semantic traces do not begin until this gate is
accepted. SwarmOps is out of scope for this repository pass.

Status meanings:

- **Complete**: implemented, documented, and covered by the current automated suite.
- **Partial**: useful infrastructure exists, but the listed exit criterion remains open.
- **Open**: required work has not started or is not yet adequate for a production claim.

## Audit and checklist

| # | Area | Status | Evidence and remaining exit criterion |
|---:|---|---|---|
| 1 | Public API consistency | Partial | Canonical pre-release command, execution-handle, telemetry, capability, and error models exist. Health state now uses optional booleans end to end so unknown is not encoded as healthy. Complete a public-header/API review before the first stable release. |
| 2 | Ownership and lifetime | Partial | Backends are uniquely owned; subscriptions and swarm fan-out use owned handles. `StopTelemetry()` now has an explicit callback-quiescence postcondition. Audit every public callback owner and document callback capture lifetime. |
| 3 | Thread safety | Partial | Shared Agent/client/backend paths use mutexes, atomics, and bounded queues; callback exceptions are contained at normalized telemetry ingress. Linux TSan is in CI. Add repeatable stress tests for concurrent connect, stop, reconnect, and server shutdown. |
| 4 | Deterministic shutdown | Complete | Client subscriptions are stoppable, Agent signal shutdown is joined, and the shared backend contract proves that simulator and MAVLink stop wait for in-flight callbacks. The MAVLink case is driven through real loopback UDP ingress. |
| 5 | Cancellation and deadlines | Complete | Unary deadlines/retries, stream stop, artifact cancellation, active-goal cancellation, and swarm fan-out stop tokens are typed and tested. |
| 6 | Standard error/result model | Complete | `core::Result`, `SwarmError`, gRPC conversion, retryability, correlation IDs, and command/backend outcomes are canonical. |
| 7 | Reconnect behavior | Complete | Streaming reconnect policy, bounded backoff, replay cursors, producer sequence classification, and callback-drop accounting are implemented and tested. |
| 8 | Agent session/boot epoch | Complete | One opaque `agent_session_id` is generated per Agent service lifetime and appears in identity, telemetry, goals, reports, capabilities, and evidence records. |
| 9 | Stale response rejection | Partial | Telemetry cursor/session mismatch, old-session frames, duplicate/reordered data, and exact stale goal handles are rejected or surfaced explicitly. Add a common reusable session guard for future state APIs. |
| 10 | Protobuf/API versioning policy | Partial | The canonical package is `swarmkit.v1`, project version comes from `VERSION`, and the greenfield breaking-change policy is documented. Freeze a Buf breaking baseline only when stable compatibility begins. |
| 11 | Backend interface contract | Partial | Threading, callback behavior, stop idempotence, and callback quiescence are documented. Add explicit start/stop state-machine and command-cancellation requirements. |
| 12 | Identical simulator/MAVLink contract tests | Complete | Reusable lifecycle and callback-quiescence suites run against both backends and verify idempotent start/stop, argument rejection, duplicate-start rejection, restart after stop, explicit health identity, and in-flight callback draining. Backend-specific command and decoder behavior remains separately tested. |
| 13 | Deterministic command/goal lifecycle | Complete | Acceptance, rejection, failure, supersession, retry, timeout, exact cancellation, and evidence lineage are deterministic and tested. |
| 14 | Multi-agent helper tests | Complete | Routing, bounded fan-out, cancellation, authority, all-drone telemetry, and partial failure are covered. |
| 15 | Sanitizers | Partial | Linux ASan+UBSan and TSan presets are restored to CI; local macOS sanitizer runtime limitations are documented. A green CI run is required for gate acceptance. |
| 16 | Warnings/static analysis | Partial | Project targets compile with `-Wall -Wextra -Wpedantic`; clang-tidy configuration and a CI analysis job exist. Existing test-only optional-access diagnostics still need triage before warnings can become errors. |
| 17 | Structured logging | Partial | Operational logs consistently carry event/RPC, correlation, Agent, drone, peer, and result fields. A typed sink/schema and redaction policy remain open. Correctness evidence uses the checksummed recorder, not console logs. |
| 18 | Metrics hooks | Complete | Runtime counters expose requests, failures, active streams/watchers, telemetry, messages, artifacts, and backend failures through the SDK/CLI. |
| 19 | Install/export/package | Partial | CMake install/export, Conan components, relocatable SDK packaging, and an external CMake consumer exist. CI exports the Conan package, builds both archives, compiles against the extracted SDK, and performs a staged Agent ping. A green hosted run is required for gate acceptance. |
| 20 | Supported-platform CI | Partial | Linux Debug, Release, ASan+UBSan, TSan, protocol lint, source analysis, packaging, macOS ARM64 Release, and docs workflows are defined. Green hosted runs are required for gate acceptance. |
| 21 | Documentation/examples | Complete | Installation, configuration, security, architecture, evidence, simulator, replay, and consumer examples are supplemented by an operator runbook with staging, monitoring, shutdown, incident, upgrade, rollback, and release procedures. |
| 22 | Dead/duplicated/temporary code | Partial | Compatibility-only execution APIs and report-only persistence were removed. Continue source-level dead-code review; generated build, docs, and distribution artifacts remain ignored local outputs. |

## Changes in this milestone

- Unknown armed, landed, failsafe, GPS-health, and estimator-health values remain absent through the
  backend, protobuf, SDK, CLI, and preflight paths. MAVLink no longer infers landed from disarmed or
  estimator health from the absence of failsafe.
- Autonomous readiness now requires explicit disarmed, landed, failsafe-clear, GPS-healthy, and
  estimator-healthy evidence.
- The backend stop contract guarantees that callbacks are quiescent on return. Simulator and
  MAVLink implementations enforce that boundary, and Agent telemetry ingress contains observer
  exceptions without terminating a backend worker.
- Build/correctness CI is restored for Debug, Release, sanitizers, source analysis, protobuf lint,
  and Conan package export.

## Local verification evidence

The following checks passed on macOS ARM64 on 2026-08-16:

- configure, build, and CTest for `mac-debug`, `mac-release`, and the reproducible `mac-ubsan`
  preset;
- clang-tidy over every production source changed by this milestone, with no diagnostics;
- Doxygen and Sphinx HTML documentation generation;
- release SDK and tools archive generation plus SHA-256 sidecar verification;
- external compilation of both `apps/test_tools` executables against the extracted SDK archive;
- launch of the staged Release Agent and a successful ping from the separately built SDK consumer;
- syntax parsing of both GitHub Actions workflow YAML files.

The Linux ASan+UBSan, Linux TSan, Buf lint, Linux package smoke test, and macOS ARM64 hosted build
are defined in CI and remain acceptance gates until their first hosted run is green.

## Gate acceptance

Phase 0 is accepted only after every **Open** item is complete, every **Partial** item required for
the supported release profile has an explicit disposition, the CI matrix is green, the external
package consumer passes, and the residual operational limitations are approved. Passing unit tests
alone is not an airworthiness claim.
