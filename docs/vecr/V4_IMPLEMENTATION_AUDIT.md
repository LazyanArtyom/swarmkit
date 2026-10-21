# VECR simulator validation v4: pre-change audit

Recorded 2026-10-21 before modifying product code. The v4 task specification is
`SwarmKit_VECR_Simulation_Validation_Codex_Prompt_v4.md` in the repository root;
its instructions are research requirements, not observed results.

| Item | Baseline |
|---|---|
| Git | `main`, `faea016d8eeb1a742aa2f7d955bb02ed45311de8` |
| Worktree | Only the v4 prompt untracked; tracked files clean |
| Host | macOS Darwin 25.6.0, arm64 T6030 |
| Compiler | Apple clang 17.0.0 |
| CMake / Conan | 3.31.6 / 2.25.2 |
| Intended preset | `mac-release`; build tree absent at audit time |
| Baseline CTest | Build tree initially absent; after Conan cache install and `mac-release` rebuild, 2/2 CTest entries passed in 10.90 s |
| Archive SHA-256 | Core `a949230659ec5fcfd78443530adf2b46d304ae7771e6dc8096e6b4b7ff1bd109`; research `65d484a9ee0705b7450b29f9699b3070e205736bcc177b1f0fc7700ef5c25bc3`; both match `CLEANUP_INVENTORY.md` |

## Product-path findings

- `EdgeExecutionRuntime::Publish` logs a journal append failure but still emits the original
  `EDGE_FULFILLED` report. A subscriber can thus see an apparently committed terminal result
  that will not be recoverable after a crash. This is an active P0-A defect.
- The journal converts nonterminal records to `UNKNOWN` on restart, but `Start` checks only
  the requested attempt ID. A fresh attempt ID for the same drone can dispatch while an old
  physical navigation command is unresolved. This is an active P0-B defect.
- `Cancel` and `Preempt` interrupt local monitoring and publish `INTERRUPTED` without
  requesting backend hold or confirming physical stop. This is an active P0-C defect.
- Stop-style arrival, hard-bound traversal, capture binding, straight-route rejection,
  simulator-only recovery, and a two-attempt SDK/Agent/SimBackend pilot already exist.
  Their regressions must remain green. The old v1/v2 geometric outputs and v3 pilot are frozen.
- The v3 pilot has no agent-backed B2 arm. Its private simulator truth is used by a synthetic
  camera producer, but the production verifier consumes only submitted metadata and telemetry.
  A new comparison needs a common agent policy path and a separate tested truth oracle.

No v4 scientific campaign has been run at this audit point.

## Gate 1 implementation and observed checks

The implementation now commits each report to the append-only journal before
publishing it. A failed append or fsync causes a critical, explicitly
non-durable `EDGE_UNKNOWN` publication; it never publishes `EDGE_FULFILLED`
as though it were committed. An fsync error means the bytes may or may not
survive a later process crash. Partial/corrupt trailing records fail the
journal closed. Fault injection covers no space, quota, read-only, partial
write and fsync failure. Report delivery is separate from journal commit;
there is no atomic transaction spanning vehicle, filesystem and subscriber.

Journal format `VECRJ02` adds explicit physical-navigation-unresolved state.
An old `VECRJ01` file fails closed and requires operator review or migration
before new edge dispatch; no compatibility decoder was added. Any unresolved
attempt gates every new attempt ID for that drone. The same attempt ID is
never automatically redispatched. An authorized simulator hold plus bounded,
stationary telemetry can clear the gate while leaving the survey obligation
interrupted. Live MAVLink does not have this unqualified takeover path.
Backend command ACK alone cannot establish physical stop. Safety, authority
or timeout interruption retains the physical gate until reconciliation.

Active cancellation first stops VECR monitoring, requests simulator hold,
then checks two bounded stationary samples at least 200 ms apart. If hold
cannot be confirmed, it reports unknown physical navigation and leaves the
gate. Capture submission after monitoring stops is rejected. Restart while
the simulator is moving, conflicting attempt IDs, stale reports, safety
interruption, hold refusal and terminal fsync failure have regressions.

On this host the release build, SDK install, full release CTest (2/2) and
macOS UBSan CTest (2/2) passed. The three-seed paired smoke campaign executed
72 actual agent attempts in `results/vecr/agent_campaign_v4/smoke_v4_2`;
these smoke measurements are not final scientific evidence.
