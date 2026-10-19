# Superseded research cleanup

The previous state-acceptance dissertation experiments and isolated public core are retired from the active pre-release API. This is a breaking source change. No old/new compatibility layer remains.

| Archive | Contents | Files | SHA-256 |
|---|---|---:|---|
| `archive/state_acceptance_research_2026-10-19.tar.gz` | Old `src/experiment`, `apps/experiment`, dissertation docs/results, benchmark result and two experiment tests | 84 | `65d484a9ee0705b7450b29f9699b3070e205736bcc177b1f0fc7700ef5c25bc3` |
| `archive/state_acceptance_core_2026-10-19.tar.gz` | Old state acceptance, evidence store, clock quality and replay public core plus seven isolated tests | 21 | `a949230659ec5fcfd78443530adf2b46d304ae7771e6dc8096e6b4b7ff1bd109` |

The adjacent JSON manifests name every archived path and original SHA-256 digest. Archives were verified before removal. The old statistics and SITL artifacts are historical records of another method, not VECR evidence.

Retained product components: normalized telemetry and uncertainty descriptors, MAVLink and simulation backends, authority and command gating, gRPC services, security, report hub, peer data transport, execution recorder, SDK and CLI. These support VECR or remain useful general SDK facilities.
