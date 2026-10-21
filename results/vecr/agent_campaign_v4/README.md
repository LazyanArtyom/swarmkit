# Agent-backed VECR simulator campaign v4

This directory contains a preregistered B2 full-edge-restart versus P
targeted-gap comparison. Both policies execute through the same SDK, TLS
AgentService, edge runtime and SimBackend. Truth is scored privately from
physical simulation frames and actual synthetic-camera acquisition. These
are simulator measurements only, not ArduPilot SITL or real camera data.

Build `swarmkit_vecr_agent_trial`, then run the smoke and full campaigns:

```bash
cmake --build --preset mac-release --target swarmkit_vecr_agent_trial
python3 scripts/vecr/run_agent_campaign_v4.py \
  --config results/vecr/agent_campaign_v4/config/protocol.json \
  --mode smoke --seeds 1:3 \
  --output results/vecr/agent_campaign_v4/smoke_v4_2
python3 scripts/vecr/run_agent_campaign_v4.py \
  --config results/vecr/agent_campaign_v4/config/protocol.json \
  --mode full --seeds 1:100 \
  --output results/vecr/agent_campaign_v4/final
python3 scripts/vecr/run_agent_diagnostics_v4.py \
  --campaign results/vecr/agent_campaign_v4/final
python3 scripts/vecr/analyze_agent_campaign_v4.py \
  --output results/vecr/agent_campaign_v4/final
python3 scripts/vecr/validate_agent_campaign_v4.py \
  --output results/vecr/agent_campaign_v4/final
```

The runner refuses to overwrite any output directory. Full mode requires
exactly seeds 1–100. The raw table includes every assigned attempt, including
interruptions, timeouts and false completions. Diagnostics are separate from
the comparative denominator. The analysis derives tables, paired bootstrap
and Wilson intervals, SVG figures, bounded prose and SHA-256 hashes from raw
rows. `manifest.json` identifies the binary, source revision and config.

S13 remote transport outage, S14 peer interruption and S17 simultaneous
multi-UAV operation are currently marked `UNSUPPORTED`, not counted as
successful or comparable. S20 rejects polyline requests because the agent
currently supports only straight two-point edge navigation. The historical
v1/v2/v3 results and research archives are unchanged.
