# Agent-backed VECR simulator campaign v4.1

This directory contains a preregistered B2 full-edge-restart versus P
targeted-gap comparison. Both policies execute through the same SDK, TLS
AgentService, edge runtime and SimBackend. Truth is scored privately from
physical simulation frames and actual synthetic-camera acquisition. These
are simulator measurements only, not ArduPilot SITL or real camera data.

The original v4.0 campaign stopped at S16 seed 10 because its scripted motor
failure never fired; `partial_aborted_s16_010/` preserves all 2,018 completed
rows and the failing trial. See
`docs/vecr/SIMULATION_PROTOCOL_V4_1.md` for the correction. V4.0 rows are
excluded from v4.1 statistics.

Build `swarmkit_vecr_agent_trial`, then run the v4.1 smoke and full campaigns:

```bash
cmake --build --preset mac-release --target swarmkit_vecr_agent_trial
python3 scripts/vecr/run_agent_campaign_v4.py \
  --config results/vecr/agent_campaign_v4/config/protocol_v4_1.json \
  --mode smoke --seeds 1:3 \
  --output results/vecr/agent_campaign_v4/smoke_v4_1
python3 scripts/vecr/run_agent_campaign_v4.py \
  --config results/vecr/agent_campaign_v4/config/protocol_v4_1.json \
  --mode full --seeds 1:100 \
  --output results/vecr/agent_campaign_v4/final_v4_1
python3 scripts/vecr/run_agent_diagnostics_v4.py \
  --campaign results/vecr/agent_campaign_v4/final_v4_1
python3 scripts/vecr/analyze_agent_campaign_v4.py \
  --output results/vecr/agent_campaign_v4/final_v4_1
python3 scripts/vecr/validate_agent_campaign_v4.py \
  --output results/vecr/agent_campaign_v4/final_v4_1
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
