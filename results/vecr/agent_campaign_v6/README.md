# VECR v6 — frozen comparative digital research

Primary: **confirmatory_v6_1**, 5,940 attempts, 22 conditions, 30 paired seeds,
nine policies. No infrastructure failure; no false fulfillment among 5,400 honest
observations, 540/540 false fulfillments in dishonest sensor controls. The stronger
GW baseline matches E's completion and often uses less travel. This negative result
is part of the completed research, not a failed publication gate.

* [Full validation and measured findings](../../../docs/vecr/FINAL_VALIDATION_V6.md)
* [Primary raw rows](confirmatory_v6_1/per_attempt.csv)
* [Frozen assignments and identity](confirmatory_v6_1/manifest.json)
* [All tables, paired intervals and figures](confirmatory_v6_1/analysis/)
* [Every raw trace](confirmatory_v6_1/trials/)
* [Primary SHA-256 inventory](confirmatory_v6_1/SHA256SUMS)
* [Client reuse comparison](client_reuse/reuse_comparison.json)
* [Regression, reconstruction and installed-SDK gates](gates/)

The 3,960-attempt `confirmatory/` collection is a retained protocol deviation caused
by an accidental 20-seed default. Do not pool overlapping seeds with the primary
30-seed replication. `pilot_02/` retains all 396 old-executable infrastructure
failures. Other pilots and the client-reuse campaign are separate engineering data.
Historical v1–v5.1 collections have not been changed.

## Reproduction

Use source revision `ce09d3b47d54959eb86101c251d9f2c3ea62be40` for the original
simulation executable, with its manifest environment and SHA-256. Build the
`swarmkit_vecr_v6_trial` target. In a clean checkout run, to a **new** directory:

```sh
.venv/bin/python scripts/vecr/run_agent_campaign_v6.py --mode final --output /private/tmp/vecr-v6-replication --workers 4
.venv/bin/python scripts/vecr/validate_agent_campaign_v6.py --output /private/tmp/vecr-v6-replication
.venv/bin/python scripts/vecr/analyze_agent_campaign_v6.py --output /private/tmp/vecr-v6-replication
```

The runner enforces the exact primary design before dispatch. Analysis refuses to
overwrite an existing analysis directory. Repeated seeds are replications, not
additional independent observations. Validate this archived collection without
rerunning experiments with `scripts/vecr/hash_research_v6.py --collection digital`.
The independent validator reconstructs coverage, accepted-footprint containment,
re-entry dwell and outcome accounting from raw physical acquisitions.
