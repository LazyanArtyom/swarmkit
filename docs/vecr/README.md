# VECR research

One onboard method: qualified edge execution, retained coverage and bounded
in-flight recovery. At B, report residual gaps without automatic backward
flight. Graph planning stays external.

* [Method and conditional guarantees](METHOD.md)
* [Digital protocol](SIMULATION_PROTOCOL.md)
* [ArduPilot SITL protocol](SITL_PROTOCOL.md)
* [Literature and contribution](LITERATURE.md)
* [Defense brief: exact claim, evidence and limits](DEFENSE_BRIEF.md)
* [Experiment revalidation and figure provenance](EXPERIMENT_REVALIDATION.md)
* [Claim-to-source traceability](TRACEABILITY.md)
* [Final change and gate report](FINAL_CHANGE_REPORT.md)
* [v8 digital results summary](../../results/vecr/v8/primary/analysis/RESULTS.md)
* [v8 client-path reuse audit](../../results/vecr/v8/reuse-qualified/analysis/REUSE_RESULTS.md)

The retired v7 campaigns are excluded from the current checkout and from
claims about the v8 endpoint contract; their pre-cleanup state remains in Git
history. The completed v8 primary, qualified reuse
and unsuccessful sandbox-blocked reuse attempts are separately preserved in
`results/vecr/v8/`. Their manifests, raw records, independent validators,
analyses and SHA-256 inventories define the current digital evidence. Check them
with `python scripts/vecr/hash_research.py --v8 primary`, `--v8 reuse-qualified`
and `--v8 reuse`. Revised-method SITL results are TBD. The
[offline dissertation atlas](../../VECR_Dissertation_Interactive_Website/index.htm)
includes the broader graph and platform context with active v8 results.
