# VECR research

One onboard method: qualified edge execution, retained coverage and bounded
in-flight recovery. At B, report residual gaps without automatic backward
flight. Graph planning stays external.

* [Method and conditional guarantees](METHOD.md)
* [Digital protocol](SIMULATION_PROTOCOL.md)
* [ArduPilot SITL protocol](SITL_PROTOCOL.md)
* [Literature and contribution](LITERATURE.md)
* [Defense brief: exact claim, evidence and limits](DEFENSE_BRIEF.md)
* [v8 digital results summary](../../results/vecr/v8/primary/analysis/RESULTS.md)
* [Historical v7 digital results](../../results/vecr/current/digital/primary/analysis/RESULTS.md)
* [Historical v7 SITL qualification](../../results/vecr/current/sitl/qualification/analysis/RESULTS.md)

The v7 campaigns remain frozen in `results/vecr/current/` and are excluded from
claims about the v8 endpoint contract. A new v8 campaign belongs in
`results/vecr/v8/`; its manifest, raw records, independent validator and analysis
define the current digital evidence. SITL results remain historical until the
revised runtime is qualified there.
The research inventories can be checked with
`python scripts/vecr/hash_research.py` from the repository root. The
[offline dissertation atlas](../../VECR_Dissertation_Interactive_Website/index.htm)
adds the graph and platform research context while keeping the v7 measurements
and their limitations explicit.
