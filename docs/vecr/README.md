# VECR research

One onboard method: qualified edge execution, retained coverage and earliest-gap
recovery with bounded evidence waiting. Graph planning stays external.

* [Method and conditional guarantees](METHOD.md)
* [Digital protocol](SIMULATION_PROTOCOL.md)
* [ArduPilot SITL protocol](SITL_PROTOCOL.md)
* [Literature and contribution](LITERATURE.md)
* [Primary results summary](../../results/vecr/current/digital/primary/analysis/RESULTS.md)
* [SITL qualification results](../../results/vecr/current/sitl/qualification/analysis/RESULTS.md)

All empirical campaigns are completed: 1,320 primary attempts across 22 conditions (N00–N21),
100 client-reuse trials, and 12 ArduPilot SITL qualification flights. All raw records,
traces, and independent verification logs are frozen in `results/vecr/current/`.
The research inventories can be checked with
`python scripts/vecr/hash_research.py` from the repository root. The
[offline dissertation atlas](../../VECR_Dissertation_Interactive_Website/index.htm)
adds the graph and platform research context while keeping the v7 measurements
and their limitations explicit.
