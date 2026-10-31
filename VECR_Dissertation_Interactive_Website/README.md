# VECR dissertation reader

Open `index.htm` directly in a browser. The 20-chapter reader is offline and includes the researcher's institutional context, prior SwarmOps work, the implemented SwarmKit method, measured digital results, and five verbatim paired replay examples. Revised-method ArduPilot SITL and calibrated camera results are marked **TBD**.

## Evidence

The frozen digital campaign is `results/vecr/v8/primary/` at the repository root: 22 conditions, 30 common seeds and two policies, for 1,320 attempts. It ran the full agent and backend. The reader's small `assets/evidence/v8/` folder contains the manifest, analysis tables and method documents; the root result folder contains all raw traces and validator output. Retired datasets remain recoverable from Git history and are excluded from the reported estimates.

The central runtime rule applies only to a qualified in-flight deviation before B. It selects the earlier of projected progress and the earliest missing required coordinate, then verifies physical re-entry within finite command, duration and travel budgets. On qualified arrival at B, the agent reports `ARRIVED` and the independent survey verdict; it issues no automatic endpoint-return command. B2 is a full-edge-restart research comparator, not another public runtime mode.

## Rebuild and validate

Run from the repository root with the site dependencies installed:

```sh
python VECR_Dissertation_Interactive_Website/tools/build_site.py
python VECR_Dissertation_Interactive_Website/tools/validate_site.py --browser
python VECR_Dissertation_Interactive_Website/tools/hash_site.py --write
python VECR_Dissertation_Interactive_Website/tools/hash_site.py
```

`build_site.py` reads the frozen digital campaign from `results/vecr/v8/primary/`, so rebuilding the reader requires the repository. The published `index.htm` and bundled replay data work without the repository or a network connection.
