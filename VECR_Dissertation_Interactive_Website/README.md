# VECR — From arrival to verified coverage

Artyom Lazyan's offline dissertation reader for the implemented SwarmKit method
and its completed, versioned digital-simulation evidence.

## Open

Double-click **index.htm**. Keep the `assets/`, `source/` and `validation/` folders
alongside it. Reading requires no server, internet, account or installation.
The Print button opens the browser's print/save-PDF dialog.

## What is included

* A restored researcher profile with Artyom Lazyan's institute, department,
  full E.13.04 specialization and research theme. The department is linked to
  the institute's official staff listing; original profile information is retained.
* Twenty revised chapters: the researcher's earlier graph and multi-user swarm
  work, SwarmOps mission workflow, SwarmKit control and communication APIs,
  the implemented VECR contract, conditional proofs, fair experiments, measured
  results, worked cases, limitations, defense questions and reproducibility.
* All 22 conditions N00–N21 comparing canonical VECR (earliest-gap recovery with
  bounded waiting) against the B2 full-restart baseline (1,320 primary attempts),
  paired bootstrap confidence intervals, 100 client-reuse trials, and 12 ArduPilot
  SITL qualification flights.
* Twelve real recorded attempts: seed 70001 for six conditions under B2 and VECR,
  including failed recovery and dishonest-sensor outcomes. Replay starts only
  when requested. No fictional completion percentages are animated.
* Current experimental figures in SVG/PDF/PNG, six vector method diagrams,
  and readable offline equations.
* Local CSV/manifest downloads and lossless representative raw traces. The full
  research trace directory is linked separately and lives in `results/vecr/current/`.
* A restored dark, photo-led visual edition inspired by the supplied earlier
  reader, with a measured route-progress chart, responsive light/dark themes,
  search, keyboard navigation, accessible figure dialogs and print styles.
* A separate 12-flight ArduPilot SITL qualification with independently recorded
  simulator truth. Native accuracy controls and synthetic bounded sensing are
  distinguished.
* A sourced literature comparison and candid defense-readiness assessment.
  A downloadable defense brief states the conditional invariant, measured effects,
  failure envelope and exact claim for presentation use.
  VECR is the complete execution method. A separate contribution and planner contract
  describes its intended use by external graph planners (e.g. rotor-router),
  with high-level planning explicitly external to the agent.
  Across 1,200 honest trials, zero false fulfillments occurred; 120 dishonest
  controls cleanly expose the sensor trust boundary.

The numerical campaign is a flat, one-dimensional full-strip simulator study.
Real-camera quality, complex terrain, physical flight, cloud deployment and
remote/multi-UAV performance are outside its supported claims. Neither a project
publication nor this reader establishes that a dissertation has been defended.

## Rebuild and check

The canonical current narrative is `source/dissertation.md`. The builder reads
frozen research CSVs and manifests from the surrounding SwarmKit repository;
it does not execute or alter experiments. Data reach the browser through a local
JavaScript asset, avoiding `file://` JSON-fetch restrictions.

```bash
python -m pip install -r VECR_Dissertation_Interactive_Website/tools/requirements.txt
python VECR_Dissertation_Interactive_Website/tools/build_site.py
python VECR_Dissertation_Interactive_Website/tools/validate_site.py
python VECR_Dissertation_Interactive_Website/tools/validate_site.py --browser
python VECR_Dissertation_Interactive_Website/tools/hash_site.py --write
```

To verify the delivered package without rebuilding or changing its inventory,
run `python VECR_Dissertation_Interactive_Website/tools/hash_site.py`.

The optional browser check uses an isolated headless installation of Google
Chrome on macOS. It checks offline loading, all six paired replay outcomes,
negative-control truth discrepancy, controls, mobile width, navigation, figure
dialog dismissal and print media. `validation/site_validation.json` records
the actual check outcome; `validation/screenshots/` contains visual checks.

`assets/research-metadata.json` maps copied artifacts to source paths and SHA-256
hashes. `SHA256SUMS` inventories the finished reader. The upstream research's
hash inventory remains separate from this derivative presentation.

## Relation to the supplied earlier reader

The supplied earlier website included extensive graph-theoretic, SwarmOps,
communication, platform, worked-example and bibliography material. Those
subjects are restored here with explicit status labels. Its VECR proposal,
obsolete formulas, simulated animations and unverified completion claims were
not copied into the current scientific account. The original ZIP supplied by
the user remains outside this generated reader; the repository's earlier
versions are available through Git history.

The manuscript audit also reproduced two endpoint defects in production code:
uncertain traversal endpoints were overcredited, and off-segment camera centers
could be clamped onto the route. Both have recorded before/after regressions.
The corrected full campaign is versioned separately; earlier results were not
rewritten. See the protocol and final validation records for exact revisions.

The numerical evidence presented here is sourced from the versioned v7 current
campaign. Earlier research remains in repository history and is not substituted
for the v7 measurements. Formal submission should independently verify the
author-supplied bibliographic details for any later publications.
