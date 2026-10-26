# VECR — From arrival to verified coverage

Artyom Lazyan's offline dissertation reader for the implemented SwarmKit method
and its completed, versioned digital-simulation evidence.

## Open

Double-click **index.htm**. Keep the `assets/`, `source/` and `validation/` folders
alongside it. Reading requires no server, internet, account or installation.
The Print button opens the browser's print/save-PDF dialog.

## What is included

* Sixteen revised chapters: motivation, contribution, architecture, precise
  contracts, conditional proofs, implementation, fair experiments, results,
  failure analysis, sensitivity, performance, provenance and defense questions.
* Every primary and sensitivity condition, with paired effects and uncertainty.
* Twelve real recorded attempts: seed 1 for six conditions under B2 and VECR,
  including failed recovery and dishonest-sensor outcomes. Replay starts only
  when requested. No fictional completion percentages are animated.
* Seven original experimental figures in SVG/PDF/PNG, five new vector method
  diagrams, and readable offline equations.
* Local CSV/manifest downloads and lossless representative raw traces. The full
  research trace directory is linked separately and needs the surrounding repo.
* Responsive light/dark themes, search, keyboard navigation, larger reading
  text, accessible figure dialogs and print styles.
* A separate SITL results structure with **TBD** values and an explicit
  **not yet executed** status. No SITL numbers or outcomes are invented.

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

## Changes from the supplied reader

The original site described an unimplemented proposal, invented enum/API labels,
an illustrative animation and unvalidated broader platform ambitions. This
edition uses the implemented contract, measured paired results and explicit
scope. The original 17 files are preserved byte-for-byte in
`source/archive/original_reader_2026-10-26.tar.gz`, with a manifest. That archive
is superseded historical material, not the current scientific account.

The manuscript audit also reproduced two endpoint defects in production code:
uncertain traversal endpoints were overcredited, and off-segment camera centers
could be clamped onto the route. Both have recorded before/after regressions.
The corrected full campaign is versioned separately; earlier results were not
rewritten. See the protocol and final validation records for exact revisions.
