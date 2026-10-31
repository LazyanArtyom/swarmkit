# Review disposition and consolidation

## Subject-matter correction

The supplied review confuses UAV graph-edge execution with continuum/soft robotics.
SwarmKit contains no continuum kinostatics, elastic observer, compliance estimation,
Cosserat model, tendon/pneumatic actuation, FEA comparison or measured real-camera
occlusion/lighting study. The M-series identifiers were simulated UAV experiments,
not manipulator experiments. Those claims and the proposed visual-elastic novelty
are rejected as inapplicable. No claim about exceeding typical PhD rigor or
committee acceptance can be established by this repository.

## Useful requests addressed

* **Explicit contribution:** the onboard obligation/evidence/recovery contract is
  specified in METHOD.md, including a concrete recovery rule and conditional
  survey/traversal arguments. Elementary known mathematics is labelled honestly.
* **Failure envelope:** the digital matrix includes stale/reordered evidence,
  multiple gaps, changing bounds, failed sensors, command refusal, motor failure,
  low battery and deadlines. Real optical/payload limits require future calibrated
  experiments; no numbers are invented for them.
* **Generalizability:** 20/40/80 m segments and direct/routed clients are tested.
  General graph planning remains external. No arbitrary planner, aircraft or
  camera compatibility theorem is asserted.
* **Reproducibility:** frozen source and binary identity, manifests, raw attempts,
  independent reconstruction, paired intervals, trace inspection and SHA-256
  inventories remain part of the current method's validation.

## One-method decision and provenance

The working tree exposes one production recovery rule: on a qualified in-flight
departure, return to `q=min(projected progress, first required unverified
coordinate)` within finite safety budgets. At B, report missing work without an
automatic backward command. There are no public policy selectors. The frozen v8
campaign tests this rule against a separately compiled full-restart B2 research
comparator, which is never an SDK/protobuf choice.

Retired datasets and pilot directories were removed from the current checkout
after explicit user authorization. Their pre-cleanup state remains recoverable
from Git history; no older estimate is used as a v8 result. The frozen v8
primary and qualified reuse campaigns retain their raw outcomes and failures.
