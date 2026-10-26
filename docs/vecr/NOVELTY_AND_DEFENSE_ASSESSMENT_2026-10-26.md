# VECR: literature comparison and dissertation-readiness assessment

Assessment date: 26 October 2026. Implementation/evidence reviewed: v5.1,
experimental source `7da3db3`, analysis `ee3e321a5`, publication `afa3822a3`.
This is an engineering and literature assessment, not an institutional decision.

## Judgment

VECR is an implemented, tested execution method with a plausible **applied
software/systems research contribution**. Its claim to a scientifically novel
method is **not yet established sufficiently to declare it defense-ready on
VECR alone**. Completed experiments demonstrate behavior under a specified
model; they do not establish originality or degree sufficiency.

The broad ideas of monitoring physical task completion, checking sensor
assumptions, accumulating conservative sensed coverage, and resuming interrupted
surveys have substantial prior work. The narrower contribution to investigate is
the precise reusable edge-obligation contract, its implementation and its
empirically characterized correctness/availability/cost tradeoffs.

## Search scope

Targeted web searches covered UAV task-assumption monitoring, coverage under
bounded localization error, sensor-footprint accumulation, missed-area recovery,
mission resumption and modular coverage libraries. The comparison below uses
primary papers, author/institution-hosted material and official documentation.
Publisher records and indexed author-PDF passages were examined for the 2021
paper; direct PDF fetching timed out. The 2018 comparison includes indexed
full-text passages around its cumulative sensed-area equation. This is not an
exhaustive systematic review or a proof that no identical method exists.

## Closest relevant work

| Source | Relevant prior contribution | Consequence for VECR |
|---|---|---|
| Zudaire, Gorostiaga, Sánchez, Schneider and Uchitel, ICRA 2021, *Assumption Monitoring Using Runtime Verification for UAV Temporal Task Plan Executions* | Addresses silent mission failure, photographic coverage assumptions, onboard monitors and corrective action; includes physical-flight and ArduPilot SITL examples. | The general problem and monitor/recovery architecture predate VECR. Establish a precise additional capability rather than claiming this problem was previously unaddressed. [Author paper](https://software.imdea.org/~cesar/papers/zudaire21assumption.pdf), [publisher record](https://ieeexplore.ieee.org/document/9561671/) |
| Arvanitakis, Tzes and Giannousakis, 2018, *Synergistic exploration and navigation of mobile robots under pose uncertainty in unknown environments* | Computes guaranteed visibility under bounded pose uncertainty and accumulates guaranteed sensed area over time. | Conservative geometric evidence and its union are established ideas. An edge-specific implementation may be useful, but union soundness alone is not a new general theorem. [Institution-hosted paper](https://pure.coventry.ac.uk/ws/files/13602376/1729881417750785.pdf) |
| ArduPilot, *Mission Rewind on Resume*, current official documentation | Resumes an interrupted survey behind the interruption point by a configured distance. | Return-to-track with overlap already exists. Its documented target depends on interruption history; VECR chooses from accepted/missing strip evidence. This difference requires a direct experiment, including cases where evidence gaps and interruption position differ. [Official documentation](https://ardupilot.org/copter/docs/common-mission-rewind.html) |
| Mier, Valente and de Bruin, Fields2Cover, IEEE RA-L 2023 | An extensible coverage-planning library evaluated using multiple methods/objectives and simulation/field experiments. | A reusable library can be a research artifact, but modularity alone is not original. VECR must justify its execution/acceptance contract and evaluate its added utility. [Primary paper](https://arxiv.org/abs/2210.07838) |
| Bouman et al., 2023, *Adaptive Coverage Path Planning for Efficient Exploration of Unknown Environments* | Plans with accumulated sensor-footprint coverage, time constraints, risk and dynamics; used in a DARPA Subterranean Challenge entry. | Coverage-informed replanning is an established research area. VECR's current evidence concerns a restricted straight-edge strip, not superiority over general adaptive planners. [Primary paper](https://arxiv.org/abs/2302.03164) |

The table identifies overlap. It does not claim these systems all implement the
same API, assumptions or obligations, nor that an unmentioned feature is absent
from an entire research line. No competing implementation was benchmarked in
this literature review.

## A defensible contribution statement

> We developed and evaluated a reusable onboard execution contract for assigned
> graph edges. It binds qualified navigation and sensing evidence to an exact
> attempt, accumulates conservative full-strip coverage, retains disjoint verified
> work, and performs bounded recovery from missing obligations. A controlled
> agent-backed study characterizes completion validity, evidence availability,
> recovery failures and travel cost under explicit sensing assumptions.

This states what was done. To call it an **original scientific contribution**,
the dissertation must additionally demonstrate a meaningful distinction from
the closest earlier work. Combining familiar components is not automatically
new, but a well-motivated contract and a demonstrated capability can be a valid
systems contribution. That is a judgment for peer and institutional review.

| Claim | Current assessment |
|---|---|
| Concrete reusable implementation and reproducible test artifact | Established by code, tests, manifests and execution records |
| Lower travel than B2 in the three stated equal-success conditions | Established within the simulator; 25.3–27.8%, with paired intervals |
| Correct handling of the reproduced endpoint/state defects | Supported by counterexample regressions and independent reconstruction |
| Better verification than the comparison baseline | Not established: B2 and VECR share the verifier |
| Novel general monitoring/recovery concept | Not supportable given the prior work above |
| New general mathematical theorem | Not established by the present elementary conditional union proof |
| Recovery optimality or superiority over current resume methods | Not tested/proved |
| Independence from particular high-level planners | Architectural separation implemented; multi-planner empirical validation absent |
| Real physical survey quality | Not established by synthetic metadata or geometry |
| Dissertation sufficient for defense | Cannot be certified from this artifact or targeted search |

## What the existing evidence does and does not resolve

The 5,280 assigned science attempts and independent reconstruction provide a
substantial audit trail. Zero false fulfillment in 4,880 honest-sensor attempts
is conditional on truthful bounded inputs. All 400 intentionally dishonest
controls falsely fulfilled. Stronger safety or malicious-sensor-resilience
claims are therefore unsupported.

B2 is a controlled, coverage-aware baseline with the same verifier and retained
evidence. That makes the recovery-target comparison fair. Its full-edge restart
can nevertheless be expensive by construction for localized missing work.
The measured improvement is useful but does not establish superiority over
practical resume policies or an intelligent missing-work planner.

The current proof establishes inclusion once conservative input footprints are
assumed. It does not by itself prove real calibration, timestamp alignment,
honest camera assertions, code refinement or universal recovery liveness.
The Python reconstruction is independently implemented but shares the declared
synthetic world/model; it is not an independent real-world measurement system.

Many attempts do not substitute for distinct environments. Current primary
coverage is a flat straight 40 m edge; sensitivity varies parameters, not a
representative distribution of terrain, airframes or actual imagery. Repeating
the same seeds after fixes does not create additional independent populations.

## Highest-value work before a strong defense claim

1. **Finish a precise novelty argument.** Compare the contract and failure
   semantics against the closest work above, including the limits of any claimed
   difference. Separate this contribution from the research group's earlier
   coordination/platform publications. Do not infer personal authorship from a
   coauthored publication alone.
2. **Strengthen comparisons and isolate mechanisms.** Retain B2; add a declared
   interruption-resume/fixed-rewind policy and a justified coverage-aware local
   repair comparator. Use equal observations, budgets and safety rules. A
   simplified simulator analogue must not be labeled an ArduPilot execution.
   Compare common-verifier policies for cost; use separate, clearly specified
   verification ablations for false-completion behavior. If the closest local
   repair policy is equivalent to VECR, report the equivalence and position the
   contribution in the contract/system rather than inventing an algorithmic gap.
3. **Establish a substantive technical result.** For example, analyze a complete
   observation-to-credit soundness condition including time/error bounds, a
   recovery-cost bound under explicit assumptions, or a rigorous implementation
   invariant/resource bound. Alternatively, justify and evaluate a sufficiently
   substantial systems contribution. More notation around a set union does not
   create a new theorem.
4. **Test transfer at the claimed boundary.** Complete separately versioned SITL
   validation with independent truth. Any claim about real image quality needs
   calibrated images/quality evidence. Demonstrating multiple external planner
   clients would strengthen reusability without implementing rotor-router or
   gossip. Choose extensions to match the intended dissertation claim.
5. **Obtain domain review of the precise thesis claim.** Have the supervisor and
   an external subject expert challenge the novelty, comparison and scope.
   Institutional requirements and the complete publication/dissertation record
   are outside this assessment.

These are recommendations, not executed experiments or new results. The frozen
v5/v5.1 campaigns remain complete under their protocols. Further comparisons
need a new versioned protocol and artifacts. SITL alone will not settle novelty.

## Author and institutional context

The original reader names Artyom Lazyan, the Institute for Informatics and
Automation Problems of the National Academy of Sciences of the Republic of
Armenia, and specialization E.13.04, Mathematical and Software Support of
Computing Machines, Complexes, Systems, and Networks. The institute's
[official department page](https://www.iiap.sci.am/department.php?did=2&dt=1&langcode=1)
lists Artyom Lazyan under the **Models of Computations and Logical Reasoning
Department**. The reader restores those details. It does not infer a supervisor,
approved title change or degree-award status from the staff listing.
