# Contribution in relation to prior work

VECR contributes a specified onboard edge-execution, evidence and recovery contract,
an earlier implementation lineage and an experimental validation plan. The revised endpoint-reporting semantics still require implementation and rerun. It does not claim to
invent set union, monitoring or returning behind an interruption. Engineering
quality and empirical superiority are separate from priority.

| Primary source | Established overlap | VECR scope and remaining boundary |
|---|---|---|
| [ArduPilot: Mission Rewind on Resume](https://ardupilot.org/copter/docs/common-mission-rewind.html) | Returns behind an interruption point using a configured distance, motivated by missed survey work | Fixed-distance resumption does not account for every previously unverified interval. This is not a claim that ArduPilot's entire mission system was benchmarked. |
| [Zudaire et al., ICRA 2021](https://software.imdea.org/~cesar/publication/2021-zudaire21assumption/) | Runtime monitoring of UAV plan assumptions and mitigation, with real and simulated flights | Generic monitoring plus correction is prior art. SwarmKit specifies reusable edge identity, acceptance, retained disjoint evidence and bounded agent execution; a direct reimplementation comparison was not performed. |
| [Arvanitakis et al., 2018](https://pure.coventry.ac.uk/ws/files/13602376/1729881417750785.pdf) | Conservative sensed regions with bounded localization uncertainty and their union | Conservative coverage unions are not claimed as new. VECR uses qualified acquisition-time evidence inside an assigned-edge execution contract. |
| [Nykolaychuk and Ortmeier, 2015](https://doi.org/10.1007/978-3-319-22876-1_31) | Coverage path replanning for processing faults | Repair of missed processing is prior art. Comparison here concerns a restricted assigned-edge action class; no global planning superiority is claimed. Publisher abstract was available; full chapter access was unavailable. |
| [Leucker et al., 2020](https://link.springer.com/article/10.1007/s11219-019-09493-y) | Correct runtime monitoring of asynchronously arriving real-time streams | Delayed-event handling is established. SwarmKit's contribution is its bounded physical acquisition consistency implementation; it is not a new general stream-monitoring theory. |
| [Fields2Cover, 2023](https://arxiv.org/abs/2210.07838) | Modular reusable coverage planning and evaluation | Planning remains external; prior SDK paths demonstrated reuse of the historical v7 implementation only; the revised API must be tested. |
| [Bouman et al., 2023](https://arxiv.org/abs/2302.03164) | Adaptive coverage planning incorporating accumulated sensing and execution constraints | VECR handles already-assigned edges; it does not outperform general coverage planners on their problem. |

Primary sources were consulted in the 26–28 October review. This is a targeted
comparison, not a systematic exhaustive priority search. It identifies overlap;
it cannot prove absence of any earlier equivalent method.


Future rotor-router integration is an application-level dependency on physical
execution outcomes, not an implemented graph algorithm. Abstract rotor-routing
models are reviewed by [Holroyd et al.](https://arxiv.org/abs/0801.3306). Their
guarantees are not automatically inherited by asynchronous physical execution.
