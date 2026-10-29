# VECR — defensible revised contribution and evidence briefing

**Scientific claim.** VECR supervises physical execution of a graph-assigned UAV edge. It conservatively tracks verified survey intervals, detects sustained in-flight deviations, attempts bounded local recovery toward the first required unverified interval *before reaching B*, and returns an exact arrival/coverage result without autonomously revisiting gaps from B. Rotor-router or another graph algorithm remains external.

**Two interfaces.** Pull `getEdgeStatus(id)` whenever a caller needs progress, accepted/unverified/pending intervals, current execution phase, recovery attempts and reasons. Push exactly one terminal `onEdgeExecutionFinished(result)` on verified arrival, bounded failure, cancellation, or unknown outcome. An arriving drone with an unresolved interval receives `ARRIVED/INCOMPLETE`, not a hidden correction command. Control is handed back to the calling mission application.

**Why locally bounded?** The SDK can own finite repairs on an assigned edge but cannot decide global UAV scheduling or task reassignment. Recovery retries, deadline, energy, safety and command-authority rules bound its work. An unreachable re-entry point produces an explicit terminal failure.

**Conditional guarantee.** Given truthful calibrated capture metadata and hard pose/height bounds, each conservative accepted footprint is a subset of actual acquisition. Union preserves containment; if every required interval is accepted, coverage is supported under those assumptions. The rule does not prove that real imagery is truthful or that any deviation is always recoverable.

**Originality boundary.** Combining the local edge execution contract, evidence union, explicit in-flight correction guards, independently reported physical arrival/coverage and algorithm-independent SDK surface is the research contribution. Backtracking, interval arithmetic and rotor-router mathematical results are established prior art.

**Evidence honesty.** The uploaded v7 binary automatically revisited missing intervals after arrival at B. Thus v7 whole-campaign completion/distance statistics are **not measurements of the revised method**. The frozen N07–N09 in-flight-only subset demonstrates a related mechanism and is labelled historical. The old N03/N04/N06/N17/N18 distance advantages must not be used. A new compiled source revision and evaluation campaign are required.

**Defense demonstration sequence.** 1) Show sustained deviation at ~14 m during A→B, correction before B, and resumption after physical re-entry. 2) Show safe B arrival with a missing [18.2,21.6] m interval, then a single `ARRIVED/INCOMPLETE` callback and zero post-arrival `GOTO` commands. 3) Show `getEdgeStatus` polling during execution. 4) Show command refusal or exhausted budget with one `FAILED` event. 5) Present historical N07–N09 cautiously and state exactly which new experiments remain.

**No overclaim.** This revision does not yet establish the new implementation's numerical performance, actual SITL behavior, global coverage guarantee, optimal re-entry route, real-world camera accuracy, or rotor-router scheduling outcomes.
