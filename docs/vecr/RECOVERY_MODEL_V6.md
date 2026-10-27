# VECR v6: conditional recovery under bounded sensing

Written 2026-10-27 before experimental results. This specifies a restricted method,
not a claim of first invention or global optimality. Runtime assurance, guaranteed
sensed sets, local coverage repair and asynchronous monitoring have prior work.

## State and acceptance

An external client assigns a straight directed edge of length L and a full-width
survey strip. C is the union of accepted conservative capture intervals; G is
[0,L] minus C. Evidence carries acquisition time and exact mission/revision/edge/
attempt/calibration identity. A bounded pose history supports consistency checks
at acquisition time rather than at delivery time. History retains at most 2,048
poses and 10.5 seconds; capture age is explicitly limited to at most 10 seconds.
Eviction can reduce availability, never justify invented evidence. Capture inputs
still require a calibrated, honest, flat-ground, route-aligned nadir adapter.

Only accepted observations change C. A planned capture, a successful command,
waiting for evidence, or a recovery target never changes C. Acceptance additionally
requires the existing bounded stationary arrival condition at B. All policies use
this same verifier, authority checks, deadline and recovery budget.

## Restricted recovery action class

The vehicle returns to an on-edge coordinate q no farther forward than current
projected progress p, establishes stationary bounded re-entry, then proceeds to B.
Let g be the first missing coordinate. Conditional future-sensing assumptions are:

* a: hard radius of actual re-entry position about q, established by the agent;
* v: physical upper bound on speed (not commanded speed);
* T: maximum time to the first qualifying capture after continuation, and maximum
  interval between qualifying captures during the repaired sweep;
* e: upper bound on each capture's declared hard horizontal position error;
* H: lower bound on the uncertainty-free along half-footprint computed from
  (estimated altitude minus its hard error) and calibrated field of view.

Assume each such capture genuinely qualifies across the full strip; acquisition
positions are correctly time-aligned; camera and vehicle origins coincide; along
progress proceeds toward B; the controller and sensor remain within this envelope.
Failures of these assumptions are experimental cases, not guaranteed closure.

The accepted half-interval is at least H-e. At the first capture its estimated
center can be at most q+a+vT+e. Its accepted left endpoint is therefore at most

    q + a + vT + 2e - H.

The robust first-capture overlap constraint is q <= g-a-vT-2e+H. The decision is

    q* = min(p, g-a-vT-2e+H),   provided q* >= 0.

If this expression is negative, there is no on-edge q satisfying this sufficient
constraint. The implementation retries at A and explicitly reports that closure
is not guaranteed. Actual observations still determine success. The additional
condition vT+4e <= 2H ensures consecutive accepted intervals cannot separate under
these worst-case independent estimator errors. This condition does not prove that
B is covered; an accepted endpoint capture or overlapping footprint must do that.

## Conditional results and their limits

**First-capture start bound.** Substitution of q* into the upper endpoint bound proves
that the first accepted interval begins no later than g, provided the envelope
holds and a feasible q* exists. For independent interval uncertainties attaining
the stated extremes, choosing any larger q can violate this overlap constraint.
This is a robust constraint result, not a new general coverage theorem.

**Restricted travel minimum.** For actual current coordinate (x,y), on-edge
re-entry q <= p = clamp(x,0,L), straight transit and forward sweep have idealized
cost sqrt((x-q)^2+y^2) + L-q. Its derivative in q is
(q-x)/sqrt((x-q)^2+y^2)-1 <= 0 (use the continuous piecewise form at y=0).
The largest feasible q therefore minimizes this cost in this restricted action
class. It does not optimize acceleration, yaw, waiting, obstacles, energy, new
faults, image acquisition or arbitrary routes. An omniscient offline solver is
not a fair online comparator. The theorem does not promise measured cost gains.

**Acceptance soundness.** If every qualified interval lies within genuinely
surveyed ground and arrival bounds hold, reported fulfillment implies the stated
physical obligations up to tolerance. History, waiting and target selection do
not weaken this condition. False calibration/quality/bounds invalidate it.

**Bounded waiting.** At a currently verified stationary B, a declared wait W may
allow in-flight evidence to arrive. Every telemetry update still checks safety,
arrival validity and the overall deadline. Departure triggers immediate recovery;
leaving B resets the wait. W is a maximum wait per endpoint visit, not a claim that
all pending captures will arrive. There is no future delivery oracle or hidden
knowledge of which gaps will disappear. Missing evidence after W causes the
chosen policy's repair. Waiting can waste time when evidence is permanently lost.

No unconditional completion/liveness theorem follows: repeated failures, unknown
bounds, infeasible first-capture conditions or exhausted budgets can interrupt.

## Mechanism separation

B2 restarts at A. D resumes at the last observed departure point (or the current
endpoint if no departure exists). R2/R8 rewind 2/8 m from that point. P repairs the
first gap with 2 m overlap. G repairs the first gap with zero overlap; GW adds the same endpoint wait as E. E0 uses q*
without waiting. E uses q* plus the same declared delivery horizon as its endpoint
wait. All retain disjoint evidence and share capture history and acceptance.
R2/R8 are simulator analogues, not executions of ArduPilot's mission-rewind code.
G is an intentionally strong simple coverage-aware comparator. Equality or better
performance by G limits the claim; it must not be concealed.

## Research positioning

The candidate contribution is the specified and tested combination of acquisition-
time evidence retention, conditional overlap calculation, endpoint waiting and
bounded onboard execution through a reusable contract. The algebra above is
specific and modest. Neither set union, monitor/recovery architecture, delayed
streams nor local gap repair is claimed as a first invention. Comparative results
must determine the useful operating region and cost of conservatism.
