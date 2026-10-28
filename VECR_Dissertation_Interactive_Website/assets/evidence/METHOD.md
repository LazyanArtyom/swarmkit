# Verified Edge Coverage and Recovery

## Contribution

VECR is a reusable onboard execution, verification and recovery method for assigned
UAV graph edges. The external application owns graph selection and coordination.
The SwarmKit agent owns exact attempt identity, qualified physical evidence,
retained partial work, missing-interval detection and bounded local recovery.

The systems contribution is this explicit contract and its tested composition.
Returning to an interruption point, conservative set unions and runtime monitoring
are established techniques. No universal superiority or worldwide priority is
claimed. A future rotor-router application can depend on this physical execution
interface; no rotor-router or gossip algorithm is implemented or validated here.

## One production method

The SDK and protobuf expose one VECR recovery method with no policy selector:

1. Bind the request to mission/revision/edge/attempt/drone identity and a fixed route.
2. Accumulate conservative qualified survey intervals and bounded traversal evidence.
3. On sustained corridor departure, re-enter at the earlier of current projected
   progress and the earliest missing interval. Without missing survey work, use
   current progress to restore the route.
4. At a verified stationary endpoint, wait for the configured bounded evidence
   delivery period. Waiting never credits a future acquisition and remains subject
   to endpoint freshness, safety and the attempt deadline.
5. If a gap remains, return to its beginning, verify physical stopped re-entry,
   then resume toward B. Retain disjoint previously verified work.
6. Fulfill only when the requested predicates hold. Exhausted retries, unknown
   evidence, backend refusal and safety intervention are explicit outcomes.

For progress p and earliest gap start g, the local target is q=min(p,g) during
sustained departure and q=g for endpoint repair, clipped to the route. This is a
repair heuristic, not a guarantee that another pass will acquire valid evidence.
No sensing-envelope optimizer or fixed-overlap policy remains in the public API.
Uncertainty remains essential to evidence acceptance and physical arrival.

The research-only full-restart baseline chooses q=0. Its separately compiled,
non-installed executable shares verification, evidence waiting, faults, dynamics,
retry budgets and safety. It is a comparator, not a second production API.

## Physical obligations and identity

Arrival, traversal, survey and traversal-plus-survey are distinct obligations.
ACK is not arrival; arrival is not surveying. Survey alone need not imply travel
inside a corridor because a wide camera can observe from outside it. The agent
supports straight local metric segments and rejects bent or multipoint routes.
WGS84 conversion uses a local equirectangular approximation; global routes, poles
and longitude wrap require a separately qualified adapter.

Requests and captures bind mission/revision/edge/attempt identity. Captures also
bind a configured calibration. Duplicate, stale, mismatched and future captures
are rejected. The agent keeps at most 2,048 bounded position samples (up to 10.5 s)
and checks capture consistency at acquisition time against a sample within 500 ms.
Timestamp separation needs a declared physical speed bound. Capture age is bounded
by configuration (at most 10 s). Delivery after the attempt deadline is rejected.

Stopped arrival requires qualifying bounded position, altitude and speed samples
at least 200 ms apart. Later invalid or away evidence revokes current endpoint
qualification. Final sensing cannot use an endpoint observation older than 1 s.
A terminal result describes satisfaction at its recorded time, not eternal presence.
The simulator oracle checks sampled dwell; continuous dwell is not empirically proved.

## Conditional guarantees

**Survey soundness.** Let C_j be the accepted conservative footprint of acquisition
j and A_j the ground actually imaged at the required quality. If C_j is a subset of
A_j for every accepted acquisition, their union is a subset of the actual observed
union. Full required-strip containment in that union therefore implies actual
coverage under those assumptions. Nonzero missing allowance and numerical
comparison tolerances qualify the conclusion. This elementary inclusion argument
is not claimed as a new set-theoretic theorem.

The substantive implementation obligation is conservative input qualification:
truthful calibration and quality assertions, hard localization/altitude bounds,
accepted timestamps, flat ground and rectangular nadir route-aligned footprints.
Authentication proves adapter identity, not physical truth. A real calibrated
camera producer has not been validated.

**Bounded traversal.** With endpoint route distances D0,D1 including hard pose
errors, physical speed bound V and interval duration dt, route distance throughout
the interval is at most (D0+D1+V*dt)/2. This follows from the 1-Lipschitz distance
function and the two endpoint motion bounds. The bound must fit the corridor.
Only forward progress [s0+e0,s1-e1], when nonempty, can be credited. Impossible
jumps, overlapping error intervals and unknown bounds cannot create traversal.

**Retained gaps.** A sorted interval union preserves holes and disjoint work. A
maximum progress coordinate cannot replace that union. Camera centers retain
signed along-route coordinates; clipping applies to footprints, not their centers.

**Conditional mission composition.** If an external application assigns every
required strip and each is soundly fulfilled, their union is covered to the declared
allowances. This gives no liveness, scheduling or fault-tolerance guarantee for the
external graph algorithm. A future adapter must define pending, interruption,
unknown, retry and reassignment semantics. Postponing rotor updates alone does not
prove preservation of a classical rotor-router theorem.

## Execution and safety

Recovery remains in the agent, including command arbitration, health, authority,
battery and navigation checks. ACK does not release recovery: physical re-entry
and stopped qualification precede continuation. Attempts, cooldown and deadline
bound recovery. Native production MAVLink recovery remains disabled until its
uncertainty and operational safety envelope are qualified. An explicitly isolated
ArduPilot SITL adapter permits synthetic bounded observations for research.

An append-only attempt journal is fsynced before dispatch. Exact duplicate requests
cannot dispatch twice; durable terminal results can be replayed. An unfinished
attempt after restart remains unknown. Changed request/client identity, corrupt or
unwritable journals and concurrent locks block dispatch. This is fail-closed
duplicate suppression, not guaranteed mission resumption after a crash.

## Evidence limits

Digital simulation uses production agent/SDK/backend execution and independent
geometric reconstruction. SITL runs actual autopilot software with simulated flight
dynamics; positive sensing uses synthetic bounded navigation and an ideal camera.
Native accuracy estimates are not silently converted to hard bounds. Real images,
lighting, visual occlusion, terrain, payload saturation, physical aircraft,
obstacle/geofence-aware recovery and operational multi-UAV reliability remain
outside this validation.
