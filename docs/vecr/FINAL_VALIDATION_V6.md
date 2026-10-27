# VECR v6 digital validation — 28 October 2026

## Completed research question

Does a bounded sensing-envelope re-entry rule improve an onboard edge-obligation
executor over practical resume/rewind and strong coverage-aware recovery rules?
The frozen experiment answers **no measured advantage over the strongest simple
coverage-aware comparator in this matrix**. It does establish useful differences
between missing-work tracking, delayed-evidence waiting and conservative targeting.
No condition, failed attempt or comparator was removed to improve that conclusion.

## Implementation and conditional result

The production core, AgentService protobuf, C++ SDK, agent and CLI expose one
canonical recovery-policy model. All policies preserve accepted disjoint intervals,
share exact attempt identity, bounded arrival, safety, evidence validity and retry
budgets. Acquisition-time pose history validates delayed observations against their
acquisition context; it is bounded by 2,048 poses and 10.5 seconds. Invalid, expired,
future, mismatched or inconsistent observations cannot create coverage. Missing
hard-speed evidence cannot justify temporal position alignment.

For earliest missing coordinate g, current progress p, arrival radius a, physical
speed bound v, maximum qualifying capture interval T, maximum declared pose error e
and minimum uncertainty-free half-footprint H, the implemented rule is
q = min(p, g − a − vT − 2e + H), when nonnegative. Otherwise it retries at A with
no closure guarantee. Its first accepted interval starts no later than g under the
stated envelope. This does not imply that the first image contains g. Consecutive
interval overlap additionally needs vT + 4e ≤ 2H. Under straight transit plus
forward sweep, the largest feasible q minimizes that restricted idealized distance.
It is not an optimality result for measured dynamics, energy, arbitrary routes or
failed sensors. Full derivation: [RECOVERY_MODEL_V6.md](RECOVERY_MODEL_V6.md).

Only accepted captures establish coverage. Waiting never credits future evidence.
At stationary B a bounded wait can avoid repair caused only by metadata latency;
safety, freshness, endpoint validity and the overall deadline continue to apply.

## Exact digital experiments

| Collection | Assigned / executed | Interpretation |
|---|---:|---|
| pilot_01 | 40 / 40 | Initial eight-policy mechanism check; led to addition of stronger GW comparator |
| pilot_02 | 396 assignments | Infrastructure failure: old executable did not support GW; excluded from inference |
| pilot_03 | 396 / 396 | Full pilot, independently reconstructed |
| pilot_04 | 198 / 198 | Final preflight, independently reconstructed |
| confirmatory | 3,960 / 3,960 | Preserved protocol deviation: accidental 20-seed runner default; not pooled |
| **confirmatory_v6_1** | **5,940 / 5,940** | Primary: 22 conditions × 30 paired seeds × nine policies |
| client_reuse | 200 / 200 | Five conditions × 10 seeds × two policies × two API clients |

Primary seeds are 30001–30030. Every primary assignment returned a result without
an infrastructure error. Source revision:
`ce09d3b47d54959eb86101c251d9f2c3ea62be40` (simulation implementation from
`4b9e19032`). Primary `per_attempt.csv` SHA-256:
`5c67d8b7bc8ff1f4b868ef32e041d8873ed2299ddc08d98053ca6ed25b8e32f7`.
The binary hash, frozen assignments, environment and full parameters are in the
campaign manifest. Later SITL integration changes do not change this identity.

Independent Python reconstruction checked **516,751 delivered captures** and
**3,984 physical re-entry dwells**. Geometry is reconstructed from private physical
acquisitions, without using the production interval-geometry implementation.
Evidence tiers remain separate: these are agent-backed SimBackend experiments,
not ArduPilot, physical aircraft or actual image-quality measurements.

## Policies and fair comparison

B2 restarts at A; D resumes at the recorded departure point; R2/R8 rewind 2/8 m
from that point; P is previous VECR with 2 m gap overlap; G targets the earliest
gap with zero overlap; GW adds a three-second endpoint wait to G; E0 uses the
derived envelope target without waiting; E adds the same wait. All share the
same bounded history/verifier. R2/R8 implement specified simulator analogues;
they do not reproduce the complete ArduPilot mission rewind subsystem.

## Measured findings

There are **5,400 honest-sensor attempts**, with **zero false fulfillment** under
all nine policies. This is conditional finite-sample evidence, not zero risk.
The separate dishonest controls expose the trusted sensing boundary. Per honest
policy, B2/P/G/GW/E0/E each truly and reportedly complete 360/600; D/R2/R8 each
truly complete 270/600 and report 210/600, leaving 60 false negatives. The 600
attempt mixture is a fixed stress design, not an operational mission distribution.

| Condition, contrast | Paired distance difference, m | 95% paired bootstrap interval | Joint successes |
|---|---:|---:|---:|
| N01 delayed metadata, GW − G | −6.22695 | [−6.29127, −6.16246] | 30 |
| N01 delayed metadata, E − GW | 0 | [0, 0] | 30 |
| N04 multiple gaps, E − GW | +6.24008 | [+6.23995, +6.24020] | 30 |
| N07 departure, E − GW | +5.02459 | [+5.00644, +5.03914] | 30 |
| N07 departure, P − B2 | −21.78619 | [−21.83863, −21.73499] | 30 |
| N09 hidden deviation, E − GW | +5.26398 | [+5.25725, +5.27067] | 30 |

N01 waiting saves 3.79333 s versus G (95% CI [−3.89333, −3.69333] for
GW−G). E and GW are identical in that condition. In N04, waiting adds about
2.99667 s (GW−G CI [2.98333, 3.00667]) because missing captures never arrive.
D/R2/R8 report 0/30 in N04; coverage-aware policies report 30/30. E buys no extra
completions over G/GW in the full matrix, and often travels farther. Sparse sensing
and the short deadline defeat all policies in their respective conditions.
Failures must not be ranked as efficient simply because they terminate sooner.

Intervals use 10,000 paired bootstrap resamples, RNG seed 20261027; success
proportions use Wilson intervals. Cost estimates restrict to jointly true and
reported successful pairs and show their denominators. Intervals describe random
variation in this simulator, not uncertainty in its fidelity; they are descriptive
pointwise intervals, not multiplicity-adjusted tests across the full matrix.
The complete 198 condition/policy rows and all contrasts are in `analysis/`.

## Reuse and reliability gates

The same executable agent serves direct `Client` and routed `SwarmClient` requests.
All **100 matched client pairs** agree in terminal phase, true/reported completion,
false fulfillment, distance, time, recovery count and reported coverage. Independent
reconstruction checked 11,120 captures and 80 dwells. This demonstrates two public
API entry paths, not two independent planning algorithms or universal reuse.
An installed SDK consumer also configures and compiles successfully.

Release and UBSan complete suites passed. V6 regressions cover target distinctions,
invalid/infeasible sensing envelopes, delay acceptance, expired/inconsistent capture
rejection, missing-speed bounds, endpoint wait and safety, and the native MAVLink
recovery guard. See `results/vecr/agent_campaign_v6/gates/` for actual logs. Further
SITL-specific regressions and results are reported separately with their revision.

## What the evidence supports

Supported: a reusable implemented onboard contract, conditional conservative
coverage accounting, acquisition-time evidence validation, bounded recovery and
quantified tradeoffs under a frozen stress matrix. Gap-aware repair can outperform
resumption and full-edge restart. Endpoint waiting helps bounded latency but hurts
permanent data loss. Conservative envelope targeting has an explicit sufficient
bound and a measurable cost without a demonstrated acceptance benefit here.

Unsupported: that E is the best general recovery algorithm, first invention of
local repair or uncertainty-aware coverage, a new general coverage theorem, real
camera quality, arbitrary terrain/obstacles, fleet scalability, operational risk
rates, or dissertation acceptance. The stronger comparisons narrow the claim;
they do not justify inventing another positive result. See the dated v6 literature
assessment and separately labeled SITL qualification record.
