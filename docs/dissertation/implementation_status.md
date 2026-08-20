# SwarmKit dissertation implementation status

Last validated: 20 August 2026

Branch: `main`

Audited base commit recorded by the campaign: `be71251d25f7544b7e913e6f6f984f88b450e1fc`

## Acceptance-gate status

| Gate | Status | Evidence |
| :--- | :---: | :--- |
| Common oracle uses selected-evidence contract health only | PASS | Shared `EvaluateAcceptedOutputValidity`; oracle regression matrix |
| No undeclared current truth-health predicate | PASS | Healthy selected evidence remains valid across current-health transition test |
| Actual clock operands, version, and incarnation bound in `CERT_V3` | PASS | Engine-captured `ClockEvaluationEvidence`; E1/`clock-v1` to E2/`clock-v2` regression |
| Independent verifier reconstructs certificate semantics | PASS | Verifier does not invoke `StateAcceptanceEngine`; rehashed semantic mutation matrix |
| Participant set and membership revision checked | PASS | Direct verifier reconstruction and mutation regressions |
| Propagation model/version and speed assumptions checked | PASS | Direct verifier reconstruction and 3D propagation regressions |
| GPS minimum-quality semantics checked by engine and verifier | PASS | Missing, wrong-variant, below-threshold, and threshold-boundary tests |
| Request event stores canonical contract hash | PASS | Replay binding check and changed-contract rejection test |
| Persisted post-`r*` / pre-`t*` regression | PASS | JSON-lines reload selects A and excludes causal-but-late B |
| All required semantic mutations rejected after rehash | PASS | 44/44 classes; 1,011,164/1,011,164 cases |
| Debug and Release test gates | PASS | 163 cases, 1,523 assertions, 0 failures; CTest 2/2 in both builds |
| macOS UBSan gate | PASS | CTest 2/2; no UBSan finding |
| Nine-scenario smoke activation | PASS | 3×9×100 smoke; all intended mechanisms nonzero |
| Protocol frozen before final campaign | PASS | `final_experiment_protocol.md` was completed before the Release run |
| Final 50×9×100 Release campaign | PASS | 45,000 requests per method; manifest build type `Release` |
| Persisted replay agreement | PASS | 22,981/22,981 agreements; 0 disagreements |
| Replicate-cluster statistics and complete CIs | PASS | 10,000 draws, seed 123456789, 50 replicate clusters |
| Calculated containment result | PASS | 0 failures observed among 68,943 evaluated enclosures; ratios exported |
| Exact end-to-end Release benchmark and raw samples | PASS | 100 warmups + 1,000 saved samples for N=3,5,10 |
| Documentation regenerated from final artifacts | PASS | Final summary, report, status, and machine-readable results agree |

ASan and TSan were not reported as executed: the repository exposes their presets for Linux,
not for this macOS campaign.

## Final generated measurements

- `B0`: FV 20.591%, availability 98.920%, UAR 20.369%.
- `B1`: FV 23.387%, availability 86.944%, UAR 20.333%.
- `P`: FV 0.000%, availability 51.069%, UAR 0.000%.
- Paired `P-B1` FV difference: -23.387 percentage points, 95% replicate-cluster bootstrap CI
  [-23.426, -23.349] percentage points.
- Deterministic enclosures: 68,943 evaluated; 0 observed containment failures; maximum
  truth-error/enclosure-radius ratio 0.807200.
- Persisted replay: 22,981 agreements out of 22,981 decisions; 0 disagreements.
- Mutated/inconsistent certificate verification: 44/44 classes and
  1,011,164/1,011,164 instantiated cases rejected.
- Release N=10 end-to-end benchmark: mean 406.746 µs, p95 426.000 µs, median certificate
  7,744 bytes on Apple M3 Pro macOS ARM64.

No false-valid events were observed for `P` in the final campaign. This is an observed finite
campaign result, not a universal failure-probability claim.

## Implemented subsystem status

| Subsystem | Status | Canonical implementation |
| :--- | :---: | :--- |
| Evidence frontier and deterministic selection | COMPLETE | `state_acceptance_engine.cpp` |
| Clock interval arithmetic and incarnation binding | COMPLETE | `clock_quality.h`, engine selection state |
| `CERT_V3` canonical binding | COMPLETE | `state_acceptance_certificate.*` |
| Independent semantic verifier | COMPLETE | `state_acceptance_verifier.*` |
| JSON-lines persisted replay | COMPLETE | `replay_trace.*`, experiment runner |
| Shared accepted-output oracle | COMPLETE | `state_acceptance_experiment.cpp` |
| Fault activation and diagnostics | COMPLETE | `fault_injection.cpp`, replicate CSV export |
| Replicate-cluster bootstrap and distributions | COMPLETE | experiment/statistics exporters |
| Mutation verification | COMPLETE | 44 semantic/content-consistency classes |
| Release scalability benchmark | COMPLETE | experiment CLI, raw sample export |

## Scientific boundary

The implementation is ready to supply the dissertation result artifacts under the frozen
protocol. Its deterministic guarantees remain conditional on valid declared upstream hard
bounds. `SimBackend` results, unsigned consistency hashes, and host benchmarks must not be
interpreted as universal field safety, adversarial authenticity, or target embedded timing proof.
