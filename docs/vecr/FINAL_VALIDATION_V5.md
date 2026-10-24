# VECR v5 validation record

## Engineering gates before confirmatory execution

* Baseline audit completed before source changes; v4.2 2,400 rows and 107 artifact
  hashes independently revalidated. No historical result changed.
* Release CTest: 2/2 targets passed (full library/integration suite).
* UBSan CTest: 2/2 targets passed (same source runtime fixes).
* Focused new v5 tests: 5 cases, 14,819 assertions, including 3,000 randomized
  measured/actual footprint pairs and independent subcell-hole/rotation controls.
* SDK install and external C++ consumer compile/link: passed with the installed
  `swarmkit::client` target and the release dependency toolchain.
* Final engineering pilot: 234/234 attempts completed; 20,302 delivered captures
  and 207 physical re-entry dwells independently reconstructed and checked.
* Pilot 02 retained one startup infrastructure failure. Its generic error did not
  preserve the specific cause; later runs use isolated state and detailed errors.
  Pilots used work-in-progress source, not a clean source freeze; tracked patches
  are retained, but early untracked source snapshots are not complete. They are
  engineering evidence, excluded from confirmatory statistics.
* Initial sandbox-only integration invocation could not bind loopback sockets;
  reruns with authorized local sockets passed. The installed-consumer check first
  used the wrong exported target spelling and then passed with `swarmkit::client`.

Confirmatory campaign and isolated benchmark results are recorded below after
execution against the preregistered source commit. No SITL is authorized in v5.
