# ArduPilot SITL v6 — separate integration qualification

The final `confirmatory/` protocol assigns 24 flights (six conditions × GW/E ×
two noise seeds) using native Copter-4.7.2 flight dynamics. Raw autopilot accuracy
remains backend-specific; positive acceptance uses explicitly synthetic bounded
navigation and an ideal flat, route-aligned camera. No physical aircraft, real
image-quality or broad reliability claim is made.

* [Protocol and engineering amendments](../../../docs/vecr/SITL_PROTOCOL_V6.md)
* [Actual qualification report](../../../docs/vecr/SITL_VALIDATION_V6.md)
* [Frozen manifest](confirmatory/manifest.json)
* [Every reconstructed outcome and descriptive tables](confirmatory/analysis/)
* [Parameters, raw traffic, flight logs and truth](confirmatory/)

`pilot_01` preserves 11 infrastructure failures and one executed native-accuracy
control. Port collisions, Python/C++ clock mismatch and bootstrap handling were
corrected. `pilot_02` contains two nominal flights. `pilot_03` contains twelve
flights before the MAVLink speed fix; `pilot_04` repeats twelve after that fix.
These pilots are not pooled with the final qualification. Native controls and
safety interruptions are intended negative cases, not infrastructure errors.

## Reproduce to a new directory

SwarmKit experimental revision: `df65c61f57fade8ba88f877329ffd31d0ecbe175`.
ArduPilot release Copter-4.7.2:
`fb773567896014da007bca58358704d4a0d8516f`.
Use the manifest executable hashes, `host_environment.txt`, `python_environment.txt`
and `ardupilot_submodules.txt`. The latter explicitly lists uninitialized modules;
only the native SITL dependency subset was needed, not hardware board modules.

In a separate ArduPilot checkout, initialize the pinned waf, mavlink (recursive),
gtest, DroneCAN/DSDL, DroneCAN/pydronecan, DroneCAN/dronecan_dsdlc,
DroneCAN/libcanard, littlefs and lwip modules. With the recorded Python environment:

```sh
python waf configure --board sitl --no-submodule-update
python waf copter -j6
```

Build SwarmKit target `swarmkit_vecr_sitl_trial`. With committed experimental
source, from the SwarmKit root:

```sh
.venv/bin/python scripts/vecr/run_sitl_v6.py --mode final --ardupilot /path/to/ardupilot --output /private/tmp/vecr-sitl-replication --workers 4
.venv/bin/python scripts/vecr/validate_sitl_v6.py /private/tmp/vecr-sitl-replication
.venv/bin/python scripts/vecr/analyze_sitl_v6.py /private/tmp/vecr-sitl-replication
```

The runner creates only owned child simulators and localhost bridges; it does not
connect to physical hardware. Fixed nonoverlapping port blocks must be free.
Arming checks stay enabled. Its Python/C++ clock-domain probe rejects mismatched
epochs before dispatch. The executable is copied to an immutable run-specific
path before any assignment. Frozen directories are never overwritten.

Verify archived hashes with `scripts/vecr/hash_research_v6.py --collection sitl`.
The independently reconstructed arrival and re-entry are sampled, not continuous
guarantees. SIM_STATE has no acquisition timestamp: the assumed <=50 ms upstream
transport age is not proven. Synthetic camera geometry does not validate actual
images. Pilot_01's `.tlog` files are raw MAVLink streams without timestamp framing;
later pilots and final flights name that content correctly as `mavlink.raw`.

Final execution completed: 24/24 records independently validated, 12 fulfilled and 12 intended negative outcomes. See the report for measured costs and limitations.
