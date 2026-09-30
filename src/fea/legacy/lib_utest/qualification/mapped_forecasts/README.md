# Read-only mapped participant forecasts

QEPH, T3, QBAT and TYPE25 expose `ForecastMapped`, returning their own
`MappedForecast { report, footprint }` by value. No allocation, owner lookup,
coefficient readback, state mutation or source admission occurs. A prospective
fresh stamp describes the requested configuration; it is not an authenticated
owner identity. Every existing `InitializeMapped` proof remains required.

Each wrapper calls the same private `MakeForecast` used by initialization with
the real private object width. Q/T's existing failure-section forecast now also
returns the exact already-computed mixed, failure and optional genuine one-point
device arena sum. No layout, hard cap, force arithmetic or history changed.

`device_bytes` is the exact explicit arena total, excluding CUDA runtime overhead.
`startup_host_bytes` is the existing inclusive admission bound. It partitions
into `source_host_bytes`, `participant_host_bytes` and `startup_scratch_bytes`.
The source charge is the physical handle's inclusive retained payload, so a
composer must prove shared immutable backing before deducting repeated charges.
The participant charge remains an upper bound: small temporary curve offsets
and existing alignment allowances remain charged. Scratch conservatively reserves
all initial upload arenas plus the raw-owner proof together even though the proof
finishes before upload. This preserves the older limit semantics and supplies a
safe per-construction-phase reservation, not measured RSS. No previous 12/8 GiB
app source-construction cap should be treated as a new runtime allocation.

The host qualifier covers all four value APIs, exact device and complete host cap
admission, one-byte-short rejection, overflow partition rejection, wrong complete
family and oversized borrowed witness count with retry. The mapped fixture has
actual PART/plain/CIN roles, a true NIP1 sidecar and the four-point QBAT family.

Author: six production units syntax-pass, all three host functions pass under
1 CPU/512 MiB using freshly compiled changed units and cached qualified fixture/
unchanged module libraries. Report `/tmp/mapped-forecast-host-1.json`; no native,
GPU or CUDA runtime operation was run by the author. Root owning CMake/Bazel
builds remain required. Existing mapped/source gates remain separate evidence.

Root owning gate:

```sh
cmake -S lib_utest/qualification/mapped_forecasts -B /path/to/build
cmake --build /path/to/build --target mapped_forecasts_host_test --parallel 1
ctest --test-dir /path/to/build -R '^mapped_forecasts_host$' --output-on-failure
```

Affected compile targets: QEPH/T3/QBAT/TYPE25 batch values and batches, shared
shell plasticity storage. Owning Bazel target is
`//lib_utest/qualification/mapped_forecasts:mapped_forecasts_check`; it links the
existing TYPE25 batch library but the executable only invokes host forecasts.
