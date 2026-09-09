# Prescribed nodal wall phase diagnosis

This optional executable diagnoses the retained 3.405312 ms median failure;
it does not replace or relax the separate 2 ms median, 5 ms maximum and 20x
contemporary integral speedup gate. The first profile passed; no optimization has been applied. The
[first-execution manifest](../../../crash-work/checkpoints/source-nodal-wall-first-execution-1/manifest.json)
retains 233 files / 49,289,881 bytes, including the actual binary, workspace
source dependencies, linked workspace runtime, inputs, reports and XML. Its
SHA256 is `5feec919f950106c8c68f84d71d1c1681b4324defad4e88df8d6403ad3a32be8`.
The [diagnostic supplement](../../../crash-work/checkpoints/source-nodal-wall-first-execution-1/diagnostics.json)
binds read-only binary resource inspection: the original nodal kernel used
255 registers and 808 stack bytes per thread. These figures alone do not
identify which operation dominates runtime.

`SourceNodalWallCuda.cu` instantiates the same kernel with a compile-time boolean.
The false instantiation performs the existing arithmetic without clock writes;
the true instantiation adds `clock64()` timestamps and trace writes at the
existing phase boundaries. Both retain one block of 128 threads, every original
barrier, all 117 source nodes, 94 parents and 100 actual wall triangles. The law,
face query, fixed ordering, validation, budgets and failure publication stay
the same. Separate caller objects and device allocations avoid trace data in
the original `Storage`; their combined explicit allocation is below 2 MiB.
No stack limit, register cap, cache preference or device setting is changed.

The single test uses the authentic coherent source fixture. It evaluates the
owning CPU oracle once, warms each kernel once, then executes five calls of
each. Every control result is checked against that CPU result with the existing
certificate/arithmetic bounds; every instrumented result must match the control
field by field exactly. Both event durations and checked end-to-end durations
are recorded, so instrumentation overhead remains visible.

Raw unsigned SM cycles are retained for serial input validation, every node's
actual face query and law/share reduction, node-status scan, every parent's
certificate reduction, serial global reduction and final publication. All
threads run in one block on one SM; timestamp ordering is checked across the
existing barriers. Thread spans include warp scheduling delays. Query and law
windows can overlap across warps, so their fractions are descriptive and are
neither additive exclusive costs nor converted to milliseconds. The global
event time is the measured whole-kernel duration. Trace output is diagnostic;
the instrumented compiler may schedule differently from the control.

Read-only attributes for both kernels include registers, local/shared memory,
architecture, maximum threads, and occupancy API estimates. The estimates are
not achieved occupancy. Free-memory readings separately identify context,
attribute/module loading, object allocation, warm execution and measured
execution. They include CUDA runtime backing beyond the explicit allocations.

Root registration: `robo_dyna_source_nodal_wall_profile` contains
`SourceNodalWallCuda.cu` and `source_nodal_wall_profile.cpp` only, links the same
nodal fixture, wall tessellation, canonical artifacts, CUDA runtime and
`GTest::gtest` as the existing gate. The new test has its own main. It needs the
same strict FP64 flags and `--expt-relaxed-constexpr`. Do not add the original
test main or old integral CUDA source to this profiling target. Root owns build
registration and execution. The first profile now passes as recorded below.

Planned create-only report invocation from the workspace root after building:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/source-nodal-wall-profile-1.json \
  --lock crash-work/reports/workstation.lock \
  --cpus 1 --min-available-gib 32 --max-rss-gib 1 --timeout 120 \
  --gpu 0 --min-gpu-free-gib 8 --max-gpu-growth-gib 4 -- \
  crash-work/build/source-contact-c5c1/qualification/source_contact/robo_dyna_source_nodal_wall_profile \
  crash-work/reports/yaris-part-2000157-readiness-1.json \
  crash-work/assets/yaris-wall/manifest.json \
  --gtest_output=xml:crash-work/reports/source-nodal-wall-profile-xml-1/
```

## First measured result

The [bounded profile report](../../../crash-work/reports/source-nodal-wall-profile-1.json)
and [XML](../../../crash-work/reports/source-nodal-wall-profile-xml-1/) pass all
clock ordering, CPU/control and exact control/instrumented field comparisons.
The control median is 3.572256 ms and instrumented median 3.378944 ms; the
0.945885 event ratio illustrates scheduling/compiler variability and is not a
speedup claim. Both kernels report 255 registers, 808 local bytes, zero static
shared memory and an estimated two resident blocks per SM. Combined explicit
control/profile storage is 770,392 bytes; measured process RSS peaked at
174,997,504 bytes, with no post-warm device-memory growth.

In representative run 2, actual face-query span is 72.53% of the block cycle
window (whole node stage 75.78%), serial validation 11.26%, global reduction
7.63%, parent certificates 0.86% and publication 2.10%. Query/law overlap and
instrumentation caveats above apply. Face query is the measured first target.
The next candidate is a conservative prepared-face bounding-box rejection
that preserves actual closest-triangle calculation, stable owner ties and
coverage/error behavior. Keep the old integral and this profile immutable.
The source contact cost gate still fails until its complete unchanged test
passes with the candidate; these measurements do not waive it.
