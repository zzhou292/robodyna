# T3 startup/rates port: first execution passed

On 2026-09-09 the first fresh configure/build and all **27 function executions**
passed: ten new host port functions, three new actual CUDA functions and the
fourteen unchanged native startup/rates regressions. No equation, acceptance
threshold or numerical tolerance changed after execution.

Evidence:

- [Configure](../../../../../crash-work/reports/t3-port-r1r2-configure-1.json)
  and [build](../../../../../crash-work/reports/t3-port-r1r2-build-1.json).
- [Host/native report](../../../../../crash-work/reports/t3-port-r1r2-host-tests-1.json)
  and [host/native XML](../../../../../crash-work/reports/t3-port-r1r2-host-xml-1/).
- [Actual CUDA report](../../../../../crash-work/reports/t3-port-r1r2-cuda-tests-1.json)
  and [CUDA XML](../../../../../crash-work/reports/t3-port-r1r2-cuda-xml-1/).

The unchanged [pre-execution contract](README.md) and source manifest retain
the frozen domain and budgets. Its 54-record verifier passed against the
independent native source closure (50 original files, 17 extracts, 42 include
files). Manifest SHA256:
`2d2ee6a9cfa0a1fbf641ea8fefd5a4e7837b6a32eb8e5b4487af0c58a9e3b166`.

The CUDA tests exercised 27 startup and 54 prescribed-rate configurations,
plus cutoff, malformed, late-arithmetic and clean-retry cases. The same
production headers ran on host and GPU; comparisons retained the native
R1/R2 references and independent long-double startup/affine/angular oracles.
The actual test packet allocation was **1,264 bytes in one allocation**, with
one thread per block. This records explicit test storage, not total CUDA
context memory or an element-batch throughput result.

Root executed each stage under the shared workstation guard. Configure/build
used two CPUs; host and CUDA tests used one CPU. Guarded elapsed times were
1.757s configure, 9.779s build, 0.252s host/native and 0.439s CUDA. These short
qualification timings do not establish production performance or scalability.

Qualified scope is three-node startup geometry, native angle-weighted mass
and A/4.5 inertia, plus prescribed current geometry and complete selected
quarter-step rates. The FP64 cutoff bands remain numerical admission, not
exact-predicate guarantees or native cutoff acceptance parity. Force/history
porting, resident T3 batches, joint dynamics, source MAT024/NIP3 behavior,
full-part integration and vehicle crash simulation remain outside this gate.
