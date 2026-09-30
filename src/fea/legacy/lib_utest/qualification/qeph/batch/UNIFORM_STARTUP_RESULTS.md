# Explicit uniform-translation startup results

All five new CUDA functions now pass, along with 41 retained batch/coupled,
CW0, T3, immutable-binding and mixed-publication functions. The new paired
flight/known-force tests cover 24 native cell intervals. First binding requires
authentic live-owner sources before measured initial kinetic energy becomes
visible; no extra owner, clock, allocation or dt=0 force call is introduced.

The first numerical execution passed four of five new functions and all 16
QEPH/CW0 regressions. The final new function reused a six-node snapshot while
reading a four-node owner, leaving two untouched tail entries. A fresh local
snapshot restores the intended exact whole-buffer comparison. Production,
shared helpers and tolerances are unchanged. The failed source, executable,
XML and guard report are retained; the corrected five-function run passes.

Both actual production Bazel archives and the CMake targets pass. The ABI probe
measures config 128->160 B, model 3240->3272 B and storage 14832->14864 B;
control remains 264 B and each slab 5664 B. Owner allocations remain six and
QEPH remains one, without per-step growth. The five-node mixed fixture owns
23,583 B in nine allocations: owner 2,191, QEPH 14,864, T3 6,224 and coordinator
304. The initial ABI invocation lacked its temporary include root, and one
mixed build invocation used a nonexistent target name; corrected invocations
pass without source changes. Those guard reports remain retained.

All six sustained default-rest CUDA runs and both frozen refinements pass.
They cover 57,344 accepted owner intervals and 86,016 native cell intervals
through 244.140625 microseconds, with 257 common samples per run. Every parsed
scientific field matches the preceding mixed-publication baseline exactly.
Only elapsed/provenance/executable/GPU-memory metadata and the expected +32 B
QEPH allocation differ. Matrix admission fingerprints remain identical.

The corrected [input map](uniform-startup-source-map-r2.json) pins 376 inputs,
SHA-256 `294e45c727630ddfdf113e971747bf6415f57463d48586966c459449927fee16`.
The first [input map](uniform-startup-source-map.json) and its failed evidence
remain immutable. Runtime provenance pins 1,358 records, SHA-256
`2fb1aa21a772c398663d36c107f994f9d925b19d1d804ffc8b0eb1ff56e712c2`.
Reports use `crash-work/reports/qeph-uniform-startup-*` and
`qeph-uniform-response-*`; runs use `crash-work/runs/qeph-uniform-response-*`.
The `qeph-uniform-startup-1` checkpoint preserves sources, actual binaries and
reports with delegated native/toolchain prerequisites, not a hermetic image.

All heavy work was serialized: one compiler worker, at most two affinity CPUs,
12 GiB build RSS ceiling, one CPU/1 GiB test or response ceiling, at least
32 GiB RAM and 8 GiB VRAM free, and at most 4 GiB GPU-memory growth. No guard
failed. Known-zero-stress standalone motion is now qualified; joined moving
startup, wall-impact timestep selection, incoming contact, mixed force feedback
and vehicle response remain unqualified.
