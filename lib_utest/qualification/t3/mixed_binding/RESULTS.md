# Mixed shell binding: first execution passed

2026-09-09. All eight host functions passed on the first build and run; the
24 existing T3 startup/rates and native-reference regression functions passed
in the same guarded run. The owning Bazel `elements:shell_batch_binding` target
also builds. No numerical operand, tolerance or source formula changed after
execution. This is immutable host startup metadata, not mixed dynamics.

The tests cover all zero-/one-/two-/three-node sharing patterns, exact source
identity (including high uint64 IDs and signed-zero coordinates), independent
mass/inertia oracles and dimensional scaling. The actual QEPH and T3 producers
supply each contribution; native total J and its diagnostic partitions retain
their distinct arithmetic. A union whose total overflows is rejected even
though each individual native startup succeeds. Rejection preserves the
binding, and valid retry succeeds. Prepared bindings cannot be replaced.

The production library and test link only host C++ and GTest. The analytic
native-oracle header adds no Fortran execution or runtime library, and no CUDA
runtime is linked or invoked. CMake's shared qualification project still
contains the existing opt-in native/CUDA targets. No device allocation or
additional state owner is introduced. The CMake build used one worker/two
affinity CPUs and peaked at 268,423,168 sampled RSS bytes; the test used one CPU.

Reports have prefix `crash-work/reports/shell-mixed-binding-`, with all five
passing XML reports in `shell-mixed-binding-xml-1/`. The source map retains
26 declared local source/include/build inputs, SHA
`2b2581b2920f0bb55bdc883c8f3998726230fe2f64ff4a7fab13f0bea2e6e07b`.
It delegates existing startup provenance to its pinned manifests and does not
claim a hermetic compiler image. The live T3 manifests changed only to record
the optional host build registration; point mechanics and their tests are
unchanged. The pre-execution README/source map remain frozen.

The next gate joins both resident batches to this complete union. Both
candidates and their exact inventory must pass before one owner commit and
two infallible history/cache publications. This startup result alone supplies
neither a candidate receipt nor mixed time-integration admission.
