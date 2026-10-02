# Build and binding migration boundaries

Source review:2026-10-01. These boundaries remain relevant while the demo matrix
is expanded; compiling more examples does not close them automatically.

1. **Canonical definitions coexist with compatibility APIs.** The live
   [declaration registry](../../build_defs/bindings/declaration_views.json) has
   eight explicit views: Body, Mesh, System/NSC/SMC, auxiliary-reference/easy-body
   families and Assembly. Their canonical definitions are first-party Robodyna
   code. Remaining Chrono classes, inherited include paths, Python/C# names and
   factory/archive identities still have compatibility obligations. Do not
   infer a complete namespace migration from branding or example build coverage.

2. **SWIG parses an authenticated declaration view.**
   [declaration_view.py](../../tools/bindings/declaration_view.py) and the registry
   use narrow inverse recipes and original header identities. These parser-only
   declarations are not alternate native implementations. A class move must keep
   its canonical header, forwarder, recipe, registry dependencies, generated
   wrappers and retained CMake interface installation in agreement. Generation
   alone is not managed/native runtime qualification.

3. **Shared implementation and wrapper profile admission are different.**
   [optional/defs.bzl](../../build_defs/bindings/optional/defs.bzl) gives shared
   module DSOs explicit `dynamic_deps` and SONAMEs. Wrappers remain thin. The
   [profile policy](../../build_defs/bindings/python/profiles/policy.bzl) currently
   rejects Multicore, extra FE fields and TDPF for these shared binding profiles;
   newer YAML/SPH/OptiX/OpenCRG/ROS routes use named coherent configurations.
   The baseline wrappers retain their narrower compatibility guard. Do not
   remove guards to make an aggregate appear complete or combine differently
   configured native cores in one process. Keep ELF ownership and real
   cross-module object/lifetime tests for every newly admitted route.

4. **The domain implementation split is incomplete.**
   [NEXT_SEAMS.md](NEXT_SEAMS.md) records the completed generic visual boundary and
   four-operation participant service declaration. The service implementation
   still includes the full System; Body still has concrete reporting/contact
   dependencies, and System retains mixed Assembly storage. Multicore stores
   addresses of its collections. Canonical names and header-only service tests
   do not establish independent FEA/MBD linking or permit changing these
   ownership/lifetime relationships without the planned tests.

5. **Import identities remain immutable.** The live
   [source ledger](SOURCE_TRANSFORMATIONS.json) currently contains 558 exact
   source histories and one declared added implementation. The
   [inverse verifier](../../tools/migration/source_transform.py) reverses only
   reviewed edits and checks original bytes/forwarders. Do not repin an old
   baseline to accept an unrelated edit. Newly admitted optional-header fixes
   require their own recorded transformations and focused generation/runtime
   gates. Historical videos and qualification receipts keep their original scope.

6. **Module availability is not GPU execution or full demo success.** The
   [compile matrix](../../examples/BuildMatrix.json) separates312 inherited native
   examples,26 retained CUDA programs,97 Python examples and17 C# examples.
   Queries prove declaration/source associations; completed-target build events
   prove compilation. Numerical trajectories, MPI/DDS exchange, GUI rendering,
   GPU execution and performance comparisons each require their own evidence.
   The26 implicit/ANCF/cuDSS examples remain separate from the explicit Yaris
   implementation and its accepted archived trajectory.

The next architectural step remains the thin System owner/storage boundary and
Body reporting/contact interfaces described in `NEXT_SEAMS.md`. Keep exposing
retained capabilities through real owners and coherent profiles while that
extraction proceeds; avoid expanding the mixed backend's claim of independence.
