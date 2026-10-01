# Bazel product bootstrap

Robodyna owns the root module and dependency graph. The first migration stage
imports qualified FEA sources into `src/fea/legacy` and presents native targets
through `//src/fea`. `@legacy_fea` is a temporary label/include boundary for these
owned sources. It does not use the imported MODULE, install another product or
run another build system for the explicit mechanics libraries.

The source directory is excluded from the main repository's recursive package
scan so it is not also loaded with incorrect root-relative inherited labels.
Existing explicit state, native rigid regions, CIN and common publication stay
together. No independent FEA/MBD extraction or physics equivalence is established
merely by these aliases. Implicit CUDA backends remain separate opt-in targets.

The `implicit_newton`, `implicit_adamw`, `implicit_nesterov` and
`legacy_dem_contact` aliases carry `manual` tags. Root wildcard builds do not
select these unqualified migration profiles automatically. Build an alias by
its explicit label when qualifying that retained backend and its dependencies;
the tag does not disable the implementation or alter numerical settings.

The root requests `rules_cc` 0.2.17 explicitly: the inherited qualified lock and
the first root graph both resolve that version with the same registry source
hash. The inherited MODULE's older 0.2.14 request was a minimum constraint, not
its resolved compiler-rule version. Root graph regeneration remains required
after changing the request.

## Qualification sequence

1. Run `python3 -B -m unittest discover -s build_defs/tests` from this root.
2. Run `python3 -B build_defs/source_boundary.py --workspace .`. This verifies
   critical source sentinels, aliases, patch hashes and root policies without
   evaluating Bazel, loading CUDA or compiling. The complete import manifest is
   separate; these sentinels are deliberately not a claim of full-tree parity.
3. Under the workstation guard, let Bazel resolve the root graph with an explicit
   one-time `--lockfile_mode=update`. Review and retain `MODULE.bazel.lock` before
   using the default `--lockfile_mode=error`. The imported lock is historical
   input and is not a valid replacement for root graph resolution.
4. Qualify host values/tests first, then native CUDA libraries and focused runtime
   tests. Build the short source-authenticated Yaris producer before claiming that
   this root reproduces the delivered simulation. Root target declaration is not
   compilation or runtime evidence.

Proposed commands for the guarded build stage:

```sh
bazel build --config=host //src/fea:host_values
bazel test --config=host //build_defs/tests:source_boundary_test
bazel build --config=cuda --config=sm120 //src/fea:explicit
```

Choose the actual deployment architecture (`sm75`, `sm86` or `sm120`); these
profiles set both native rules_cuda and optional DEME foreign-build architecture.
CPU-only targets have a narrow dependency graph. `--config=host` by itself does
not make a CUDA-dependent target run on the CPU.

Numeric flags stay with the qualified targets. Never put fast math, FMA or FTZ
policy in the root rc: inherited explicit and implicit implementations differ.
Four Bazel action slots and one local test slot supplement the existing process
guard. They do not enforce RSS, free RAM, GPU reserve, affinity or locks by
themselves. GPU acceptance must use the shared resource guard and fresh execution.

Python also uses explicit dependency boundaries. Both the native Bazel option
and the rules_python 1.7 replacement disable automatic imports from every external
repository root. Otherwise the watchdog's data-only `legacy_fea/tools` package
can shadow Robodyna's `tools/migration`. Imported Python utilities and the runfiles
library declare their own import roots; watchdog sibling imports use the selected
guard's existing runtime contract. No source-tree `sys.path` workaround is used.
Qualification must confirm `_IMPORT_ALL = False` in packaged bootstrap metadata
and exercise packaged migration, driver and CLI workflows.

## Optional legacy DEM dependency

The root maps the retained DEM-Engine source separately and supplies a reviewed
temporary `rules_foreign_cc` wrapper. It uses four nested compiler workers and
the selected architecture; no absolute CUDA path is embedded in the build.
Before using this optional target, declare `CUDA_PATH`, `CUDAToolkit_ROOT` and
`CUDACXX` consistently with the CUDA toolchain admitted by the run guard. The
wrapper, its runtime data and shared helper still require their own build/run
qualification. The qualified Yaris explicit target does not depend on DEME.

The three compatibility patches are copied byte-for-byte from qualified TL
commit `f0cdeffaef85ea1f97c2162790dbd091fb2e4853`'s migration source
contract; the authoritative full commit and exact hashes are recorded in
`legacy/source_contract.json`. Upgrade them only with affected build/test evidence.

Reviewed fixes to imported Bazel declarations live in
`legacy/build_overlays.json`. Each record identifies the qualified baseline hash,
current hash and exact reversible text changes. The source-boundary inspector
reconstructs and checks the original bytes; it does not replace the original
source hashes with new ones. This exception admits only `BUILD`/`BUILD.bazel`
metadata. Physics sources and numerical policies retain their qualification
boundary. Additive include/guard BUILD packages are separately recorded by the
import inventory.
