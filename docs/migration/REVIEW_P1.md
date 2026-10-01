# P1 architecture and build review

Review snapshot: native backend build 3, native viewer test 2 and product Yaris
run 3 with 101 accepted steps. This review inspected source, build declarations and closed JSON
receipts. It did not rebuild, rerun physics, render or hash simulation payloads.

## Qualified progress

- The CUDA vehicle backend builds through native Bazel targets. The production
  application retains 95 libraries and 473 translation units with per-target
  arithmetic policies. Its C++ executable does not link GoogleTest or a Fortran
  reference solver.
- The owned Chrono core/FEA aggregate and VSG frontend compile natively. The
  separate CMake bridge is retained transition evidence, not the production
  backend's compilation route.
- The real product CLI completed 101 steps. The independent comparison receipt
  reports all 50 archive files totaling 177,167,032 bytes, the viewer receipt and every
  non-timing summary value exactly equal to the qualified baseline. Only the
  seven named timing fields differ. This proves the recorded short trajectory;
  it is not a rerun of the 100 ms delivery or a CPU-reference validation.
- FE nodes, native rigid regions, CIN transfer/recovery and contact still share
  their qualified owner/publication. No directory-driven second integrator was
  introduced. Current TL implementation differences from its import are BUILD
  metadata only, covered by reversible overlay evidence.

Evidence in the outer workspace:
`robodyna-native-backend-build-3.json`,
`robodyna-chrono-native-tests-2.json`,
`robodyna-viewer-native-tests-2.json`,
`robodyna-product-package-tests-2.json`, and
`robodyna-yaris-101-3-independent.json` under `crash-work/reports/`.

## Controlled debt and follow-up gates

1. **FEA/MBD separation is still pending.** `src/fea` is a compatibility façade;
   Chrono's native aggregate still compiles both domains and mixed-system code.
   Its rigid and FE unit tests do not establish independent link closures.
   Extract neutral frame/inertia/variable-block seams first, then test both
   standalone domains and a shared-solve flexible-body attachment. Preserve the
   existing Yaris owner during this work.
2. **Rooted includes and public visibility are transitional.** The app macro
   supplies explicit compatibility include roots; header owners are declared
   separately rather than hidden in a global header blob. Narrow the APIs and
   visibility at each actual extraction. Do not promote historical `test::`
   resource helpers or every imported target into the final public SDK.
3. **Optional backend selection is now explicit.** The four implicit/ANCF and
   DEM façade aliases are tagged `manual`. Root `//...` builds do not select
   these separately qualified migration profiles automatically. Their source
   and direct target labels remain available for deliberate qualification.
4. **Configuration exports need a later cleanup.** The inherited native Chrono
   header target publicly exports `NDEBUG` and a fixed CPU feature profile.
   This is the reviewed release profile, but is unsuitable as an unnoticed
   default for future debug/configurable consumers. Retain current parity while
   introducing tested configuration targets; avoid changing flags mid-gate.
5. **Local SDKs are declared, not hermetic source packages.** CUDA, OpenSSL,
   VSG/Vulkan and the platform watchdog interpreter remain environment inputs.
   The SDK receipts and interpreter capability/hash records make these visible.
   Do not describe a fresh checkout as dependency-free or all-platform qualified.
6. **Full inherited coverage remains a matrix, not one build claim.** Source
   retention and LFS restoration are recorded. Several optional gitlink checkouts
   and most optional Chrono build/runtime profiles remain unqualified. Keep the
   import-time `SOURCES.json` snapshot and use `EXECUTION.md` for live status.

## Verification wiring

The source-boundary/overlay and app parser tests already have Bazel targets.
The actual checkout source audits are explicit checker commands; their fixture
unit tests alone do not execute those audits. Import-preservation and archive
comparator tests are now declared under `//tools/migration:behavior_tests`, with
the Bazel execution gate still pending at this review snapshot. Source-audit aliases
are `//tools/migration:check_source_boundary` and `:check_app_sources`.
The viewer source manifest also deserves an
automated parity target when its build profile is finalized. Do not substitute a
long vehicle run for these cheap metadata/ownership gates.

## Current comparator contract

`tools/migration/verify_run.py` matches the current CLI's
`robodyna.launch_result.v1` and `robodyna.native_vehicle_driver.v1` schemas. It
requires a completed zero-exit guard, verified product result and full C++ replay
before reading candidate archives. Failed pidfd/watchdog launches and active runs
are rejected. No driver-report or timing-schema identity is required; the accepted
archive, viewer receipt and non-timing summary remain exact.

The newer CLI also binds small product receipts in `launch-result.records`.
The comparator now reuses the shared product-receipt verifier for the exact record
inventory, file hashes and launch-to-request linkage before inspecting payloads.
This supplements the existing closure checks and complete archive byte comparison.

The successful 101-step receipt closes the immediate numeric packaging gate. Remaining
P1 work is reviewed native rendering, final operator commands and consolidated
evidence; independent FEA/MBD ownership is the next architectural gate.

## Subsequent qualification

The final bundled gate passed all 20 targets after restricting Python imports to
declared dependencies. The post-extraction 101-step run again matches every
recorded archive file and non-timing summary field. The new native product rendered
the preserved 100 ms archive into a byte-identical qualified MP4; full decode,
input immutability and initial/middle/final visual review passed. Evidence is in
`QUALIFICATION.json`. All original numerical source bytes remain unchanged.
