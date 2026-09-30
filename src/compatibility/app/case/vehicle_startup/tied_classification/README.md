# Original tied classification after search finalization

Root qualification (2026-09-11, app `33f2ba7`, TL `7527fbe`) passes all14 C++
functions and one Python importer test. All11,165 original slaves select CIN,
zero PEN; every observed field, all8192 ITF words and the native classifier
counters agree. Complete forecast515,134,232 B fits512 MiB; source context
reserves408,609,922 B including the rigid source reservation371,091,422 B.
The1.382 s guarded test samples369,557,504 B under2 CPUs/2 GiB.
Canonical evidence uses `crash-work/reports/tied-classification-source-root-*`;
`tied-classification-evidence-index.md` documents the earlier report-prefix
collision and preserved old native XML. Post-KINCHK remains a separate phase.

`TiedSearchClassification::Prepare(finalized, context)` requires the same
immutable declaration backing. It reuses finalized compact NSV/MSR maps and TL
`Classify`, with the independently qualified observed-slave projection. Original
node IDs are retained; no position or search is recomputed. The contact source
line is a caller association key, not a generated Radioss interface ID.

The result contains every compact slave NID/original NSV row, native IRUPT and
all five observed IKINE fields, all 8,192 ITF words and classifier KINSET/penalty
warning counters. Unobserved projected fields stay private. Native phase is
`InterfaceTaggedBeforeKinChk`; `post_kinchk()` remains pending. The original
finalizer warning events stay in the retained finalizer; classifier counters
are not a complete log of all native starter printed messages.

Preflight includes the source context reservation, distinct packing/geometry/
assessment/finalizer backing, input staging, output rows and bounded native
classifier. Default remains 512 MiB, including a 64 MiB native reservation. No
device allocation is added. Copy/move construction retains handles; assignment
is disabled. Exceptions preserve previously published handles.

Host targets are `robo_dyna_tied_classification_values_check` (two bridge
functions) and `robo_dyna_tied_classification_source_check` (five new source plus
five auxiliary regressions). Optional original-source target is
`robo_dyna_tied_classification_actual_check` (two functions). It reuses the actual
geometry/main/auxiliary fixtures and complete native classifier. It deliberately
does not link the native finalizer oracle into the same executable because
those independent Fortran contexts own overlapping symbols.
One Python importer test additionally rejects high-order solid tails and
multiline/incomplete records before any source-array append. Run this owning
slice with `ctest -R '^tied_classification_'`; transitive source-I/O CTest entries
refer to other executables that are not built by these bounded target commands.

Root configure arguments, alongside explicit toolchain/Chrono paths:

```
-DROBO_DYNA_TL_ROOT=/absolute/path/to/Total-Lagrangian-FEA
-DROBO_DYNA_TIED_CLASSIFICATION_ACTUAL=ON
-DROBO_DYNA_TIED_CANONICAL=/absolute/path/to/crash-work/assets/yaris-vehicle
-DROBO_DYNA_TIED_SCOPE=/absolute/path/to/crash-work/reports/yaris-full-shell-scope-10.json
-DROBO_DYNA_VEHICLE_DECLARATIONS=/absolute/path/to/crash-work/reports/yaris-vehicle-declarations-1.json
```

The original gate reports the actual rigid-source reservation, complete forecast
and independently derived CIN/PEN counts. It compares every observed field,
decode word and classifier warning count with native output, then exercises
complete byte limits, late wall authentication failure and exact retry. It does
not admit an owner, inertia producer, penalty force or post-KINCHK state.
