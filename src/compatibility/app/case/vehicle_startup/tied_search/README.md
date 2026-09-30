# Original tied geometric assessment

`TiedSearchAssessment::Prepare` combines immutable `TiedShellSearchGeometry` with
the bounded TL startup driver. It retains the same canonical/declaration/packing
backing, copies only typed working positions and master inputs, and returns one
row for every NSV node. `secondary()` supplies original NID and canonical index;
`selected_master()` and `selected_part_id()` refer to the declared IRECT patch,
including when equivalent coincident glass witnesses supplied its thickness.
The geometry handle supplies original four-slot coordinates and source cards.
There is no geometry JSON, source parser, material admission or second owner.

The result exposes the native gap/projection coordinates, selection distance in
working units, outside flag, complete candidate/box counts and separately
prepared SI force patch. Unmatched rows and selected singular patches remain in
the result. `finalization()` and `classification()` are explicitly `Pending`:
these projection flags are not the complete I2TID3 warning/compaction result or
an IKINE/classifier outcome. No coefficients, M/J, force, runtime eligibility,
accepted step or mechanics clock are constructed.

Preparation publishes an immutable handle only after all source associations,
driver work and result extents pass. A driver failure includes source NID/EID
when its failing row is known. Prior handles remain usable after any rejection.

`Forecast` runs before typed staging or driver allocation. Its 512 MiB default
combines retained canonical arrays/metadata, declaration/packing/geometry payload
once, input staging, fixed objects and a 128 MiB driver host reservation. It
charges actual string/vector capacities (conservatively including short string
storage) and excludes already-freed parser/member buffers. The native driver
separately admits exact CUB scratch and candidate capacity inside 128 MiB device
and the reserved host cap. Payload accounting excludes allocator/control-block
metadata, CUDA context/driver overhead, unrelated caller buffers and a separately
retained prior result; it is not RSS. Test/oracle memory is separate from the
production forecast.

Host checks cover working bit identity, T3 repetition, source ordering, late
association rejection/retry, complete rows and exact host budget. The owning
actual test retains all 171,813 masters / 11,165 slaves, releases input handles,
then compares every public selected rank, projection and native box count against
the independent native bucket helper. The native helper never consumes GPU
candidates. It reuses the existing primitive comparison budget. Root qualification
must run this test; author host checks do not establish the full GPU result.

Root gate `tied-assessment-root-tests-1` now passes all five host/source functions.
All11165 selected ranks, per-node exact-box counts and native projections match
the independent oracle.31104 candidates,0 unmatched,0 outside flags and0 rejected
selected force patches. Total host forecast276211441 B includes source backing
and driver reservation; device forecast106955790 B. The complete original CTest
takes0.96 s and samples334950400 B RSS under2 CPUs/2 GiB. Independent code review
found no association, lifetime, budgeting or publication blocker. Finalization
and classification remain pending.

Configure from this directory with `ROBO_DYNA_TL_ROOT`, installed `Chrono_DIR`,
and `ROBO_DYNA_TIED_ASSESSMENT_CUDA=ON`, plus explicit `ROBO_DYNA_TIED_CANONICAL`
and `ROBO_DYNA_TIED_SCOPE` (scope10) fixtures. Targets are
`robo_dyna_tied_assessment_values_check` and
`robo_dyna_tied_assessment_actual_check`; CTest names are `tied_assessment_values`
and `tied_assessment_actual`. Native bucket and primitive sources stay owned by
TL's separate qualification targets.
