# Loaded finite-wall stages on the existing physical owner

`LoadedWall::Prepare` returns `VehiclePhysicalDynamics`. It prepares the existing
source-bound owner and contact adapter, then installs one private `WallContribution`.
There is no second run driver, publication selector or clock. `Preflight` composes
the same source/startup/geometry/contact forecasts before device allocation.
An optional trailing `JointModel` is passed through both factories and the actual
contact source; the common publisher authenticates its seventh TYPE45 participant.
Omitting it preserves the existing six-family path.

The named loaded settings select `EnvelopeRectangleV1`, 35 mph, a 20 mm leading
gap and a 0.25 m transverse motion margin. The selected mesh adds another margin
around the projected envelope and retains its generated feature IDs separately
from the authenticated original wall. The requested durations are explicitly
5, 20 or 50 ms; each recomputes nominal travel/envelope. The default is 20 ms.
`LoadedWallConfig` supplies a candidate fixed interval of `3e-7 s` and enables
`NativeOrdinaryRigidTrace` with factor 0.8. Neither duration nor configuration
asserts a completed or stable run. Every attempt retains the actual envelope,
penetration, rotation and combined current-response checks. The factory rejects
a smaller transverse margin or a disabled structural screen. An attempt cannot
advance beyond the declared duration; no final-step time is fabricated.

## One attempt

1. Capture actual accepted shell activity for the complete CIN witness roster.
   Begin the existing owner trial and assemble all declared accepted structural
   forces/stiffness, including TYPE45 when supplied.
2. Assemble accepted wall force through `NodalWallMappedContact`, including its
   current constrained response bounds and CIN stiffness contribution. Upload
   accepted CIN witnesses, then seal this same assembly.
3. Run the existing post-CIN current-coefficient structural screen and the native
   CIN/rigid staggered step. No independent inverse mass or fixed member inertia
   is substituted. Borrow its prepared nodal state.
4. Evaluate every declared material/joint candidate and prepare the common
   publication. Evaluate wall candidate geometry/work using that exact prepared
   state and actual prepared shell activity. Copy the complete physical fields.
5. The caller may inspect or discard the prepared result. `CommitStep` performs
   the one common owner commit and existing infallible participant/observation
   swaps. No allocation or readback follows a successful commit.

Any earlier failure calls contact, owner and publisher discard. The contact
adapter has scratch/results, not independently accepted physical history.
`WallStageError` preserves its actual rejection status, physical node/parent and
point report. It supplies no guessed stable step absent from the TL report.
`WallObservation.enabled=false` means unavailable. Otherwise its `accepted` and
`prepared` values are the original typed TL observations, preserving phases,
owner/attempt identity, force intervals, penetration, potential, drift work,
accepted and prepared activity, and removed potential. Same-mask work and
removal accounting are kept separate by that existing API. A prepared record is
not an accepted archive row; only the existing accepted accessor grants that
phase. `wall_setup()` and `wall_forecast()` expose immutable settings/source and
budget for a later archive observer. This slice does not write a physical archive.

## Storage and qualification

The complete preview includes the existing owner/source reservation, physical
readback workspace (including both enlarged observations), geometry/contact
retained storage and maximum startup scratch, then `sizeof(Stages)+256` bytes
for the installed stage/control reserve. The same immutable canonical/source
backing is charged once by the existing identity-checked composition. Optional
joint retained bytes and its arena come from the startup forecast. The default
limits remain 20,000,000,000 host bytes and 8 GiB device payload; contact retains
its existing module limits. Allocator/driver overhead and measured process/GPU
growth remain separately guarded. The startup-only root result before this
increment was 8,610,834,935 host and 3,119,764,763 device bytes; the loaded preview
recomputes its own complete sum, including supplied joints, before allocation.

The author gate `vehicle-loaded-wall-author-final-1` passed nine host functions
and ten production/fixture C++ syntax units in 17.544 s with 471,093,248 B sampled
peak RSS under one CPU/512 MiB. This is not CUDA or original-source execution.
The preserved first host run rejected one test's strict outward-rounding
expectation; the shared bound is exactly representable there. Correcting the
test to include the exact bound changed no arithmetic or tolerance.

The owning `vehicle_loaded_wall_cuda` gate has two functions using the existing
complete small physical fixture and the same concrete loaded operations. It
checks actual force/work, complete accepted-history preservation on rejected
common publication and invalid material receipt, stale result rejection and
retry. Its pre-existing fixture step configuration is retained; the enabled
full-case structural screen belongs to the next gate.

The owning `vehicle_wall_loaded` gate uses all original 372,435 physical nodes,
349,645 shell parents and 38 retained joints. It declares a **1 micrometre gap**
and 5 ms envelope solely to reach contact within two accepted intervals. The
first prepared contact interval is discarded/retried; the next interval applies
nonzero accepted contact force before common publication. It records actual
completed time, force, penetration, potential and TYPE45 startup evidence. This
is neither the default 20 mm approach nor a completed 5 ms crash. Root owns both
GPU gates and must retain any real timestep/force admission failure without
relaxation. No native constitutive, contact or joint equation changes are made.
