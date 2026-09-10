# Original Yaris part elastic case

`SourcePartElasticCase` composes the original PID2000157 input adapter with one
TL physical-node owner, joined QEPH/T3 batches and their atomic publication
coordinator. All 117 source nodes and 94 parents remain present. The source adapter
retains density and thickness while explicitly selecting experimental LAW1
elasticity. Source MAT024 behavior and the part's original attachments are not
implemented by this free-part experiment.

The physical tuple and promotion criteria are frozen in
[`planning/SOURCE_PART_ELASTIC_PILOT.md`](../../../planning/SOURCE_PART_ELASTIC_PILOT.md).
`SourcePartElasticPilot.cpp` supplies that configuration to runners and tests.

- `SourcePartElasticInitialize.cpp` constructs immutable native mass/inertia,
  spatial pulse forces and component ownership. It allocates once at startup.
- `SourcePartElasticStep.cu` assembles accepted force caches and the pulse,
  advances the existing TL clock, evaluates both candidates and commits once.
- `SourcePartElasticObservation.cpp` checks the candidate before publication.
  Its global kinetic sum uses native total inertia once; source work retains
  the separate family ledgers. Native timestep diagnostics impose a conservative
  case guard, with actual response refinement required separately.
- `SourcePartElasticCase.cpp` provides the small Initialize/Step/Capture API.
  Capture returns a complete accepted cache, including actual owner timing.
- `SourcePartElasticArtifacts` outside this directory owns accepted output and
  Chrono mesh publication. Rendering does not participate in mechanics stepping.

The spatial pulse is a mass-centered quadratic along the configured axis,
multiplied by a smooth sine-squared temporal profile. It has zero resultant to
roundoff but may apply a resultant moment. Maximum pairwise chord-length change
therefore measures deformation independently of rigid translation/rotation.
This derived metric is computed by Capture at output cadence. Its value in
per-step diagnostics is zero; every-step mechanics envelopes remain active.

Accepted raw velocity and angular velocity retain their actual midpoint phase.
Separate derived arrays reconstruct endpoint velocities from the complete new
internal and external force/couple. They support the physical energy monitor
and refinement comparison without changing the stored solver fields.

All force readbacks, geometry/rotation/strain checks, source-work accounting,
discrete kinetic-work identity and synchronized energy checks finish before the
existing joint commit. A failed trial preserves nodal state, both material
histories and the last accepted diagnostic/capture. No native Fortran oracle,
additional solver clock or per-step allocation appears in production stepping.

The focused qualification executable takes the pinned readiness file as its
first positional argument. Its tests cover immutable startup, 64 actual-source
intervals against existing native QEPH/T3 oracles, late T3 failure after QEPH
preparation, observer rejection, full-history preservation and exact retry.
Native references are linked only to the test executable. Execution evidence
belongs in the workspace reports; this document does not claim an unrun gate.
