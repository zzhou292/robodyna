# Robodyna: Yaris delivery checkpoint and next work

Updated 2026-09-30 during local source consolidation.

The selected native V6 Yaris assembly completed a **100 ms wall crash with
self-contact**, and both reviewed videos are delivered. The run accepted
666,667 intervals at a fixed 150 ns step, ending at
0.10000005000066203 s. The accepted archive contains 301 saved states. The
front close-up and whole-vehicle overview each show all saved states in a
60.2-second, 1,806-frame video with part colors and deformation scale 1.

The successful simulation and rendering are evidence for the qualified source
revisions below. Consolidating those histories does not itself qualify a new
binary or a unified multiphysics runtime.

## Qualified sources and preserved evidence

| Component | Qualified revision |
| --- | --- |
| Native vehicle application | `3b6ef1c566061246679d56b4ffd538315dde74dc` |
| TL mechanics | `f0cdeffaef85ea1f97c2162790dbd091fb2e4853` |
| Physical renderer | `7260743c19a6082fa3a717b2b9c88881a120bc56` |
| Native vehicle postprocessing | `41774b5b267e59c79a8b41ec1db4df5b15c0e8f8` |

Evidence remains outside the source repository, relative to the original
`chrono-work` workspace:

- Run: `crash-work/investigations/native-v6-yaris-100ms-1/accepted`.
- Producer guard: `crash-work/reports/native-v6-yaris-100ms-1.json`.
- Archive audit: `crash-work/reports/native-v6-100ms-long-prefix-independent-1.json`.
- Video qualification: `crash-work/reports/native-v6-yaris-100ms-video-qualified-1.json`.
- Videos: `crash-work/renders/native-v6-yaris-wall-self-100ms-1/`, in
  `front-closeup-video/movie.mp4` and `overview-video/movie.mp4`.

The guard exited 0 with complete cleanup. Elapsed time was 40,358.713 seconds
(11 hours 12 minutes 39 seconds), with peak sampled RSS of 5,456,846,848 bytes.
Throughput was 16.5283 accepted intervals per second. These measurements do not
establish a long-impact GPU-versus-OpenRadioss CPU speed ratio or prove absence
of all memory leaks.

The closed 100 ms archive preserves the earlier 30 ms trajectory: all 200,001
prior interval rows, 95 shared frame/activity files and 35 static source and
environment files match. Differing saved-state schedules were not interpolated.
The completed 30 ms and original 10,000-step/~2 ms runs, failed receipts,
frozen binaries and old videos remain preserved.

## Current physics and implementation boundary

The selected assembly uses CUDA explicit structural mechanics, native shell
plasticity, selected solid and beam families, rigid groups, CIN and connection
support, and native TYPE25 contact with friction and retained history. One
physical owner publishes accepted state and all participating histories.
Application code composes the run; Chrono renders accepted geometry.

The current profile supports source-declared shell removal with fixed solid
contact topology and the admitted CIN policy. General solid exposure/erosion,
failed-CIN release and reactivation still require separate qualification.
The selected V6 population is scoped: omitted raw-deck primitives are not
silently included, and proper inflated tires remain absent. Earlier frictionless
fixed-triangle M2/V5 descriptions concern a historical profile; they do not
describe the completed native V6 run.

The final activity record contains 34 removed parents: 31 glass and three
radiator fan-cover elements, comprising 31 QEPH and three T3 elements. Native
plastic history and visible deformation do not alone prove permanent residual
deformation after full unloading. Full energy, momentum, contact-work, load-path
and failure ledgers remain separate work. Visualization archives are not
physical restart checkpoints.

The qualified long-run executable is currently hosted by GoogleTest, although
the reusable `PreparedRun` library owns its execution. A normal product CLI must
call that library; it must not wrap a test executable permanently. No production
OpenRadioss library is linked to advance mechanics.

## Next work and migration gates

1. Consolidate the qualified app, renderer, postprocessing, TL and Chrono source
   histories into the user-selected Robodyna codebase. Preserve original licenses,
   attribution, root-only work and the accepted artifacts.
2. Establish one authoritative Bazel build and a normal CLI around the existing
   preparation, execution, archive and rendering libraries. Keep FEA and rigid-body
   dynamics as distinct modules with shared lower-level contracts and explicit
   coupling. Absorbing all inherited functionality is distinct from qualifying it
   for CUDA execution.
3. Verify migration with source manifests, focused host checks, a matched short
   Yaris regression and replay of existing accepted archives. The old 100 ms
   delivery is not a test of the newly consolidated source. Select longer physics
   checks when executable changes warrant them rather than for documentation alone.
4. Establish a controlled longer CPU OpenRadioss versus GPU comparison with matched
   source population, material/contact/failure controls, physical interval,
   timestep/mass policy, precision and output workload. Separate startup, solve
   and output time before publishing a speed ratio.
5. Profile representative impact/removal intervals and optimize reusable batching,
   residency, reductions or scheduling. Preserve numerical and regression guarantees.
6. Add physical restart, complete balance ledgers and broader contact/failure
   support through separate qualified milestones. A small fluid/FEA coupling
   example should then exercise the same product lifecycle and output workflow.

The workspace architecture and execution documents hold the detailed migration
design and current resource policy. No new simulation or build is implied by this
source consolidation. Preserve the numerical profile: no hidden timestep change,
mass scaling, softening, geometry perturbation, dropped candidate pairs or disabled
validation. Keep other GPU jobs running and use the existing resource guards.

## Historical status preservation

The root branch's three documentation commits remain in the merged Git history.
Its uncommitted 2026-09-24 delivery-plan update was preserved without modifying
the original checkout under
`crash-work/reports/robodyna-app-consolidation-1/preservation.json`.
The preserved document SHA-256 is
`c750141fe9af202dd0681b7e9a7fcbf3c8fd6511207e7a45870aa6a0371a524b`.
Those records describe the earlier 400 ns CPU-native checkpoint and failed native
GPU probe; they remain historical evidence rather than the current delivery status.
