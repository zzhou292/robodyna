# Robo-dyna restart checkpoint — 2026-09-09 PDT

Execution stopped at the user request for a machine restart. Do not resume
builds, numerical runs or agent work until the user requests continuation.
No push. The Yaris deliverable is incomplete; estimated capability completion
remains 25–30%. No Yaris crash video has been generated.

## Last qualified boundary

TL-FEA `9be796a`: immutable mixed Q4/T3 startup binding, eight new host functions
and 24 startup/rates regressions; 974 retained distinct passing functions overall.
See `crash-work/checkpoints/shell-mixed-binding-1/manifest.json`.
Robo-dyna `89b975f` records that milestone. Chrono remains at `96af26597b`;
preserve existing dirty third-party submodules and untracked fmu_tools.

## Saved integration — NOT runtime qualified

The joined Q4/T3 implementation and eight CUDA test sources are saved in a
local WIP commit on TL-FEA `work/cuda-surface-contact`. It uses the immutable
union, one coordinator, one shared kinetic ledger and one owner commit before
both typed history swaps. Native force arithmetic remains in the existing ports.

No mixed executable has been built or run. CMake/Bazel registration and a frozen
source map are still missing. A host-only size probe compiled and ran: QEPH
storage 14,832 B, T3 storage 6,224 B, coordinator scratch 304 B, all alignment 8.
These are ABI forecasts, not measured GPU allocations. Prior standalone source
maps/checkpoints remain historic evidence and must not be described as current
qualification of the changed participant files.

Initial reciprocal review passed, but subsequent adversarial integration review
identified an UNRESOLVED concern: initial AssembleAccepted validates the supplied
NodalAssemblyView mass/J pointers and sets bound=true; coordinator Initialize
trusts that flag. Later BorrowPrepared/SamePrepared authenticates kinematic
buffers but not immutable mass/J/mask pointers. Forged initial mass pointers
might therefore bind native masses against a different actual owner. No
reproduction exists yet. Review actual view authority and add an appropriate
negative test before promotion. Normal direct-owner mass checks pass previously.

## Resume order

1. Re-read this note, active plan, repository status and saved source checkpoint.
   Confirm workstation RAM/GPU reserves and that workstation.lock is free.
2. Resolve/reproduce the initial-binding authenticity concern in owning reusable
   nodal/batch APIs. Keep fixes modular and preserve standalone behavior.
3. Wire the joined production library and mixed tests into existing CMake/Bazel;
   add measured ABI notes and freeze the source map/live manifest revisions.
4. Run the eight new CUDA tests, old T3/QEPH/owner gates, sustained QEPH h/h2/h4
   regressions and scoped shell-wall tests with serialized bounded commands.
   Compare scientific fields against retained baselines; preserve every failure.
5. Continue CW1 native full-state shell/contact screening from
   `planning/QEPH_WALL_RECURRENCE_SCREEN.md`, then actual incoming-impact startup
   and refinement. That document is prospective design only, not admission.
6. Mixed contact, plasticity, attachments/self-contact, connected capacity and
   vehicle-scale output remain on the critical path. Keep 25–30% separate from
   the broader LS-DYNA-like product and from time-to-completion forecasts.

Resource policy remains <=16 GB RAM overall; heavy stages use at most 12 GiB
process-group RSS, two affinity CPUs and one worker, serialized through
`tools/run_bounded.py` and workstation.lock. Tests normally use one CPU/1 GiB.
Do not push; preserve local branches and failed/accepted immutable evidence.
