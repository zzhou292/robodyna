# Cooperative rigid-group motion prototype

This source-only experiment starts from TL2733. It is not promoted, compiled,
CUDA-qualified or measured. The delivered10ms run and all frozen binaries remain
unchanged. One64-thread block owns each disjoint rigid group; retained owner
storage, clocks, reports and the final group-order failure scan stay unchanged.

The production `group_motion` folder separates shared scalar storage, admission
and primary preparation, wrench assembly, member publication, orientation and
CUDA orchestration. The shared tile is4368bytes; the native primary/member
math objects remain thread-private. Every barrier is block-uniform.

The wrench phase calls the existing one-member AggregateWrench leaf and folds
its contributions in source order with every original finite check. It does
not prevalidate mass/axes/durations: a later wrench failure still wins before
primary validation. Members use the original general/two-member leaves and
source units/coefficient policy. A failing tile publishes only its successful
source prefix; prior tiles remain. Group state and capture publish only after
all members succeed. Quaternion preparation uses the pure existing value leaf
and has a separate prefix after full group publication. No force atomics,
retained arrays, second clock, source-specific scheduling or public API changes.

## Qualification to execute only after source review and lane delegation

Configure this directory with CIN_COOPERATIVE_MOTION_CUDA=ON and the existing
strict CUDA math flags. Build only cin_cooperative_motion_host and
cin_cooperative_motion_cuda. CTest must use --no-tests=error, -j1 and fresh XML.

Four new host tests compare reverse-prepared tiles with the unchanged original
group body. Three new CUDA tests plus four reused caller/owner tests exercise the actual production LaunchMotion,
complete private output/capture prefixes, boundary errors, capture admission
and untouched reports on prior failure. The target also compiles the existing
frozen full caller and actual owner tests unchanged, including accepted-state
preservation, failed readback, discard/retry and allocation invariance.

Source proof pins36 unchanged baseline files, including numerical leaves,
owner admission and frozen caller/owner fixtures. It reverses only the exact
Groups.cu dispatch replacement and BUILD header registration to baseline bytes.
The historical cin_parallel_groups identity test remains frozen and unselected:
its old launch-string assertions are not rewritten to bless this experiment.

Before any vehicle gate: inspect actual production register/stack/shared/local
resources and compare baseline33896fc. Speculative lane work can repeat native
validation and increase register pressure; no speed benefit is assumed. Then
require the ordinary short owner and exact101-step archive comparison before
paired timing. Do not infer whole-vehicle speed from this group's microtiming.
