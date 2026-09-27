# QBAT and solid local final controls

Source-only candidate from qualified2733. Production fe9a3dff changes four files:
only Control destination plumbing and terminal publication. The live vehicle
source/binary are untouched. No numerical/resource/performance pass is claimed.

Eight host and six CUDA tests reuse qbat_measurement_operands::Fixture /
DeviceFixture and solid_measurement_operands / solid_candidate_validation
HostRig / DeviceRig. No geometry, material law or force implementation is added.
All complete Control fields are compared through the existing bitwise diagnostic
comparators, excluding undefined struct padding. Tests cover nonzero/valid identity,
poisoned old Control, initial identity overrides, both solid finalization routes,
all families, competing status/result/overflow failures, unvisited partial fields,
QBAT maximum-displacement fallback, cancellation and same-device repair. Actual
admitted layouts prove Control separation from every consumed source/operand slab.

The full four original2733 source files are frozen with hashes. Exact reviewed
hunks reverse current files to complete originals;32 layout/type/numeric/owner/
fixture sources are separately pinned unchanged. The old numerical oracles and
historical manifests remain unchanged. Their broader source gates are not silently
relabelled as passed by this new destination-only proof.

generate_oracles.py adapts frozen sources by includes/namespace and explicit
local-call qualification only. The host current solid finalizer is extracted
literally from production (qualifier/name adaptation only); CUDA tests invoke the
actual Candidate.cu body through a test-only suffix. QBAT tests call its actual
mapped pipeline as well as compare the exact2733 finalization leaf. Existing
older native/serial numerical comparisons are retained in the reused fixtures.

After the live run/render lane is released: guarded configure/build, exact8 host
plus3+3 CUDA GTests and source proof, compiled kernel REG/STACK/LOCAL and terminal
store inspection, relevant existing public-owner/native regressions and direct
production Bazel closure, then exact all-budget/owner/101-step comparison and
paired timing. No floating reassociation, state/ABI allocation change, new clock,
physics change or performance promise is part of this candidate.
