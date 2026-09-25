# Native PEN3 cohort-composition experiment

No production algorithm or runtime routing change. This target compiles the
complete pinned I25PEN3 routine at a62b27e6 with MVSIZ2 and an explicit two-row
BIND(C) wrapper. The source/ABI manifest hashes the unmodified donor and reused
native constants. The wrapper only copies supplied columns to the donor arrays;
it does not translate closest-point or penetration arithmetic.

Each of2064 finite synthetic cases has four calls: target T3 alone; identical
T3 with another T3; identical T3 first in a mixed T3/Q4 packet; and identical T3
second in that mixed packet. The auxiliary shape, margin and native NRTM2 remain
fixed. All target coordinate bits, source IDs, gap and symmetry flags are unchanged
between modes. The corpus covers regular, collinear/coincident, outside, tilted,
three axes, two sides, dyadic scales/offsets, exact and adjacent gap boundaries,
solid symmetry flags and fixed-seed finite perturbations. It is not Yaris data.

The JSONL output records every input bit and both output slots per call.
Active-row counts are [1,2,2,2]; the second slot in single-T3 mode is unused,
wrapper-initialized zero, not a computed native result. Compare
all-T3 against mixed for exact bits, strict native nonzero membership, and a
separate64-epsilon scalar rounding bound. Native PENE is squared search clearance,
not the current physical penetration fed to the force law. Membership receives no
tolerance. The all-T3 control changes cohort length while the target remains first; the
mixed control changes the target ordinal. Each control must be bit-identical;
nonfinite results or failed controls return2. A completed experiment returns0
even if all-T3/mixed differ, because such differences are the evidence being sought.
The summary exposes every mismatch and explicitly says physical_qualification=false.

Before deciding a GPU policy, inspect mismatching inputs/output bits and determine
whether they reflect arithmetic or a different effective geometric branch. A
finite corpus cannot prove complete equivalence. If differences only fit the
approved numerical contract without changing membership/state, qualify a clearly
named canonical GPU packing policy. Stateful selection/history order still needs
its own proof. Do not require emulation of CPU voxel/list scheduling merely from
source branch shape, and do not hide membership differences with tolerance.

Root owns bounded configure/build/source/probe execution. Use GNU11.4 Fortran
wrapper and normal no-fast-math/no-contraction flags. No CUDA or vehicle startup is
needed. Retain the entire JSONL plus source/compiler/object/binary hashes. This is
source-authored and has not been compiled or run by its author.
