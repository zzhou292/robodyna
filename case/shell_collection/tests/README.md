# Vehicle contact geometry qualification

`ShellContactGeometryLimits::Vehicle()` explicitly prepares up to 524,288 native
parents and physical nodes within a 2 GiB startup payload reservation. It reuses
the existing Q4/T3 reference measures, immutable binding and TL nodal wall weight
producer. Source order within each parent, coordinates and source identities are
unchanged. The original default limits remain unchanged.

The owning host gate configures `case/shell_collection` with an explicit
`ROBO_DYNA_TL_ROOT` providing vehicle wall weights. Two functions exercise profile
and byte admission, exact-budget retry, immutable ownership and complete mapping
at 328,344 Q4 / 21,301 T3 / 359,785 nodes. The geometry at those counts is synthetic;
this gate does not admit original Yaris materials or execute wall forces.

The conservative reservation includes complete retained binding, position and
source-map arrays, temporary per-parent native reference measures, and configured
weight-owned/scratch caps. Allocator bookkeeping remains bounded by the external
workstation RSS guard. The contact device owner and case remain separate gates.
