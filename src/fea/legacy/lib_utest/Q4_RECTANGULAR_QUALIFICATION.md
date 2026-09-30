# Rectangular C2 qualification

This operation is an explicit opt-in C4 backend; scalar squares remain the
default. `Q4RectangularIntegration.h` and its types live in the owning collision
module so reuse does not depend on test sources. The existing C4 owner binding,
C3 area expansion, assembly, interval work and failure handling remain shared.
Backend and independent axis depths are carried in diagnostics and parent
outputs, with exactly one selected allocation and no runtime fallback.

The original scalar integrator remains the execution oracle. Its only source
change extracts the existing per-Gauss point body into `EstimateSample` without
changing its arithmetic, sample order, mapping, physical mass/Jacobian, normal
law or force transpose. Existing scalar host/CUDA regressions remain required.
The rectangular variant invokes that same sample operation at rectangular
Gauss points, so its estimates and partition may differ from square refinement.
Comparison requires certified interval overlap and unchanged absolute budgets;
it does not claim bitwise identity across different partitions.

## Partition and certificate argument

A cell's U endpoints are `-1+2*c*2^-du` and `-1+2*(c+1)*2^-du`; V is analogous.
Both depths are bounded by the existing 16. Global corner shape products have
at most 34 significant binary digits, so the existing exact dyadic-corner
restriction argument remains applicable. The four original corner gaps are
always retained. No coordinate or width variation is flattened or snapped.

Each split replaces one cell with its two disjoint equal children on one
axis. The union is unchanged. Its area is the outward-scaled input area times
`2^(-du-dv)`, and each Gauss weight is the input area times
`2^(-du-dv-2)`. Affine restriction of a bilinear shape to any rectangle is still
bilinear: original `Restrict`, `ActiveMoments`, and `MixedBounds` apply with
that area. The original directed aggregate operations, positive-area logic
and `Certify` decision are reused. A rounded estimate may lie outside its
truth interval provided its error covers both endpoint distances.

The refinement score bounds the absolute gap variation over each pair of
parallel edges, using cross-endpoint interval differences. Select the larger
axis score, break exact ties toward U, and fall back to the other axis when
the preferred axis is capped. Scores control work only and are never an error
certificate. A score overflow saturates to infinity, with deterministic ties;
it does not reject otherwise admissible physical arithmetic. A mixed cell
remains refinable while either axis is below its cap.

Leaf count increases by one and visit count by two per split, beginning at
one each. Limits stay 4,096 leaves and 16,384 visits. The result reports both
maximum axis depths; legacy `deepest_leaf`/report depth means their maximum,
not the binary-tree path length `du+dv` (which can reach 32). The complete
output is staged until success; scratch can change on any failure.

Each cell is exactly 104 bytes. Full leaf/heap scratch is 442,368 bytes,
32,768 bytes more than the scalar scratch. No allocation is performed by the
operation. CUDA tests allocate bounded reusable storage explicitly; driver
context/module/stack storage remains outside this owned scratch count. The
qualification does not change CUDA stack limits. Prescribed saved-state timings
and complete integration checks are recorded separately in the workspace
[contact execution review](../../planning/GUIDED_CONTACT_EXECUTION_REVIEW.md);
they do not establish dynamics or vehicle throughput.

## Independent tests and next evidence

Host tests use exact integer dyadic-grid coverage/disjointness, independent
closed-form uniform/cut/corner/saddle integrals, and direct polynomial
integration of `g(s,t)=s-a+d*t` with nonzero width variation. They include U/V
interchange with physical IDs/masses preserved, thin strips, nonbinary area,
selected second parent, per-axis depth extremes, every hard limit, invalid
scratch, late Gauss-mass failure, lost positive energy and clean retry.
Actual CUDA tests independently exercise the same physical oracles and
CPU/device execution/certificate consistency. Runtime failures are failures,
not skips. Source staging alone is not a numerical pass.

After these gates, profile immutable saved parent inputs at epochs 12,000,
12,400 and 12,500 with their actual area, mass, force/energy limits and source
hashes. Compare the complete scalar/rectangular results and record leaf,
visit, axis-depth, scratch and timing evidence. Production promotion, C4
transaction regression, a complete guided horizon, and h/h2/h4 are separate
gates. No result in this module admits vehicle mechanics or arbitrary contact.
