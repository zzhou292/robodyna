# Native projection working length

ReferenceInput declares the immutable projection_working_length_m, default1m.
This is metres per original native projection length unit, bound to the physical
source/reference identity. It is not an adjustable per-step penalty or geometry
normalization. The one-metre path calls the original projection leaves directly,
retaining their arithmetic order and existing public failure fences.

All physical public positions, areas, rates, forces and couples remain SI. Only
the algorithmic DI/DB diagnostics retain their original working metric; the
appended Kinematics::projection_metric identifies its working_length_m. These
mixed coefficients are not mislabeled SI tensor inverses. GeometryWork embeds
that same tagged record; no duplicate18-double matrix cache is retained.

For rate projection, the private coherent SI packet is converted immediately
before unchanged CZCORP5 algebra: x/z and translational velocity divide by L;
area, squared diagonals and LL divide by L²; reciprocal area multiplies by L².
Angular velocity and the orthonormal frame are unchanged because time stays in
seconds. Projected translational velocities multiply by L on return; angular
rates remain per second. Original physical geometry is retained directly rather
than round-tripped. Native planar clearing of Z1 is still propagated. Returned
DI/DB values remain exactly in their working metric with the immutable tag.
Subsequent material/rate/history operations consume SI values.

Force projection verifies the tag against the same immutable reference length.
It converts local x/y/z to working lengths, retains raw DI/DB and normals, leaves
local forces in N, and divides local couples by L (N times working-length).
The unchanged CZPROJN leaf runs on that packet. Result world forces stay N;
world couples multiply by L to return N*m. There is no whole-element/material
unit conversion and no change to force sign, integration phase or assembly.

Finite descriptor, reciprocal and squared-scale admission precede use. Each
consumed conversion rejects nonfinite results or nonzero values collapsing to
zero; new-mode arithmetic is staged before publication. Invalid/mixed cache
metric cannot silently fall back to1m. The existing outer force/history stages
continue to publish only complete successful candidates. Raw CZCORP5/CZPROJN
production leaves are unchanged.

The captured native packet replay establishes same-input agreement and reveals
working-unit dependence in the warped mixed metric. It does not prove every
coupled discrepancy has this cause. Legacy1m, source identity, actual CUDA,
complete accepted-step and matched end-to-end performance gates remain distinct.
