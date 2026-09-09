# QEPH Q3a startup port qualification

This standalone gate compares the TL one-cell host/device startup operation to
the retained actual native Q1 reference. The initial 48-configuration gate passed
all four host and two actual-CUDA functions on its first numerical execution
(`qeph-q3a-{host,cuda}-tests-1.json/xml` under `crash-work/reports`). Independent
review then added skewed parallelograms. The enlarged 72-configuration gate
passed all five host and two actual-CUDA functions on its first execution
(`qeph-q3a-skew-{host,cuda}-tests-1.json/xml`). Both gates used the original
tolerances. No force, history, batch, owner, clock or dynamics admission is
introduced. The native Q2 force/history reference is separately qualified;
its retained source equations remain unchanged.

The owning API is `lib_src/elements/qeph/QephStartup.h`:
`InitializeReference(const ReferenceInput&, ReferenceData&)`. It publishes only
a complete successful result and can replace an existing result, including when
the input aliases the prior output's immutable input. Values are binary64 and SI.
The record has four cyclic node identities and coordinates, rho/E/nu/t, native
axes, projected area, unnormalized CDERII derivatives, projected node0-relative
coordinates, mass and separately physical/area-added/total inertia. Native total
inertia retains its original expression; the partitions are not recombined to
replace it. Warped area is mean-plane projected area, not true bilinear area.

`QephStartupFrame.h` retains selected CNEVECI/CLSKEW3 IREP0 arithmetic;
`QephStartup.h` retains CDERII and selected centered CINMAS expressions. The
OpenRadioss pin is `a62b27e6baa555d222a580d6218867d0be4d70b5`; full AGPL notices
and routine/file hashes are in the native reference and port source manifest.
`math/Fixed3.h` shares only Vec3/Matrix3 records with compatibility aliases in
ReissnerFrame. No Reissner frame/chart or inertia equations move into QEPH.

## Explicit domain and error budgets

Input density, Young's modulus and thickness are positive finite; `0<=nu<.5`.
Vertices are finite and node IDs distinct. The actual binary64 native frame
DET must exceed `1e-20*(1+64*DBL_EPSILON)` m², excluding the source normalization
floor and an explicit adjacent band. The original MAX expression remains in
the admitted path. Projected corner turns normalized by the largest original
node0-relative distance must exceed 128 epsilon. Frame orthogonality is checked
at 1e-10; coordinates, mass and each inertia partition must be finite, and every
mass/inertia strictly positive. Failure preserves every prior output byte.
This is a conservative bounded domain, not an exact geometry certificate or
equivalence to the native wrapper's long-double input-admission boundary.

The tolerance was frozen before execution:
`abs(actual-reference)<=2e-12*(dimension+abs(reference))`. Dimension is 1 for
frame entries, original node0-relative length L for coordinates/derivatives,
L² for area, and that independently nonzero reference mass/inertia partition
for mass quantities. Physical inertia is never compared against a total-inertia
absolute floor. No tolerance is fit after a result is observed.

Five host functions and two actual-CUDA functions passed. The parity
functions each cover 72 configurations: metre-scale flat/saddle/skew, 10/20 mm
square flat/saddle and scaled skew at t=.001648 m, four cyclic identity
permutations, and two proper world transformations. The skew vertices are
`(0,0),(2,0),(2.5,1),(.5,1)` times 1, .005, .01 m, all z=0. Its independent native
IREP0 basis is `e1=normalize(2+sqrt(5),-1,0)`, `e2=(-e1.y,e1.x,0)` and area 2
times scale². A separate 24-case closed-form test checks that basis, constant
Jacobian cofactors, local coordinates and individual mass/inertia partitions.
The original dimensional tolerance is unchanged. The small inputs use
rho=7890 kg/m³, E=200 GPa and nu=.3;
these are representative startup fixtures, not authenticated source-part meshes.
Independent rectangular/saddle geometry and long-double mass identities are
separate from native comparisons. Rejection tests include concavity, invalid
identity/material/coordinates, late mass overflow and t³ underflow, adjacent
represented inputs around a 2.5e-11 m half-side and a predeclared safe-side margin.
Complete failure bytes and clean retry are checked. CUDA uses one thread and
one packet below 4 KiB, with canaries; missing CUDA fails instead of skipping.
Device-context memory is additional and belongs in the external runtime guard.

## Build and provenance

Root serializes all builds/tests through the workspace guard. Source verification
is read-only and requires no compiler/GPU:

```sh
python3 Total-Lagrangian-FEA/lib_utest/qualification/qeph/verify_sources.py
```

The verifier checks eighteen declared records plus the owning native verifier's
full original/extraction/include closure. Fortran-to-C++ changes are explicitly
listed as a port, not falsely described as an exact textual transformation.
The old classic-shell CUDA K1/K2/K3 kernels are not substituted: their geometry,
normalized forces and five-state CHVIS3 history differ from QEPH.

The standalone CMake exposes `TL_QEPH_ENABLE_CUDA` and links the native oracle
only into qualification tests. Root supplies the existing CUDA/compiler options
and retained Fortran compiler, and builds with one job. Targets are
`qeph_startup_check` and `qeph_startup_cuda_check`. Fast math, contraction/FMA
and CUDA flush-to-zero are disabled; CUDA division/square root remain precise.
The final build passed in 3.512 s with 331,087,872 bytes peak sampled host RSS.
The final host guard completed in .252 s; the CUDA guard completed in .461 s
(GTest .244 s). These include process/transport overhead and are not kernel
throughput measurements. XML records one thread and exactly 688 owned device
bytes. The guard imposed two CPU affinity slots, one build job, at least 32 GiB
available RAM, at least 8 GiB free GPU memory, and a 4 GiB GPU-growth cap for
the CUDA test. Its sparse samples do not establish peak CUDA-context memory.

Exact report identities and limits are in
`planning/QEPH_Q3A_FINDINGS.md` at the workspace root. Complete geometry/rate
and force/history parity precede any resident batch or temporal/physical-owner
integration. In particular, Q1's measured first-order rigid-path shear-rate
residual remains a separate temporal limitation; startup parity cannot qualify it.
