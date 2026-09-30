# S1 and S2a operator fixtures

These numerical sources are byte-identical copies of the frozen shell operator
checkpoint. The captured pre-migration results are 16 S1 and 13 S2a GPU tests;
the relocated tree must reproduce them. Build and integrity recipes are in the
[parent README](../README.md).

`ShellFixture` evaluates unchanged K1 geometry and K3 force assembly for 1–16
planar convex Q4s with prescribed physical membrane, bending and shear resultants
and zero hourglass. Independent boundary traction/couple, force/moment/work,
geometry, indexing and failure-publication assertions are retained.

`ElasticFixture` evaluates one fresh rectangular Q4 elastic increment with
original and corrected K2 variants. It exposes per-point histories, strains,
resultants, thickness and work for analytic checks. Tests cover membrane,
cylindrical/biaxial bending, engineering twist, transverse shear, zero plasticity,
thickness scaling, coordinate covariance, in-plane spin and transactional output.
A mixed rotated case sends computed world resultants through K3 and checks
independent nodal forces/couples. The high-yield synthetic predictor is not an
admitted vehicle material law and cannot advance saved constitutive history.

The original NPT3 midpoint rule retains the exact 8/9 physical bending stiffness
and energy defect as a diagnostic. The generated Gauss3 variant uses the existing
TL quadrature literals and passes the unchanged physical expectation
D=E*t^3/(12*(1-nu^2)). Only thickness coordinates and stress-resultant weights
change. Other branches/history updates remain unchanged and unqualified outside
this elastic slice; midpoint history must never be restarted as Gauss history.

Original source, headers and AGPL-3.0-or-later license reside in
`../reference/cuda/`; generated source/patch/metadata/license are in `generated/`.
`generate_gauss3.py --verify` checks exact pinned inputs, generated contents and
migration fingerprints. The script's numerical transformation is unchanged;
relocation adds path and source-integrity plumbing plus two regression tests.

The fixtures use private donor kernel templates to avoid donor launchers that
terminate the process. MYREAL8 applies to every donor translation unit. S1
explicitly allocates at most 19,584 device bytes and S2a 1,248 bytes; runtime
context memory is additional. No fast math or relaxed numerical expectation is
introduced by relocation.

S2a is a narrow elastic gate. Finite-step objectivity with staggered state,
warped geometry, full hourglass/constitutive coupling, mass/inertia, stepping,
plasticity, T3/membrane families, offsets and mesh contact remain separate gates.
Native source shares the same quadratic velocity correction and DT1C=DT1;
coordinate covariance and same-instant in-plane spin do not qualify an arbitrary
finite-time rigid-motion trajectory.
