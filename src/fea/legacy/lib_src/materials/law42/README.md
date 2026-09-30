# Selected LAW42 material point

`Prepare` and `Update` implement the one-term alpha2, IFORM1, ISMSTR10 branch
used by the MAT007 converter mapping. Original shear modulus is24 MPa,
density1980 kg/m3, and mapped Poisson ratio0.463. The effective bulk expression
comes from HM_READ_MAT42, including its GS=2*Mu convention. The caller must
supply the resolved tension cutoff in SI; the native1e20 MPa sentinel is1e26 Pa,
not1e20 Pa. Parameters are checked on both host and device.

Input is the actual native total-strain measure in the caller material frame,
with doubled off-diagonals and eigenvalues+1 equal to squared stretches.
The caller supplies current density separately. No strain-to-density inference
or element deformation-gradient convention is hidden in this point entry.
Output includes Cauchy stress, principal extrema/material activity, relative
volume, sound speed and the HEPH tangent factor ET. Prony viscosity is zero in
this profile. Hyperelastic loading is reversible until the native tension cutoff;
there is no rubber plasticity surrogate. No thermal, bulk-curve, small-strain
fallback, multi-term Ogden or Prony-series branch is admitted by this API.

The original TL `MooneyRivlin.cuh` supplies device PK1/tangent operations for
deformation-gradient input, with a different caller and determinant-floor policy.
It does not supply native LAW42 principal cutoff, ET or density-driven sound
speed. The new small module adapts those complete native material expressions
instead of changing that existing constitutive contract. Geometry and force
drivers for HEPH and wedges will share this one point implementation.

Tensor decomposition reuses Eigen's fixed-size3x3 direct solver through
`math/SymmetricEigen3.h`. Eigen's iterative3x3 specialization in3.4 has a
host-only internal path, so it is not suitable for this CUDA call. CUDA consumers
use `--expt-relaxed-constexpr`; no dynamic matrix allocation or rewritten
eigensolver is introduced. Represented positive stretch is checked before the
eigen solve, so rounded near-zero roots cannot admit an exactly collapsed tensor.

Successful value preparation is not vehicle/source admission. The original
material and hourglass cards remain immutable, and the demo's explicit HEPH24/
S6Z selection lives in app case resolution. Actual element kinematics, work,
physical stabilization, nodal coefficients and accepted-state integration remain
separate responsibilities. Invalid input or arithmetic leaves output unchanged.
