# Selected circular beam18 force values

`InitializeForce` constructs virgin OFF1, four zero point histories, the native
source section seed, current endpoint force/couple and stiffness at TT0. It has
sample zero and no completed interval. `EvaluateForce` requires a positive
supplied interval and the exact prepared reference/material/accepted stamp. It
stages the complete output; every rejection preserves the caller's output.
The caller retains immutable material curve backing. This is a value API with
two endpoint contributions, without an owner, device arena or independent clock.

The closed profile is TYPE18 circular four-point, stored ISMSTR0, LOCAL2,
unreleased endpoints, active OFF1, tabulated LAW44 CA0/YSCALE1/ICC1/VFLAG2,
neutral-fiber filtered rate, no thermal/preload/failure/scaling branch. It
matches the selected cards for the 142 structural beams in PIDs2000514,
2000520,2000948,2000950. Crossing the finite native axial/plastic failure
sentinels is an explicit atomic rejection. Other beam sections, release
patterns and material profiles are not admitted by this interface.

The order is PEVEC3 frame transport, PDLEN3 stiffness/monitor, PDEFO3/PCURV3
rates, MAIN_BEAM18 neutral-fiber filter and generalized increments, MULAW_IB
point increments and three total strains, SIGEPS44PI projection, rounded-PLA
plastic work, ordered four-point section resultants and the two EINT channels.
PDAMP3 then changes force/moment copies; its damping does not rewrite accepted
FOR/MOM or EINT. PFINT3 forms the two endpoint loads and PFCUM3/PMCUM3 subtract
their frame transforms into nodal RHS. N3 only prepares the section seed; it is
never a runtime force, mass, stiffness or rotational endpoint.

The qualified LAW44 reader/curve preparation is shared. The beam update uses
XX/XY/XZ with its native 5/6 shear factor and three-stress radial metric;
it does not invoke the solid return mapping or pressure/EOS path. Four point
stress/PLA/cursors/total strains, filtered neutral rate, section seed,
undamped FOR/MOM, EINT[2] and WPLA form the accepted history. Diagnostic yield,
tangent and unrounded plastic increments do not replace those histories.

Reference input retains source working units. Force input/output is SI.
Prepared section lengths, areas and inertias are converted once by L, L² and
L⁴; native stress failure sentinel provenance remains in material preparation.
PDAMP3's EM20 is a time-squared floor; PDLEN3's EM30 inertia floor is scaled by
the declared working L⁴. The reported element monitor uses DTFAC1(5)=1,
unscaled PM27=sqrt(E/(1-nu²)/rho), and applies no owner timestep policy.
PDLEN3 TYPE18 KPHI=max(1,A L²/max(Iy,Iz,EM30)) is kept distinct from PMASS's
startup expression. `AccumulateForces` reuses the existing atomic bounded
two-node force/couple scatter; the owner controls transaction and lifetime.

Independent native/CUDA qualification is owned by
`lib_utest/qualification/beam18_force`. This slice does not admit beams into
the full vehicle coefficient ledger or common resident publication.
