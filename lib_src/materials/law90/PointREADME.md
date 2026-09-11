# Selected engine LAW90 point values

`InitializePointSI(material, kinematics, result)` constructs virgin history and
executes the native TIME0 phase. `UpdatePointSI(material, history, kinematics,
time_s, result)` requires positive caller time and valid initialized history.
The point owns no clock and accepts no synthetic dt, current density, volume,
physical EINT or work. Both calls publish the whole result only on success.
Prepared material still borrows caller-owned immutable curve arrays; this is
not an owning/source-authenticated material or element declaration.

The selected profile is the prepared single-curve nu0/Hys1/Shape1/Alpha1,
IFLAG1 or IFLAG2, IDAM0/TFLAG2/FAIL0/ISRATE0 material. Inputs are total B−I and engineering
rate in the actual material frame, with doubled off-diagonals. The represented
B = I + (B−I) tensor must be positive definite, using the unchanged shared LAW42 guard;
the native principal stretches must also be positive. The dedicated native
spectrum retains native order/basis decisions, including degeneracies.

All ten UVAR values and three native-order cursors are retained. The unused
slot5 is carried unchanged; history path energies are algebraic Pa values,
not physical EINT. Both native VINTER2 passes execute in principal order.
TFLAG2 uses the linear E0 tension response before the later FAIL0 tensile cap.
IFLAG2 sets ET to minimum scaled queried slope/E0, including zero on a
plateau. IFLAG1 instead retains the native MAX reduction initialized at1e20
(lines405–440); its ET is1e20/E0 for the original curve. This literal donor
behavior is preserved, not replaced by a minimum or unit tangent. The final IHET0 explicit caller does not overwrite ET.
Tension capping precedes each pair of sequential transverse-stretch divisions.

IFLAG2 loading/unloading selection, scalar-rate limiting, trapezoidal path energy,
the Hys1 damage-expression chain, previous precap stress norm, residual strain
and effective modulus follow the complete engine order. Hys1 is not used to
erase its history updates. IFLAG1 skips the entire IFLAG2 path/damage block,
carrying UVAR1/2/4/6/7 unchanged; common rate, quasistatic energy, residual
strain and effective modulus still evolve, with all three cursors. Returned SSP uses the updated modulus and prepared
PM1 reference density; it is distinct from ET and from caller current density.
VISCMAX0 and OFF1 are explicit selected outputs. Partial removal, failure,
other rates/curves, strain measures and implicit/IHET overrides are outside
this value contract, not guessed by default.

Nonfeedback native locals (`E`, `E_MIN`, second-pass energy before replacement,
and `DE` before storing residual strain) do not create additional retained
state. The first-pass ILOAD expression is consumed only by excluded IDAM>0;
no selected point-to-point history dependency is dropped. Operations that
affect history/cursors/stress/SSP/ET retain their native order.

Source: complete engine SIGEPS90 at
`a62b27e6baa555d222a580d6218867d0be4d70b5`, SHA256
`e1bd7623d72d25d952b59f733298ca235b46c8b1b92e40220da193d4f812d0a5`.
The owning `law90_point` qualification compiles this complete routine with
native spectra and VINTER2, using the independently prepared native33 packet.
The native EMIN1e20 is a finite SI-packet slope sentinel; this point gate does
not claim arbitrary working-unit floor equivalence or app card conversion.
The separately qualified selected LLPIJ72 geometry/current/force caller reuses
this point. Original source admission and resident integration remain distinct
gates; the actual blank-HU extension requires its new native/CUDA controls.
