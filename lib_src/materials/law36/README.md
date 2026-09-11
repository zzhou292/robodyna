# Selected three-dimensional LAW36 point

Use SolidLaw36Point.h. The pure value API admits the explicit isotropic
single static curve branch: NRATE1, VP0, no pressure-dependent yield, no
variable modulus, no EOS, no nonlocal model or external failure. Six components
are XX, YY, ZZ, XY, YZ, ZX; strain/rate shear components are engineering values.
All dimensional values use SI. This is a solid response, with bulk pressure
coupling and no plane-stress or thickness constraint.

Prepare borrows immutable 2..1024-point nondecreasing positive yield data.
The caller owns its lifetime and host/device accessibility. Curves use the
native left segment at knots and last segment beyond the table. Parameter
coefficients retain HM_READ_MAT36 operation order. Histories are independent
values; update never allocates, advances an owner clock or publishes accepted
state. Only explicit success publishes the complete staged output.

Update takes the actual native material-frame rates and relative-density
variable AMU. It uses the explicit IPLA1 update from SIGEPS36: old-PLA curve
evaluation, one linearized hardening increment, radial return, then bulk*AMU
pressure. Equivalent deviatoric rate is computed without filtering. Returned
plastic increment and rounded accumulated-PLA subtraction are distinct.

UpdateCaller takes Kinematics and native Measures. It computes
AMU=rho/rho0-1 and Vavg=Vnew-.5*dV and retains the native pressure/deviatoric
internal work expression. Current quadrature volume weights plastic work.
Storage volume converts native EINT to/from energy density. New/old bulk
viscosity pressures are explicit caller values: this API does not calculate
solid viscosity or infer selective-pressure geometry.

The MMAIN storage-volume denominator retains its literal floor as 1e-20 m^3
in this SI packet API and its independent SI native oracle. This is not a
working-unit-invariant interpretation of the original t/mm/s model: converting
an original 1e-20 mm^3 floor would give 1e-29 m^3. Results near either floor
therefore do not qualify working-unit equivalence for the original model.
Original Yaris solid volumes are far above this floor; this point gate does
not compute or certify those element volumes. Future element admission must
establish that domain from its actual native geometry measures.

Positive dt is the ordinary interval contract. Zero dt is admitted only for a
virgin zero-stress/strain/rate/plastic history, AMU0, and (for the caller)
zero work/pressure/dV with original density and equal current/storage volume.
That value condition is not an authenticated owner transaction identity.
Source frame transport, derivatives, selective-volume computation and owner
phase authentication remain the future element caller's responsibility.

The original finite native failure sentinels are not infinite IEEE values.
The default card limits are clamped to CONSTANT_MOD INFINITY, a promoted REAL
1E20, and its multiples. This API rejects PLA at that cap and total-strain
absolute sum at one quarter of it, a conservative bound below every principal
strain threshold. It implements no failure/removal branch. Original explicit
zero failure-limit and finite-default native packets are separate oracle
controls over this admitted domain.

No solid mass, eight-point force integration, material/source catalog,
resident batch, contact or tied-interface admission follows from this module.
Native provenance and executable scope are owned by
lib_utest/qualification/solid_law36_point.
