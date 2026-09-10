# LAW44 analytic hardening with filtered total rate

This optional branch implements the pinned MAT024 LCSS=0, blank-inline-point
conversion: A=SIGY, B=ETAN*E/(E-ETAN), n=1, MFUNC=0. The same LAW44 predictor,
three evaluated plane-stress correction iterates, independent transverse shear,
thickness additions and work diagnostics serve the existing tabulated path.
`detail/Law44Hardening.h` selects/prepares hardening; it does not implement a
second stress update. The virgin native slope remains E. At PLA>0 the slope is
B multiplied by the source rate factor.

`PrepareLinearLaw44ShellPlasticity(E,nu,rho,{SIGY,ETAN},rate,output)` requires
positive finite SIGY, 0<=ETAN<E, and positive-C/P filtered VP2. The converter
multiplication/division order is retained; overflowing or underflowed coefficients
reject before output changes. Inputs are in SI and output owns all scalar data.
`UpdateLaw44ShellPlasticity` dispatches the immutable tag; the old
`UpdateTabulatedShellPlasticity` remains a table-only compatibility entry point.
There is no analytic curve endpoint/extrapolation limit. The selected SI branch
rejects `HardeningDomainExceeded` before reaching the native default SIGM/EPSM
caps (`CONSTANT_MOD INFINITY=1E20`, promoted from its source real literal).
This is an explicit supported-domain boundary, not a shell failure substitute;
finite-state and thickness guards remain. No allocation or clock is added.

Trailing fields in `TabulatedShellPlasticityParameters` and
`ShellPlasticityMaterialInput` preserve old positional aggregate initialization.
This is source compatibility, not a C++ binary ABI promise: rebuild users.
The prepared parameter grows by 32 bytes on the qualified 64-bit ABI; history
and section result sizes remain unchanged. Existing active byte forecasts use
`sizeof(PointParameters)`, and opt-in resident plasticity keeps the same one
allocation/two section slabs. Elastic batches receive no material allocation.

`ShellBatchPlasticityBinding` accepts mixed table/analytic catalogs or analytic
catalogs with zero curves. An analytic material uses curve_id=0, explicit
`LinearLaw44` tag and `linear={SIGY,ETAN}`. All supplied real curves must still be
referenced. Only table pointers are rebound into owned host/device pools; an
analytic record always has null pointers/count0. Full source inventory, parent
order, scalar bits, tag and all material declarations belong to SameScope.
The old single-material `ShellBatchPlasticityConfig` remains table-only.

The original full-shell census contains 52,483 non-failing ordinary NIP3 shells
eligible for a later source adapter. This library does not admit them itself or
increase shell/catalog capacities. It does not implement FAIL/JOHNSON, zero-C
filtered history, Ishell12/NIP1, kinematic hardening or nonlocal plasticity.
The additional original FAIL-bearing shells must not be silently admitted.

Native evidence lives in `lib_utest/qualification/native/law44`: complete,
hash-verified SIGEPS44C and converter/starter sources pinned at
`a62b27e6baa555d222a580d6218867d0be4d70b5`. `NativeAnalytic.F` supplies
MFUNC=NVARTMP=0 and independently computes B without production headers. The
shared Fortran packet only initializes the same native arrays; all constitutive
arithmetic remains in the unchanged original routine. The prior table C ABI and
its MFUNC=NVARTMP=1 setup are preserved.

Qualification owners: `qualification/law44_analytic` for source conversion,
native point/section histories and actual CUDA point parity;
`qualification/plasticity_binding` for complete immutable mixed catalogs;
`qualification/resident_plasticity` for actual joined QEPH/T3 yield, zero-curve
allocation/rebase, late triangle rejection and exact retry. Native work remains
a diagnostic within shell work; these tests do not establish vehicle accuracy
or a complete physical energy balance.
