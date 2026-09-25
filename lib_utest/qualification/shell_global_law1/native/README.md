# Independent global LAW1 reference with explicit thickness policy

This qualification-only extension reuses the complete already pinned native
QEPH/T3 global force leaves and their existing private modules and COMMON
contexts. It does not call production force or coefficient code. Old native
adapters remain unchanged. `prepare.py` authenticates every inherited adapter
and donor, then mechanically produces separately named material/law/stiffness/
force adapters with one explicit `ITHK` argument. Numerical equations are not
translated or replaced. The generated receipt records every resulting byte.

The selected context remains ordinary centered LAW1, global NPT0, ISMSTR-1,
explicit ISMDISP0, and the inherited IRESP2/IDRIL0/CVIS1/DM/DN defaults. ITHK is
exactly 0 (reference) or 1 (accepted). The new T3 force adapter explicitly initializes
ISMDISP0 because the ITHK1 branch newly consumes it; the old ITHK0 wrapper did not
need that value. Complete geometry, reported-thickness update, viscosity, work,
stiffness, projection and force/couple output retain the original ordering.
The C++ bridges reuse the old complete packet packing/decoding and atomic output
checks mechanically. Only T3's effective-thickness check follows the selected
reference/accepted input instead of assuming reference thickness.

`QephThickness` and `T3Thickness` call the same complete native CNCOEF3B/C3COEF3
through the new material adapter with benign material/area inputs. They observe
only the source-defined THK0 channel. CNCOEF3B selects `MAX(EM20,THK)` when
ITHK>0 and ISMDISP0. C3COEF3 selects THK directly when ITHK>0, ISMSTR!=3 and
ISMDISP0. `NativeEm20()` returns the actual compiled native constant. No helper
reimplements the MAX, changes its units or claims full-force validity from this
single channel. Reference thickness must be finite/positive and accepted
thickness finite/nonnegative; the full-force wrappers retain stricter history
and geometry admission.

All native input/output numbers use a single caller-chosen consistent working
unit system. The inherited type comments use SI names, but these oracle calls do
not convert units. The owning tests explicitly construct metre and millimetre
packets and scale physical force/work/history independently. A production
coefficient working-length descriptor is never passed to or read by this oracle.
The QEPH projection working length is a separate concern.

NPT0 uses SIGEPS01G and its ONE_OVER_12 bending factor. NPT3 uses the already
qualified point/section adapter and the promoted default-real 0.0833333 moment
weight; the two are not assumed identical. CMAIN3's NPT0→MULAWGLC call does not
pass ITHK. CZFINTN1's NPT0/other COEF1=16/25 distinction remains in the original
source, but its use is behind the yield branch and is unconsumed for the selected
LAW1 SIGY=EP30. This slice does not claim NPT1, nonlinear geometry options,
stack/offset/thermal branches or a full native material dispatcher.

The owning project adds the existing native QEPH/T3 targets, then includes
`Native.cmake` and links `shell_global_law1_native_reference`. That target is
qualification-only and retains the inherited GNU Fortran precision and private
symbol flags. No production executable should link it.
