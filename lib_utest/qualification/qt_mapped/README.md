# Mapped QEPH/T3 physical-owner participant gate

This increment adds explicit `InitializeMapped` and tokened
`AssembleMappedAccepted` entry points. Each participant retains one complete
`ShellPhysicalBinding`, authenticates the actual initial raw nodal ledger and
CIN roster with the shared owner proof, and subsequently borrows that owner's
current force and CIN stiffness destinations. Initial inverses are not treated
as a replacement for source mass/inertia. No owner, clock, timestep controller,
or publication coordinator is added here.

`physical.execution()` is required for explicit rigid-skin rows. Its prepared
PART binding is checked against the actual owner's compact source records.
The skin role has zero material points, zero internal force/stiffness/work, and
active surface identity. Its cache advances only source/endpoint bookkeeping;
zero cache kinematics are not a deformed rigid constitutive evaluation. Skin
and constitutive parents can share physical node sets through the authenticated
complete formulation binding. Old standalone/joined admissions stay strict.

One accepted/trial selector controls force, layered/one-point, and failure
payloads. True T3 NIP1 remains a genuine one-point payload. Reserved dense
NIP3 storage for skins is unavailable and is never advanced as a material
history. Typed readback validates the complete family shape, law, force source,
endpoint, and failure activity before caller output publication. Inactive union
bytes and object padding are not serialization/equality contracts.

Initial stiffness is a coefficient-only operation through the qualified native
family material/geometry helpers. QEPH applies the native alternating FAC1/FAC2
slot factors. T3 adds full STI/STIR at all three slots; true NIP1 has GS=DM=0
and positive STIR. Later assembly uses the accepted force diagnostics, including
native removal masks. The neutral checked scatter is extracted unchanged from
the existing mapped QBAT helper. It does not add nodal mass or inertia.

An all-skin family reports native timestep zero as unavailable. A common
coordinator must omit that family from its material timestep minimum using the
explicit catalog role, and must dispatch witnesses by authenticated source EID
and concrete QEPH/T3/QBAT family, not by cell arity alone.

## Evidence and boundaries

The author ran 3 stiffness host functions, 6 startup/result functions, 28
production host syntax units, 4 unchanged mapped QBAT host regressions, and
3 CUDA-test host syntax units under one CPU /
512 MiB. An earlier fixture incorrectly assigned a QEPH material to a T3 with
different reference coefficients; the preserved failed host report led to a
fixture correction, without a production admission change.

The native and actual CUDA targets are authored for root execution. The native
gate reuses the complete independently compiled placed QEPH/T3 callers and
CNUPDT3/C3UPDT3 scatter wrappers, plus the original EID 2357656 one-point T3
caller/source fixture. It compares virgin coefficients and recurrent current /
removed force-stiffness scatter. Native histories remain independent of the TL
trajectory. No donor or material/force equation is changed.

Ten actual CUDA functions cover both families: complete PART/plain/CIN owner
and ordinary solid J0; initial native CIN stiffness destinations; stale tokens;
late prepared geometry rejection and exact accepted-history retry; typed NIP1
and NIP3 readback; coherent foreign mass/roster rejection before allocation;
and explicit PART skin zero-point/force/stiffness behavior and protected source
output ranges. The synthetic CIN patch has explicit prescribed activity,
stiffness and load inputs; this is not evidence of all other contributors.
These tests stop at a prepared uncommitted family candidate. Common mapped
publication, full original vehicle startup and a full simulation are later gates.

## Owning commands

Use the root's serialized resource guard and authenticated Fortran/CUDA
toolchain. Host-only configuration leaves both options off.

```sh
cmake -S lib_utest/qualification/qt_mapped -B BUILD_DIR \
  -DQT_MAPPED_NATIVE=ON -DQT_MAPPED_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120 -DCMAKE_BUILD_TYPE=Release
cmake --build BUILD_DIR --target qt_mapped_stiffness_host_test \
  qt_mapped_startup_host_test qt_mapped_qbat_regression_test \
  qt_mapped_native_test qt_mapped_cuda_test
ctest --test-dir BUILD_DIR --output-on-failure -R '^qt_mapped_'
```

Affected gates: existing mapped QBAT host/native stiffness scatter, mixed
resident sections, resident constant failure/TAB1, true T3 one-point resident,
and QEPH/T3 native family/placement regression. The composing native CMake graph
retains their complete owning source checks. Bazel owners are
`//lib_src/elements/qeph:batch`, `//lib_src/elements/t3:batch`,
`//lib_src/elements/qbat:batch_values`, and the three `qt_mapped_*_check` targets
in this directory. Root should build/test the affected resident and publication
owners after the new gate passes. Donor baseline records are preserved; the
T3 manifest changes only register the reviewed host target split/new wiring.

## First root runtime gate and focused correction

`qt-mapped-root-tests-1` passed 13 host, three native and six CUDA functions.
Four CUDA functions failed. The QEPH failures exposed a production retry defect:
source binding survives discard, but the epoch-zero accepted cache remains
virgin. Selecting stiffness by `!bound` therefore read unavailable completed
coefficients on retry; QEPH rejected zero FAC while T3 silently scattered zero
STI/STIR. Both mapped paths now select virgin stiffness by accepted epoch zero.
The existing retry test additionally compares every scattered coefficient after
discard to the first native coefficient packet, closing the silent T3 gap.

The T3 failure-history assertions incorrectly requested the legacy NIP3 payload
for true NIP1. They now use the existing typed section payload, which includes
the actual one-point damage/timestamp/activity, and explicitly verify legacy
readback rejection preserves destination bytes. Loads, intervals, constitutive
operations and tolerances are unchanged. The author ran only host syntax for
three CUDA TUs and the 27-record source receipt; corrected numeric execution
belongs to the next root gate. The failed first root log remains preserved.
