# LAW44 family sound-speed handoff

The ordinary TYPE1, NLay1/NIP3 LAW44 caller replaces its startup sound speed
before membrane viscosity and the family timestep. This applies to QEPH and
T3, including constant failure and TAB1 glass at centered or qualified shifted
reference planes. The shared force cores now use the existing validated LAW44
`PointParameters.sound_speed` after the section update. No new state or clock
is introduced.

At the fixed OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`:

- `native/law44/original/sigeps44c.F:195–196` writes
  `SOUNDSP=sqrt(A11/RHO0)` and zero material viscosity, even for an already
  inactive parent. `NativePoint.F` exposes this exact return as `values(11)`.
- Complete `MULAWC:1316–1327` passes the shared SSP to this point law;
  `1990–1993` retains the declared membrane damping maximum; `3018–3023` uses
  the returned SSP in membrane viscosity. Its `2964–2978` composite/stack
  coefficient resets do not apply to this ordinary TYPE1 profile.
- `C3FORC3:563–607` and `CZFORC3:600–664` pass the same SSP through CMAIN3 into
  C3DT3/CNDT3. The selected LAW44 stiffness branches retain elastic A11/G;
  C3DT3 `106–131` resets them from PM24/PM22. The sound-speed correction must
  not rescale these stiffness coefficients or replace them with a tangent.

The complete caller files are retained in workspace
`crash-work/deps/openradioss-shell-johnson-failure-1/original/engine/source/`.
The point law, exact caller extracts, and complete family leaves remain under
their existing qualification owners and original source hashes.

## Corrected reference boundary

The old LR section declared SSP and family M as `INTENT(IN)` and discarded
the point's returned `values(11)`. The failure/TAB1 family adapters explicitly
held the initial SSP override. This caused production and reference to agree
on `sqrt(E/rho)` instead of the native post-law value. Their prior numerical
agreement did not qualify this handoff; the earlier fixed-SSP association test
and its interpretation are superseded here. Historical reports and frozen
baseline files remain unchanged.

The ordinary section now propagates each actual native point return. The
failure callback loop does the same, and its optional SSP argument is an
input/output value: initial family SSP enters, final native SSP returns. The
TAB1 and placement adapters propagate that value without recomputing it from
production parameters. Standalone default section packets already initialized
the same plane-stress value and retain their signatures and numeric meaning.

Production force-stage sound-speed diagnostics, membrane viscous force/work,
and native timestep diagnostics change. At fixed prescribed kinematics, point
constitutive histories and elastic STI/STIR are unchanged by this handoff.
Future coupled trajectories must be qualified with the correction; existing
archives are neither rewritten nor retroactively requalified. Global LAW1,
NIP3 LAW1, startup mass/inertia and accepted publication ownership are unchanged.

## Qualification

`LayeredSoundSpeedValueTest.cpp` tests both families, zero and nonzero Poisson
ratio, table and analytic/FilteredZeroC LAW44, preserved LAW1/stiffness values,
viscous stress and timestep ratios, and finite false-SSP rejection with retry.
The native normal recurrence explicitly checks the returned SSP. Shared native
failure/TAB1 comparisons now require all three actual point SSP values to match
the family packet, including removal and two inactive intervals. Those same
checks run in the existing device-owned CUDA recurrence tests, including late
failure/retry, all masks, and all three qualified reference planes.

Run the existing `shell_failure_force` owner with `TL_FAILURE_FORCE_NATIVE=ON`
and `TL_FAILURE_FORCE_CUDA=ON`, and `shell_placement_force` with
`TL_PLACEMENT_FORCE_NATIVE=ON` and `TL_PLACEMENT_FORCE_CUDA=ON`. The latter also
owns the reused TAB1 and normal recurrence targets. Root qualification must
include the affected resident mixed/failure/TAB1 and actual-source case gates.
Author host checks do not establish Fortran or CUDA parity.
