# S2b nonzero hourglass qualification fixture

This isolated experiment reuses unchanged OpenRadioss K1 geometry and K3 CHVIS3-style stabilization/force assembly at commit `a62b27e6baa555d222a580d6218867d0be4d70b5`. It preserves the frozen S1/S2a folder and does not introduce a production shell backend. Original donor files, license and SHA256 manifest remain in `../reference/cuda/`. CMake verifies that checkpoint at configure and build through its existing read-only verifier.

The public API separates immutable configuration, borrowed host world kinematics, owned accepted state, and owned trial results. Configuration stores explicit test coefficients, rather than inferred Yaris hourglass settings. `EvaluateTrial` reads accepted state into initialized, bounded device scratch and publishes a valid trial only after checked launches, copies and synchronization. `Commit` checks the owning state and revision, then swaps prepared storage without allocation. `Discard`, failed evaluation and stale commit cannot advance accepted state or time. No contributor has an independent simulation clock. `accepted_time` is an accepted-increment diagnostic, not a timestepper implementation. Generic `CudaError` reports a failed operation; it is not a production fatal-device classification or device/context recovery contract. Injected post-assembly rejection tests do not simulate a lost CUDA context.

Supported batches have 1–16 Q4 elements and 4–64 nodes. `UniformRectangle` selects ISMSTR1/IHBE1 and admits rectangles; `CorrectedPlanar` selects ISMSTR2/IHBE1 and admits strictly convex planar quadrilaterals. NPT3 is fixed. Positions may undergo a whole-fixture rigid transformation between accepted increments, but edge/diagonal lengths must remain fixed; deformation and reference-frame evolution belong to S2c. Velocities are m/s, angular velocities rad/s, positions/thickness metres. Input arrays remain borrowed until evaluation returns. A fixture-local xyz value avoids an element-to-contact module dependency.

The owned history preserves the native distinction: HOUR1–3 accumulate local-frame force histories; HOUR4–5 are instantaneous rotational moments. OFF/SMSTR and the two EINT work components are also transactional. OFF advances from 1 to 2 under ISMSTR1, while the selected ISMSTR2 path retains 1. EINT is donor endpoint-force work and can decrease under elastic unloading. It is not asserted to be recoverable elastic energy or conserved total energy.

Native comparison links the independent `chvis3_native` target under `../native/chvis3/`, built from the unchanged pinned Fortran routine and separately recorded context adapters. Its scalar controls are `[HVISC, HVLIN, HELAS]`; the CUDA `HourglassParams` field order is `[... HVISC, HELAS, HVLIN]`. The fixture marshals both explicitly. Geometry is supplied from K1 for this cross-implementation comparison; independent five-mode, affine-nullspace, scaling, work and balance checks supply additional physical oracles.

Independent rectangle modes are the alternating nodal vector `(1,-1,1,-1)` in each of three translations and two tangential rotations. This vector interpolates as xi*eta and has zero center value/gradient. Corrected translational modes on a distorted quad are derived independently from signed cofactors of its `[1,x,y]` matrix. Only the translational branch is expected to annihilate affine fields there: the donor's angular branch retains its uniform mode, and the corresponding nonzero response is explicitly characterized. No angular-affine-annihilation claim is made.

Tests cover native CPU/GPU force/couple/history/work agreement, zero response, five independent modes, linear/quadratic viscous scaling and reversal (including transverse H2 and both rotational HELAS/H3 modes), elastic load/unload/reload, stationary history retention, instantaneous rotational-state replacement, corrected affine preservation, corrected null-mode force direction, rigid coordinate covariance, shared-node assembly, sixteen-element strides, bytewise accepted-data preservation on rejection/retry, owner/epoch checks, input rejection and the 128-accepted-increment limit. Two persistent-state cases specifically probe a proper 3D coordinate transformation after nonzero history has been committed, and an ISMSTR2 corrected-planar load/unload/reload sequence with a rejected trial and clean retry. The transformation case compares an unrotated reference with corotating native/GPU responses and checks zero additional work during its stationary-history probe. It establishes coordinate covariance for this operator/history convention, not a finite-time rigid-motion trajectory or objective history transport under arbitrary changing frames. The identity `work_increment = -dt * assembled_force_power` checks the donor work convention, not trajectory energy conservation.

There is no K2 constitutive update, contact, failure, NPT1 membrane path, CHSTI3/yield clamp, IHBE2 branch, mass/inertia, timestep-stiffness qualification or explicit dynamics loop. FOR/MOM are zero and compute_sti is zero. Finite positive material inputs must produce representable positive thickness squared, shear modulus and plane-stress stiffness; zero/overflow/underflow configurations are rejected. Poisson ratio lies strictly in (-1,.5); stabilization coefficients lie in [0,10]. Geometry scale is within [1e-6,1e6], and dt is within [1e-8,1] seconds. These are bounded qualification domains, not recommended crash-model parameters.

Maximum explicit device storage is 19,584 bytes, below the enforced 1 MiB cap. CUDA context/runtime memory is additional. The root resource guard owns all compiler and GPU execution, with one job at a time; native COMMON access is separately serialized. The isolated Fortran compiler is supplied through `CMAKE_Fortran_COMPILER`; no compiler install, network fetch or whole OpenRadioss build occurs through this CMake project.

## Captured checkpoint and relocation

All 17 GPU/validation tests and six independent native tests passed in the
frozen workspace checkpoint on 2026-09-08. Numerical source/test/header bytes
are preserved in this tracked copy; only build/documentation paths change.
Reproduction under these paths is a separate migration acceptance gate.
Historical reports remain in workspace `crash-work/reports/`, with names
`hourglass-s2b-tests-final.xml` and `hourglass-native-tests-final.json`.

Use the [parent focused CMake recipe](../README.md) with
`TL_SHELL_ENABLE_NATIVE_ORACLES=ON` for the independent comparison tests.
The hourglass fixture library itself has no Fortran dependency. This source
migration does not extend the fixed-shape qualification or introduce a production
shell/vehicle backend. Full phase-consistent dynamics remains a separate gate.
