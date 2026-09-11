# Rigid normal response for the physical mesh-wall profile

Pure values in `lib_src/collision/RigidNormalResponse.h` reuse existing TL
principal-frame math and contact interval arithmetic. The normal impulse/force
at one member responds through aggregate M and principal J. No member mass is
invented; zero-M/J physical members are handled by their authenticated body.

`EvaluateRigidNormalResponse` evaluates n^T n/M plus the principal-inertia
weighted squared body-frame moment arm. It returns the nominal value and an
outward upper bound from represented input coordinates, frame and coefficients.
`AccumulateRigidContactTrace` sums upper k times response for one independent
body. The resulting trace bounds the largest eigenvalue of its frozen symmetric
contact operator. The caller takes the maximum across independent blocks.

This is local contact step screening. It does not admit the nonlinear solver,
joint/gyro tangent, CIN projection or finite member update. The caller supplies
the authenticated CURRENT force-stage frame and center; accepted group snapshots
can hold the previous force frame and must not be passed without that phase
conversion. Source registration, current activity and force scatter remain in
the mapped contact runtime. No clock, allocation or owner registry is added.

Four host functions check analytic center/offset response, long-double scalar
enclosure, objective rotation/reversal, invalid input/overflow preservation and
an independent dense six-DOF eigenvalue bound. Two native functions reuse the
existing pinned OpenRadioss wrench, body-frame acceleration and frame-update
wrappers, including epoch-zero and later force-stage reconstruction. One CUDA
function compares actual device values and invalid-tail behavior with the host.
The source verifier is reused unchanged. No reference solver is used at runtime.

Configure the owning CMake directory with `RIGID_CONTACT_NATIVE=ON`, the local
GNU Fortran wrapper and `RIGID_CONTACT_CUDA=ON`, explicit NVCC and architecture120.
Build the three `rigid_contact_response_{host,native,cuda}` targets under the
shared workstation guard; run CTest `^rigid_contact_response_` with XML output.
