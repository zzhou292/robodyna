# Native force-stage group kinetic observation

`ObserveGroupForceStageKinetic` consumes pre-kick primary/member velocities,
actual constrained translational/angular accelerations, the already updated
principal axes and the native `(DT1,DT12,DT2)` duration tuple. It evaluates native
`V + A*(DT1/2)` at the base force/position time. First-step DT1 is zero; subsequent
fixed steps use DT1=h, DT12=h. Unequal pure-packet durations are also admitted when
DT12=(DT1+DT2)/2 and all input velocity/frame times agree with DT1.

This value API does not authenticate an owner, advance a frame or integrate a
state. The caller supplies readable arrays in the immutable metric's exact source
order, authenticates that all values came from one actual owner packet, and keeps
those values alive during the call. Finite schedules, native metric partitions,
duplicate source/global member IDs and output/input range overlap are checked.
Failure leaves every caller output and input byte unchanged. An orthonormal frame
or finite velocity from a wrong historical phase cannot be authenticated from its
numbers; tests establish that those wrong choices change the observable.

The output retains group/set IDs, member count, full phase, reconstructed primary
motion, native member kinetic channels, aggregate full-tensor channels and a
per-group replacement. The unchanged quadratic/decomposition implementation is
shared with the stored observer through `NodalRigidKineticValues.h`. Existing
stored observation types, phase admission, arithmetic order and ABI are unchanged.
Native TOTAL J remains authoritative; physical/added/primary regularization and
principal correction are diagnostics counted once. No primary proxy J is added.

The function uses a bounded 256-entry local motion array and no heap/device
allocation. It performs no force evaluation, new eigen solve, clock operation,
state publication or energy admission. Owner capture is a separate future slice.
An enclosing accepted endpoint is one step later than this observable's force
position time; do not relabel it endpoint kinetic energy or compare it with the
existing stored-kinetic publication value.

Qualification uses nonzero DT1/A/AR in exact retained RGBCOR primary, small/large
member and correction-publication fragments at OpenRadioss
`a62b27e6baa555d222a580d6218867d0be4d70b5`. The authored wrapper is
`lib_utest/qualification/nodal_rigid_group/native/NativeForceStageKinetic.F`.
`verify_sources.py` checks the complete originals and extracted bytes. The old
zero-DT1 wrapper remains unchanged. Independent long-double world-tensor tests,
64 native force-stage packets and one optional CUDA gate cover this value scope.
Neither wrapper proves complete ENCIN/ENROT output parity or physical energy
conservation; its baseline native output-producer audit is still separate.
