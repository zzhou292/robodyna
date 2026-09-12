# Frozen original-node weighted surface algebra

The additive helper maps exactly three or four original ordered nodes. It
accepts fixed nonnegative linear weights, including physical-facet compositions;
it does not substitute Q4 shape values at averaged facet parameters. Weights
use the existing 1e-12 partition tolerance and are never normalized. Therefore
partition/momentum identities have floating-point and admitted partition error,
not a promise of exact bitwise conservation for arbitrary weights.

Position-only evaluation reads no velocities or masses. The combined helper
evaluates all positions then all velocities, preserving each source-slot
Add/Scale schedule. It checks zero-weight inputs and stages all output. Existing
Q4/T3 APIs and their different failure-output contracts remain unchanged.
Native adapters reuse EvaluateQ4Shape or copy validated T3 barycentrics. These
are numeric descriptors, not source/offset/geometry/owner admission.

Qualification: four host functions cover independently strided views, 25 Q4
points bitwise against the existing map/projection, T3 mapping, transpose work,
composed-facet counterexample, late NaN rollback, topology/range checks, signed
zero and subnormal inputs. No native or CUDA gate is claimed by this slice.

Build under the author guard with CMake at this directory; target
`weighted_surface_host`, then CTest. Bazel owning target is
`//lib_utest/qualification/weighted_surface:host_check` (root execution only).
