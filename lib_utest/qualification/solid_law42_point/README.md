# LAW42 point qualification

Root `law42-point-root-tests-4` passes10 numerical functions:5 host,4 native,
1 CUDA, plus complete native source identity. The fixture parameters represent
the original MAT007-to-LAW42 mapping; this is not an all-source vehicle gate.

Tests cover virgin/hydrostatic/isovolumetric finite response, rotation covariance,
reversible loading, tension cutoff and inactive material, supplied current
density, native pressure enhancement, original working-unit conversion and
invalid input/late retry. CUDA executes the actual shared Eigen/material path
and compares its outputs to the native routine after rejection and retry.

The native oracle retains complete SIGEPS42, complete VALPVEC_V/VALPVECDP_V
subroutines from the original SIGEPS33 owner, complete precision/matrix helpers
and an exact HM_READ_MAT42 bulk slice. Includes/modules reuse existing pinned
paths. All bytes and Git blobs are checked; namespace rewriting changes no
expression. Unselected bulk interpolation fails explicitly. The native spectral
routine uses its actual MVSIZ512 strides; the ABI only selects the tested profile.

Analytic comparisons use2e-10 relative/dimensional scale. Native comparison adds
2*sqrt(binary64 epsilon) for repeated-root cubic spectral sensitivity. The
native diagonal(-0.19,0,0) residual explicitly observes its split double root
and checks the bound; the independent Eigen decomposition has the same bound.
A1e-5 relative shear-modulus change produces stress outside the allowed comparison. This is
numerical equation qualification, not a crash-accuracy criterion.

Preserved failed attempts:

- Runs1/2 exposed the native unsuffixed REAL0.81 literal. The implementation now
  preserves its float-rounded value before promotion; no ET tolerance workaround.
- The initial Eigen iterative3x3 CUDA path returned zero roots. A tiny independent
  diagnostic traced this to its host-only specialization. The existing device
  direct solver fixes it; details in `law42-cuda-eigen-diagnosis-1/`.
- Run3 caught an exactly zero stretch being rounded slightly positive by the
  spectral solver. Explicit positive-definite input validation now rejects it
  before decomposition and preserves prior output.
- Initial native comparison was too tight for the original cubic double-root
  split. The added residual/control test documents the conditioning separately
  from unchanged analytic checks and the corrected native literal.

Configure this directory with `LAW42_NATIVE=ON`, `LAW42_CUDA=ON`, the local
gfortran11.4 wrapper and CUDA13.2 architecture120. Use the serialized workstation
guard for builds and GPU execution. No element force/history or full-vehicle
trajectory is certified by these point tests.

Owning point/helper/host targets pass `law42-owning-bazel-build-1`; Eigen is
the repository's declared3.4 dependency. CMake independently executes the GPU
and native gates; a successful Bazel build is not counted as a numerical test.
