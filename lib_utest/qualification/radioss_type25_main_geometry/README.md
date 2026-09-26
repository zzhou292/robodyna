# Native exterior main geometry qualification

The value API covers one already-resolved exterior EightSlot support:
NORMA1D area/pre-reversal normal, original INSOL3D centers and primary-only
orientation, and VOLINT signed raw volume. It preserves the before-INITIA
reader packet, original native coordinates and raw repeated PENTA slots.
It does not establish membership, X/X_C offset policy, material/property roles,
complete main/gap coefficients, topology or runtime admission.

The oracle mechanically extracts the complete numerical blocks from pinned
OpenRadioss sources. Its wrapper uses IC=1/IR=0, source-shaped local X/IRECT/IXS
arrays, and serial routine-local storage. Skipped membership and diagnostic
printing are explicit wrapper boundaries. No production numeric function is
used to create expected results. All six result doubles, four corner indices
and the actual native reversal count are compared; there are no undefined
output channels in this unique-support packet. The unchanged source donors
retain their AGPL notices and are authenticated by SHA256 plus Git blob.

The corpus has 184 packets: Q4/T3, actual raw PENTA repeats, rotations, translated
warps, both directions, native area floor, signed zeros, zero/negative volume
and nearest DDS sides. Separate failures preserve every output and retry.
The native cyclic determinant witness prevents replacing VOLINT with a
mathematically equal structural volume reduction. Raw finite negative/zero
volume is observable but still rejected by the existing solid coefficient leaf.

Configure the owning project here with `-DTYPE25_MAIN_GEOMETRY_CUDA=ON` and the
workspace pinned gfortran-local compiler. CMake includes complete unchanged
startup/current-normal regressions because their double normal implementation
is factored through the same value helper. Run CPU and actual CUDA CTests in
separate guarded steps, then the two owning Bazel targets and startup/current
normal Bazel regressions. The standalone consumer must inherit precise flags
without private options and must not link the native reference library.

No actual V5 geometry packet is baked into production or this synthetic corpus.
The planned 741-row source assessment and any post-I25GAPM topology phase must
be separately source-bound and qualified. The former coated assessment remains
frozen at its original scope. See workspace
`planning/openradioss-alignment-2026-09-24/V5_MAIN_COEFFICIENT_GEOMETRY_DESIGN.md`.
