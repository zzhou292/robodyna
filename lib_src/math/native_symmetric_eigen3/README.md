# Native ordered symmetric spectrum

`NativeSymmetricEigen3(tensor, result)` is a fixed-size host/device adaptation
of the complete OpenRadioss `VALPVEC_V` at pinned revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`. Tensor order is XX/YY/ZZ/XY/YZ/ZX;
off-diagonals are true tensor entries, not engineering shear. The output uses
the existing row-major `Matrix3`, with principal vectors in its columns.

Native cubic order, compression exchange, maximum-column ties, distinct and
double-root direction decisions, near-triple coordinate fallback and final
vector signs are retained. Near-triple fallback returns original diagonals
and coordinate axes and can be unsorted. Its ignored tiny off-diagonal values
are native behavior, not an exact eigendecomposition claim in that branch.
Native absolute floors and dimensional threshold comparisons are preserved;
the routine does not normalize or rescale its input. It is intended for
callers requiring this native branch/order contract, initially LAW90 B−I.

The caller owns material stretch admissibility and its frame. Nonfinite input
or arithmetic is rejected without publishing any output. No allocation,
curve, history, state owner, density, work or timestep is introduced here.
Existing `SymmetricEigen3` and every existing material caller are unchanged.

LAW90 stores a principal-rate-derived history. An arbitrary different basis
inside a repeated eigenspace can change that history despite agreeing on
isotropic stress. Reversing Eigen's sorted values alone does not establish
the native contract. Tests therefore compare ordered vectors and the actual
engine LAW90 projected-rate expressions, as well as reconstruction controls.

`FLOATMIN` stores `2.2e-16` in default REAL. `SQRT(FLMIN)` is also REAL before
multiplication by WP TWO: `2 * double(sqrtf(2.2e-16f))`. Neither binary64
epsilon nor a binary64 square root is substituted. Constants and this
promotion are compared to the native ABI independently.

The owning qualification is `lib_utest/qualification/native_symmetric_eigen3`.
Its manifest authenticates complete enclosing donors. This math freeze alone
does not qualify a LAW90 point recurrence or any source/element admission.
The operation-order gate compiles without fast math or floating contraction;
CUDA also uses precise division/square root and retains subnormals.
