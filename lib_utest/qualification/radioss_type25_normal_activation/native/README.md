# Native normal-activation reference

The serial qualification reference runs complete original I25FREE_BOUND then
complete I25TAGN under NSPMD1/ISPMD0/NSNR0/IEDGE0/FLAGREMN2. Source routines are
unchanged except private module/COMMON/barrier names. All four donors have
pinned original SHA256 and Git-blob identities. Foreign and edge declarations
only permit unselected source branches to compile; those arrays are not read.

The C ABI has17 arguments. Source connectivity changes from C++ zero-based
node indices to native one-based indices; ADMSR, neighbor and CSR main IDs are
retained. Full original NRTM and signed MSEGTYP are never compressed or replaced.
IRTLM is copied from the actual post-Begin/OPTCD staged OptimizedRow history.
The passed optimized list is exactly the source suffix; prefixcount0 merely
reindexes a loop that never reads the excluded retained prefix. The separate
native retained-row loop still consumes all staged histories.

Expected free IDs are derived independently by I25FREE_BOUND, not copied from
the input list. The reference returns those IDs and both complete mask arrays.
It does not read source normals/bisectors or call a production mask helper.
Its bounds are4096 nodes/mains/secondaries,16384 references and32768 entries per
CSR/optimized list. These are serial test limits, not production capacity or
completeness authority. Output masks must be exact0/1. No clock, physical owner,
source authenticity or accepted publication is created by this adapter.

Generated sources: Constants.F90, Boundary.F90, Tag.F, Free.F, Wrapper.F90.
Compile with GNU Fortran, the generated include/module directory, precise FP
flags and bounds checks. The existing selection Sources.py constant reader is
reused. NativeReference.h/.cpp belongs to the qualification target only. Source
generation is not an owning compile or numerical pass.
