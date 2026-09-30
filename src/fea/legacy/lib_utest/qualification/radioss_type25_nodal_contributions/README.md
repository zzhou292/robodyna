# Native startup nodal contact contribution values

This owns the SBULK3 H8/Penta6 share values and RINIT3 property TYPE13/TYPE25
translational STR substage. It does not build a contributor ledger, bind source
models, compute masses/structural STI, or admit a contact/runtime profile.
Existing ASSTIFI and beam endpoint STP remain unchanged.

Solid output is a common per-occurrence volume and pressure-times-volume share
plus a defined raw-slot mask: H8 0xff; Penta6 0x77 (slots 1,2,3,5,6,7). No value
for raw Penta slots 4/8 is claimed. Shape is an explicit native NNC choice, not a
unique-node census. Source signs and zero values remain arithmetic inputs;
material/geometry admissibility is the caller's separate responsibility.

Spring inputs are resolved native slope/scale channels, I7STIFS and ILENG.
TYPE25 never reads channel 3; nonpositive ILENG never reads the supplied geometric
length and selects native ONE. I7STIFS zero leaves native STR unwritten; the value
API returns UnsupportedProfile without publication. Native-only inputs are
intentional because slope dimensions depend on property and length mode. The
value function is not a replacement for earlier geometry/NOISE checks.

The independent oracle compiles whole pinned SBULK3 and exact RINIT3 STR/length
blocks from a62b27e6. Native wrappers marshal source-shaped arrays only. The STR
oracle receives prepared XL. A separate original length block observes actual
XL and diagnostics from supplied endpoints, with explicit MTN0/no material114
correction. That fixture control is not a whole-vehicle material observation.
Zero-length/floor coupons are labeled arithmetic substages, not admitted full
RINIT3 geometry. Penta unused and extended scratch channels retain distinct
seeds; their preservation does not turn them into native numerical outputs.

Eight host groups compare primitive bits, masks, association, signed-zero MAX
ordering, ignored fields, native length preparation, overflow/failure/retry and
composition with the existing ASSTIFI oracle. Three CUDA groups reuse the bounded
packet fixture at several launch shapes/orders. Primitive comparisons are exact;
the unchanged ASSTIFI fractional-power comparison reuses its qualified tolerance.
A production-only consumer inherits precision flags without Fortran/GTest/CUDA.
The old coefficient host/CUDA/source gates are included as regressions.

Configure this owning directory with TYPE25_CONTRIBUTIONS_CUDA=ON and the pinned
local GNU Fortran compiler. Source generation/check is `native/prepare.py`; it
never reads production numeric implementations. All numerical execution is under
the shared guarded qualification lane. Source pins or authored tests alone are
not a numerical pass or source-binding authority.
