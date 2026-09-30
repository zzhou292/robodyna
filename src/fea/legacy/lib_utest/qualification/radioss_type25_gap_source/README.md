# Native startup gap values

This bounded host module implements I25STI3 secondary shell masking followed by
truss/beam/spring additions and scale/cap, and the independent I25INI_GAP_N main
nodal reduction. It accepts explicit IGAP1/ILEV1, property1, IINTTHICK0/1, no
free-edge zeroing and no contact thickness update. Springs13/25/45 have two
physical endpoints; TYPE12 is rejected because its third-node branch is outside
this first scope. All inputs are resolved native working-unit operands.

PhysicalShell and MainGapFields plus the property1 half-thickness expression
are reused from the old shell producer, whose source is unchanged. Young's
modulus and structural thickness in that shared record are not consumed by this
gap producer. Positive line part thickness suppresses the unused area operand;
zero/negative spring part thickness performs no update. The native finite MAX/
MIN later-equal rule is local to this module and has exact signed-zero tests.

The caller supplies complete physical source populations, native storage order
within Q4/T3 and line/spring families, authenticated expanded main roles and
NSV/MSR order. The producer checks the complete main-node set and unique rosters,
but these numerical checks are not source completeness authority. It creates no
owner or clock and changes no runtime admission. The output does not resolve
the separate interface GAPMIN/search-bound reductions. No production source
table, captured gap array, beam area heuristic or nodal stiffness substitute is
used. Genuine beam GEO1 and post-I25GAPM topology binding remain app work.

Preflight is descriptor/count/profile-only and reserves all output/staging and
two nodal work arrays plus tags. Build checks alignment/ranges/aliases before
using scratch, validates all rows, completes all arithmetic, then publishes.
Caller output and input are preserved on failures, including late overflow.
RN/gradual-underflow arithmetic and inherited precise GNU compilation are the
qualified environment. No CUDA implementation or GPU-performance claim is made.

The independent qualification oracle copies the original full selected gap
blocks; it does not call the production branch helper or compute expected DX in
C++. It retains source-shaped IXC/IXTG/IXT/IXP/IXR/GEO/THK/IPART arrays with one
resolved property per fixture row. It is bounded to32nodes/32shells/16rows per
line or spring family/64mains for tests only. Original branches remain in the
source blocks, while the wrapper fixes the explicitly admitted profile.

Eight host groups compare all secondary/main-node/corner/max/extrema bits:
noncontact masking and restored beam gaps, property precedence, scales/caps,
roster permutations, signed zeros, source-unused fields, solid-role clearing,
empty rosters, alias/cap/row errors, late overflow/retry, and unchanged legacy
ordinary shell gap results. The owning CMake project also executes the old
shell-source host/source tests. Production-only consumer uses inherited flags
and must not link a Fortran oracle. Source checks validate original byte extents,
SHA256 and Git blobs. CMake and Bazel execution belongs to the lane owner.
