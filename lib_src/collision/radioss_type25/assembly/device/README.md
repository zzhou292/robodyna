# Device contact-to-node incidence

`RadiossType25AssemblyDevice.h` provides bounded numerical staging for the
source-ordered assembly leaves. It introduces no physical owner, history,
clock, force law or independent physical publication. The common coordinator
authenticates selected rows and borrows the returned CSR for its private gather.

`Initialize` allocates one retained arena from an explicit forecast. `Preflight`
queries CUB/CUDA and is a startup resource operation. The arena includes both
64-bit key arrays, all occurrence ranks, all nodal offsets, the integer failure
word and aligned CUB scratch. No association array or CSR is copied to the host
by `Stage`; only the 8-byte diagnostic word is returned. Host storage is fixed.

The device validates original logical cohort boundaries, packs each of the five
endpoint occurrences as `(node, source operation rank)`, radix-sorts the complete
list and builds node offsets by parallel lower bounds. Exact rank decoding uses
the same `Schedule.h` as the host oracle. Repeated slots and zero-weight slots are
retained. Integer diagnostic atomics choose the earliest bad cohort, then earliest
bad occurrence; there are no force/stiffness atomics or within-node sums here.
Invalid input never grants a view, and all decode reads remain bounded even when
the cohort schedule is invalid.

The synchronous staging API drains its explicit startup stream before returning,
once work is launched, including device errors; a CUDA error poisons the workspace.
Host admission errors enqueue nothing and do not drain already queued caller work.
There is no CPU
fallback. Inputs must remain device-readable and immutable during the call.
Host admission rejects capacities, pointer extents/alignment, missing provenance
and overlap with the entire owned arena. Pointer descriptors cannot establish
allocation validity; that remains the producer/coordinator's responsibility.

Every stage attempt or discard invalidates the old opaque view. An accepted CSR
that must survive a trial rebuild belongs to a separate retained instance; both
arenas must be included in the coordinator's resource forecast. `IsCurrent`
authenticates the workspace/generation only. Source/physical epoch and packet
association equivalence are separate checks, never inferred from the CSR.

The returned incidence may be used with the exact same immutable connectivity,
cohorts and prepared endpoints in `GatherNode`, with a unique writer per node.
All gathered outputs remain private until the common owner publishes them. An
empty contact roster produces complete zero offsets without a zero-grid kernel
or a radix sort. No runtime allocation, host reconstruction, physical commit or
reference solver call occurs in this component.

The owning device tests compare complete CSR arrays against the existing ordered
host builder, then compare complete GPU nodal results against the native ASS0
oracle. They also exercise dense aliases, changing row/cohort counts, malformed
cohorts/connectivity, bounded deterministic errors, alias/capacity rejection,
view expiration, retry, empty contact and separate accepted/trial storage. A
separate consumer links only production C++/CUDA. These are correctness gates;
no whole-step or CPU OpenRadioss speed result is claimed by this module.
