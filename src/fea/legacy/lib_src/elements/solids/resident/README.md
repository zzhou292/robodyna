# Shared solid resident participant

`solids::Batch` retains one immutable `solids::Model` and uses one device arena
and one accepted/trial selector for its three typed parent spans. Solid18 uses
the qualified LAW36 eight-point recurrence; Solid24 and S6Z use their qualified
mapped LAW42 force profiles. Family traits call existing force constructors,
updates and native stiffness helpers. They do not contain a material solver or
family geometry arithmetic. Rubber tensile cutoff rejects the complete trial;
this first profile does not publish deleted rubber parents.

`InitializeJoined` allocates the model, deep-copied device curve pool and typed
state slabs, then constructs native TT0 force caches with sample zero. Ordinary
updates still require positive owner intervals. Constructor diagnostics have
`has_completed_interval=false`; material and physical hourglass histories retain
their native initial values. They are not reset to a manufactured zero cache.

The private common coordinator must call `PreflightAttach` with the actual
initial owner, complete coefficient ledger, rigid assembly binding, CIN witness
source, Model and exact configuration. It authenticates the same domain and
solid coefficient snapshot, the owner's retained PART/plain binding, and the
complete initial raw coefficient/kinematic proof through `ShellPhysicalOwner`.
Only after every participant's fallible preflight succeeds may the coordinator
claim this participant. Batch retains its Model and compact claimant/phase
identity; the coordinator retains the ledger, rigid and CIN authorities.

`AssembleAccepted` authenticates the owner/token and actual CIN destinations
before any write. It adds native source-slot RHS and translational STIFN in
fixed Solid18/Solid24/S6Z, retained-parent, original-slot order. It writes no
couples or STIFR. The shared scatter stages each bounded six/eight-node
contribution; any assembly failure discards the whole owner attempt. This
deterministic SI addition order is not a native global reduction-order claim.

`EvaluateCandidate` uses the actual prepared owner's positions and velocities.
Its three kernels update only the trial slab. All named histories, force caches,
diagnostics and phases must validate before a candidate becomes publishable.
Solid18/Solid24 label the endpoint sample and S6Z labels its interval base;
traits translate these existing value APIs into the same owner epoch. Failed
candidates require a new owner attempt. A CUDA failure poisons the participant.

The private publication preflight authenticates the actual claimant, owner
token, prepared view and complete pending diagnostic identity. The coordinator
commits the sole owner once, then switches participant slabs without allocation,
CUDA calls or other fallible work. The public Batch cannot commit an owner.
Production `ShellBatchPublication` composition with every other producer is a
subsequent integration step; the qualification peer exercises this exact private
protocol with explicit prescribed other-producer inputs.

Readback stages every complete typed family before copying any caller output.
It validates device curve identities without dereferencing them on the host,
finite named history/cache values, native point applicability and sample phase.
Public packets contain all retained histories and cached observations, not
temporary derivatives, reference duplicates or device pointers. Output ranges
must be disjoint from the participant and its Model backing. A coordinator must
also protect its external retained authorities and other participants' ranges.

The forecast includes the Batch/Impl headers, complete retained Model payload,
upload arena, complete result staging and temporary whole-domain owner proof.
It is a bounded owned/startup payload forecast, not a CUDA-driver/RSS estimate.
Hard limits remain 16,384 parents, 1,024 MIDs, 1,048,576 curve points, 524,288
domain nodes, 128 MiB device arena and 256 MiB host payload. Original2412
qualification records exact measured arena/forecast bytes at the selected cap.
