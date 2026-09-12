# Mapped QBAT scheduling increment

Baseline 01a9a39; isolated worktree qbat-mapped-gather. Root has qualified the V5
forecast, initial owner, two loaded intervals and replay before authorizing this
increment. No constitutive, force, stiffness, timestep or signed work formula is
changed. Author checks remain one CPU / 512 MiB; native/CUDA/full-source run is root-owned.

Mapped assembly currently validates each parent's four nodes and complete
accepted result, computes its native diagonal FAC*STI/STIR, then writes six
force/couple and two stiffness destinations in one thread. Reuse the qualified
mapped_shell flat incidence, staged node values and integer first-error schedule.
QBAT has BatchResult.internal_force_n/internal_couple_nm and no mixed law array;
a narrow typed access seam must expose those actual fields without copying
forces or fabricating a law tag. Existing Q/T wrappers retain their interfaces.

Parents prepare independently. Per-node gather preserves ascending source
parent/local-slot order and actual incoming eight-channel values. Integer keys
2*p for validation and 2*p+1 for addition retain original first-error precedence.
Publish only after all checks pass; failure leaves the complete incoming arrays
unchanged. No floating atomics, reassociated node sums or per-attempt allocation.

A mapped-only arena tail contains offsets, encoded four-slot incidence, parent
stiffness/status, eight-channel staged nodes, one failure key and bounded
maximum-displacement block summaries. Initialization, rebase and host/device
forecasts use one checked layout; legacy initializers allocate no tail.

Candidate Advance already runs per parent. Run ValidResult independently and
cache its boolean verdict, but consume it in the existing serial parent loop so
failure prefixes and every signed sum remain identical. Whole-owner displacement
uses the unchanged per-node sqrt/dot expression and a deterministic maximum.
A nonfinite node requests the old serial displacement scan, preserving its rare
failure-prefix diagnostics. No parallel signed observer reduction is included.

Qualification: freeze complete baseline mapped assembly and candidate measure
sources independently of the new helpers. Host tests cover ordered incidence,
nonzero/signed-zero/cancelling input sums, exact arena caps and full-count layout.
CUDA compares all eight destinations and diagnostics against frozen serial
kernels, including competing early/late failures, complete rollback, changed
input retry, epoch zero and carried native states. Existing qbat_mapped owner and
native tests, qbat_resident and common publication remain affected regressions.
Shared accessor changes also require unchanged qeph_mapped_gather and
t3_mapped_gather serial-oracle gates. Final app timing uses the same retained V5
loaded-prefix task and existing synchronized stage timers; no speedup is assumed.

Authored boundary: four QBAT host functions, ten affected Q/T gather host
functions and 14 syntax units pass; five source verifier entry points pass.
The optional original V5 mapped tail is 29,024,792 bytes, with full startup
host/device forecasts using the same checked layout. Four frozen-serial CUDA
functions are authored but root execution is pending. See the owning qualifier
README for exact commands and preserved author reporting correction.
