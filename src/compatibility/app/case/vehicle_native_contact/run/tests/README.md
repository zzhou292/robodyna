# Native long-run output admission

The optional `ROBO_NATIVE_VEHICLE_ARTIFACT_FILE_BYTES` preview control chooses
only the existing archive file/chunk bound. Unset preserves 32 MiB; an empty
or malformed value rejects. The 30 ms Yaris run uses 24 MiB so normal replay
fits its unchanged 512 MiB allocation envelope. RunConfig carries the bound
to the same request builder during preflight and writer construction.

The owning host gate covers default configuration equivalence, bounded input,
a count-only context with the real Yaris record dimensions, existing replay
and native-contact record contracts, and complete original-source reassembly
with 24 MiB chunks. The latter reads the authentic source through the existing
fixture and compares its retained arrays and mapping. No physical owner is
created by these tests.

The old `output/physical_run/tests/replay_preflight/source-proof.json` remains
frozen historical evidence; its old complete caller hash predates this shared
request builder. It must not be rewritten to make this new change appear old.
Current qualification uses the owning tests and review of the narrow source
delta. The normal replay preflight remains before creation of a physical owner.

A subsequent actual vehicle qualification must compare physical frame and
interval payloads. Under an explicit smaller cap, source-key chunks, their
descriptors and the enclosing manifest change; reassembly still authenticates
the original member size and digest. Default output remains unchanged.
