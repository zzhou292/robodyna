# Upstream normal-replay admission

Source-only candidate from app1ea; hardware/build qualification pending.

`Replay::Preflight` performs allocation admission on the supplied actual context and configuration. It reuses the existing sequential-lifetime Budget, source/record envelope and interval workspace check. It creates neither source authority nor accepted state and does not replace Open's validation against its source/record limits. Native Prepare passes the exact environment request/profile/context already used for writer admission, before Execute can create a physical owner. Existing512MiB outer replay reservation and all reader limits stay unchanged.

Open preserves early cap rejection before I/O, retained Budget before index, and interval workspace rejection after index/coverage validation. ReadIntervals still plans once and evaluates staging once; its explicit early cap guard preserves the original short-circuit order before the shared byte-admission helper.

Seven new public-path tests plus four existing Budget tests separate the default384+128MiB envelope boundary from actual retained-peak rejection. A large count-only Context exercises real exact/one-byte-short Budget admission without allocating frames. Chunk transitions use existing PlanChunks and staging readers; malformed index/coverage still wins over a later workspace rejection. These are allocation/record tests, not vehicle physics acceptance.

Run focused CTests `physical_replay_preflight` (11 GTests) and `physical_replay_preflight_source`, then the real native Prepare.cpp syntax check using the qualified product compile command rebound to this isolated app. Full source construction and owning GPU acceptance remain separate; do not rerun the live simulation or promote this source-only candidate.
