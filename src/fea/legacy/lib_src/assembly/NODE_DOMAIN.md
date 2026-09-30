# Declared node domain and exact shell map

Root qualification (2026-09-11, `7527fbe`, owning dependency correction
`61e9180`) passes all10 host functions and the owning Bazel targets. The
full-count synthetic shell map retains318,738,552 B including both producers;
the524,288-node domain retains25,166,056 B. Root test sampled393,236,480 B RSS
under2 CPUs/2 GiB. Evidence is `crash-work/reports/nodal-domain-root-tests-1`
and `qbat-domain-owning-bazel-build-3`; this remains identity/capacity evidence.

`NodalNodeDomain` owns a caller-supplied source instance ID and an ordered table
of positive, unique source NIDs and finite represented SI coordinates. Distinct
NIDs may have coincident coordinates. Lookup uses the existing retained
`SourceIdentityIndex`; sorting its entries never changes domain order. Exact
identity includes the full uint64 IDs, source instance, order and binary64
coordinate bits, including signed zero. It neither converts units nor
authenticates an external source file; source association remains the caller's
obligation.

`ShellNodeMap` retains the unchanged `ShellBatchBinding` and domain. Every active
shell-local node must match a domain node by NID and all coordinate bits. Both
inputs enforce unique NIDs, so the derived map is injective. Additional domain
nodes are allowed. An identity map requires equal extents and every local index
unchanged. Complete binding inventory remains part of map identity, including
source parents, family, placement and original native coefficient inputs.
QEPH/T3/QBAT collections, connectivity, reduction order and mass values are not
rewritten. The legacy binding still rejects uncovered local nodes.

These values prove declared identity and shell-map closure only. They do not
prove that every domain node has a mechanically valid contributor, or that all
required contributors are present. A later typed composition must close source
coverage, authenticate contributor maps and native reduction phase/order, and
resolve independent, dependent, prescribed and absent DOFs before owner
initialization. No coefficient addition, inverse mass/inertia, TYPE13 append,
solid contribution, rigid registration, device allocation or clock is added.

Both classes publish only after complete startup validation. Rejection leaves
an empty handle available for retry; initialized handles cannot be replaced.
Copy/move construction shares immutable backing and leaves the original usable.
Borrowed node input can be destroyed immediately after successful initialization.
There are no mutable views or caller-supplied unchecked index overrides.

Default domain limits are 2,048 nodes / 1 MiB; explicit `Vehicle()` admits up to
524,288 nodes / 128 MiB. Map defaults are 2,048 nodes / 8 MiB, with an explicit
524,288-node / 1 GiB vehicle cap. These opt-ins confer no resident admission.
Complete count and payload arithmetic precedes borrowed node reads. Domain
payload includes its handle, implementation, typed node arena, retained sorted
index and reserved shared-control bytes. Map payload includes its handle,
implementation, index arena and complete retained shell/domain payload, with
embedded handles counted once. Existing `BoundedArenaLayout` and
`BoundedStartupArray` conventions reserve 64 bytes per shared control. Allocator
bookkeeping, call-stack sorting scratch and caller-owned inputs are outside this
payload budget. No separately allocated startup scratch survives or is omitted;
`startup_payload_bytes()` equals the complete retained payload. This is a
deterministic admission budget, not a process RSS measurement.

Qualification lives in `lib_utest/qualification/nodal_domain`. It checks exact
identity, ordered duplicate rejection, lifetime, invalid ranges and counts before
borrowed access, exact byte-cap admission, final-node failures/retry, unchanged
strict binding and native coefficients, all three shell families, and 524,288
declared nodes. The larger source-count map uses synthetic qualified Q/T geometry
at 349,645 parents / 359,785 shell nodes plus two extras; it is capacity evidence,
not an original Yaris source or mechanical-closure gate.

Owning host commands (schedule the larger map test separately):

```sh
cmake -S lib_utest/qualification/nodal_domain -B BUILD_DIR -DCMAKE_BUILD_TYPE=Release
cmake --build BUILD_DIR --target nodal_domain_check -j1
ctest --test-dir BUILD_DIR -R '^(nodal_domain_values|nodal_domain_full_count)$' --output-on-failure
ctest --test-dir BUILD_DIR -R '^shell_node_map_full_count$' --output-on-failure
```

Bazel owners are `//lib_src/assembly:nodal_node_domain`,
`//lib_src/assembly:shell_node_map`, `//lib_utest:nodal_domain_values`,
`//lib_utest:nodal_domain_full_count` and the manual larger target
`//lib_utest:shell_node_map_source_count`. Production targets own no native test
oracle, CUDA runtime or external engine source.
