# Actual common publication with structural beams

This gate uses the real `ShellBatchPublication`, not a private qualification
coordinator. It reuses `physical_publication`'s complete six-participant source,
CIN roster, attempt operations and accepted snapshot helpers. The new source
wrapper appends three beam18 parents to the same domain and V5 ledger, rebuilds
the same complete PART/plain groups, and creates a physical binding from that
ledger. The endpoints include ordinary, PART, plain-rigid and shell nodes.
Beam native-total inertia is carried directly through the explicit unpartitioned
plain-rigid channel. No additional geometry, fake tie or substitute coefficient
producer is introduced.

Only the startup function selects the new physical binding for the existing
participants. Existing accepted assembly, loaded owner/CIN advance, evaluation
and full snapshot functions are linked unchanged. The beam Batch is declared
before the reused Rig, so the actual publisher releases its claim before beam
storage is destroyed. Caller parent declarations and curve vectors die at the
end of source initialization; the immutable beam model owns the retained values.

Two host functions check exact source/domain/member associations, all producer
coverage, honest beam inertia attribution, and curve lifetime. Four CUDA tests
cover:

- Six loaded accepted intervals, nonzero beam and shell forces, ordinary/PART/
  plain endpoint handling, one shared epoch, and unchanged accepted state before
  common commit.
- Missing and genuinely foreign beam candidates, complete rollback and retry.
- A beam numerical failure after six valid candidates, altered expected beam
  and shell diagnostics, complete owner/beam/shell/history preservation, retry.
- Missing/uninitialized and foreign-source beam participants, late attach
  rejection without claims, valid retry, a second claimant rejection, and
  diagnostic output aliases into retained beam parents and curves.

The existing mixed snapshot records owner kinematics, raw CIN coefficients,
rigid state, all six existing participant histories/activity/observations and
the complete common diagnostics. A new pointer-free named beam snapshot records
every public beam result field. No floating tolerance is relaxed. Independent
native beam equivalence remains owned by the force/resident qualification gates;
this gate establishes common transaction behavior, not a full vehicle run.

Root commands under the shared resource guard:

```sh
cmake -S lib_utest/qualification/beam18_publication -B <build> \
  -DBEAM18_PUBLICATION_CUDA=ON -DCMAKE_BUILD_TYPE=Release
cmake --build <build> -j2
ctest --test-dir <build> --output-on-failure
```

The nested existing `physical_publication` host/CUDA regressions are included.
Owning host target: `//lib_utest/qualification/beam18_publication:host`.
Author evidence: two host functions and eight C++/CUDA-shaped syntax units pass
under 1 CPU / 512 MiB. No author native, NVCC, CUDA or Bazel run is claimed.
