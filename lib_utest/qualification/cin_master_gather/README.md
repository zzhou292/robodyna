# CIN ordered master gather qualification

This qualifier covers the optional owner-private scheduling change from complete
serial CIN force application to independent, source-ordered per-master folds,
one serial numerical-mass recurrence, and disjoint success publication. Raw or
foreign launch views remain on the complete serial route.

The checked reversal in `gather_proof.py` removes only this gather slice and its
narrow build wiring. Every restored tracked file must equal the `edab8e1`
baseline digest. The three files under `reference/` are immutable full baseline
copies; `prepare_reference.py` generates the complete pre-gather caller prefix
without changing those copies.

Host coverage compares the frozen force path for source permutations, shared
masters, repeated T3 slots, nonzero incoming cancellation, signed zero,
first-error/partial-write fallback, poisoned rejected payloads, retry reset,
raw descriptor eligibility, incidence policy, exact capacity boundaries, V5
count-only storage, and fixed current ABI sizes. CUDA sources add complete
caller, raw-route, cancellation, failure/retry, owner exact-cap/tight-cap,
groups, capture, and inherited complete owner controls. A shaped C++ pass is
only a syntax check; it is not NVCC, CUDA ABI, race, barrier, or execution proof.

Author-only light checks:

```sh
python3 -B lib_utest/qualification/cin_master_gather/verify_sources.py
python3 -B lib_utest/qualification/cin_master_gather/abi_check.py
python3 -B lib_utest/qualification/cin_master_gather/syntax_check.py
cmake -S lib_utest/qualification/cin_master_gather -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DCIN_MASTER_GATHER_CUDA=OFF
cmake --build BUILD --target cin_master_gather_host --parallel 1
BUILD/cin_master_gather_host
bazel query //lib_utest/qualification/cin_master_gather:all
```

Root hardware qualification must configure with
`-DCIN_MASTER_GATHER_CUDA=ON`, build both focused targets, enumerate the actual
GoogleTest inventory, and run CTest on the exact frozen commit. It must then run
the affected native/owner/source identities, owning Bazel targets, four V5
forecast/startup/loaded/limiter gates, exact accepted-output comparison, and an
unchanged-case throughput measurement. No CUDA qualification or speedup is
claimed by the author checks.

The pre-gather ABI receipts are `sizeof(nodal_detail::CinLayout) == 400` and
`sizeof(nodal_detail::CinStorage) == 664` on this host ABI. The focused current
host gate requires 648 and 992 respectively, plus a 64-byte private gather
view. These fixed metadata values are distinct from retained CSR and temporary
startup storage and do not replace complete owner forecast checks.

## Reuse on the current OpenRadioss-aligned owner

This branch ports donor `3bb0cb1a` onto `1dfd6754` (current owner `90a4a806`
plus independent post-node input validation). All seven gather production files,
the shared incidence utility, and numerical transfer leaf remain byte-identical.
`reuse-source-manifest.json` records those identities. The original full caller
references and historical manifest are preserved. The active source proof checks
exact reversal of all eleven owner/build hooks back to `1dfd6754`, preserving its
explicit-empty CIN path, observer support and current source APIs.

The existing numerical, frozen-caller, ABI, exact-cap and owner retry tests are
reused. Two added host cases cover dependent/master overlap before incidence
writes and explicitly empty CIN declining the optional gather allocation. The
old broad proof remains as `verify_historical_sources.py`; it describes its
historical tree, not current unrelated numerical implementations.

Dependency proof: immutable admission rejects repeated secondaries and every
secondary/master overlap before constructing incidence. A prepared row reads
accepted coordinates, its own secondary loads/current coefficients, and the
complete pre-transfer entry-inertia snapshot. Earlier row application changes
none of these inputs. Each gathered master starts at its original destination
and consumes all row/slot terms in original order, including triangle repeats.
On any failed prepared leaf, gathered value, or numerical-mass prefix, original
destinations are untouched until the existing serial apply reproduces its exact
failure and partial packet. A raw/foreign descriptor or insufficient optional
capacity retains the complete serial path.

The current baseline ABI is measured as `CinStorage=672` by the executed
`cin-post-node-inputs-host-1` XML property. Explicit-empty support adds the
existing StageView field relative to the donor's historical 664-byte baseline.
The reused gather still adds 328 bytes of metadata: current expectations are
`CinLayout=648`, `CinStorage=1000`, and private view 64 bytes. These fixed ABI
assertions do not change allocation formulas or any numerical tolerance.
