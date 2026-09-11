# Ordinary-node CIN scheduling qualification

The existing owner now launches the unchanged serial force-transfer/screen
prefix, a parallel ordinary-node loop, and the unchanged serial completion.
The private `cin_advance::Input` only borrows the actual owner's arrays. It is
not an admission, publication, selector or clock API. Production uses it only
from the existing `Impl::LaunchCinAdvance`.

Each node writes its own current inverse M/J, motion, reactions and A/AR. CIN
dependents only receive inverse zeros here. Rigid members retain the exact
nonnegative dependent-coefficient branches and are advanced by the serial
completion. The immutable source admission already forbids CIN/rigid overlap.

One unsigned 64-bit atomic minimum selects `(node_index << 32) | status`.
Node index, rather than source NID or status value, preserves the old traversal
priority. Each worker reports only its first local error. No floating-point
atomic operation is introduced. Completion calls the old `Fail` before any
rigid/recovery/capture operation if an ordinary node failed. A structural-screen
rejection retains its computed dt; a motion rotation-limit error still clears
dt through `Fail`.

The optional CIN arena gains exactly one aligned eight-byte key, with no new
allocation or host seed. Existing `sizeof(CinStorage)` accounting includes the
new pointer and region metadata. The successful prefix initializes the key on
every attempt; failed-prefix consumers check status and return without reading
it. Neither CIN history tails nor the 9*n work layout change. Owners without
CIN keep their existing layout and execution path. A failed parallel trial can
contain more private writes than the old serial trial; the mandatory discard
contract, accepted fields and returned first error are the invariant.

`serial/ExplicitNodalCinStep.cu.txt` is the complete caller at commit
`12a957fc4d7a4b55c68a6d26b2bcf09bfd123a81`. `serial/Kernel.inc` retains that caller
through its complete kernel, changing only the two CUDA function qualifiers
into macros so the same extract also validates fixture packets on the host.
`verify_sources.py` authenticates the frozen bytes and 12 unchanged live files,
then proves exact prefix/suffix text and ordinary arithmetic after explicit
view-name/local-status substitutions. This is a frozen serial CUDA comparison;
the existing complete native CIN/rigid leaves remain the native oracle.

Six host functions cover integer arrival permutations, full-count late-cap
retry, node role rules, and valid/failing complete serial packets. Four CUDA
functions are authored:

- Two deliberately ordered atomic arrivals must select the same earlier node.
- Every named successful state/history/load/entry-IN/A/AR/capture/patch field
  matches serial CUDA bitwise for 272 nodes across three intervals, with and
  without capture, screen and two disjoint rigid groups. The attachment fixture
  is reused; it contains shared masters and a repeated triangle slot. Added
  rigid groups are synthetic selected-caller values, not a new source ledger.
- Seven faults preserve prefix, ordinary-node and group traversal priorities;
  accepted state remains bitwise unchanged and a fresh retry matches serial.
- The public owner rejects an ordinary rotation limit, exposes no candidate,
  preserves every accepted node/coefficient field and stamp, and then retries
  with exact force capture under the sole selector at epochs zero and one.
  It reuses `rigid_assembly_owner`'s actual PART/plain+CIN binding because the
  existing capture admission requires rigid startup. Both group tails and
  group A/AR are included; plain CIN-only capture remains rejected.

Root owning commands, under the workstation heavy/GPU guard:

```sh
cmake -S lib_utest/qualification/cin_parallel_ordinary -B <build> \
  -DCIN_PARALLEL_ORDINARY_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build <build> --parallel 1
ctest --test-dir <build> --output-on-failure
```

Bazel targets are `//lib_utest/qualification/cin_parallel_ordinary:host`,
`:serial_fixture`, and `:cuda`. The source verifier is also a CTest entry.
CUDA compilation uses strict division/sqrt, no FTZ, and disabled FMA.

Required affected owning regressions are `tied_cin_runtime` host/native/CUDA,
`cin_physical_timestep`, `cin_physical_main_summary` (TT0 query then the same
commit), `rigid_assembly_owner`, and the common physical publication/loaded
vehicle gate. Those retain native current/saved coefficient semantics,
recovery/drift priority, actual PART/plain source admission, capture availability
and owner transaction contracts. No speedup or full-vehicle result is claimed
before root runs those gates and the existing call timers.
