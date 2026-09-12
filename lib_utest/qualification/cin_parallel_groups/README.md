# Independent CIN rigid-group scheduling

This gate freezes the complete `01a9a39` CIN caller, screen and screen values.
The new owner evaluates each immutable, disjoint rigid group on one CUDA thread.
Within each group the existing body preparation, trace, force reduction, member
motion, and orientation expressions and visitation order are unchanged. No
floating sums are reassociated. Shared-master CIN force/motion transfers remain
in source order. This is a scheduling qualification, not a new native solver law.

The ordinary screen finishes before independent group screens. A 24-byte report
per group records the limit and original visited-node/error semantics. One
ordered scan chooses the first group failure and strict minimum, including the
old carry of the preceding visited node on a malformed range. Equal bounds keep
the ordinary node or earlier group. The report tail is then reused for motion:
ordinary failure wins before group workers launch; complete candidate/member
motion precedes orientation within each group; an ordered report scan precedes
unchanged CIN dependent recovery and force-stage capture.

The actual owner retains this tail in its existing CIN allocation. Both public
forecast and allocation use the authenticated group count. It costs 18,696
device bytes for 779 groups, plus the `sizeof`-accounted host layout/pointer
metadata. There is no extra state slab, clock, allocation per step, or floating
atomic. Zero groups add no tail. Private input without reports preserves the
old serial group path. Source admission already proves disjoint group members
and no CIN dependent/master intersection with those members.

Failure can leave additional **private trial/capture** writes in later groups.
Accepted state, published failure status/node/bound, failed readback, and
discard/retry behavior remain exact. The CUDA tests intentionally compare only
the applicable partial-output contract on failure, then compare all successful
retry fields bitwise. Screen errors retain the old key/bound behavior; an
orientation rotation-limit failure clears the bound as before.

Six host functions cover reverse group completion/order, 0/1/2/64/65/129 groups,
zero M/J physical members, two-member and general bodies, half/full kicks,
capture on/off, exact ties, malformed ranges, late orientation/candidate
failure, untouched output and exact host/device caps. CUDA tests compare the
complete frozen caller through three accepted intervals, both capture/screen
settings, multi-block group populations, earliest failure and clean retry.
The actual PART/plain/CIN owner tests cover ordinary and later-body failure,
all accepted node/CIN/group fields, capture, failed readback and stable allocation
across epoch-zero and full-kick retry. The test packet adapter allocates guard
storage only for qualification; the actual owner test exercises retained scratch.

`prepare_reference.py` changes namespace/include spelling, aliases the existing
summary ABI and qualifies calls to suppress ADL. The entire frozen caller prefix
and launch are independently compiled; only its owner dispatch is omitted.
`group_proof.py` verifies exact extracted arithmetic and reversible scheduling
changes before earlier CIN identity verifiers consume their original view. All
older frozen baselines and native donors are unchanged. Updated manifests retain
the previous hashes of each changed production/packaging record.

Root qualification:

```sh
cmake -S lib_utest/qualification/cin_parallel_groups -B <cache> \
  -DCIN_PARALLEL_GROUPS_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build <cache> -j1
ctest --test-dir <cache> --output-on-failure
```

Batch affected existing CIN force-input, ordinary, screen and capture tests;
physical timestep/main-summary, rigid-assembly owner and tied-CIN native/CUDA
gates; then the unchanged V5 loaded/discard/replay and allocation forecast gate.
Owning Bazel targets are `//lib_src/solvers:explicit_nodal_state` and this folder's
`:host`/`:cuda`. Author verification is bounded host, C++-shaped syntax and source
identity only. Native/GPU correctness and timing are root execution boundaries.
