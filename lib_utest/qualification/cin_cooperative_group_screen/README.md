# Cooperative rigid-group structural screen

This changes scheduling of the existing `NativeOrdinaryRigidTrace` screen only.
The ordinary scalar and rigid trace screens remain analytical surrogates; this
is not a new nonlinear/contact tangent admission. No source/count/activity fast
path or source admission change is introduced.

One 64-thread block handles each group. A fixed 64-member tile freshly evaluates
all three original `EvaluateRigidNormalResponse` calls per member. The leader
consumes those values in original member then x/y/z order through the literal
original `AddRigidMemberTrace` body. Translation products/sums, rotational
quotients/products/sums, their short-circuit order, and final trace rounding are
unchanged. The regular wrapper and optional winning-group limiter capture still
make fresh direct calls. Prepared responses are private shared values that never
outlive the launch; they are not a cached verdict or a public staged-input API.

Each worker fully initializes its active packet. Invalid range/body checks stop
a group before member preparation. Bad member indices/membership are checked
before dependent reads. Later workers can compute unused responses, but the
leader stops at the first original failure, including earlier trace overflow
before a later unavailable response. All barriers/exits are group-uniform;
separate blocks share no packet storage even for overlapping read-only raw test
ranges. Such ranges are a scheduling rejection/value control, not new owner
admission. Ordinary-node errors and global group/limiter selection retain their
existing precedence and publication behavior.

The trivial shared tile is 4,776 bytes (64 × 72-byte responses + 168-byte state),
with no dynamic shared memory, arena/header/Input change, host staging, transfer,
new allocation, or forecast delta. Root must confirm actual PTX shared usage,
register/local-memory use and occupancy. The remaining leader trace arithmetic,
one body preparation/group, barriers, global report fold and optional capture
are still serial in their original order. No speedup is claimed before timing.

## Evidence and tests

The measured CUDA-event diagnostic `vehicle-throughput-kernel32-timing-1.json`
recorded 32 active EvaluateGroups calls, mean 70.444466 ms (779 groups in V5),
versus 0.236568 ms for FinishGroups. That instrumented run serializes event
regions; it is not a normal throughput measurement. The V5 member census is
12,961 (38,883 full response calls), but neither count appears in production.

Seven complete reference files are byte copies from `21729f3`: member trace,
scalar screen, group evaluation, group wrapper, ordinary summary, complete CUDA
screen and limiter capture. `prepare_reference.py` changes only namespaces,
includes and lookup qualification. `cooperative_proof.py` restores the complete
old CUDA screen and checks the literal original member fold; prior limiter,
force, ordinary, group, recovery and capture receipts remain active. Historical
records are retained when their reviewed scheduling/build-proof inputs change.

Six host functions cover bitwise direct/frozen/staged response and trace values,
343 stiffness/rotation/trace combinations including signed zero, subnormals and
overflow, unavailable suffix short-circuit, invalid axes/body/coordinates,
2–260 member multi-tile groups, rotated current frames, source-order reversal,
overlapping raw ranges, malformed ranges, ordinary priority, stopped-input
non-dereference, limiter values and retry. CUDA tests compare the complete old
screen's reports, all control/witness fields, failure key and unchanged state
for success/failure/retry, including early response failure with later invalid
member indices. The target also executes the retained full caller and actual
owner tests from `cin_parallel_groups`. GPU comparison is within one backend,
bitwise; no tolerance change is used.

## Root qualification

From the workspace root, inside the normal root guard/shared lock:

```sh
cmake -S Total-Lagrangian-FEA/lib_utest/qualification/cin_cooperative_group_screen \
  -B crash-work/build/cin-cooperative-group-root-1 -DCMAKE_BUILD_TYPE=Release \
  -DCIN_COOPERATIVE_GROUP_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_CUDA_FLAGS=--ptxas-options=-v
cmake --build crash-work/build/cin-cooperative-group-root-1 --parallel 4
ctest --test-dir crash-work/build/cin-cooperative-group-root-1 --output-on-failure
```

Rebuild/run affected existing `cin_parallel_groups`, `cin_physical_timestep`,
`rigid_contact_response`, `cin_limiter` and current CIN recovery/ordinary owner
caches (host/native/CUDA where already configured). No new native source is
needed. Then repeat the root's complete V5 32-interval run: compare all 54 archive
files and every nontiming native JSON field, exact device/host forecasts and
limiter outputs before assessing normal Prepare throughput and kernel timing.
Author gates are only bounded host compilation/tests, source identity and
CUDA-shaped C++ syntax. They do not establish GPU execution or performance.
