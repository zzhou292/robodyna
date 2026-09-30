# Ordered mapped T3 assembly

This replaces the mapped T3 serial accepted-force/stiffness scatter with the
qualified QEPH gather schedule. The small shared `elements/mapped_shell` headers
own only flat incidence, eight-channel node accumulation and five ordered CUDA
launches. Each family's result validation and initial/accepted stiffness remain
local. QEPH observers, candidate equations and both native force libraries are
unchanged. No production clock, selector or publication API changes.

Startup builds each node's incidence in ascending source-parent/local-slot
order. Parallel nodes start from the actual incoming force/couple/STI/STIR and
use the unchanged `AccumulateNodalForces<1>` and `ShellNodalStiffness::Add<1>`
leaves. Only the integer first-error key uses an atomic operation. Parent
validation precedes that parent's additions; node errors retain the original
node-only identity. All eight destinations remain unchanged on failure, and
the next attempt replaces scratch. Accepted element/history slabs are read only.

Saved OFF0 does not erase the current accepted force packet. The caller's
native ordering can retain current force after removal while accepted stiffness
is zero. Only the explicit RigidSkin law skips both contributions. The initial
stiffness branch still uses accepted epoch zero, including discard/retry.

The optional T3 arena tail contains uint32 offsets `(N+1)`, three uint32
incidences per parent, one 56-byte prepared parent, one 72-byte eight-channel
node, and one eight-byte failure key. At 21,301 parents and 372,435 owner nodes,
this adds **29,753,540 bytes**, including alignment. The existing odd-parent
status region ends at offset4 modulo8, so the combined offset/incidence arrays
reach the next eight-byte boundary without additional padding. The private device header
grows by 40 bytes and the private layout by 128 bytes. These are counted through
actual `sizeof`, not excluded from budgets. Mapped startup charges the entire
device arena again as temporary host staging and subtracts it before optional
section allocation. Old initializers allocate no capacity-sized gather tail.
The existing complete host/device limits remain unchanged.

Five new host functions cover ordered connectivity, late rejection before
scratch mutation, count overflow, exact caps/rebase and full-count forecast,
all eight physical-parent masks, virgin/accepted packets, signed zero and a
cancellation case that distinguishes reassociation, stiffness overflow/retry,
and node/result error priority. The fixture includes true NIP1 LAW44, layered
LAW44, layered LAW1, coincident layers, an explicit rigid skin and 139 owner
nodes (including untouched nodes across a CUDA block boundary). It supplies prescribed valid caches to isolate assembly; it does not
claim a new force solver trajectory.

Three CUDA functions compare to the complete frozen serial caller from
`3eca05d`: virgin and two accepted endpoints/all masks, exact channel bits and
history preservation; competing early/late failures and complete retry; saved
OFF0 with retained current force and zero stiffness. The source verifier checks
the frozen caller after reversing only test namespace/name/include changes,
along with unchanged numerical leaves, QEPH observer files and its frozen
serial oracle. Native force/stiffness evidence remains owned by `qt_mapped`.

Author validation is source identity and C++ syntax only, including temporary
launch-stripped CUDA bodies. Numerical host, NVCC, native, GPU, full vehicle and
performance gates belong to root. No tolerance was changed.

Owning commands (under the root workstation guard):

```
cmake -S lib_utest/qualification/t3_mapped_gather -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DT3_GATHER_CUDA=ON
cmake --build BUILD -j2
ctest --test-dir BUILD --output-on-failure
bazel test //lib_utest/qualification/t3_mapped_gather:host
bazel build //lib_src/elements/t3:batch //lib_src/elements/qeph:batch
```

Rerun unchanged `qeph_mapped_gather` (QEPH_GATHER_CUDA=ON), `qt_mapped`
(QT_MAPPED_NATIVE=ON, QT_MAPPED_CUDA=ON), QEPH observer gates and common physical
publication/loaded vehicle checks. The only old fixture change selects
`InitializeMapped` for its direct mapped T3 model construction. Prior identity
records are retained in the ownership manifests. Full source timing should
compare accepted T3 assembly stages using the existing app timers; do not mix
constructor/virgin costs into later-interval throughput or add GPU syncs.
