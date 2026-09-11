# CIN force input staging

Only the complete node input scan and entry-inertia copy become parallel.
Shared-master load/coefficient transfer, structural screen, ordinary node
arithmetic, rigid update, CIN motion recovery, dependent drift and capture keep
their existing expressions and ordering. The public serial `PrepareForceTrial`
retains pointer/node/numerical-mass/witness/row validation, followed by the same
entry IN copy and ordered row/slot transfer.

The owner uses a private input-key path in `cin_advance::Launch`. The old null-key
private path remains available to the existing frozen ordinary-stage fixtures;
actual owner storage always supplies its separately owned key. No new public
admission, claimant, participant, publication mechanism or clock is introduced.

The four input kernels execute on the owner's stream:

1. Validate existing control/row/limit identity and force pointers; initialize
   the input key only after those checks pass.
2. Check independent nodes, using an unsigned 64-bit integer minimum of their
   original indices. Each node preserves its exact mass, inertia, stiffness,
   position and six load-channel read/check order.
3. Resolve the first node failure before numerical mass, witness aliases,
   row ranges, witness activity and master-role validation.
4. Copy current inertia into entry IN only after all validation succeeds.

Then the existing serial kernel transfers in original NSV/four-slot order,
including the triangle's repeated slot. It retains the separate current versus
entry inertia semantics and conditional saved-secondary M/J. Structural screen
success still initializes the later ordinary-motion failure key. Rejected
inputs leave entry IN, all force/coefficient destinations, patch/capture data
and that incoming ordinary key untouched. Kernel launches check errors without
additional synchronization or per-attempt allocation. No floating atomics are
used. A partial private later-stage failure still requires ordinary discard.

Storage adds one 8-byte device key, one pointer and one arena-region descriptor.
Existing `sizeof(CinStorage)` and complete owner forecasts charge the metadata.
No host seed array is needed: the device prefix initializes the key before read.
The old scratch count remains `9*n`, and accepted/trial tails remain
`4*n+2*r+1` doubles each. Default caps remain unchanged. Full-count layout tests
exercise both exact caps, one byte too small, output atomicity and retry.

The qualification freezes the complete pre-change force header, separately
from all extracted production helpers. A test-only symbol bridge substitutes
that complete frozen force function into the already frozen complete CIN caller;
its other native primitives are reused, without another donor copy. Source
identity proves exact pointer/node/post-node/transfer bodies, the unchanged
public serial sequence, Screen, ordinary arithmetic and complete suffix.

Four host and three CUDA functions cover 272 nodes across block boundaries,
both initial and later intervals, groups/capture/screen on and off, repeated
triangle slots, zero dependent coefficients, inactive rotational DOFs, all
named successful fields, first/last/boundary input errors, numerical/witness/
row priority, untouched entry IN/capture and ordinary key, then complete retry.
Existing public owner/native tests remain required to qualify the actual
storage path. No standalone speedup is claimed; root's pre-change complete CIN
stage was about 2.51 s per attempt, with inner sub-stage costs unmeasured.

Author checks are source/C++ syntax only, including standalone new headers and
launch-stripped CUDA bodies. Root owns numeric host, NVCC/native/CUDA and actual
loaded-case gates under the workstation resource guard:

```sh
cmake -S lib_utest/qualification/cin_force_inputs -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DCIN_FORCE_INPUTS_CUDA=ON
cmake --build BUILD --parallel 2
ctest --test-dir BUILD --output-on-failure
bazel test //lib_utest/qualification/cin_force_inputs:host
bazel build //lib_src/solvers:explicit_nodal_state
```

Also rerun `cin_parallel_ordinary` (including its public owner discard/retry),
native CIN runtime, physical structural screen/main summary, rigid/common
publication and the complete loaded accepted-prefix gate. Compare every physical
field and native transfer result exactly, with no tolerance or step relaxation.
