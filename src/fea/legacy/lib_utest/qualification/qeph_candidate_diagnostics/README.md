# Mapped QEPH candidate diagnostics

At baseline `b939d7645dc371f684c451700bcf4d4bce8deaa6`, candidate material,
section history and force work already ran independently in 64-thread blocks.
The complete original QEPH population uses 5,064 such blocks. Its finalization
then ran in one thread: first element failures, full mapped-result validation,
all 372,435 owner-node displacement/quaternion checks, and the parent diagnostics
and signed cache-work folds. Root's first loaded timing measured 33.641 seconds
in QEPH evaluation across three attempts; that call total does not distinguish
material kernels, diagnostics, host checks and existing synchronization.

This increment changes only the mapped diagnostic finish. A 256-by-128
grid-stride preparation kernel independently performs the existing
`mapped::ValidResult` checks and the exact nested `hypot`/unit-quaternion node
checks. The final kernel retains the complete error order:

1. First element-update failure in ascending family index.
2. Missing mixed catalog, then first invalid mapped result in family order.
3. Kinetic-availability consistency, then each node's finite displacement,
   maximum update, and quaternion check in owner order.
4. Parent ratios and diagnostic sums, then the complete final finite check.

Thus a later failed material update still outranks an earlier invalid result,
and a bad quaternion still retains that node's already-folded displacement in
the private failed control. Public diagnostics remain unpublished on failure.
The original `MeasureParents` statements are extracted verbatim. Signed work
retains all parent/local-slot subtractions through `AccumulateInternalWork`;
there are no per-parent work subtotals, floating atomics or reassociated sums.
First-material minima and all-skin defaults are unchanged. The nonmapped
finalizer and candidate material kernel remain byte-identical function bodies.

The existing mapped assembly records are transient after assembly completes.
Named diagnostic helpers reuse `AssemblyParent::status` for result validation
and `AssemblyNode::value[0]/touched` for displacement/quaternion validity. Every
candidate overwrites these fields; every later assembly initializes its own
records again. Incidence, coefficients, histories, owner state, accepted selector
and public interfaces are unchanged. No reinterpret casts or C++ lifetime
overlays are used. There are no new pointers, arena bytes or allocations.
The full mapped base arena remains 1,183,491,056 bytes for 324,094 parents and
372,435 nodes, excluding its unchanged material sidecars.

Three host functions compare every named control/diagnostic field against a
frozen independent copy of the old serial finish: three endpoint epochs, all
eight active masks, skin defaults, coupled/prescribed diagnostic branches,
signed cancellation with a reassociation negative control, competing failures,
finite-input overflow, partial failed controls, missing catalog, retry, exact
full-count cap, scratch reuse and legacy kinetic arithmetic. The supplied valid
force packets isolate diagnostics; they do not claim a new constitutive path.

Two CUDA functions exercise the actual preparation/finalization kernels against
the frozen serial device routine, including cache immutability, late failures,
retry and reuse of diagnostic scratch by the next actual mapped assembly.
The source receipt authenticates the old complete files and copied test routine,
unchanged shared leaves, candidate mechanics and exact extracted reduction body.
Existing native Q/T and material/source recurrence receipts are not repinned.

Author evidence: three host functions, six C++/temporary CUDA-shape syntax units
and three new/affected source receipts pass under one CPU and 512 MiB. No author
NVCC, native numerical or GPU execution. Root owns:

```
cmake -S lib_utest/qualification/qeph_candidate_diagnostics -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DQEPH_DIAGNOSTICS_CUDA=ON
cmake --build BUILD -j2
ctest --test-dir BUILD --output-on-failure
bazel test //lib_utest/qualification/qeph_candidate_diagnostics:host
bazel build //lib_src/elements/qeph:batch
```

Rerun unchanged mapped Q/T native/CUDA and deterministic assembly gates, then the
complete loaded two-interval/discard-retry case with its stage timers and existing
6 GiB device-growth guard. This change leaves the serial ordered parent/work
fold intact. If evaluation remains expensive, measure the material and finish
kernels separately before selecting another increment; the present stage total
alone is not a material-kernel performance measurement.
