# Exact row maxima during owner seal

This slice replaces only the private owner's serial row validation/max loop.
The complete `ExplicitStepStability.h` and seal caller at `301a750` are retained
under `frozen/`. `prepare_frozen.py` changes namespaces/includes and the matching
test-only control type; it does not extract a reduced numerical oracle. The
source verifier proves that the public serial finalizer retains its complete
prefix, ordered loop and scalar suffix. It also compares the unchanged seal
identity/contributor checks, force validation and failure/status completion.

The optional owner path scans disjoint rows into at most 256 summaries, with
256 lanes per block. Maxima contain no floating-point additions. Equal positive
values select the lowest original node. Initial positive zero wins over both
signed zeros. A final single writer combines at most 256 summaries and invokes
the exact original scalar formula. The maximum extra device payload is 8192 B,
appended to the existing control allocation; the control type and its ordinary
readback size stay unchanged. Both owner initialization and combined CIN
forecast charge this tail. There is no per-step allocation or new allocation
count, and no accepted-state write.

Only the actual owner row layout and exact private control tail select this
path. Invalid headers, unexpected row layout/tail, or any invalid row use the
complete old serial finalizer. Identity/contributor/first-node-and-axis errors
precede row errors. Failed finalization keeps original cleared limit/row flags;
invalid row position is not invented as a public failure node. Same-stream
ordering makes all summaries fresh for the current attempt; early failures
never consume stale summaries. CUDA failures keep the existing owner poisoning
path. Native force assembly, sums, admission and all CIN math are untouched.

The host gate covers exact ties/zeros, reordered partitions, subnormal and
near-overflow values, minimum-step rejection, prefix error priority, invalid
row fallback and exact tail capacities. Root CUDA gates compare complete
frozen/production control fields and unchanged scratch bytes across block and
stride boundaries, malformed row layouts and simultaneous failures. Real owner
tests cover loaded bounds, first maximum node, exact stable dt, failure readback,
discard/retry, successful later commit and exact allocation cap. Existing
node/axis seal controls and the original host/CUDA stability gates are batched.
Private header kernels have internal linkage so the actual owner and direct
comparison can coexist in the same test binary without duplicate symbols.

The measured full seal stage was approximately 0.12 s per interval. The isolated
row-loop cost and the new implementation's speedup have not been measured.

## Root gate

From the integrated TL tree, use the shared workstation guard and ordinary
resource limits. The author has not run NVCC, native code, CUDA or full source.

```sh
cmake -S lib_utest/qualification/nodal_seal_rows \
  -B ../crash-work/cache/nodal-seal-rows-root-1 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DNODAL_SEAL_ROWS_CUDA=ON
cmake --build ../crash-work/cache/nodal-seal-rows-root-1 --parallel 2
ctest --test-dir ../crash-work/cache/nodal-seal-rows-root-1 --output-on-failure
```

CTest entries are `nodal_seal_rows_host` (five new functions),
`nodal_seal_rows_affected_host` (the existing stability host file),
`nodal_seal_rows_cuda` (four new CUDA functions plus existing seal/stability
controls), and `nodal_seal_rows_identity`. The Bazel targets are
`//lib_utest/qualification/nodal_seal_rows:{host,cuda}` and the affected owner
`//lib_src/solvers:explicit_nodal_state`.

After this gate, rerun the existing combined CIN forecast/exact-cap owner and
full-capacity `nodal_vehicle_owner` rollback gates. Run the unchanged V5 loaded
prefix and compare its accepted archive before reporting a timing benefit.
The V5 maximum device forecast grows by exactly 8192 B; no guard is raised.
