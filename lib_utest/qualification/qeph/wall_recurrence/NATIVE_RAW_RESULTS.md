# Full CW1 native raw capture results

All six frozen fixture/velocity jobs completed their raw collection on first
execution: one/two QEPH cells and physical world-X boosts -8, 0 and +8 m/s.
The files retain all 108 centered native matrices, 72 contact-branch records
with all physical directional samples, full 109/194-coordinate dictionaries
and actual moving baselines. Exactly 57,942 native cell intervals completed.
This is raw evidence, not a spectral/switching decision or impact timestep.

The 390 raw files occupy 47,302,880 bytes, including six exact provenance copies,
progress inventories and final indices. The 96 MiB raw-plus-derived cap leaves
53,360,416 bytes for all derived reports/selection. Every final inventory's
64 earlier files was independently byte/hash checked; externally pinned index
hashes are retained in `crash-work/reports/qeph-wall-native-raw-inventory-1.json`.
Strict semantic reading and recomputed derived decisions remain subsequent gates.

All 38 retained model/raw/analytical/support functions passed after extracting
the unchanged one-sided quotient and directed-comparison helper. Fourteen CLI
preflight rejection cases passed without creating an output directory. The
runner verifies exact input provenance and the actual executing image before
collection; caller-supplied hashes alone do not authenticate source files.
Root checked all declared source/build records before actual execution.

The [runner source map](raw-runner-source-map.json) pins 170 inputs, SHA-256
`e2be0e22572847b822942381a445ef9ef42b4f321b01f72a41b87eb3039ca4eb`.
Actual runner SHA-256 is
`561a28a3a9bf6b2e7a0c9a87efe80c6a563ecbaebc53a590eb554c72e588cc08`;
42,002-byte raw provenance SHA-256 is
`a40043bb3fcdf51cce776af9c6abf226d980c14c261ef3b08a7cd1150c753ae9`.
Declared compiler/native dependencies are delegated to owning pinned inputs;
this is not a hermetic toolchain image.

Runs are `crash-work/runs/qeph-wall-native-raw-{one,two}-{minus,zero,plus}-1/`.
Reports share that prefix; helper/CLI checks use `qeph-wall-raw-runner-*` and
`qeph-wall-raw-cli-*`. Each raw job used one CPU, no GPU, 1 GiB RSS ceiling
and 120-second timeout, with at least 32 GiB RAM available. All guards passed;
guard elapsed times were 0.268-0.274 seconds, too short for sampled RSS to be
an exact peak measurement. The `qeph-wall-native-raw-1` source/runtime checkpoint
preserves actual producer, raw inputs, reports and all observed bytes.

No full-state Schur/Gram verdict, boost-consistency decision, selected contact
step, actual incoming CUDA impact or rendering is claimed by raw completeness.
