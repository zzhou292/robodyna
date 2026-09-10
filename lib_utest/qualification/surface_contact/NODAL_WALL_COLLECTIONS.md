# Bounded nodal-wall collections

This change admits up to 128 incident physical nodes and 128 native Q4/T3 parents in the existing `NodalWallContactDevice`. It consumes `ShellCollectionLimits.h` and also checks the actual nodal owner's admitted node cap. The host `reference-area-lumped-nodal-wall-v1` law, immutable Q4 A0/4 and T3 A0/3 shares, parent ordering, native mass distinction, finite-wall prepared query and all-destination scatter are unchanged. This is contributor capacity qualification; it grants no structural or vehicle trajectory admission.

The module retains one CUDA allocation and one block of 128 workers on the borrowed owner stream. Each worker initializes/copies one compact node and parent record; no 79 KiB result is passed as a kernel argument or constructed as a whole lane-local temporary. Per-node sorted share arithmetic, parent certificate sums, global node reduction and staged six-component scatter keep their existing arithmetic order. Candidate work still uses accepted-base force and actual kick duration. A failed node or parent prevents scatter; a late destination overflow only changes private scratch. No owner, clock, history participant, stream or runtime allocation is introduced.

Root's compiler-only ABI probe confirmed these 64-bit host layout sizes before the first numerical execution:

| Record | Bytes |
| --- | ---: |
| Immutable Model | 105,976 |
| Control | 28 |
| Point / parent result | 384 / 224 |
| Complete Results | 79,248 |
| Complete device Storage | 471,864 |
| Approved module allocation cap | 524,288 |

The old storage was 99,384 bytes. The explicit increase is 372,480 bytes; the single allocation now reserves the full capacity even for a small selection. Full result readback grows from 3,984 to 79,248 bytes. The fixed prepared wall packet is retained once. At the bound, each active worker scans at most 128 parents and 512 native share slots; reductions/scatter remain bounded serial operations. No latency or sustained-step throughput claim is made from this capacity change. Actual runtime allocation checks are still required by root.

The new fixture prepares an 8 by 16 rectangular grid of 128 nodes using the already qualified QEPH and T3 startup producers. Material is rho = 1024 kg/m3, E = 2e6 Pa, nu = 0.3, t = 1/32 m. Native mass and TOTAL isotropic inertia accumulate Q4 first, then T3; contact shares separately follow sorted parent IDs above 2^54. Q4 reference objects retain their existing one-parent preparation contract. The 94-parent layout has 88 Q4 and 6 T3; the 128-parent layout has 82 Q4 and 46 T3. Both retain every node, including 127. These counts describe synthetic capacity fixtures, not the authenticated Yaris source geometry. Root's app gate separately reuses its actual 117-node/94-parent source fixture and assembled native mass/J.

Five functions are frozen before execution:

1. Host model: full capacity, reversed input invariance, exact allocation ledger, late high-node mass/motion faults, byte-budget and 129-node/parent rejection with unchanged outputs.
2. CUDA: all 94 and 128 parents against every host node/parent/certificate field and an independent long-double area/force/potential/moment/power calculation. All six preseeded force/couple arrays are checked, including node 127; unused T3 slots stay zero. Retry and allocation counts remain fixed.
3. CUDA: two contact-only intervals at h = 1/1024 s, with first h/2 and later h kicks, immutable accepted state until commit, endpoint force excluded from its own kick, host endpoint parity and independent kick/drift/kinetic/impulse ledgers. This uses the existing narrowly labelled contact unit admission, not shell feedback.
4. CUDA: late node-127 borrowed-mass, candidate depth/envelope/NaN and fixed-motion faults; late parent accuracy rejection; all exposed diagnostics/results and six destination arrays remain unchanged as applicable, followed by clean retries.
5. CUDA: a finite high-stiffness arithmetic-only fixture overflows the 128th force destination after earlier active nodes have been staged. All six arrays and accepted state remain unchanged; fresh unseeded retry exactly matches the host law.

Normal fixture law is kappa = 16 N/m3, cap = 0.5 m, unchanged per-parent force budget 5e-7 N and potential budget 1.2500000000000005e-12 J. Independent arithmetic retains 2e-12 times (1 + absolute truth). Work checks retain 256 binary64 eps times absolute terms plus 1e-12 times the 1/64 J scale; impulse uses the same formula and 1/1024 N s scale. No numerical tolerance was raised. The overflow-only tuple retains the previous kappa = 1e308 and per-parent budgets = 1e296; it is not a physical parameter choice. The late accuracy test deliberately uses 1e-30 J and must reject.

Root registration: one CUDA test target, suggested `utest_nodal_wall_collection_cuda`, compiling `NodalWallCollectionFixture.cpp` and `NodalWallCollectionTest.cu`. Reuse `tl_nodal_wall_contact_device`, `tl_q4_parametric_contact`, TL startup headers and the existing GTest/strict floating-point options. No native interval bridge is needed. Existing model, owner and native contact tests remain required regressions; only the old exact storage-size assertion changes to the measured new layout. The source117 app adapter needs no new public fixture API: existing prepared `NodalWallWeights`, global position/mass/mask views and `Initialize` already accept its authenticated records after this bound change.

## Execution result

All five new functions and28 existing owner/contact functions pass, including
94/128-parent host/device agreement, actual128-node two-kick contact motion,
last-node failure preservation and exact retry. The measured one-allocation
storage is471,864 B within the512 KiB cap. No law, numerical budget, input
geometry or applied load was changed after execution. Peer review corrected
a test-only Q4/T3 node-ID width mismatch before the first build; both use
1000+n while contact parent/feature IDs remain wider than2^54.

Reports: `crash-work/reports/contact-collection-{configure,build,tests}-1.*`
and tests-1 XML. Build elapsed24.061 s, peak sampled RSS745,631,744 B. All
CPU/RAM/GPU guards passed. Six owning component Bazel targets also pass in
`shell-contact-collection-bazel-1.*`. The actual original117-node/94-parent
source data separately pass host native structural startup in robo-dyna; these
contact capacity fixtures are synthetic and do not establish part dynamics.
