# QEPH Q3c prescribed force and material history

Q3c passed all nine host and three actual-CUDA functions on its first numerical
execution, at the budgets frozen below. Each backend's parity function checked
432 prescribed one-cell configurations. Independent source review found no
blocker. Q3b geometry/rates remains an unchanged dependency; the full native Q2
one-cell reference remains the force/history oracle.

The public `QephForce.h` operation returns a complete `ForceTrial`, with
caller-owned proposed `History`, pre-CNDT3 `Kinematics`, four positive world
internal forces/couples and ten native diagnostics. `QephHistory.h` prepares
zero or explicitly supplied finite material values. Exact reference-input
binding, base time and next sample index are required; all failed operations
preserve the entire base/output. This is no owner, clock, native restart,
zero-dt force operation, batched solver or dynamics admission.

The selected donor branch remains centered isotropic LAW1, ITHK0, CVIS1,
DM=DN=.015, ISROT0, IDRIL0, active1. All 12 HOURG values, separate total FOR
and persistent FOR_G, stress-like MOM, STRA, reported THK, EINT[2] and EVIS
are preserved. Reported THKN below native EM30 is explicitly rejected before
the source MAX; the force thickness stays fixed at the reference value.
EINT/EVIS and their increments retain native meanings, not a potential or
general nonnegative dissipation claim. The Q1/Q3b O(h) general-rigid shear
residual remains a separate temporal limitation.

## Numerical budgets frozen before execution

All named native/host/device comparisons use the existing port coefficient
`2e-12*(dimension+abs(reference))`. Set L to the largest current node0-relative
distance, t to reference thickness, E to the supplied Young's modulus and
c to the independent source sound-speed expression. Q3b's 80 fields retain
their own unchanged field budgets. New fields use:

| Field | Dimension floor |
| --- | --- |
| FOR/FOR_G | E |
| MOM | E*t/L |
| HOURG Pa / Pa per metre components | E / E/L |
| STRA membrane/shear / curvature | 1 / 1/L |
| Reported/effective thickness | t |
| Membrane EINT and EVIS, including increments | E*t*L² |
| Bending EINT, including increment | E*t³ |
| World internal force / couple | E*t*L / E*t*L² |
| Sound speed / viscosity | c / 1 |
| CNDT3 STI / STIR / DTEL | E*t / E*t*L² / L/c |

These fixed constitutive scales support dimensional roundoff comparisons;
they are not an accuracy bound for arbitrary small loads. Independent stress,
physical bending moment, work and zero-force tests use the already frozen
native Q2 budgets `2e-11 + 2e-10*max(abs(a),abs(b))`, work absolute `2e-22 J`,
and explicit source-scale rescaling of those same physical expectations.
Covariance uses the separately declared `2e-11` coefficient and the table's
dimensions; no tolerance is adjusted after execution.

Host/device parity each covers 432 configurations: Q3b's 216 geometry/rate
configurations times zero and explicitly seeded full history. Additional tests
cover all eight material strain/curvature modes, physical moment conversion,
load-hold-reversal, all twelve stabilization entries, fixed IDRIL0, static and
rigid characterization, force/moment balance, malformed/stale/foreign inputs,
late thickness failure, complete byte preservation and retry.
CUDA uses one thread and one reusable packet below 8 KiB. CUDA absence is a
failure, and resource/time measurements belong to root's bounded runtime.

## First execution evidence

The retained guard reports under `crash-work/reports` are
`qeph-q3c-{configure,build,host-tests,cuda-tests}-1.json`. The host XML is
`qeph-q3c-host-xml-1/qeph_force_port_check.xml` (9 tests, no failures); the
device XML is `qeph-q3c-cuda-xml-1/qeph_force_port_cuda_check.xml` (3 tests,
no failures). The packet owns 2,744 device bytes and uses one thread. Build
wall time was 6.272 s with 373,948,416 bytes peak sampled host RSS; the host
and CUDA guards completed in .252 s and .489 s respectively (GTest .018 s
and .321 s). These process measurements are not per-element throughput or
peak CUDA-context memory measurements. No tolerance or donor arithmetic
changed after execution. This gate qualifies prescribed one-cell value
operations, including history sequences and failures; resident assembly,
joint nodal/material publication and coupled dynamics remain unqualified.

## Source precision and reuse

`materials/ShellElasticLaw1.h` shares only the actual PM elastic coefficients
and centered SIGEPS01G linear point update. QEPH keeps its rate packing,
reported-thickness update, work composition, damping, stabilization and force
projection. A future T3 port can share that point helper where contracts match.
No classic CHVIS3 normalized stress or five-state history is substituted.

Named MYREAL8 constants preserve source expression order. ONEP414 is
`(1+4/10)+1/100+4/1000`, not sqrt(2); THREEP464 is
`3+4/10+(1/10-4/100)+4/1000`, not sqrt(12). FIVEP333 is 16/3. The selected raw `4.`
is exactly representable; the noninteger raw `1.2` is confined to unselected
LAW58. Q3b's four promoted binary32 constants stay unchanged. Source originals
and qualified native arithmetic are never edited to match the port.
