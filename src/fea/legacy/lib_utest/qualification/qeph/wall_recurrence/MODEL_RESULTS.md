# CW1 native contact model and probe results

Eight new host functions and ten retained support functions passed on first
execution. The new numerical modules reuse the native full-state map and
owning contact law. The support implementation now has one reusable static
library; its source and the ten existing tests are unchanged.

The model keeps all 109/194 coordinates, native reference geometry at the
initial gap, a distinct touching probe baseline, and actual native mass/J.
The penalty uses the frozen upward interval chain, with finite-wall coverage
and every native mass/contact stiffness coefficient checked independently.
All six step coefficients pass the independent dictionary/scatter oracle.

At H0, both fixture sizes and all three physical velocity baselines pass
complete native differentiation and both strict contact sign cones. The
three physical amplitudes yield two one-sided derivative checks per direction.
The tests verify the actual contact kick before shell cache consumption,
shared-node accounting, force/potential/moment/power certificates, omission
controls, late native rejection, exact output preservation and clean retry.
The test bodies and budgets are frozen in [MODEL_CONTRACT.md](MODEL_CONTRACT.md).

The guarded CMake build took 10.082 s, with a largest sampled process-group
RSS of 403,861,504 bytes. The model and support GTest bodies took 0.105 s and
0.005 s respectively. All guards passed with one test CPU, a 1 GiB RSS limit
and no GPU requirement. Sampling of these brief tests is not an exact memory
peak measurement. Reports are `crash-work/reports/qeph-wall-model-*-1.json`
and `qeph-wall-model-xml-1/` in the parent workspace.

The [pre-execution source map](model-source-map.json) records 100 inputs,
SHA-256 `c92791767112e81c9fd5551927d571f1280cb671cca13c42fcb27e009cc500a1`.
The `qeph-wall-model-1` checkpoint retains source/build inputs, test binaries
and execution evidence, with the native/toolchain scope explicitly delegated.

This is the model/directional-probe slice. Full-grid amplitude/boost comparison,
spectral and switching-product screens, authenticated decision reports, actual
incoming-velocity startup, nonlinear CUDA impact and h/h2/h4 refinement remain
open. The new tests do not admit an impact timestep or mixed-shell dynamics.
