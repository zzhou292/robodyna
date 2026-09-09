# Nodal-wall owner contributor execution

2026-09-09: all four host and six actual CUDA functions pass on their first
numerical execution. The 85 existing host/CUDA/source-part regressions also
pass: 95 function executions across the guarded CMake runs. The tests retain
the declared law, budgets and fixtures in `NODAL_WALL_OWNER.md`.

The contributor binds the actual finite triangle wall and the existing nodal
owner. The gate covers additive assembly, eight contact-only spring steps,
half/full kicks, base-force work/impulse, endpoint potential, activation,
late failures, changed borrowed stream, output preservation and clean retry.
No shell/contact recurrence or physical impact case is admitted.

The measured complete allocation is 99,384 bytes, one allocation and one
128-thread block. The host/device layout assertions agree. The sm_120
candidate/assembly kernels each use 255 registers, with 424/432 stack bytes
respectively, and no static shared memory. These are bounded unit-fixture
resources, not a vehicle throughput measurement. The guarded CMake build
peaked at 764,379,136 sampled RSS bytes; CUDA tests at 169,816,064 bytes.

Two authored compile failures are retained: an eight-byte overestimate of
the model/storage sizes, then a missing namespace closing brace. Corrections
change declarations and explicit bounded integer casts only. A separate
direct Bazel CUDA build exposed a missing dependency on the existing
`nodal_force_assembly` target; the host wrapper build alone had not compiled
that CUDA archive. The corrected direct CUDA archive and host wrapper now
build successfully, and both owning host tests pass. Failed reports remain
distinct from passing numerical runs.

Evidence: workspace `crash-work/reports/nodal-wall-owner-*.json`,
`nodal-wall-owner-{host,cuda,source}-xml-1/` and
`nodal-wall-owner-cuda-resources-1.txt`. Failed source checkpoints are
`nodal-wall-owner-first-compile-1`, `nodal-wall-owner-second-compile-1` and
`nodal-wall-owner-bazel-compile-1`. Source-part tests continue to use their
declared provisional test masses; they do not establish structural mass.

QEPH response admission, a separately qualified combined contact/history
recurrence and precommit case composition remain the next integration work.
This contributor adds no material history, accepted clock or postcommit CUDA
operation. TL-FEA remains the mechanics owner.
