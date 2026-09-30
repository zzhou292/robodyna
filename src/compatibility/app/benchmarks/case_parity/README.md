# Resolved-case parity gate

This offline module compares **declared resolved physics and actual bound evidence**.
It creates no owner, solver, GPU context or scientific authority. It is deliberately
stricter than the existing six-part timestep pilot; that pilot's semantics are unchanged.

API: read_contract(pin,parent,artifacts), compare_contracts(reference,candidate),
and assess(request_pin,parent). The CLI is:

    python3 -B -m benchmarks.case_parity REQUEST.json --sha256 REQUEST_SHA --report NEW_REPORT.json

Exit0 means a report was written, not that a benchmark passed. Read status:
incomparable, numerics_unqualified, comparable_without_speed_win, measured_gpu_win.
Every v1 verdict is declared_case_only. full_vehicle_requirement_met is always false,
even for a contract labelled full_vehicle. Project delivery needs a separately reviewed
P5 workload/acceptance protocol; a short case cannot establish it.

## Trust and scope

This is a consistency gate, not proof that arbitrary caller-declared hashes or
scientific claims are true. A pinned comparator must be independently reviewed
and qualified. The gate verifies that a specific completed named test bound these
contracts, producer programs, tolerances and exact timing-run manifest; a supplied
passed=true flag is never enough. It cannot make a dishonest producer trustworthy
or replace numerical validation of the equations. Backend truth, GPU UUID/driver,
producer dependency pins, effective OMP/worker settings and hardware identity remain
trusted-producer/qualified-adapter responsibilities. Comparing opaque platform IDs
and observed guard affinity is not independent hardware verification.

The real fixture current_unmatched contains copied investigation evidence for
704.352kg V5 versus1256.152kg complete native Yaris. Its unresolved ledgers/catalogs
stay unknown. It must remain incomparable regardless of a claimed fast timing.
No current production CPU/GPU win is claimed by this implementation.

Version1 supports identical complete recorded time grids and shared dense binary64
field output. Adaptive schemes may be described, but different actual histories or
unqualified equivalence policies cannot be admitted as matched timing. Native raw
animation formats need an explicitly qualified adapter before using this protocol.

## Required contract domains

Topology/geometry; population; formulations/materials/units; mass/inertia;
constraints; initial state/loading/wall; contact/history; timestep/mass control;
recorded time grid; output work/precision.

Every domain has status resolved, absent or unknown, a nonempty definition, and
evidence pins. Resolved and absent need evidence; unknown never equals a known
absence—even when both inputs say unknown. Definitions compare completely,
including unknown future physical keys, signed zero and bool/integer distinctions.
Different file paths are provenance rather than physical differences when the
resolved values are identical.

Resolved recorded_time_grid.definition has kind physical_steps or
prescribed_updates, planned_steps, initial_time_s and requested_end_time_s.
Selected/full vehicles require physical_steps. Their recorded grid has exactly
planned_steps+1 finite, strictly increasing times and exact declared endpoints.
Prescribed normal-response packets may use repeated input times but are not a
physical trajectory. The selected scope and boundary stay in the report.

Resolved output_work_precision.definition has binary64 precision,
case_fields_f64_v1 format, named fields with scalar component counts, and ordered
sample_epochs spanning0..planned_steps. Each run's output manifest must reproduce
this definition and pin exactly one dense payload per field. Actual byte counts
must equal samples × components ×8. Scientific values are checked by the separate
numerical comparator, not by pretending equal output work implies equal response.
The output manifest also binds run_id, producer_sha256 and contract_sha256.
Its manifest and each payload must have distinct invocation-owned resolved paths
and file identities across repeats; shared case inputs and byte-identical newly
written payloads remain allowed. Fresh create-only writes during the timed invocation
are a qualified producer-adapter responsibility; offline hashes cannot prove when a
file was written.

## Numerical and measurement evidence

A contract may leave numerical_protocol null; then it can never produce a win.
A protocol supplies its version ID, exact Suite.Case test name and tolerance-file
pin. The numerical evidence pins reference/candidate producer programs, comparator,
GoogleTest XML, passing guard and a timing manifest listing the exact measured runs.

The XML must contain exactly one executed, completed named test, zero failures/
errors/skips, consistent counters, and properties binding both contract SHA256s,
both producer SHA256s, numerical_protocol, tolerances_sha256 and
timing_manifest_sha256. The guard must actually invoke that comparator/test/XML.
The promoted output.gtest_evidence helper also serves the unchanged
viewer.postprocess.require_exact_replay wrapper, whose Chrono test name stays fixed.

Each guarded measurement wraps a producer_record, its actual guard receipt and
that same invocation's bound launcher_interval.
The producer record binds backend, run ID, producer/case hashes, requested/completed
steps, endpoints, parsed time/output manifests, warm timing and platform/boot ID.
Safe invocation admits only the actual executable or env assignment/unset prefix.
It requires --run-id and --benchmark-record binding, rejects forecast-only, and
does not accept a producer filename passed as an argument to another executable.
A native Engine comparison therefore needs a small qualified record-producing
benchmark adapter; this gate does not parse arbitrary shell pipelines.

The warm record binds run_id, producer_sha256 and contract_sha256, and supplies first_step, step_count, warmup_steps, total_seconds,
timer_resolution_seconds and boundary. Its mean is derived, not accepted as an
unbound scalar. Vehicle/synthetic complete-step boundary is accepted_complete_step;
normal packets use complete_normal_response_update. The window must follow warm-up,
fit completed work and span at least1000 declared timer ticks. Paired runs must use
the same window and complete boundary.

At least three alternating reference/candidate pairs are required. A pinned launcher interval records start/end in one explicit CLOCK_MONOTONIC_NS
clock around Popen/wait, and binds both guard and completed producer-record hashes.
These actual intervals enforce nonoverlap/order and supply complete elapsed time.
Child start ticks are never added to a duration that starts before preflight. All repeats must
retain the same declared work, actual grid, output definition, hardware/boot and
host resources. Guard identities/run IDs cannot be reused as extra samples.
Warm records, producer records, output manifests and payloads cannot share paths or
underlying files between invocations (including hard-link aliases).
Both warm advancement and complete guarded elapsed must have nonoverlapping
observed ranges: max(candidate)<min(reference). The report includes paired medians
and conservative observed variation bounds, not a population confidence interval.
This is a minimum consistency/performance gate, not a substitute for full-workload
profiling, long-tail evidence or the project's2× engineering target.

## Bounds, mutation and ownership

Metadata is capped at1MiB; XML at2MiB; at most512 unique files and8GiB cumulative
pinned bytes are admitted per invocation. Large payload hashing streams in1MiB
chunks. Evidence is rehashed before report publication. Malformed/duplicate JSON
keys, NaN/Inf including exponent overflow, stale hashes, symlinks, missing fields,
unsupported schemas, invalid windows and incomplete work reject. Reports are
create-only, written only after the complete comparison succeeds.

No production numerical libraries are linked. The only shared changes are an
additive bounded argument on the existing strict JSON reader and promotion of its
existing strict named-test XML logic to the neutral output package.

## Owning checks and the first matched case

Configure this directory with CMake (LANGUAGES NONE), then CTest case_parity_host.
It runs contract/measurement/real CLI tests, existing exact Chrono evidence
regressions and the old accepted-payload comparator regressions. Tests with
synthetic XML/timing records are clearly parser/consistency tests; they are not
real performance results.

P1's first future matched contract is normal_response_packet, not a shell scene:
actual native mm,s,metric-ton units and SI UnitScale{.001,1000,1}; I25FOR3 normal
branch with resolved Engine KDTINT/IDTMINS/IDTMINS_INT all0, VISC.05/IVIS2=1,
given post-offset penetration/sideK/weights/masses/normal velocity/history/DT1.
Its native .002mm penetration,400N/mm stiffness,−20mm/s velocity, DT1=1e-5s,
secondary .002tonne, main masses .004/.002/.008/.001tonne and quarter weights
are a concrete synthetic input, not Yaris pair geometry. Initial packet time0
and repeated prescribed-update times must not be relabelled physical steps.
Geometry/search/friction/full source stiffness remain outside P1 and unknown.

After P1 numerical qualification, a record-producing paired driver and a named
comparison test can populate this protocol. The first matched shell/contact scene
must then use one resolved geometry/material/contact/time definition and validate
history/force/stability on CPU and CUDA. An exported selected-vehicle case follows
only after all original/custom profile differences are resolved explicitly;
the existing full-native/V5 mismatch is not promoted by filling labels.
