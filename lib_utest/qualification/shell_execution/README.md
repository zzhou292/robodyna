# Explicit rigid shell execution binding

`InitializeExecutionCatalog` retains one complete QEPH/T3/QBAT catalog. An
explicit `RigidSkin` material retains the original reference E, density and
Poisson ratio, with canonical unused constitutive controls. It requires a
`Nonconstitutive` section with zero through-thickness points. It has no LAW1 or
LAW44 point parameters. `MaterialPointCount` reports the actual applicable
0/1/3/4 points; curve-pool size and allocated storage are separate quantities.
QBAT remains constitutive with four in-plane points. Older catalog entries
reject rigid skin declarations.

`ShellExecutionBinding::Initialize(catalog, ledger, rigid, limits)` joins these
declarations to the exact prepared physical ledger and PART assembly. It retains
every original catalog row and native family index. For each rigid skin, the
original PID must identify a prepared PART, every shell NID must belong to that
original PART's bare member list, and every mapped domain node must belong to
its final merged root. The root identifies the actual PART group/primary in the
rigid binding. A merged child retains its own PID and original member authority;
equal final roots cannot authorize swapping PIDs.

A constitutive PID absent from the original PART declarations remains
constitutive even when all its nodes belong to a rigid body. Conversely, an
original PART PID cannot be disguised as a LAW1/LAW44 declaration. Source card
authentication remains the app producer's responsibility; this TL value binds
already supplied original identities and native reference bits.

`ShellBatchFailureBinding::InitializeExecution` requires the complete execution
catalog and permits only canonical `None` on rigid skins. It can represent an
all-None execution scope. The old failure initializer retains its old all-None
rejection and rejects execution catalogs. `ShellPhysicalBinding::InitializeExecution`
retains the same execution authority and complete failure catalog. The old
physical/formulation entry remains closed. No second failure sidecar, material
arena, coefficient reduction, inverse value, or owner is created here.

The complete geometry and native shell M/J remain in the ledger exactly once.
This change does not alter shell startup coefficients or PART tensor arithmetic.
It does not yet admit runtime participants, skip force kernels, recover rigid
motion, or declare contact/surface activity. The mapped Q/T follow-on must
authenticate the actual owner rigid binding, skip only explicit rigid rows, and
protect execution rows plus retained rigid/PART/topology payloads in output
alias checks. Existing output-range helpers alone do not know these new rows.

Preflight charges the new immutable object/control reserve, complete retained
catalog and rigid binding payloads, source-order row/family-index arena, and both
startup identity indexes. The rigid binding already retains its ledger and
domain, so they are not added again. The catalog may share inventory with the
shell binding; its complete report is conservatively charged. Physical execution
copies the exact ledger handle retained by execution and charges that backing
once through execution. Its separately prepared failure catalog is charged in
full. These are checked payload reservations, not allocator overhead or RSS.
Default count caps remain small; explicit `Vehicle()` permits 524288 parents and
nodes with a 2 GiB host cap. Budget and semantic failures leave an empty,
retryable handle; invalid forecasts leave the output untouched.

The synthetic host fixture combines coincident QEPH and QBAT layers, NIP3 and
NIP1 triangles, an original rigid primary and a disjoint merged child. Tests
check zero rigid points, ordinary shells on rigid nodes, distinct original
membership, foreign ledgers, late failure/retry, exact cap boundaries, borrowed
input lifetime, canonical failure controls, and legacy admission. It proves no
original-vehicle execution admission by itself. The later app gate must bind all
349645 original rows, including 5102 rigid skins, to the actual 22 PARTs/20 roots.

Standalone host gate:

```sh
cmake -S lib_utest/qualification/shell_execution -B /tmp/tl-shell-execution -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/tl-shell-execution --parallel 1
ctest --test-dir /tmp/tl-shell-execution --output-on-failure
```

The targets are `shell_execution_host` and `shell_execution_legacy`. The latter
reuses existing QBAT catalog/failure and physical-binding tests. Bazel owns the
new host target at `//lib_utest/qualification/shell_execution:shell_execution_host_check`.
No native numerical formula changed; the PART and shell coefficient producers
retain their independently qualified donor/native gates.
