# Native QEPH full-recurrence audit

Source staged; no numerical execution result yet. The fixed pre-run contract is
`planning/QEPH_FREE_RESPONSE_ADMISSION.md` in the workspace. This native-only
qualification does not start a CUDA owner, allocate GPU memory, change a native
coefficient/inertia, or implement the subsequent long trajectory.

The numerical library separates the 109/194-coordinate dictionary, single
native full-kick map, centered probes, structural identities and nonsymmetric
matrix/finite-horizon Gram analysis. It uses the retained native Q2 and shared
quaternion operation. `RecurrenceAuditTest.cpp` has six focused host functions:
native shared m/J and history dimensions; independent cache kick and uniform
membrane traction; actual derivative/neutral columns and wrong-map controls;
identity/Jordan/direct Gram; unstable/nonnormal examples; failure preservation.
The three report functions exercise retained rejected probes, shape/claim
validation, exact byte/hash binding and the reused create-only artifact writer.

Configure a separate CPU-only cache through the parent qualification project:

```sh
cmake -S Total-Lagrangian-FEA/lib_utest/qualification/qeph -B crash-work/build/qeph-recurrence-audit \
  -DCMAKE_BUILD_TYPE=Release -DTL_QEPH_ENABLE_CUDA=OFF \
  -DTL_QEPH_ENABLE_FREE_RESPONSE_AUDIT=ON \
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local
```

Targets: `qeph_recurrence_audit_check`, `qeph_recurrence_report_check`, and
`qeph_recurrence_audit`. Root executes builds/tests through the workstation
guard. Eigen is compiled with `EIGEN_DONT_PARALLELIZE`; numerical core/test
dependencies are native Q2, Eigen and GTest only. The CLI/report alone reuse
`ArtifactIO.cpp`, OpenSSL and Chrono's RapidJSON headers. `ROBO_DYNA_ROOT` and
`CHRONO_SOURCE_DIR` are explicitly overridable qualification-only source paths.

Invocation (inside a one-CPU bounded guard):

```sh
crash-work/build/qeph-recurrence-audit/free_response/qeph_recurrence_audit \
  crash-work/reports/recurrence-audit-provenance.json crash-work/runs/recurrence-audit-new
```

The parent directory must exist and the result directory must be absent. The
CLI reads a <=1 MiB nonempty provenance JSON inventory and binds its exact bytes
and SHA256. Root must verify that inventory against the actual sources/binaries
before launch. Include this directory, the frozen planning contract, native Q2
source/prepared manifests, actual native/numerical/report libraries, executable,
CMake cache/compile/link inputs, compiler identity, Eigen headers, ArtifactIO,
RapidJSON and licenses. This is declared and source-verified provenance, not a
hermetic toolchain or source-deck authentication claim.

`raw-matrices.json` is created after all attempted probes and **before** any
decision checks. It preserves every finite completed column and failure status;
unevaluated columns are explicitly distinguished by `completed_columns`.
`decision.json` records all matrix diagnostics and binds the raw report's exact
bytes/hash. Each file is capped at32 MiB; both survive a numerical rejection.
ArtifactIO requires external serialization and does not promise atomic exclusive
creation or a multi-file transaction. No existing destination is overwritten.

Exit0 means the stated native finite-horizon audit selected H0 orH0/2. Exit2
means a complete rejected/unresolved report; exit1 means the report did not
complete (an already written raw artifact survives). Every report fixes
`simulation_ready=false` and `trajectory_execution_qualified=false`.
There is no long-run permission inferred from a positive matrix decision.
