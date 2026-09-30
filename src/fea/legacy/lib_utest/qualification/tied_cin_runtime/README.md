# Ordered CIN stage in the common nodal owner

`NodalCinStartup` is an explicit opt-in to the existing staggered owner. It
retains one immutable attachment/domain handle, complete current M/J, and
source-identified Q4/T3 activity witnesses. Default initialization and layouts
remain unchanged. CIN masters and dependents must be ordinary free nodes and
cannot intersect a rigid group; disjoint groups keep their existing arithmetic.

The two existing accepted/trial slabs gain the same tail:
`M[n], J[n], inverseM[n], inverseJ[n], SMAS[r], SINER[r], DMAST`.
One optional arena carries immutable row indices plus private attempt work.
The existing accepted selector publishes the entire state once. An invalid last
witness, force transfer, recovered motion, or validation receipt discards the
attempt, including all current coefficients and conditional saved history.

Each attempt requires explicit activity for every declared witness, with repeated
source EIDs agreeing. An exact active witness containing the four physical slots
is sufficient for the pinned native no-release branch, including repeated
triangle slots and coincident layers. Missing activity or absence of a positive
witness returns Pending admission. It does not imply release or restore M/J.
The caller must also declare no explicit interface stop/cleaning event.

The ordered force stage receives accepted current positions and force/couple
assembly, snapshots current entry IN, transfers loads and current coefficients,
then the existing owner kicks independent nodes. Native motion recovery uses
that force-stage patch before one dependent drift. The original source M/J is
never reintroduced automatically. Solid-dependent J=0 is preserved; dependent
DOFs have no conventional inverse or ordinary kick even before the first
transfer. Initial SMAS/SINER are literal INIEND seeds. Native numerical DMAST is
separate from a physical source-mass subtotal.

At 359,785 nodes, 11,165 attachments and 11,165 witnesses the optional device
payload is **52,338,608 bytes**, including both 1,461,471-double state tails.
The per-owner device cap remains unchanged. Checked host admission charges
retained attachment/domain/post-KINCHK payloads, complete owner staging, optional
rigid storage, immutable witnesses and the temporary identity index. Actual
vector capacities are checked again before CUDA allocation. The 128 MiB optional
host/device caps measure payload, not allocator overhead, CUDA driver memory or
whole-process RSS. No allocation or extra accepted-state copy occurs per step.

The initial qualification values use explicit synthetic positive independent
M/J and source-shaped zero-J dependents. They do not establish full-vehicle
coefficient producer closure. Future composition must supply the actual ordered
shell, solid and beam stiffness/force contributions and source coefficient
ledger. The restricted-history timestep admission is explicit; this stage does
not infer a native stability limit from zero scratch arrays.

Small host gate:

```sh
cmake -S lib_utest/qualification/tied_cin_runtime -B /tmp/tl-cin-host -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/tl-cin-host --parallel 1 --target tied_cin_runtime_host_test
ctest --test-dir /tmp/tl-cin-host --output-on-failure
```

Root owning native/CUDA gate adds:

```sh
-DTL_TIED_CIN_RUNTIME_NATIVE=ON -DTL_TIED_CIN_RUNTIME_CUDA=ON
-DTL_CIN_NATIVE_CACHE=/home/jsonzhou/Desktop/chrono-work/crash-work/deps/openradioss-tied-interface-1
```

Targets are `tied_cin_runtime_host_test` (11 CIN functions plus 3 unchanged layout
functions), `tied_cin_runtime_native_test` (3 functions), and
`tied_cin_runtime_cuda_test` (4 owner functions plus 1 native comparison when the
native option is enabled). CMake owns the Fortran qualification. Bazel registers
the host and CUDA owner targets and the ordinary production library.

Root `tied-cin-runtime-root-tests-1` passes22 numerical functions (14 host,
3 native,5 CUDA) plus source identity after integration as `5640868`.
Original-size optional device storage is52338608 B. Actual original live
witness/activity source admission remains separate; owning/affected gates
are recorded in the workspace execution status.

Affected Q/T/QBAT owner regression groups PASS in `cin-affected-qbat-root-tests-1`;
owning host/CUDA builds PASS in `cin-wedge-owning-bazel-build-1`.
