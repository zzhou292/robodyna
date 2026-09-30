# Native TAB1 point and glass any-point NIP3 values

This increment qualifies a separate TAB1 point failure state and a centered
NLay1/NIP3 glass caller. It adds no resident material admission, source model
admission, contact deletion, nodal mass removal, archive or mechanics clock.
Noncentered windshields and QBAT membrane integration remain separate work.

The point table is explicitly constant-valued with three finite, increasing
triaxiality abscissae. It preserves native weighted arithmetic and segment
selection at/beyond table boundaries. It does not silently return the repeated
ordinate, clamp triaxiality, or introduce arbitrary table dimensions/functions.
The original glass profile is (-.3,.015),(0,.015),(.3,.015). Its damage is the
uncapped native UVAR1; native DFMAX is a separate bounded display channel.
A newly failed point records the supplied actual endpoint time. An already
failed point, or a point on an inactive parent, retains its failure history.
Finite-output admission rejects overflowing retained damage atomically instead
of changing the native history by capping it.

The NIP3 adapter admits analytic LAW44 with the explicit FilteredZeroC policy.
It uses the existing point observer loop, distinct saved masked and current
unmasked stresses, and a separately named native AnyPoint parent policy.
Source NUMINT1 must later be retained by the source adapter with the explicit
correction of the pinned converter's hardcoded IFAIL_SH2. The native policy
used here is IFAIL_SH1/PTHKF=EM06 with the existing positive property threshold.
No claim is made that the original converter already preserves NUMINT1.

The shared work helper is extracted from the previous Johnson wrapper without
changing operation order. Both typed wrappers validate their own history, then
use native raw current resultants plus viscosity before final parent masking.
Removal keeps old-force work; post-OFF0 constitutive filtering/auxiliary fields
continue under the existing LAW44 gates. Existing nonfailure and Johnson
resultants contracts are retained.

## Native evidence and independence

Pin: `a62b27e6baa555d222a580d6218867d0be4d70b5`.
`native/source-manifest.json` authenticates 141011 bytes of complete TAB1,
TABLE module/tools, starter/CFG and license donors by Git blob and SHA256.
The prepared library compiles complete FAIL_TAB_C, complete TABLE_INTERP and
TABLE_VINTERP routines, and exact starter default/packing extracts. Only private
symbol names are changed. Native unsupported extra functions/diagnostics stop
explicitly; no production value helper appears in the native oracle.

The table scalar specialization preserves native left-segment selection at
interior knots and first/last extrapolation. Extremely remote finite queries
can produce a rounded zero ordinate even from a positive constant table;
the point leaf retains native `EPSF>0` gating. Nonfinite table results reject.
Native UVAR initial geometry fields are finite harness values, unused by the
selected no-thinning/no-size branch. This is not a full native restart packet.
No unassigned native TDEL/DMG_SCALE output is read.

The test-only LF_CALLER has one internal loop and explicit failure/parent
callbacks. Its existing C wrapper/three-field Johnson layout/default sound
speed remain unchanged. A separate explicit-SSP wrapper supports complete
family drivers; TAB1's standalone section gate uses the original default and
compares work with the actual returned viscosity. Full family force admission
is not inferred from this standalone gate.

A hashed frozen910e346 authored caller/parent/interface harness is compiled only
for regression: all point histories, failure values, material/current forces,
work, phase and trace field bits are compared with the refactored default
wrapper. This baseline is labelled authored wrapper history, not a new native
physics donor. Existing native driver extracts and reference pins are unchanged.

## Gates

Six host functions cover table interpolation/caches, tiny increments, retained
overshoot/DFMAX, invalid parameters/history/input, eight selected point masks,
removal/post-inactive histories and failed output/retry. The work parity test
compares every work field bit against the frozen pre-extraction implementation
under nonzero mixed increments and both finite/overflowing viscosity inputs.
Eight prior Johnson host functions remain part of this owning configuration.

Four native functions cover exact defaults and weighted lookup, point history
at both sides of the native EM20 tiny-stress denominator boundary,
all eight any-point removal selections with full work/diagnostics, and frozen
Johnson callback parity. Two CUDA functions prepare/update their own point and
section histories, compare the independent native trajectories, and verify
late failure preservation and clean retry. They do not create a second runtime
owner or claim connected vehicle simulation.

Author evidence: 6 new/shared + 8 legacy host functions passed under 1 CPU /
512MiB. Four native C++ and two CUDA-shaped host syntax units passed, along with
source verification and host-only CMake configuration. Reports are
`crash-work/reports/shell-tab1-glass-author-{1,2,3}`. The final six-function
host rerun and syntax/source checks passed in 3.896s with 200604KiB maximum
child RSS. One fixture SCOPED_TRACE pair
was split across lines after host syntax found duplicate generated names.
Fortran/native and actual CUDA runtime gates are pending the root's scheduler.

Root owning commands (run through the workstation guard):

```sh
cmake -S lib_utest/qualification/shell_tab1_glass -B BUILD_DIR \
  -DCMAKE_BUILD_TYPE=Release -DTL_TAB1_GLASS_NATIVE=ON \
  -DTL_TAB1_GLASS_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel 2 --target \
  shell_tab1_glass_values_check shell_tab1_glass_native_check \
  shell_tab1_glass_cuda_check shell_layered_failure_values_check \
  shell_layered_failure_native_check
ctest --test-dir BUILD_DIR --output-on-failure -j1 \
  -R '^(shell_tab1_glass_(values|native|cuda)|shell_layered_failure_(values|native))$'
```

Owning Bazel host target:
`//lib_utest/qualification/shell_tab1_glass:shell_tab1_glass_values_check`.

Root qualification at TL `fe6e1b8`: all **24 functions pass**, comprising six
new host, four new native, two actual CUDA, eight legacy host and four legacy
native functions. All **13 failure-force functions** also pass after rebuilding
the common work helper, including both CUDA force trajectories. Reports are
`shell-tab1-glass-root-{configure,build,tests}-1`,
`shell-tab1-glass-force-regression-{build,tests}-1`, and matching per-function
XML directories. No skips or tolerance changes were used. The owning Bazel
glass and failure-force targets build after granting the glass qualification
package access to the existing test-only failure fixture.

This qualifies the point and centered section composition. Complete glass
family forces, noncentered placement, resident/source admission and contact
activity still require their own integration.
