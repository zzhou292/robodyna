# Resident layered-plasticity publication

This opt-in path reuses the existing QEPH/T3 batches, physical nodal owner,
assembly, history caches and joined publication. It stores the separately
qualified three-point section history beside each original ForceTrial slab.
Existing configuration and device Model/Storage records retain their original
layouts; the default LAW1 path has the same allocation count and byte size.

The new Initialize/InitializeJoined overloads accept
`ShellBatchPlasticityConfig`: explicit material/curve IDs and a bounded curve.
Both families derive material coefficients from each original reference and
deep-copy all curve data. This initial interface admits one material per batch.
Joined publication compares immutable material/curve values and IDs, including
the difference between LAW1 and plastic mode; caller pointer identity is not
a material identity. Caller curve/config storage can expire after initialization.

The optional device slab contains shared immutable curve values, prepared point
parameters and two fixed section arrays. Its allocation is forecast together
with the unchanged batch allocation before admission. No allocation or host
section readback occurs during stepping. The current accepted shell pointer
determines the section array index; there is no second accepted index, pending
flag, epoch or clock in the section storage. The existing sole Publish operation
continues to swap only the original shell pointers. This selects both families'
material and section histories together after the owner commits.

Each candidate runs the qualified family adapter, retaining its original
ForceTrial and private proposed section. Parent evaluation stays parallel,
failure selection stays ordered, and the existing serial Measure is unchanged.
A late failure may write private earlier-parent section scratch, but it cannot
publish that scratch. Discard invalidates the original candidate receipt, and
the next evaluation reads the same accepted shell/section pair.

`CopyAcceptedSectionHistory` and `CopyPreparedSectionHistory` authenticate the
existing stamp/diagnostic receipts and stage the complete bounded readback
before writing user output. `ShellBatchSectionState` includes point histories,
the section diagnostic and its accumulated plastic-work diagnostic in joules.
That diagnostic uses accepted interval force thickness and actual candidate area. It
is never added again to the shell's total internal-work ledger.

The three new focused tests reuse the existing mixed actual-owner fixture:
four prescribed load/hold/reverse intervals with both families plastic and a
host-adapter cross-check; late T3 rejection with accepted-cache preservation,
stale readback rejection and exact retry; immutable material mismatch and
allocation-cap rejection with unchanged default LAW1 allocation. Point-law and
section/native equations remain covered by their separate suites. These are
short transaction gates, not a coupled plastic wall-response claim.

The production CMake and Bazel graphs register the owning material, section,
family adapters and shared optional storage separately. The CMake test entry
point remains the native qualification route. Additive Bazel registrations
carry an explicit T3 build-file provenance revision; donor source, numerical
port records, native recipes and test inputs retain their original hashes.
No build/run was performed while preparing these patches; the integrating
guarded test run is required.

# Source rate extension

Optional rate parameters are copied with the curve at initialization. Each of
the three points carries its own filtered total rate in the existing section
slab; the same accepted index publishes shell stress and filter history. The
rate test uses the existing 1/1024 s prescribed fixture with a declared 10 Hz
cutoff to exercise retained history, not the source demo's 10000 Hz cutoff.
The source cutoff and evolving-thickness sequence have separate native point
qualification. Default LAW1 allocation remains unchanged; opt-in allocation
continues to be exactly the reported fixed DeviceStorage size.
