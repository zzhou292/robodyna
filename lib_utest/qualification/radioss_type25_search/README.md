# Native TYPE25 search maintenance

This component owns search-reference data, not a contact inventory or physical
state. A positive remaining distance never grants permission to omit geometry,
forces or contact histories. The future native contact coordinator must bind a
complete candidate inventory to the same reference generation and apply interface
activation/sensor rules. No solver clock or timestep selection is added.

`Source` contains host maps copied at initialization, explicit native working units,
resolved margin, fixed/current-main-gap mode and immutable source/topology/activity
identities. It admits a single process, no edge paths and converged updates.
`Current` contains borrowed device views on one explicit stream. Each operation
finishes all borrowed reads/readback before returning, including errors. One owner
requires external serialization. Caller streams must outlive it.

InputUnits::Si reads existing SI X/V/gap arrays directly and converts each consumed
value into native arithmetic. No full converted device arrays are introduced.
DT1 converts through the same time scale. Native constants, margin, snapshots and
returned distances remain in native units. Secondary stiffness supplies only its
finite nonnegative zero/nonzero activity; no force is computed here. Negative
STFN is explicitly UnsupportedLifecycle. The complete native routine first
observes a negative nonzero row and only then clamps STFN for NSPMD=1. A caller
must not pre-clamp input merely to pass this component's admission. Full support
requires a staged observe/normalize phase plus coherent activity/reference and
candidate generations in the common contact transaction. The independent oracle
retains that clamp and uses its own copy; a test proves the pre-clamp contribution.

`StageReference` captures the inactive snapshot slab and returns a private token.
`PublishReference` swaps only a completed token from this instance/current staging.
`DiscardReference` invalidates pending staging. Failed staging invalidates older
pending tokens and preserves the published reference. Tokens are bound to a bounded
per-instance identity, sequence and generation; those are cache identities, not
physical owner IDs. Changed source generations or changed secondary activity masks
return UnsupportedLifecycle, including during a new capture. Empty active sides
also return UnsupportedLifecycle; no successful reuse result is invented.
Successful Report and token outputs are unchanged on failure. `last_failure()`
provides a copied status, current-query stamp when present, and the first flattened
role/gap row when device validation identifies one. Host admission failures have
no row; Publish failures have no query. Discard preserves the last diagnostic;
the next Stage/Evaluate/Publish replaces it. It carries no physical authority.
Typed alignment and checked address spans precede host-map reads or launches.
Outputs cannot overlap Current metadata, borrowed field extents, or private owner
storage, including managed inputs visible from the host.

The native compact/dense XSAV rule is retained. Main1D entries contribute to both
sides, inactive main sentinels are skipped, and inactive secondary fields are not
consumed. Current-main-gap mode uses max(current gap minus saved gap), including
negative changes; fixed-gap mode supplies native MAXDGAP=0. One startup arena owns
both snapshots, copied maps, masks, partial reductions and readback control. Exact
host/device forecasts include this storage; no per-query allocation or floating
atomic reduction is used. CUDA failures poison the instance, never retry on CPU.

The scalar block preserves both ONEP01 factors, raw/stored VMAXDT, raw/final
DIST0, forced-sort and strict2x/5x high-velocity thresholds. High-velocity Error is
an explicit Budget diagnostic, not a physical acceptance result or a swallowed
warning. Global native error handling remains the future coordinator's duty.

The Fortran oracle compiles the complete pinned I25BUCE_CRIT routine (one reference
thread), the verbatim selected INTCRIT block and the exact gap-difference loop.
Test-only boundaries supply source constants/common shapes, suppress I/O and reject
unselected MPI calls. They never call production numerical helpers. Reference
layout packing and SI input preparation are independent. Source extraction is
rechecked before compilation. Equal extrema zeros may have either sign because
native parallel max/min tie order does not define a physical distinction; all
nonzero values and budget decisions are compared exactly in the authored gates.

Owning CMake: configure this directory with GNU Fortran, Release, and
`-DTYPE25_SEARCH_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120`; build and run host/source/
CUDA CTests under the existing resource guards. Header/O0 forecast and source gates
also have Bazel targets. The CPU forecast target contains no GPU execution object or CUDA header dependency.
`TYPE25_SEARCH_CUDA=OFF` does not locate the CUDA toolkit; the public owner uses
CUDA's forward-declared opaque stream pointer without changing its ABI.
Eight host groups and thirteen CUDA groups cover source values, layouts, caps, token
lifecycle, stale/foreign/discard, native/SI and SoA/AoS loads, gap changes, inactive
nonfinite data, empty sides, negative-STFN rejection, malformed map/device ranges,
managed output aliases, copied failure evidence, multiple blocks and injected CUDA
copy failure.

Not yet implemented: full native candidate search/build, inventory binding, topology
or activity replacement, distributed/FI ownership, contact geometry/force/history
integration, physical publication, or a matched full-scene performance claim.
The scene controller must stage reference publication with the common owner commit.
This source has not been compiled or numerically qualified by its author.
