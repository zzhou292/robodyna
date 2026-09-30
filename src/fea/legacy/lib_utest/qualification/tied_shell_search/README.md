# Native tied-shell candidate projection

This gate qualifies the selected TYPE2/Spotflag28 Ignore2/DSEARCH0 geometric
candidate and ordered selection. It reuses TL fixed-vector operations; force,
motion and coefficient transfer remain separate. Native candidate bounds and
reuse of the existing GPU broadphase are qualified below; source packing has
its own application-owned gate.

The public input contains SI coordinates/thickness and explicit original working
length (.001 m for the Yaris millimetre deck). I2BAR3 area and I7LIN3 edge floors
are evaluated in those working units. The candidate exposes SI distances and a
separate native-unit selection distance; rounding the latter into SI must not
invent a search tie. All candidates in a choice use the same declared scale.

The native final search gap uses **ZEP05=.05**, the smaller master diagonal,
and .6 times the two shell thicknesses. A secondary beam/solid node's shell
thickness is zero when it has no shell incidence. It is not its beam diameter.
Q4 uses centroid fans; a T3 repeats its third physical vertex, preserves native
unclamped barycentric/tip rules, and uses fan zero. Strict positive penetration
and strict -1.5<s,t<1.5 admission remain distinct from the +/-1.02 warning flag.
That flag is a geometric diagnostic, not an emitted native startup warning.

`ConsiderCandidate` consumes caller-defined native master order. It selects
smaller native distance, then smaller max(abs(s),abs(t)); exact ties preserve
the first. It does not sort EIDs, assign NSV/MSR/IRECT order, classify CIN/PEN,
remove unmatched nodes, or install a force attachment. A source adapter must
qualify the converted ordering independently.

Finite input is insufficient when intermediate products overflow. Such candidates
are rejected before native min/max floors can hide the overflow. Original SI
triangle repetition is checked before unit conversion can merge two distinct
representable coordinates. All failures preserve prior outputs/choices; retry
does not allocate. Native zero-area regularization is retained, so successful
search does not establish structural patch rank: `PreparePatch` must still
reject singular force/coefficient geometry before later attachment admission.

## Independent qualification

Complete pinned I2DST3 (including I2BAR3) and I7LIN3 compile with the existing
authenticated native constant module. Exact I2COR3 lines183–198 compute gap
from independent source positions and thickness. The wrapper calls native
selection with independently carried distance/ST/master values. No production
projection, barycentric value or selection result seeds the oracle. Complete
donor byte/Git-blob hashes, exact fragment and AGPL attribution are retained.

Root qualification passes **8 functions**:4 host,3 native,1 actual CUDA,
plus source identity. Native coverage includes5,415 flat/warped/triangle grid
packets, edge/corner/tip branches, strict boundaries, reversed exact ties,
more-central candidate selection, and working-unit floor sensitivity. CUDA
checks native parity and rejected NaN/finite-overflow/one-ULP topology attempts
with exact retry. Independent review found the overflow and pre-conversion
topology gaps; both are fixed and covered.

Accepted reports: `tied-search-root-tests-3` / functions3; build4. Build1's
duplicate GTest trace macro line and tests1's mistaken .5 diagonal factor remain
failed evidence. The compiled native constant exposed that factor-ten error;
no oracle arithmetic or tolerance was changed. A fast test's sampled RSS does
not measure its peak allocation.

```sh
cmake -S lib_utest/qualification/tied_shell_search -B BUILD \
  -DCMAKE_Fortran_COMPILER=PINNED_GFORTRAN -DTL_TIED_SEARCH_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD --parallel APPROVED_WORKERS
ctest --test-dir BUILD --output-on-failure --no-tests=error
```

Run through the existing workstation guard/lock. Owning Bazel host target:
`//lib_utest/qualification/tied_shell_search:tied_search_values_check`.

## Existing GPU broadphase reuse

Two additional CUDA functions pass in `tied-search-broadphase-tests-1` /
functions1. The existing `HydroelasticBroadphase` uses four-slot masters and
four-repeated-slot secondary points in separate mesh groups. Per-master native
gap inflation is rounded outward. All three SAP axes retain every independently
native-accepted pair and the same ordered choice on1,805 secondary points,
including flat/warped quads, true triangles and duplicate-master exact ties.
The caller restores explicit master rank before consuming SAP candidates.
One-pair-short capacity fails with empty results; exact-capacity retry agrees.

The collision implementation is unchanged. Its CMake declaration and CPU
utilities now have reusable owning modules; the existing contact harness uses
aliases to those targets. Both existing utility and GPU broadphase test suites
pass (`tied-search-broadphase-contact-tests-1`). The first new-target build ran
before CMake reconfiguration and had no target; configure1/build2 pass.

This establishes a small-fixture reuse path, not full-deck native bucket
traversal, degeneracy admission, CIN/PEN classification or an attachment owner.
Actual source ordering, conservative geometry-domain admission and original
full mapping comparison remain necessary before production attachment startup.

## Native candidate bounds and exact filtering

`TiedSearchBounds.h` preserves two distinct native decisions. I2BUC1 inflates
each master by max(.05 times its **larger** diagonal, .6 times its thickness
plus the interface-wide maximum secondary shell thickness). I2COR3's final
projection uses the **smaller** diagonal and the individual secondary shell
thickness. Both calculations use explicit original working coordinates.
`WithinSearchBounds` applies I2TRIVOX's inclusive box filter after conservative
candidate generation. The caller must separately exclude a secondary whose
physical node ID is one of that master's four slots.

This exact filter is required for native geometric admission: the native
zero-area floor can admit a distant point in raw projection even though the
startup box excludes it. A successful search still does not establish force
patch rank; attachment installation must also pass `PreparePatch`.

Root `tied-bounds-root-tests-1` passes **15 numeric functions** (4 host,
7 native, 2 CUDA projection/bounds and 2 CUDA broadphase) plus source identity.
The SAP fixture now includes unequal diagonals and a zero-area master, builds
bounds in original units, filters candidates using the exact native box and
restores IRECT rank before selection. All three axes agree with independently
native-filtered exhaustive choices; exact-capacity retry still passes. Separate
tests cover global secondary thickness, inclusive faces, one-ULP outside points,
overflow and rejected-output preservation. Six complete donors and six exact
fragments pass byte/hash verification. Independent read-only review found no
blocker. This remains a bounded fixture, not the full original mapping.

Build3 and tests1 are accepted. Build1 retained a fragment extraction-offset
error (I2TRIVOX line240 instead of241); correcting the copied range to241–244
from the same pinned donor fixed the wrapper. No donor arithmetic or comparison
tolerance changed. The native-only intermediate gate passed seven functions
and source identity before the complete CUDA gate.

## Original source coordinates without a unit round trip

The additive `WorkingSearchInput` and `WorkingSearchBoundsInput` overloads take
genuine original coordinates/thicknesses and explicit `working_length_to_m`.
They share the existing projection/bounds arithmetic. `CandidateProjection`
continues to expose SI distances and a separate original-unit selection score;
`WithinWorkingSearchBounds` consumes an original-unit point directly. Existing
SI convenience overloads retain their conversion contract. Source adapters
must authenticate the source-to-SI relation rather than reconstruct original
doubles from rounded SI coordinates.

Root `tied-working-root-tests-2` passes **20 numeric functions** (4 host,
11 native, 3 CUDA, 2 SAP) and source identity. The native packet now accepts
original doubles directly. An exact-bound negative control proves that a
source-to-SI round trip changes an inclusive-box decision; native projection,
selection, rejected values and device retry are also covered. Independent
review found a narrow legacy SI admission change for positive thickness
underflow during unit conversion. The shared arithmetic now preserves that
diagonal-based native domain; the new public working overloads still require
positive original thickness. The native regression covers both contracts.
Review of the corrected delta found no blocker. Build2/tests2 are accepted.
