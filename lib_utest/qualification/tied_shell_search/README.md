# Native tied-shell candidate projection

This gate qualifies the selected TYPE2/Spotflag28 Ignore2/DSEARCH0 geometric
candidate and ordered selection. It reuses TL fixed-vector operations; force,
motion, coefficient transfer, broadphase and source packing remain separate.

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
