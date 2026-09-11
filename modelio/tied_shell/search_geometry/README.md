# Original tied search geometry

`TiedShellSearchGeometry` prepares immutable geometry and thickness inputs from
an authenticated `TiedShellPacking` and the original source member. It retains
the packing handle and its canonical backing. It does not create a mechanics
owner, classify constraints, or publish final search associations.

The named first profile requires zero secondary-node incidence across **all
original shells**, including shells outside the selected master parts. Both
the per-secondary shell thickness and the interface maximum are consequently
zero. Declared IRECT patch order, physical NIDs and true triangle topology stay
unchanged. The physical-own-node helper compares those identities.

Required NODE cards are read at their original line associations. Source
working doubles, codes and blank masks must reproduce the canonical SI arrays
bit for bit under the declared `.001` length conversion. Retained positions
and thicknesses are in original millimetres; they are not SI values divided by
`.001`. The raw member is borrowed during preparation and is not retained.

Native INCOQ3 matches physical-node membership. The ordinary source profile
ranks GEO thickness and then the authenticated material modulus. It retains
every equal winner only when all winners have the same family and bit-identical
consumed thickness. Part/element overrides retain native nonzero precedence.
Canonical match rows identify source evidence; they are not native incidence
order or an invented winning EID. In particular, a matching glass shell can
provide thickness without replacing the declared midlayer master patch.

Plain PART/ELEMENT_SHELL and ordinary SECTION_SHELL coefficients are explicit
source conditions. Unsupported overrides, ambiguous material coefficient
paths, mixed-family matches and unequal consumed ties reject the complete
preparation. Constitutive admission of rigid or midlayer shells is independent
of this geometry-only coefficient path.

## Storage and qualification

`Forecast` validates counts and the complete startup payload budget before the
factory reads borrowed member bytes. The default cap is 512 MiB. It charges
shared canonical/declaration/packing backing once, borrowed member bytes,
metadata/decode/index scratch, selected cards and final arrays. The retained
`owned_payload_bytes` counts this module's owned arrays, strings and descriptor
object; allocator bookkeeping, shared control blocks and process RSS are not
payload. No per-master geometry allocation is retained. Actual-source tests
print the forecast before preparation and the final retained payload afterward.

Six small host functions pass, including signed zero, a decimal with a lossy
SI round trip, complete unselected shell incidence, late source-card mutation,
exact budget rejection and retry. Native/actual test translation units pass
C++ syntax checking. Native compilation and full-source execution are owning
root gates, not author results. The independent native packet and its source
identity are documented in [native/README.md](native/README.md).

The actual-source gate expects 171,813 master patches, 11,165 secondary nodes,
194,622 unique working nodes and 180,315 matching-shell occurrences. It checks
all 4,251 original three-layer witnesses and calls complete INCOQ3 for all six
permutations of each witness, plus every unique match: 193,068 native calls.
It must establish these results before claiming actual geometry readiness.
The expected two equal glass winners are PIDs 2000023/2000523, with thickness
2.28 mm and modulus 70,000 MPa, alongside declared midlayer PID 2000524.

## Owning build

Run through the root's serialized resource guard. From the app checkout:

```sh
cmake -S modelio/tied_shell/search_geometry -B /tmp/robo-tied-search-geometry-owning-1 \
  -DChrono_DIR=/home/jsonzhou/Desktop/chrono-work/crash-work/install/chrono-vsg-r0/lib/cmake/Chrono \
  -DCMAKE_BUILD_TYPE=Release \
  -DROBO_DYNA_TL_ROOT=/home/jsonzhou/Desktop/chrono-work/Total-Lagrangian-FEA \
  -DTL_SOURCE_ROOT=/home/jsonzhou/Desktop/chrono-work/Total-Lagrangian-FEA \
  -DROBO_DYNA_TIED_SEARCH_GEOMETRY_NATIVE=ON \
  -DROBO_DYNA_TIED_CANONICAL=/home/jsonzhou/Desktop/chrono-work/crash-work/assets/yaris-vehicle \
  -DROBO_DYNA_TIED_SCOPE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-full-shell-scope-10.json
cmake --build /tmp/robo-tied-search-geometry-owning-1 \
  --target robo_dyna_tied_search_geometry_check -j1
ctest --test-dir /tmp/robo-tied-search-geometry-owning-1 --output-on-failure
```

The three CTest groups are `tied_search_geometry_values`,
`tied_search_geometry_native` and `tied_search_geometry_actual`. The existing
create-only source fixture utility authenticates the original ZIP/member and
sets the explicit source paths for the actual group. No GPU target is added.
Subsequent broadphase/projection assessment and constraint classification are
separate consumers of this handle.
