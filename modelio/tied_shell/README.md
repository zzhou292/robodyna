# Original tied-shell declaration

`TiedShellDeclaration::Prepare(canonical, original_member, limits)` creates an
immutable source census for the original `CONTACT_TIED_SHELL_EDGE_TO_SURFACE`
card. It retains the authenticated `CanonicalSource` handle; coordinates,
connectivity, units, file identities and the complete source inventories remain
in that shared backing. It does not create a native contact or mechanics owner.

The factory authenticates the complete supplied original member before reading
selected blocks. It reuses the existing canonical/scope reader, bounded array
codec, source identity readers and fixed-column helpers. Selected PART,
SECTION, MATERIAL, contact, part-set and main-member nodal-rigid-group/set cards
retain exact raw blocks, hashes, line locations and card order. Optional list
header fields stay in those cards without native interpretation. Other original
constraint blocks retain raw evidence. Auxiliary-file constraint blocks retain
their exact canonical file/block identities with `retained_source == SIZE_MAX`;
their native interpretation and memberships remain unresolved. The shared
canonical manifest retains the rest of each include's declaration inventory.
The caller must not infer a complete native IKINE mask from this census.

`Data` exposes original part-set order, sorted original PIDs, canonical shell
parent order, physical slave incidence in shell/beam/solid order, and slave NIDs
sorted by source identity. Those choices are source traversal conventions, not
native NSV order. T3 parents have arity three while their original repeated
fourth slot stays in the canonical backing. Every original slave connectivity
slot is retained, including repeated slots; a beam's orientation node is not a
physical endpoint. Main-member rigid-group membership retains its original card
order and reports intersections with both roles. Original node TC/RC codes are
retained without manufacturing constraint masks. `ordering`, `classification`
and `search` are all explicitly `Unresolved`.

The actual-source test expects 171,813 shell masters (160,896 Q4 and 10,917 T3),
183,457 master nodes, and all 11,165 slave nodes from 4,442 beams plus 908 solids.
Their 16,148 physical incidence slots are retained. The 759 main-member rigid
groups include 409 intersecting the master nodes and none intersecting slaves.
The 11 auxiliary rigid-group block identities remain unresolved. These are
source-census expectations, not successful pairing, CIN/PEN classification,
solid-weld material admission, native coefficients, or completed load paths.
The search/classification follow-on is documented in the workspace's
`planning/YARIS_TIED_SHELL_STARTUP.md`.

`Forecast` checks explicit count/byte caps before inspecting supplied member
bytes or allocating declaration storage. The default 512 MiB startup budget
charges shared canonical bytes once, member bytes, conservative parser/decode
scratch, selected raw/cards, indexes and staged output. It is a simultaneous
payload reservation, not an allocator/RSS measurement; source loading itself
is an already completed `CanonicalSource` operation. The 64 MiB member and
8 MiB selected-metadata limits are separate. `owned_payload_bytes` reports
owned vector/string capacity (including string terminators), excluding shared
canonical storage, allocator/control blocks and RSS. The small-object string
allowance is conservative. Preparation publishes only a complete immutable
value; failed preparation leaves prior handles and source backing unchanged.

The library target is `robo_dyna_tied_shell_declaration`. Configure the owning
host fixture with:

```sh
cmake -S modelio/tied_shell -B /tmp/robo-tied-declaration-root-1 \
  -DChrono_DIR=/home/jsonzhou/Desktop/chrono-work/crash-work/install/chrono-vsg-r0/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=/home/jsonzhou/Desktop/chrono-work/Total-Lagrangian-FEA \
  -DROBO_DYNA_TIED_CANONICAL=/home/jsonzhou/Desktop/chrono-work/crash-work/assets/yaris-vehicle \
  -DROBO_DYNA_TIED_SCOPE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-full-shell-scope-10.json \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/robo-tied-declaration-root-1 --target robo_dyna_tied_declaration_check -j1
ctest --test-dir /tmp/robo-tied-declaration-root-1 --output-on-failure
```

Run that actual-source gate only under the root's scheduled resource guard.
`TiedDeclarationValues.*` uses a tiny synthetic card/array fixture to check the
internal serialization/admission boundary. `TiedDeclarationActual.*` goes
through the public authenticated canonical reader and immutable factory, checks
every original master/physical slave incidence, and exercises budget rejection,
late master-cap rejection, retained backing lifetime and retry. No solver,
CUDA, geometric pairing or native search is run by either group.
