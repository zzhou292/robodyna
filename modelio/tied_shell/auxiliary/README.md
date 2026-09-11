# Auxiliary source evidence for TYPE2 classification

`TiedAuxiliaryConstraints` retains the missing original auxiliary rigid-group
evidence alongside an immutable `TiedShellDeclaration`. It authenticates the
complete `set-yaris-coarse-v1l.key` bytes against the retained canonical manifest,
then checks each consumed raw block hash, line extent and keyword. The same
canonical handle retains source archive identity, units, source arrays and
explicit shell/tire selection. It never reconstructs that authority from a
membership census.

The producer reads every auxiliary nodal-rigid declaration, resolves its plain
node list and retains exact source-order member NIDs. Existing fixed-column
`CardId`, `ListIds`, `SourceScalar` and `ReadRequestedSources` helpers own the
parsing. Native group options remain source values: the first profile admits
blank or explicit zero optional fields and plain lists with blank optional
headers. Blanks and explicit zero are retained separately, including the last
original group's explicit `CID=0`. Unsupported nondefault options/set operators
fail rather than acquire guessed native defaults.

Every member has a checked canonical index or an authenticated auxiliary NODE
source row. Required auxiliary NIDs, TC/RC values and blank masks are retained;
their exact raw NODE block also preserves original coordinates. Coordinates are
not converted into new mechanics nodes here. Member nodes are listed by NID;
group lists retain original card order. Master/slave intersections are separate
and never replace the complete membership. All auxiliary node-list and NODE
blocks used to index this source namespace are retained, including source rows
which do not occur in a selected rigid group.

The expected original-source gate is 11 groups, 228 member occurrences, nine
groups touching 125 distinct masters, no declared slaves and 88 noncanonical
group-member nodes. The existing declaration still owns all main-file groups,
including the separately measured 409 groups touching masters. These are source
topology counts, not native registration events or a CIN/PEN decision.

## Explicit boundary scope

The required `OriginalWallPolicy` argument is either `RetainUnresolved` or
`ReplaceWithMeshWall`. Both retain the complete original wall block identities
and their source-file hashes from the authenticated canonical manifest.
Replacement explicitly excludes that entire inventoried population; it does
not evaluate original wall node-selection rules or instantiate a mesh wall.
The whole auxiliary member is authenticated; other wall members are represented
by authenticated inventory identities, not a claim of reading their raw bytes.
Every original wall keyword's block count must match its manifest census.

Other unresolved constraints remain visible, including the selected tied card
and its unresolved native phase. Their `retained_source` indices still refer to
the retained declaration's main-file sources. Original node codes and absence
of slave intersection do not establish zero IKINE. Native hierarchy/main-node
generation, registration order/scratch boundaries, other context producers,
matched post-search NSV and later KINCHK remain separate gates. No all-CIN or
original-condition-absence result is produced.

## Lifetime, limits and checks

Preparation returns an immutable shared handle. Copy and move construction keep
both the canonical and declaration backing alive; assignment is disabled.
Failed preparation cannot mutate an existing handle. No FE owner, CUDA state,
native mask table, mechanics, mass or clock is constructed.

Limits default to 512 MiB host budget, 1 MiB auxiliary input, 512 KiB retained
metadata, 64 groups, 8,192 member occurrences and 8,192 source blocks. Preflight
charges retained canonical/declaration payload plus input, parser, node decode,
index and bounded draft storage before inspecting auxiliary bytes. The reported
own payload separately counts retained vector capacities/string storage. These
are storage budgets, not an RSS or allocator-bookkeeping measurement.

The author gate compiled fresh producer/helper/test units against unchanged
qualified host libraries under one CPU/512 MiB. Five new small tests and five
legacy declaration tests pass. The initial run retained a fixture failure: it
expected only the joint unresolved record and overlooked the intentionally
retained tied card. Only that test expectation changed. No native/GPU or
full-source execution was performed by the author.

Root's standalone owning gate:

```
cmake -S robo-dyna/modelio/tied_shell/auxiliary \
  -B crash-work/build/tied-auxiliary-root-1 \
  -DChrono_DIR=/absolute/path/to/Chrono/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=/absolute/path/to/Total-Lagrangian-FEA \
  -DROBO_DYNA_TIED_CANONICAL=/absolute/path/to/crash-work/assets/yaris-vehicle \
  -DROBO_DYNA_TIED_SCOPE=/absolute/path/to/crash-work/reports/yaris-full-shell-scope-10.json
cmake --build crash-work/build/tied-auxiliary-root-1 --target robo_dyna_tied_auxiliary_check -j1
ctest --test-dir crash-work/build/tied-auxiliary-root-1 --output-on-failure
```

The two actual-source functions authenticate full membership/roles and immutable
lifetime, plus late member corruption and a one-byte-short startup budget.
The fixture extracts only the 44,991-byte auxiliary member itself, then invokes
the existing bounded main-member fixture helper. Full-source execution remains
the root-owned qualification boundary.
