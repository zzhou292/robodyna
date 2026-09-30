# Original Yaris shell scope

`yaris_full_shell.compile_full_shell_scope(archive, assets, tire_policy)` produces
an authenticated, bounded **inventory**, with `simulation_ready=false`. It does
not initialize a model, recompute mass, replace materials or release connections.
The existing six-/seven-part source compiler and runtime remain unchanged.

The default `retain_all` selects every original `*ELEMENT_SHELL` record and the
union of its source nodes. `omit_original_tire_shells` removes only the eight
authenticated tire body/tread PIDs declared in `full_shell_coverage.py`. It checks
each PID, title, SID, MID and source family. Rims, hubs and disks stay selected.
Unknown policies fail before reading source files. No tire exclusion is inferred
from a substring, coordinates, material stiffness or topology.

Run from the app checkout:

```sh
python3 tools/compile_yaris_full_shell.py \
  --source-archive /path/to/source_model.zip \
  --canonical-assets /path/to/yaris-vehicle \
  --output /new/path/scope.json
```

The CLI is create-only. Report size is capped at 32 MiB before opening output.
Use the workspace resource guard for actual source compilation. The report is
not an accepted replay bundle and must not be passed to a runtime admission API.

Ownership stays focused:

- `full_shell_source`: reuse pinned ZIP/README authentication, stream all source
  geometry, verify all four original include hashes/censuses and all 17 canonical
  arrays byte-for-byte. The legacy connector PID is an authentication anchor
  only; it does not limit the selected source scope.
- `full_shell_coverage`: complete shell coverage, precise optional exclusion,
  non-shell incidence, source EID/NID digests and interface NID-to-PID mapping.
- `full_shell_declarations` / `full_shell_materials`: preserve all source cards,
  source options, supported-parser candidates and retained unsupported parts.
  Literal LCSS/SIGY/ETAN/C/P and supplied-option summaries distinguish first
  parser failures from additional source options; they do not admit new laws.
- `full_shell_connections`: retain all literal nodal groups/spotweld endpoints,
  source sets, tied candidate scopes and remaining include/constraint/load
  obligations. Internal/cross-boundary classifications describe node selection,
  not qualified mechanics or load-path closure.
- `yaris_full_shell` and the CLI compose/serialize those reusable modules.

`selected_shell_part_ids` plus the authenticated canonical arrays identify every
selected shell and node without duplicating large geometry arrays in JSON.
`declarations.parts` lists all original parts, including non-shell ones;
`tables` retain original sections, materials and curves. `connections` retains
literal endpoint order and complete family/PID incidence for those endpoints.
General joints and tied projections are source obligations, not fabricated pairs.

Non-shell mass/inertia is unknown, never zero. In particular, omitted beam welds
can connect to selected shell parts through tied contact even when they share no
literal shell node IDs. Source shell-only coverage does not make these ties
irrelevant. Full-vehicle load-path and mass closure require separate admission.

Focused host gate:

```sh
python3 -m unittest discover -s tests -p test_full_shell_scope.py -v
```

It checks complete mixed-family coverage, exact tire identity, unsupported/missing
declarations, source field summaries, source rehash mutations including signed
zero, policy preflight, bounded publication and create-only failure. Actual
source audit evidence lives in `crash-work/reports/yaris-full-shell-scope-*`.
