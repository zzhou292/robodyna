# Original self-contact source selection

`OriginalSelection` authenticates the original `combine.key` and
`set-yaris-coarse-v1l.key` members against the retained canonical source
inventory. It resolves the single `CONTACT_AUTOMATIC_SINGLE_SURFACE` part-set
reference through checked `SET_PART_ADD` and `SET_PART_LIST[_TITLE]` operators,
then inventories the selected original shell, solid, and beam records.

This is source admission only. Original FS, FD, DC, SOFT, and IGNORE fields are
retained as provenance; they are not silently applied to the first explicitly
frictionless self-contact profile. The module creates no facets, exclusions,
current activity, pairs, force, stiffness, history, owner, or clock.

The source helper reuses the established bounded raw-block reader from
`modelio/tied_shell`. Set expansion rejects missing sets, unsupported operators,
cycles, duplicate final PIDs, altered bytes, and capacity exhaustion. The
canonical no-tire shell disposition remains separate from non-shell source rows.

Standalone host gates:

```text
cmake -S modelio/self_contact -B BUILD \
  -DChrono_DIR=/path/to/Chrono/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=/path/to/Total-Lagrangian-FEA \
  -DROBO_DYNA_SELF_CONTACT_ORIGINAL_TESTS=ON \
  -DROBO_DYNA_VEHICLE_CANONICAL=/path/to/yaris-vehicle \
  -DROBO_DYNA_VEHICLE_SCOPE=/path/to/yaris-full-shell-scope-10.json
cmake --build BUILD --parallel 1
ctest --test-dir BUILD -R '^original_self_contact_selection_' \
  --output-on-failure
```

The actual fixture extracts the two exact source members into a temporary
directory, while the existing canonical fixture supplies the authenticated main
member. Full selected physical-facet construction and offset/exclusion policy
belong to the vehicle-startup contact adapter that consumes this result.
