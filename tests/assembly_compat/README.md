# Mixed assembly baseline

`//tests/assembly_compat:baseline_test` passed all five cases before moving the
actual Assembly definition. Receipt:
`crash-work/reports/robodyna-assembly-baseline-1.json` in the enclosing workspace.
The tests use the same legacy entry points before and after the rename.

Coverage includes mixed body/shaft/link/mesh admission and offsets, nested owner
and time propagation, removal and shared lifetime, pending batch admission, and
copy/Clone/assignment/ADL swap. Existing frozen System archives cover the original
assembly tag, represented fields and top-level body restoration; no second archive
producer or physical-restart claim is added.

Several inherited behaviors are intentional baseline expectations: copy/Clone
retain counters but omit child collections; assignment and swap exchange metadata
without replacing topology; Clear retains queued batch insertions. A naming change
must preserve these behaviors. Any later behavioral correction needs separate
design, tests and compatibility review.
