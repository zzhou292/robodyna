# Pre-existing checks requiring scope review

The inherited `output/physical_run/tests/replay_preflight/verify_sources.py`
checks whole-file hashes for an earlier source-only replay-admission change against
baseline `1ea39473365a3853c346d80df10c6efd9317e706`. It rejects current Prepare.cpp,
which is byte-identical in the qualified100ms app and the consolidated import:
`57cf2b99d7238aab0bd2b4bd6317b8a4a5d16ee0abbad2a7bfa20169ff372449`.
The later100ms horizon change predates this migration. Keep the historical proof
unchanged; do not refresh its hashes to claim that old reversal evidence covers
new source. Current functional replay-preflight tests and exact import/short-run
comparisons are separate migration gates. Numerical runtime validation was not
disabled. This issue is not classified as a new physics regression.

## Populated glyph property archives

The pre-change visual-model baseline exposed an inherited archive limitation:
after FE node glyphs populate their color properties, restoring the visual model
throws `Cannot call CallConstructorDefault(). Class not registered, and is without
default constructor` (the emitted message has an empty class name). Receipt:
`crash-work/reports/robodyna-rename-baseline-build-2.json`; test log is the
`tests/visual_model:compatibility_test` result from that run.

`ChGlyphs::AddProperty` calls `clone()`. `ChPropertyT<ChColor>::clone()` returns
the template base type, while the factory registers `ChPropertyColor`, not that
base template. The populated property therefore has no registered dynamic tag.
The relevant visual, property and archive implementations were unchanged from
the pre-rename baseline when this failed; the inertia move does not alter them.

The visual separation gates cover ordinary attachment round trips before glyph
properties are populated, populated glyph coordinates/colors/update behavior, and
the existing omission of FE attachments from serialized visual models. They do
not assert that populated glyph properties or physical FE state can restart.
Repairing the inherited clone/registration behavior belongs in a separate change.
