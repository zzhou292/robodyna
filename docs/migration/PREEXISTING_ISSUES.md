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

## Public visualization reporting with YAML disabled

Compiling the full Python core wrapper exposed an undefined symbol at import:
`ChVisualShapeFEA::Settings::PrintInfo() const`. Its public declaration was
unconditional, but its definition was inside `CHRONO_HAS_YAML`. The qualified
native profile disables YAML. This existed in the imported source and was not
exercised by the earlier C++ demos.

The binding integration moves the unchanged printing function outside that
conditional. It reads only existing settings and writes to `std::cout`; no
equations or model data change. A focused visual-model test calls the public
method in the YAML-disabled native profile. Failed import evidence is retained
in `crash-work/reports/robodyna-body-bindings-build-2.json`.

## Borrowed Python references

The inherited SWIG frame binding returns a borrowed position-vector reference.
`ChBodyAuxRef::GetFrameCOMToRef()` returns a frame value, so Python callers must
retain that frame proxy while reading its position. Chaining
`body.GetFrameCOMToRef().GetPos()[0]` can destroy the temporary owning frame
before indexing the borrowed vector. The binding smoke test now retains the
frame explicitly; its numerical expectations are unchanged. No ownership policy
or physics was changed to accommodate this test.

## Core-only Mesh proxy and FEA nodes

The inherited core-only Python `ChMesh.AddNode` descriptor names an unqualified
`ChNodeFEAbase`; the FEA module uses the qualified FE node type. Historical and
current generated functions are byte-identical. Construct meshes through
`pychrono.fea.ChMesh`, then use the existing System and cast interfaces for object
exchange. The qualified core/FEA runtime test follows that supported route.
Evidence is in the outer workspace at
`crash-work/investigations/robodyna-fea-core-mesh-metadata-1/evidence.json`.
No production binding or solver code was changed for this issue.
