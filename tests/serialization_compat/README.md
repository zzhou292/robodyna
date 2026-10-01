# Serialization compatibility during the API rename

These tests preserve the inherited Chrono object archive contract independently
of C++ spelling changes. They do not implement a restart format or change physics.

`FixtureModel` deliberately uses existing types: vector/frame values, a plain
`ChBody`, and one `ChBodyAuxRef` referenced through body, physics-item and adjusted
moving-frame pointers. It verifies dynamic factory creation, one shared ownership
control block, null pointers, mass/inertia, reference/COM frames and velocities.
Separate assertions exercise raw/shared multiple-inheritance casting and the
unregistered base/template version keys in JSON and XML. Binary archives retain
their existing implementation and platform limits.

## Freeze before changing production archives or the factory

The root operator runs the guarded build; this directory does not launch builds:

```sh
bazel build --config=host //tests/serialization_compat:write_baseline
bazel-bin/tests/serialization_compat/write_baseline /tmp/robodyna-archive-baseline-NEW
python3 tests/serialization_compat/freeze_baseline.py \
  --repo-root . \
  --generated-dir /tmp/robodyna-archive-baseline-NEW \
  --destination tests/serialization_compat/fixtures \
  --producer-binary bazel-bin/tests/serialization_compat/write_baseline
bazel test --config=host //tests/serialization_compat:baseline_tests
```

The writer requires a new directory and refuses overwrites. Before publishing
fixtures it checks semantic round trips and repeatable writes for all three
formats. The freezer records the actual compiler/ABI metadata, producer binary
and source hashes, and 24 relevant production input hashes that must match the
recorded repository HEAD. It does **not** claim that concurrent unrelated source
changes are absent or that these 24 files represent the entire linked program.

Once accepted, commit the generated fixture bytes and manifest as immutable
evidence. `compatibility_test` reads those bytes and requires new writes of the
same fixture to match them. `fixture_integrity_test` authenticates the frozen
bytes without silently updating expected values. Do not regenerate fixtures to
make a rename pass; diagnose any changed class tags, field names, version keys,
pointer graph or archived numerical values.

The literal unregistered version keys intentionally pin the supported
Linux/GCC/Itanium archive identity. Existing compiler-derived keys were never a
general cross-toolchain portability contract. Test tolerances cover semantic
floating-point values; the writer comparison additionally pins exact bytes on
the qualified build profile.

## Opt-in naming support

`CH_FACTORY_REGISTER_TAG(Type, "LegacyTag", unique_registration_id)` retains one
canonical factory/writer tag while allowing a qualified, renamed C++ type.
`CH_UPCASTING_TAGGED(From, To, "LegacyFrom", "LegacyTo", unique_conversion_id)`
retains the existing conversion graph names and real pointer adjustment. Use
these once in implementation files; do not register a second alias for the same
type. Existing registration macros retain their inherited behavior.

For an unregistered archived base or template specialization, use
`VersionWrite<Type>("captured_legacy_identity")` and the matching named
`VersionRead<Type>`. Registered types normally need only their canonical factory
tag and the original no-argument version calls. Named calls install a transient
type-specific name and delegate to the existing version policy and virtual
format hooks. The scope restores its previous value even if a hook throws;
ordinary nested requests for different types retain their own identity. This
adds no registry, read alias, serialized field, or physics state.

The owned transient fields change the C++ layout of `ChArchive`. Rebuild its
consumers and plugins with the updated headers/library; this stage preserves
the qualified archive wire representation, not binary compatibility with a
previously compiled Chrono archive implementation.

`//tests/serialization_compat:all_tests` combines frozen actual-type compatibility,
fixture integrity, the genuinely renamed probe, and version-policy tests for
clustering, disabled versions, virtual dispatch, exceptions and binary bytes.
The original test producers remain separate; the renamed probe does not link
their types. The inherited `CHNVP` macro uses an unqualified helper, so tests in
new namespaces deliberately use `using chrono::make_ChNameValue` for primitive
and standard-library fields instead of changing its behavior during this stage.
