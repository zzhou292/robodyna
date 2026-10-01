# Pre-body-rename archive evidence

`//tests/body_compat:write_baseline` is a small CPU-only producer. Run it against
the existing body implementation before applying the staged canonical body move:

```text
bazel-bin/tests/body_compat/write_baseline NEW_OUTPUT_DIRECTORY
```

The directory must not exist. The producer writes `baseline.json`,
`baseline.xml`, `baseline.bin` and `producer.json`. It first checks round trips,
BodyEasyBox constructor recovery, marker/force parent rebinding, repeated child
pointer ownership and deterministic repeated writes. Output is limited to 1 MiB
per archive. No simulation, graphics, external assets or GPU are used.

These cases supplement the existing `tests/serialization_compat` fixtures; they
do not replace them. The new stream helper only centralizes archive lifetime and
stream handling. It does not change either production serialization or the
existing qualification harness.

Generation alone is not a frozen baseline. After the N2/N4 checkpoint, extend
the existing freeze helper with the reviewed body profile and pin the producer,
archive machinery, Body/BodyEasy, Marker and Force source files. Keep the
existing requirement that named production inputs match the recorded Git HEAD.
Only then admit the canonical body implementation and compare its reader and
writer against these original files. Preserve failed producer directories.

The pre-rename producer has passed all three formats and deterministic writes.
Root owns the accepted N2/N4 checkpoint and then the freeze operation:

```sh
python3 tests/serialization_compat/freeze_baseline.py \
  --profile body --repo-root . \
  --generated-dir ../crash-work/investigations/robodyna-body-baseline-1 \
  --destination tests/body_compat/fixtures \
  --producer-binary bazel-bin/tests/body_compat/write_baseline
```

`//tests/body_compat:body_tests` reads those immutable files and checks exact new
writer bytes in JSON/XML/binary. It reuses the producer's semantic checks and
the independent fixture-integrity test. The separate upcast target exercises
raw/shared conversion for all four inherited base classes, using string and
RTTI lookup combinations, adjusted addresses, dynamic recovery and shared
control-block identity. It works with the current public alias and the later
reversed legacy alias.

The new reader/upcast tests still require root's guarded qualification. Existing
producer C++ files and prior fixture directories remain unchanged.
