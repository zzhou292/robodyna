# Static source-copy bundle

`PreparedSourceBundle` adds a self-contained static package around the existing
immutable source mapping. It is visualization/source evidence, with no restart
state, live acceptance token, native material admission or run-completion receipt.
The externally authenticated mapping-only reader remains available unchanged.

## Authority and reuse

Preparation takes an immutable `PreparedSourceMapping` plus a `BundleRequest`.
The mapping retains the already checked canonical source and caller-declared
native mapping. Preparation reauthenticates the original extracted key against
its exact expected size/hash and owns one bounded copy. It preserves all 17
original canonical array layouts, filenames and bytes, the original canonical
manifest bytes, and the entire scope report (including original raw declarations).
The ZIP is not copied; its recorded provenance remains in the original metadata.

`CanonicalSource::ReadWithMemberBytes` shares the same resource admission,
metadata parser and source checks as the file path. The chunk reader supplies
complete bounded original bytes to this authority. It does not parse source
keyword cards, select material models, classify tires or reconstruct unit defaults.
Source authority fields use one shared serializer/checker with the mapping record.

## Explicit layout

The versioned schema is `robo_dyna.full_shell_static_source_bundle.v1`.
For the default stem `source` the package contains:

- `source-canonical.json`, the unchanged original canonical manifest.
- `source-scope.json`, the unchanged complete authenticated scope report.
- All 17 original arrays at their original `arrays/` paths.
- `source-key-000000.bin` and following bounded original member chunks.
- The existing eight `source-mapping-*.bin` files and mapping descriptor.
- `source.bundle.json`, written last after all preceding files verify.

The compact descriptor records exact original source/units/selection authority,
mapping digest, file cap, named metadata references, ordered source chunk offsets,
and an ordered file/size/SHA256 inventory. Its measured `static_payload_bytes`
includes the descriptor itself. The bounded decimal byte-count fixed point does
not include a self-referential content hash. The outer caller supplies the exact
expected descriptor size/hash and source/selection/mapping identities on read.

Each chunk has a bounded size, exact contiguous offset and authenticated hash;
reassembly also checks the original complete member hash. No file exceeds 32 MiB,
no source exceeds the explicit 64 MiB original-member cap, and at most 64 chunks
are admitted. A 42,846,753-byte original key splits into 33,554,432 + 9,292,321.
The reader releases reassembled member bytes after canonical source validation.
Its returned source handle does not require access to original input directories;
copying it into another bundle still requires the explicit original member file
for the preparation API's reauthentication.

## Accounting and publication

Preparation appends all exact static source reservations to the caller's
`PlanRequest` and invokes the existing `PlanArchive` before output mutation.
Required `manifest.json`, `frame-index.json`, `configuration.json`, wall, later
per-frame metadata and other application obligations must still be explicitly
reserved. The helper validates known reservations; it cannot infer exhaustive
future output requirements. It never changes dt, frame cadence or stored fields.

For the qualified original no-tire source, N=359785, P=349645 and T=677989:

| Static content | Exact bytes |
| --- | ---: |
| Original 17 canonical arrays | 45,415,582 |
| Original key chunks | 42,846,753 |
| Original source-scope report | 13,175,122 |
| Original canonical manifest | 2,632,394 |
| Eight source mapping arrays | 40,339,684 |
| Mapping and static bundle descriptors | 7,744 |
| **Complete static bundle, 31 files** | **144,417,279** |

The 192 MiB reserve leaves 56,909,313 bytes after this package, before later
obligations. The existing worst-case three-PLA-points-per-parent, 20 ms, h=2^-26,
88 planned frames plus one prefix reserve forecast remains 2,135,428,608 bytes;
the 100-frame profile still rejects. This is a storage forecast, not evidence
that every original material has three applicable PLA points. Actual applicability
and native point counts must come from the qualified live source mapping.

Writing requires the caller to create the real root and `arrays/` directory and
serialize filesystem access, as required by existing ArtifactIO. Every destination
is checked before the first write. All copied file sizes/hashes are checked again
before the last static descriptor is written. Partial files remain create-only
evidence; the same paths reject on retry. No completed static descriptor is
returned after failure, and no operation publishes a completed accepted run.

Reading requires external expected identities, stages chunk/canonical/mapping
validation, and reconstructs the complete expected inventory through those same
authorities. Rehashed wrong chunk contents or missing source/mapping files cannot
replace the original source. The immutable mapping is returned only after all
checks; its native family meanings remain a caller obligation.

## Qualification

Eight new host functions cover source-sized complete roundtrip, exact file/static/
run limits, declared later obligations, shared byte/file source authority, chunk
gaps and rehashed corruption, units/inventory changes, late mapping truncation,
and a real partial first-key-chunk write after all original arrays were copied.
The complete no-tire fixture uses an opaque native family with unavailable PLA
and zero native points. Its 1,388,586,888-byte whole-run forecast is explicitly
that formatting fixture, not the three-point material profile above.
All 27 existing host contracts remain enabled. Author gate: one CPU, 512 MiB
address-space cap, no GPU or solver work; tests pass within that cap.

```sh
cmake -S output/full_shell -B <build> -DChrono_DIR=<install>/lib/cmake/Chrono \
  -DROBO_DYNA_SOURCE_MAPPING_CANONICAL_FIXTURE=<canonical-assets> \
  -DROBO_DYNA_SOURCE_MAPPING_SCOPE_FIXTURE=<yaris-full-shell-scope-6.json>
cmake --build <build> --parallel 1 --target robo_dyna_source_bundle_check robo_dyna_source_mapping_check
ctest --test-dir <build> -R full_shell_source_ --parallel 1 --output-on-failure
```

Full-shell live capture, accepted interval/frame association, application wall/
configuration publication, outer completion manifest and Chrono replay dispatch
remain separate integration work.
