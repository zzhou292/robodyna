# Accepted interval record foundation

`IntervalValues` is the single column and value authority for the existing
source-assembly CSV and the new binary representation. `SourceAssemblyWallIntervalValues`
checks the live accepted case; its formatting-only helper checks the supplied
common, QEPH, T3 and contact phase associations. The old CSV delegates to the
same typed values and preserves its column order and precision-17 text.
Legacy V1 archive entry points explicitly reject V2 source inventories until
their analytic hardening declarations have complete output/reader support.

The encoded core is exactly **312 bytes per interval**: four unsigned 64-bit
integers and 35 binary64 values. No native structure or padding is serialized.
Each segment owns two canonical raw little-endian arrays:

| Array | Shape | Fields |
| --- | --- | --- |
| `<stem>-0000.ids.bin`, `<u8` | rows × 4 | owner, base epoch, attempt, accepted epoch |
| `<stem>-0000.values.bin`, `<f8` | rows × 35 | base time, accepted time, then the other 33 CSV diagnostics |

`IntervalValues.h` retains the exact 39-column CSV header. `IntegerFields()` and
`RealFields()` derive descriptor field names from that header. Count diagnostics
remain doubles, as in the original CSV; owner/epoch/attempt retain all 64 bits.
Signed work, replacement and residual channels retain their original meaning.
This is not a collocated energy balance, global energy threshold, force-stage
extension, or restart format. Connector-specific channels still need an explicit
extension and its own forecast cost.

`PlanChunks` is shared by `FullShellVisualizationPlan` and interval I/O. The two
payloads together fit the declared file cap, at most 32 MiB; each individual file
therefore fits it too. A run has at most 64 segments. The full archive forecast
still charges every 312-byte interval, all selected frame fields, and the explicit
static/metadata reserve against 2 GiB. Segment descriptions are embedded in the
future authenticated manifest/index, with no uncharged per-segment JSON files.
The caller must reserve and later verify that metadata; this foundation does
not publish or authenticate a complete run manifest.

`IntervalContext` contains caller-authenticated source identity, actual fixed
timestep, planned interval count, source count limits and declared deformation
envelopes. It does not infer source/material support. The host limit bounds
staged array payloads with a conservative four-copy allowance; it is not an OS
RSS measurement. The writer preallocates one active chunk pair and at most 64
descriptor slots. It performs no row-by-row file creation or solver operation.

`Append(values, expected_accepted_phase)` requires the caller's actual accepted
common phase. It checks consecutive epochs, exact previous endpoint association,
strictly increasing attempts, first half kick and later full kicks, finite
channels, integral bounded count diagnostics, the declared deformation envelope,
plastic-history monotonicity and recorded recurrence/bookkeeping residual bounds.
Source identity and both segment cursors remain in the segment metadata.
QEPH/T3/contact participant stamps are checked by the live factory before
serialization. Those stamps are absent from the 39 columns and **are not
reconstructed or independently authenticated by binary replay**.

Invalid input leaves the writer unchanged and can be retried. Destinations are
checked before startup allocation and again before either chunk file is created.
I/O failure poisons the writer and preserves incomplete create-only evidence;
there is no successful completion result after a partial write. `rows_received()`
includes buffered rows and is not a persistence guarantee. `Finish()` requires
the whole declared horizon; `FinishPrefix()` explicitly closes a nonempty
accepted prefix. Filesystem use is externally serialized, matching ArtifactIO.

`ReadIntervalSegment` requires the authenticated accepted count, completion flag,
expected segment index and exact previous validated cursor. It verifies source,
array shape/type/field layout, size, SHA, expected row count and every row before
returning an immutable staged result. Replacing a previously visible result
therefore leaves it intact on checksum, truncation, late-value or phase failure.
Readers must chain the returned cursor from the initial zero cursor; metadata
alone is not proof of a physical trajectory.

Host qualification uses eight new GTest functions: five segment/creation tests
and three shared-factory/schema tests. They cover IDs above 2^53, exact double
bits and legacy CSV bytes, partial final chunks, explicit prefixes, missing or
rehashed invalid rows, tail NaN, checksum/truncation, wrong material/contact
association, exact budget boundaries, and a real second-file partial write
under a temporary `RLIMIT_FSIZE` in the serial test process. The factory fixture
contains synthetic accepted-shaped public value types; it runs no live solver.

```sh
cmake -S output/full_shell -B <build> -DChrono_DIR=<install>/lib/cmake/Chrono \
  -DROBO_DYNA_INTERVAL_FACTORY_CHECKS=ON -DROBO_DYNA_TL_ROOT=<TL-root>
cmake --build <build> --parallel 1 --target robo_dyna_interval_segment_check \
  robo_dyna_interval_factory_check robo_dyna_full_shell_record_check
ctest --test-dir <build> --parallel 1 --output-on-failure
```

The optional factory test needs CUDA SDK headers because existing public nodal
types include them; it links no CUDA runtime. The production interval values,
planner and segment targets have no TL mechanics/runtime dependency.
