# Component accepted binary frame producer

This slice produces the existing `full_shell::FrameRecord` from real accepted
six/seven-part `SourceAssemblyWallCase` captures. The record format is reusable;
its use here does not imply a complete vehicle, a completed archive, a restart,
or admission by the full-shell replay reader. Source V1 and actual catalog
`LayeredLaw44Nip3` are required. Other laws/source schemas reject explicitly.

`SourceAssemblyBinaryFrames` owns the existing `SourceAssemblyAcceptedOutput`,
a `Context`, and two exact-sized position/PLA buffers. `Capture` calls the case's
accepted-only readback once. The extracted `wall_fields::AcceptedFrameView`
checks the actual owner, source, common publication, both native families,
optional connector/force-stage metadata and contact phase. It is also used by
the existing JSON field factory. No TL ownership, transaction or arithmetic is
changed. The smaller contact phase helper is shared with the old contact writer;
its field order and values remain unchanged.

`ComponentFrameFields.cpp` copies every binary64 world coordinate and every
native thickness point's equivalent plastic strain, in original source-parent
order. QEPH/T3 local indices select their own histories. No maximum reduction,
field downsampling, decimal conversion, synthetic quadrature or deformation
scale enters these records. Family codes are explicitly QEPH=1 and T3=2 under
this component mapping domain. All three native points are applicable for the
admitted law; no nonplastic point is fabricated as zero.

The stamp copies the accepted epoch/time, reaction base epoch/time, actual
previous-midpoint velocity time, kick duration, and common material/contact
attempt. Epoch zero has its original collocated/zero-interval declaration.
Neither sample time nor attempt is reconstructed from output cadence.

All capture, context, source/point and phase checks finish before public buffer
selection. Storage is allocated at construction; successful capture performs no
new allocation. A failed trial leaves the accepted state intact, so a newer
uncaptured accepted prefix can still be captured. A failed readback or wrong
owner leaves the previous public record intact. CUDA poison is retained by the
existing owner/participants. External serialization is required; returned frame
views expire at the next successful capture or producer destruction.

`Write` uses the existing bounded/create-only per-frame writer and returns its
exact descriptor identity. It does not create an index or run manifest. Late I/O
failure can leave incomplete files as evidence. The last successfully captured
record remains immutable; a future outer publisher must account for every file,
prove full interval/index chronology and publish its own completion receipt.

## Component mapping authority

`PreparedSourceMapping` requires the entire selected canonical scope and is
intentionally not used for a component. `ComponentContext` instead retains the
actual authenticated component inventory identity and checks the immutable
source/native catalog against the existing `SourceAssemblySurface`.

The mapping digest starts with the NUL-terminated domain
`robo_dyna.source_assembly_binary_mapping.v1`. It then appends three named arrays
in this fixed order. Each entry is NUL-terminated name, NUL-terminated canonical
`<u8` dtype, little-endian UInt64 `(rows, columns, payload_bytes)`, then the 64
ASCII hexadecimal characters of the payload SHA256. Payloads use the existing
`BoundedArrayIO::Encode`; the final SHA uses `ArtifactIO::Sha256`.

| Array | Ordered columns |
| --- | --- |
| nodes | global node index, original source NID |
| parents | original source index, EID, PID, MID, SECID, curve ID, original ELFORM, native family, family-local index, native point count, PLA applicability, first display triangle, display triangle count |
| triangles | v0, v1, v2, EID, PID, physical local face, display subtriangle |

No C++ padding, file path, asset ID or run identity enters that digest. The
separate `Context::Identity` binds the actual inventory bytes/hash, source
instance, owner, run, topology, configuration and qualification. Original
geometry/source declarations and boundary scope remain authenticated by the
component source inventory. Native startup/source authority remains the case
composition; the formatting-only `detail::StageFrame` seam is not an acceptance
token and requires its immutable component context and disjoint staging arrays.

## Qualification

Three host tests cover actual six-part source/context mapping and limits,
synthetic formatting parity against the existing JSON serializer, exact binary
roundtrip, wrong expected attempt, late native-point NaN, contact-attempt mismatch,
short staging capacity and unchanged output with clean retry. Synthetic markers
are formatting tests only. Four unchanged legacy wall-field/source/configuration/interval tests also run in
the same target. All seven pass under 1 CPU / 512 MiB.

Four authored CUDA tests cover actual complete six/seven-part mesh-wall cases:
initial/32/64 accepted records, all position bits and all three native PLA points,
source IDs/family/display PID association, common phase/attempt, rejected interval
exclusion and retry, stable buffer/device allocation counts, actual D2H failure
via the existing qualification-only CUDA interposer, startup byte limits and a
foreign owner. CUDA execution is scheduled by the parent, not claimed by the
host gate. These short component checks do not qualify full-vehicle dynamics.

Standalone owning build:

```sh
cmake -S <app>/output/source_assembly/binary_frames -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=<chrono-install>/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=<TL-root> \
  -DROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=<six-part-inventory> \
  -DROBO_DYNA_SOURCE_ASSEMBLY_WALL=<wall-manifest> \
  -DROBO_DYNA_COMPONENT_BINARY_LIVE=ON \
  -DROBO_DYNA_SOURCE_BRACKET_INVENTORY=<seven-part-inventory>
cmake --build <build> --target robo_dyna_component_binary_values_check \
  robo_dyna_component_binary_live_check --parallel 1
ctest --test-dir <build> --parallel 1 --output-on-failure
```

Omit the live flag for the pure host target; it links no runtime owner or CUDA
mechanics. Both native readback and the full JSON field writer remain unchanged
contracts outside this small adapter.
