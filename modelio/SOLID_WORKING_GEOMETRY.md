# Selected solid working-coordinate fixture

`solid_working_geometry.select_geometry` reuses `canonical_geometry.load_array`.
It authenticates the requested canonical manifest and all nine complete input
arrays before selecting at most 4,096 parents and 8,192 nodes. Aggregate array
storage is capped at 64 MiB. Parallel arrays, original identities, index joins,
finite coordinates, and exact requested-part coverage are checked. No element
shape is inferred from unique-node count; all eight original slots survive.

`collect_working_solids` reuses `VehicleGeometry`, `scan_vehicle`, the fixed-card
`fields` utility, and the bounded line iterator. It verifies every selected
source NODE/solid, source line, code, blank mask and the exact one-way mm-to-SI
coordinate bits. It retains raw mm values directly, never SI-to-mm reconstruction.
A missing, extra, wrong-family or duplicate selected source record rejects.

`solid_geometry_export.export_geometry` composes those helpers with the complete
`scan_declarations` index. The current declaration adapter supports the plain
original low-density-foam PART/SECTION_SOLID/MAT/curve closure only. Other solid
geometry selection remains reusable without claiming a new material parser.
All declarations retain their complete original raw blocks, including comments,
blank cards and supplied zeroes. The manifest records SID/MID/curve joins and
original density field text, line, parsed binary64 bits, and the exact source
value multiplied once by `1e12` for kg/m³. No reader defaults are substituted.

The little-endian arrays follow the canonical descriptor convention
`file/dtype/shape/bytes/sha256/fields`. Nodes are in original canonical order;
solids are in source-line order, with EID/PID/N1..N8 unchanged. Local node indices
are zero-based in that exported node table. Original canonical row indices are
separate arrays. Coordinate units are explicit in the names and manifest.

The manifest authenticates the original model reference file, canonical manifest,
source member, input arrays and exporter sources. The local source member is
checked twice, on declaration and geometry passes. The ZIP hash/member identity
is retained from the matching canonical/original reference; this exporter does
not claim that it reread the ZIP container. No other source include is admitted.

Publication is create-only: source validation, output serialization and the
complete 4 MiB output-byte cap precede directory creation. The success manifest
is written last. An I/O failure can leave a newly created incomplete directory;
no existing directory is overwritten or removed. Consumers require the manifest
and authenticate every descriptor. Existing importer/runtime paths are unchanged.

Root-only original export, from the app root and under the shared resource guard:

```sh
python3 -B tools/export_solid_working_geometry.py \
  --assets ../crash-work/assets/yaris-vehicle \
  --source-member ../crash-work/assets/yaris-cli-source-1/yaris-coarse-v1l.key \
  --output ../crash-work/reports/yaris-radiator-geometry-1
```

The named CLI pins canonical SHA `c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8`,
PID 2000063, 1,345 source solids and 2,904 nodes. Python API tiny tests use their
own authenticated authored source reference. Author tests do not parse the full
vehicle, execute native/GPU mechanics, permute elements or grant source admission.
