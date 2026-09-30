# One-job wall analysis command

Source-only wiring, pending the root-owned build and protocol checks:

```
qeph_wall_analysis JOB_CONFIG_JSON EXPECTED_CONFIG_SHA256 NEW_DIRECTORY REMAINING_SET_BYTES
```

The configuration is a bounded, strictly parsed JSON object (64 KiB maximum),
whose exact bytes must match the externally supplied lowercase SHA256. Those
unchanged bytes become the analysis provenance. It requires:

- `executable`: an object with `path` and lowercase `sha256` strings. The path
  must resolve to the running `/proc/self/exe` image; its actual bytes are hashed.
- `raw`: exactly six fields: `directory`, `cells`, `normal_velocity_m_s`,
  `index_sha256`, `provenance_sha256`, and `bytes`. Paths are nonempty and bounded;
  hashes are lowercase 64-digit hexadecimal strings. `cells` is the JSON unsigned
  integer 1 or 2. Velocity is explicitly a JSON double `-8.0`, `0.0`, or `8.0`;
  integer representations and negative zero reject. `bytes` is a positive JSON
  unsigned integer no larger than 96 MiB and must equal the complete raw receipt.
- Other top-level members may bind source maps and build inputs. Authentication
  of those members remains the external launcher's responsibility.

The real raw directory, every raw file/hash, closed inventory, model and identity
are validated by `ReadRawJob` before creating any output. A missing or invalid
raw input cannot produce an analysis directory. An authenticated incomplete raw
collection is allowed for diagnosis; no missing native samples are regenerated.

`REMAINING_SET_BYTES` is a canonical positive decimal integer at most 96 MiB.
The launcher supplies the decreasing shared raw+derived allowance, currently
53,360,416 bytes before the first derived report, and reserves final-selection
space. Per-job raw bytes are a separate exact input allowance, not a debit that
may be counted again. Each writer receives the available derived remainder.
The existing writer's per-file publication and failure accounting contract in
`DERIVED_REPORT.md` applies: prior artifacts survive exceptions, and the launcher
must account for all actual files, including possible partial writes.

Exit 0 means the one-job analysis completed, including when scientific `passed`
is false (the aggregate includes diagnostic 4H0). Exit 2 means numerical analysis
is incomplete. Exit 1 means protocol, I/O or runtime-image binding failure.
Neither command selects a timestep or admits an impact trajectory.

`WallCommandLine.h` shares only command parsing and provenance/image preflight
with the retained raw CLI. Its raw collector, callback order, output and exit
semantics remain unchanged. No native map is invoked by the analysis command.
