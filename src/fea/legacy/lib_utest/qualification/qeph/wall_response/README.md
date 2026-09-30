# Screened broadside response

This opt-in qualification reuses the sole CUDA nodal owner, QEPH cache/history,
finite-mesh wall contributor and joint publication. Host modules observe the
accepted endpoint, compare refinements and retain numerical records. The
native reference is an oracle in this qualification executable only.

The fixed experiment and numerical limits are documented in
`planning/QEPH_WALL_RESPONSE.md`. Six runs cover one/two cells at h/h2/h4 through
4096H0. A completed run alone does not qualify refinement, a deforming patch or
the Yaris vehicle. The records carry explicit staggered velocity timing and
source work; they are not restart or rendering archives.

Enable `TL_QEPH_ENABLE_WALL_RESPONSE` alongside incoming/response prerequisites.
Build the `qeph_wall_response_*` targets. The four CTest targets cover six
observer, six comparison, four report and one CUDA transaction functions.
They pass with 18 affected existing regressions. The first compile exposed a
GTest name collision; qualified type names fix it. The first unit batch exposed
force/couple indexing that ignored their interleaved layout. Correcting the
six-field stride preserves all test operands and tolerances. Evidence remains
under `crash-work/reports/qeph-wall-response-{build,tests}-*`.

Root launches one complete run per process under the workstation guard:

```
qeph_wall_response_run CELLS REFINEMENT PROVENANCE.json EXPECTED_SHA256 NEW_DIR
qeph_wall_response_compare H/run.json H_SHA H2/run.json H2_SHA H4/run.json H4_SHA NEW_COMPARISON.json
```

The shared provenance schema is `robo-dyna.qeph-wall-response-launch.v1`.
It contains `executable: {path, sha256}` and
`screen: {index_path, index_sha256, selected_h}`, plus root-reviewed source/build
evidence. The command checks exact provenance bytes, the running image, the
selected index and every indexed payload before allocating an owner. Root
authenticates the source/runtime evidence. Files are create-only under external
serialization. Initial intent/provenance survive interruption; a completed or
failed execution retains its last accepted sample and summary in `run.json`.
Each numeric report is capped at 16 MiB. Readers check externally supplied
hashes and retained arithmetic; unsaved intermediate extrema remain evidence
from the reviewed producer, rather than a reconstructed trajectory.

Current resource observations: integrated build 29.001 s, largest sampled RSS
1,059,717,120 B with four affinity CPUs and two workers. Corrected focused tests
take 3.501 s with 165,584,896 B peak sampled RSS, one CPU and monitored GPU use.
The user-requested faster workflow retains commits, test logs and required
bindings, with one integration checkpoint instead of per-utility snapshots.
Full response execution results are recorded separately after the six runs.
