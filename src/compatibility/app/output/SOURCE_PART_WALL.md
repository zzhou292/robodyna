The `robo_dyna.source_part_wall_artifacts.v1` bundle records the original
117-node, 88-QEPH/6-T3 elastic source part against its actual placed finite mesh
wall. It is a separate replay kind and schema from the existing elastic pulse.
Common source tables and kinematic serialization retain the pulse's member
order and arithmetic. No solver, clock, material history or restart ownership
is introduced in output or rendering.

The configuration records the native nodal M/J partitions and source IDs,
measured common initial K0, declared physical initial velocity and the immutable
wall setup certificate. The original canonical manifest, declared X shift and
its exact binary64 bits, actual represented wall X, semantic source mesh digest
and placed mesh/OBJ hashes remain separate. The archived placed mesh retains
all 62 vertices, 100 triangles and original connectivity/identity.

Frames contain accepted endpoint x/q, raw carried midpoint v/omega and their
actual owner timing, plus separately derived endpoint v/omega. Epoch zero has
`contact_state=certified_separated_startup` and `contact=null`. Later contact
data retains `prepared_candidate_of_committed_interval` provenance beside the
matching accepted owner stamp; the case has already committed that by-value
copy with both native histories. Per-node/parent force and potential
certificates, actual wall face IDs, work, impulse, moment, penetration, native
energy and common momentum reporting are retained. Angular reporting explicitly
uses endpoint x and raw carried velocities, with no angular admission claim.

The segmented 34-column interval ledger includes every accepted step. Its
`strictly_separated_nodes` column counts nodes that are not touching, with zero
force and potential upper bounds. This supports separation-persistence checks
over unsaved endpoints without an additional device readback. Saved scientific
fields must match the corresponding ledger row. Every row retains the same
configured physical energy allowance; uncertainty is added to error, never to
the allowance. Native K0 is also bound to the archived ordered mass reduction.

`Finish` requires the whole requested horizon. `FinishPrefix` uses the additive
CSV prefix close to retain exactly the written rows and segments. A closed
prefix has a complete artifact manifest but `horizon_complete=false`, an actual
final epoch below `requested_steps`, and a nonempty stop reason. An output I/O
failure instead leaves `failure.json` and is not replayable. Neither close path
claims rebound, source attachment support, plasticity or crash accuracy.

The shared schema byte limits are 480 KiB for fields, 40 KiB for mesh JSON and
16 KiB for OBJ, totaling 536 KiB per saved frame. The first emitted dynamic
fields occupied 303739 bytes, exceeding the original 256 KiB estimate. Holding
the fixed schema/strings and reserving 26 bytes for each of its 8164 numeric
tokens and five bytes per boolean bounds the document at 476443 bytes. The same
conservative replacement bounds mesh JSON at 37798 bytes. An OBJ with 117
vertices and 182 triangles needs at most `117*83 + 182*14 = 12259` bytes with
26-byte coordinate tokens and the fixed three-digit index range. Writer and
reader enforce these limits; no accepted scientific channel is removed.

The full h/4 forecast retains all 131072 interval rows and 259 possible frames,
including the extra first-kick and last-accepted failure endpoint slots. The
34-column ledger reserves 26 bytes per column, with repeated segment headers;
adding the 536 KiB frame bound and 1 MiB static reserve remains below the
unchanged 256 MiB aggregate cap (about 247.1 MiB). The existing actual-engine
output test checks emitted sizes, expands actual JSON scalar token widths to
check the bound, checks the full ledger/frame forecast and rejects a rehashed,
otherwise-valid JSON frame padded beyond its field cap. Pulse limits and
serialization are unchanged.

The thin CLI uses the existing source engine and frozen wall factories:

```
robo_dyna_source_part_wall READINESS WALL REFINEMENT BASE_STEPS FRAME_EVERY_BASE NEW_DIR
robo_dyna_replay NEW_DIR --capture NEW_CAPTURE_DIR --fps 30
```

Refinement is 1, 2 or 4. The largest base horizon is 32768 steps; a 128-step
base frame stride records initial, first-kick, regular and final frames. CLI
exit 0 means the requested bounded horizon was archived. Exit 2 means an
explicit accepted prefix was archived after a rejected step. Exit 1 means
startup or output is incomplete. The coordinator must still apply the existing
workstation guard and run builds/simulations serially.

The Chrono/VSG scene uses the actual placed wall at physical scale. A close
incident-side camera frames the complete moving source trajectory and nearby
wall, with Z vertical. The wall remains an unscaled gray wireframe and its
whole mesh is retained even where it extends outside the view. Wall-mode
deformation magnification is rejected. Prefix status is shown in the overlay
and capture metadata.

`source_part_wall_output` supplies five small actual-engine output/reader test
functions. Existing source pulse replay tests reuse the same safe bundle clone
helper; the CSV and scene targets each add one focused check. All seven new
functions and affected replay regressions pass under the workstation guard.
The old pulse retains 21 scientific files / 695141 bytes exactly after shared
serialization extraction. Runtime evidence is in workspace reports
`source-part-wall-output-comparison-tests-2`, `source-part-wall-output-tests-1`
and `source-wall-pulse-parity-1.json`; initial cap failures are preserved.
