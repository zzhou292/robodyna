# Guided plate contact study

This is a six-node, two-Q4 elastic qualification fixture against the original
finite Yaris wall mesh. It is not a vehicle model. TL-FEA owns state, shell and
contact forces, admission and stepping. Chrono provides reference setup and
accepted mesh playback. No second clock or solver is introduced.

`GuidedPlateStudy` observes accepted scalar metrics at every step and captures
201 common-time geometry samples. Its three source modules separate physical
observables, transactional recording and refinement comparisons.
`GuidedPlateStudyIO` reads/writes bounded reports; `GuidedPlateArtifacts` owns
the optional renderable archive, with shared shell fields and canonical wall
utilities. `GuidedPlateRun` composes the existing Step/observer loop;
`GuidedPlateCli` parses commands and delegates execution. The thin
`guided_plate_main.cpp` only reports the result. `GuidedPlateWallStudyIO`
authenticates source/transform/file identities before calling the existing
same-step response comparison.

With guided plate checks enabled, the executable supports:

```text
robo-dyna-guided run WALL-MANIFEST NEW-STUDY-JSON 1|2|4 [NEW-ARCHIVE-DIRECTORY] [--wall=original|flip|subdivide] [--wall-provenance=NEW-SIDECAR] [--contact-integration=scalar|rectangular] [--experiment=original|penalty-margin-v1]
robo-dyna-guided compare COARSE-STUDY FINE-STUDY NEW-COMPARISON
robo-dyna-guided compare-wall WALL-MANIFEST DERIVED-STUDY DERIVED-SIDECAR CANONICAL-STUDY CANONICAL-SIDECAR NEW-COMPARISON
```

The named penalty-margin-v1 rectangular experiment completes 200 ms with rebound
at h, h/2 and h/4. The h/2 versus h/4 comparison and both h/2 wall variants pass.
The h versus h/2 artificial drilling-energy comparison remains failed. Consult
the [active plan](../../planning/CURRENT_EXECUTION_PLAN.md) for retained evidence
and the selected h/2 archive/rendering work.

Run each invocation through the workspace resource guard and shared workstation
lock. Each creates one case at one fixed timestep; no command automatically
launches other refinements or wall variants. All output paths must be new, with
existing parent directories. Preflight rejects output aliases, dangling output
symlinks, repeated options and unknown backends/transforms. Publication uses the
existing externally serialized artifact utilities; it is not a multi-file atomic
transaction. A write failure can retain incomplete files.

`1`, `2` and `4` select the independent `h`, `h/2` and `h/4` experiments. The baseline
archive records every accepted interval and fixed-cadence frames at every
`100*refinement` steps plus the final state, preserving 201 frames for the named
experiment. Refinement reports instead sample at exactly
`ceil(j*base_steps/200)*refinement` for `j=0..200`. Their schedules are explicit
and independent. An occasional coincident sample/frame causes two captures of
the same accepted state; neither advances mechanics. Captures never interpolate
states to align these schedules.

The default is `--wall=original --contact-integration=scalar --experiment=original`. The explicit
`--contact-integration=rectangular` option requests the separately qualified
dyadic-rectangle implementation through the owning Case configuration. It does
not change the contact law, tolerances, work accounting or leaf/visit/depth
limits. Backend qualification and a successful full response remain separate.

`--experiment=penalty-margin-v1` selects a reviewed physical experiment with
penalty 400,000 N/m^3 and a fresh qualification ID. Original remains 100,000
N/m^3. Geometry, mass, initial mode, the 0.5 mm penetration stop, 5e-7 N force
error and original 1.2500000000000005e-12 J potential error are unchanged.
Each setup recomputes contact stiffness and modal timestep admission. Before
device allocation, actual Chrono/TL forces and certified contact integrals
screen the initial modal path at 0.375 and 0.5 mm. The revised target must
exceed the existing 1% energy envelope. This is a prescribed-path screen, not
a global penetration guarantee; its values and represented-depth corrections
are archived in configuration. Original records its insufficient capacity while
remaining runnable for reproducibility. Arbitrary penalty doubles are rejected.

The five reference-screen tests, named Case/receipt/output tests and strict
reader tests pass. The revised base schedule is 19,998 steps through 200 ms.
The original rectangular full-h attempt stopped safely at the penetration cap
at 145.732 ms; the revised trajectories complete as described above. Use the same
explicit experiment and backend on every member of a refinement or wall study.

For an original-wall run that can participate in authenticated wall comparison,
request its sidecar explicitly. The canonical replay archive remains optional:

```text
robo-dyna-guided run WALL original-h.json 1 original-h-archive --wall-provenance=original-h-wall.json
robo-dyna-guided run WALL flip-h.json 1 --wall=flip --wall-provenance=flip-h-wall.json
robo-dyna-guided run WALL subdivide-h.json 1 --wall=subdivide --wall-provenance=subdivide-h-wall.json
robo-dyna-guided compare-wall WALL flip-h.json flip-h-wall.json original-h.json original-h-wall.json flip-v-original.json
robo-dyna-guided compare-wall WALL subdivide-h.json subdivide-h-wall.json original-h.json original-h-wall.json subdivide-v-original.json
```

These examples use the scalar default. To study the rectangular backend, add
`--contact-integration=rectangular` to **every** corresponding run and choose new
output names. `flip` selects deterministic convex-pair diagonal flips;
`subdivide` selects deterministic uniform four-way triangle subdivision. Plate
setup and initial pose still derive from the original canonical wall. Derived
walls require a sidecar and **cannot** create a canonical guided replay bundle;
passing an archive path for either derived kind is rejected before initialization.

The sidecar is written last, after a completed Study and optional canonical
archive. It binds the exact saved Study bytes, authenticated original wall,
declared transform, derived topology and source lineage. `compare-wall` reads
each input once and authenticates those same bytes. Argument roles are strict:
the first response must be `flip` or `subdivide`, the second must be `original`.
Both must use the same backend, fixed timestep, refinement, schedule and
physical experiment. Their process-local owner IDs may coincide; source/file
identity is established separately. The canonical response supplies relative
comparison scales. The output uses a distinct
`robo_dyna.guided_plate_wall_comparison.v1` schema with both Study/sidecar hashes.

`compare COARSE FINE NEW-COMPARISON` remains the refinement comparison: use the
same wall binding and backend, with the fine run at half the coarse timestep.
The two comparison commands do not rerun mechanics or prove that a file's
reported history was generated by the solver. Synthetic host fixtures test I/O
and accounting only and are never accepted replay bundles.

Completed reports retain a missing-rebound outcome. `run` returns 0 after the
admitted horizon and requested publications complete, even if no rebound was
observed. Completion alone is not a passing comparison: either comparison
command returns 2 and writes `passed=false` for a valid
experiment that fails a frozen response, energy, event or deformation gate.
Malformed inputs or incomplete work return 1; malformed comparisons publish no
result. Existing reports are preserved. Force peak certificates enclose the maxima over all
accepted endpoints; they do not claim a continuous-time peak. Applied-base
forces determine impulse. Endpoint force fields describe the saved geometry.
Contact numerical estimates may lie outside their truth interval when the
declared error radius covers both endpoints. Numerical limits are not relaxed
when selecting another wall or backend.

New Study v1 reports explicitly serialize configuration
`contact_integration_backend` as `scalar-dyadic-squares` or `dyadic-rectangles`.
Legacy v1 reports without that field mean scalar only. Unknown names, wrong
types and duplicate fields are rejected. Wall comparisons record the backend
explicitly; the unchanged refinement report binds it through the exact input
Study hashes. The physical experiment fingerprint remains distinct from this
execution choice.

New Study/archive records also explicitly serialize `guided_experiment`.
Missing legacy names resolve only to `original`; names and qualification IDs
must agree. Readers reject changed named penalties, absolute budgets and caps.
The original physical fingerprint serialization is preserved; the revised
fingerprint additionally binds its complete stop/budget declaration. This
identity does not claim that a completed trajectory exists.

The wall transform helper authenticates the original source, retains explicit
derived IDs and validates its complete finite mesh. Prescribed-state invariance
tests and actual h/2 response comparisons pass for flipped pairs and subdivision.
Actual rendering of the selected h/2 remains a separate gate.

Archives whose ledgers fit one file retain `guided_plate_artifacts.v1`.
Larger ledgers use `guided_plate_artifacts.v2`, with identical
`interval_ledger_segments` metadata in configuration and manifest. Each logical
CSV has deterministic complete-row segments, repeated authenticated headers,
inclusive accepted-epoch ranges and bounded row/file capacities. Every segment
has its own inventory size and SHA-256. The 32 MiB per-file and 256 MiB aggregate
limits remain unchanged. The named h/2 forecast is 164,552,257 bytes at 201 frames;
its contact ledger splits after epoch 25,810. h/4 remains above the aggregate
archive budget and can still produce its smaller Study/sidecar evidence.

`CsvLedgerSegments` handles layout and metadata, `CsvLedgerWriter` handles
create-only stream lifetime, and `AcceptedReplayLedgers` checks the actual saved
rows. The v2 reader verifies all three contributors' owner/epoch/attempt/time
identity, exact fixed-step recurrence and saved endpoint bindings, while keeping
only the current segments in memory. These checks establish archive consistency,
not a new physical accuracy claim. Legacy v1 replay retains its original rules.
