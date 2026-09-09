# Source part declarations (E1)

This package compiles the requested source PART → SECTION_SHELL / MAT024 →
DEFINE_CURVE closure. It creates no geometry, material history, force operation,
accepted owner or timestep. Every report has `simulation_ready=false`.

`source_blocks.py` scans the whole bounded include before resolving references.
It retains exact selected-family block bytes, hashes, physical source lines and
empty data cards. Duplicate requested identities fail even when the second one
appears later or uses an unsupported keyword suffix. Geometry blocks are streamed
and hashed, never flattened or loaded into a new mechanics representation.

`declarations.py` contains immutable typed records and explicit units.
`keyword_cards.py` reads the four supported keyword forms using the existing
importer's fixed-field helper and blank mask. `_legacy.py` is the small import
compatibility boundary for those standalone tools; it restores the module search
path after importing their existing utilities. `yaris_part.py` verifies the
pinned archive/include, composes the closure and stages a report. The CLI handles
arguments and create-only publication; it works from any current directory.

The current declaration domain is deliberately narrow:

- PART has its title and three required references; ancillary fields must be
  blank. SECTION_SHELL requires explicit ELFORM 2, NIP 3 and four identical,
  positive thicknesses. Other section fields remain absent, with no inferred
  centering, added mass, shear factor or control-card defaults.
- MAT024 requires supplied density, elastic modulus, Poisson ratio, positive C/P
  and positive LCSS. SIGY/ETAN are retained if supplied. Failure/LCSR/additional
  options and inline EPS/ES values are excluded. Four material data cards are
  required, including the physically empty EPS/ES cards in this original model.
  VP `0.0` becomes the exact integer flag 0 while retaining the raw field and
  its supplied status; a blank VP stays unresolved.
- DEFINE_CURVE requires explicit SIDR 0, 2..1024 ordered points starting at zero,
  positive nondecreasing stress and identity scaling/offsets. Only absent curve
  scales and offsets receive separately recorded documented defaults. DATTYPE,
  LCINT, nonidentity transforms, tables, softening and keyword suffixes are
  outside this declaration domain. There is no interpolation/extrapolation
  implementation or constitutive update here.

The original archive README explicitly declares metric tonnes, millimeters,
newtons and seconds. It is preserved with its member hash under the archive's
existing SHA pin. Those scales produce density in kg/m³, thickness in meters,
stress in Pa and C in s⁻¹. No units are inferred from plausible material numbers.
Every supplied field, nullable absence, blank mask and original block is retained
alongside those SI declarations. Unit conversion overflow/underflow is rejected.

The reviewed [Altair MAT024 input reference](https://help.altair.com/hwsolvers/rad/topics/solvers/rad/mat_024_piecewise_linear_plasticity_lsdyna_r.htm)
defines this positive-LCSS branch through a plastic-strain/stress curve, with
SIGY/ETAN ignored and VP 0 denoting total strain rate. Its documented mapping is
LAW36 for the curve branch; this is not evidence of equivalent donor or TL
mechanics. The [curve reference](https://help.altair.com/hwsolvers/rad/topics/solvers/rad/define_curve_lsdyna_r.htm)
provides scale/offset defaults. The [section reference](https://help.altair.com/hwsolvers/rad/topics/solvers/rad/section_shell_lsdyna_r.htm)
is an input-interface subset and does not justify inventing all LS-DYNA defaults.
Other fields and controls remain pending before simulation.

The 2026-09-09 guarded execution passed all **12 synthetic tests** and compiled
the actual archived connector in 1.005 s, with 22.25 MiB peak sampled RSS and no
GPU. The [compiler guard](../../crash-work/reports/source-part-declarations-compile-1.json),
[test guard](../../crash-work/reports/source-part-declarations-tests-1.json) and
[typed report](../../crash-work/reports/yaris-part-2000157-declarations-1.json)
are retained. An [independent integrity check](../../crash-work/reports/source-part-declarations-integrity-1.json)
matched the archive, all four selected raw blocks, physical card lines, blank
masks, unit authority and generator hashes without rerunning the compiler.

The actual closure is PID/SECID/MID **2000157**, `177_railfrontconnector`, to
curve **2100270**. It declares ELFORM 2, three thickness integration points,
four 1.648 mm thicknesses, density approximately 7,890 kg/m³, E = 200 GPa,
Poisson ratio 0.3, C = 8,000 s⁻¹, P = 8 and supplied VP `0.0`. Both empty
material data cards survive at source lines 3263 and 3265. The curve has
46 points, from plastic strain 0 / stress 270 MPa to 0.3 / 362 MPa; only its
blank offsets receive separately recorded zero defaults. The report still
has `simulation_ready=false`; E2a geometry and provisional mass readiness are
qualified below, while typed attachments and all source mechanics remain open.

To repeat after the shared workstation slot is released, use new report names:

```bash
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/yaris-part-declarations-repeat.json \
  --lock crash-work/reports/workstation.lock \
  --cpus 1 --min-available-gib 32 --max-rss-gib 1 --timeout 60 \
  -- python3 robo-dyna/tools/compile_yaris_part.py \
     --source-archive crash-work/assets/yaris-vehicle/source_model.zip \
     --part-id 2000157 \
     --output crash-work/reports/yaris-part-2000157-declarations-repeat.json
```

Synthetic tests live in `tests/test_source_part_declarations.py`; use
`python3 -m unittest discover -s robo-dyna/tests -p test_source_part_declarations.py`
under the same CPU guard. No original assets are needed by those tests.

E2a passed bounded CPU qualification in `canonical_geometry.py` and
`shell_mass.py`. `compile_archive_readiness` composes these
with the existing E1 declaration compiler. The default declaration-only API,
CLI and schema remain unchanged. Opt in by adding both `--canonical-assets`
and `--quadrature` to `tools/compile_yaris_part.py`.

The geometry loader caps the selection at 256 source shells / 512 nodes and
aggregate canonical arrays at 64 MiB. It hashes the existing arrays and archive,
checks compact indices against source IDs, then cross-checks selected values,
physical source lines, repeated triangle slots and blank masks against the
pinned original member. Omitting a triangle from self-consistently rehashed
arrays still fails complete source-part coverage. It applies no transform.

Mass is explicitly a uniform `rho*t*dA` midsurface lamina proxy. Native T3
moments are exact; Q4 uses the actual bilinear map, a sufficient conditioned
Jacobian certificate over the entire parameter square, and separate 4/8/16
integrations. Both diagonal areas remain descriptive diagnostics. Each element
and the aggregate must meet the fixed `1e-9` last-refinement budget. Two local
passes preserve central moments even for a tiny element far from the common
part anchor; aggregate inertia uses positive parallel-axis additions. Nonuniform
or blank thickness, malformed connectivity, nonfinite/degenerate/ill-conditioned
geometry and nonconvergence prevent a successful report.

The actual Chrono quadrature donor passed orders 4/8 and failed order 16 at the
predeclared `2e-12` independent Legendre residual gate. That failed run remains
under `crash-work/reports/shell-quadrature-tests-1.*`. The accepted export
uses existing Boost 1.74 for all three orders, retaining the measured Chrono
failure and source/license hashes. No new root solver or relaxed budget was
introduced. `load_quadrature` independently rechecks roots and polynomial
moments; no rule is inferred from a claimed qualification flag alone.

Passing tests are `tests/test_canonical_part_geometry.py` (eight synthetic source
cases), `tests/test_shell_surface_mass.py` (15 numerical, failure and end-to-end
composer cases), and six actual C++ quadrature/export cases. The mass suite
requires `--quadrature PATH` or the environment variable
`ROBO_DYNA_SURFACE_QUADRATURE`; absence fails instead of skipping the gate.
Root-coordinated CPU execution uses the existing shared guard, one CPU and at
most 1 GiB RSS. No solver or CUDA context is required by this layer.

The original PID 2000157 audit retained **88 Q4 + six native T3 / 117 nodes**
without flattening or transforming geometry. Its area is
**0.019729915620722037 m²** and provisional lamina mass is
**0.25654256843987483 kg**, with relative 8-to-16 mass refinement
`4.327636662316166e-16`. The [typed readiness report](../../crash-work/reports/yaris-part-2000157-readiness-1.json)
and [immutable evidence checkpoint](../../crash-work/reports/e2a-readiness-checkpoint-1.json)
retain full COM/inertia, source-card/array coverage, rule provenance, test
reports and source/build hashes. The
[retained-file supplement](../../crash-work/checkpoints/e2a-readiness-1/manifest.json)
preserves the exact mutable source/build entries. Reproduce with a new output path under the
same guard:

```bash
python3 robo-dyna/tools/compile_yaris_part.py \
  --source-archive crash-work/assets/yaris-vehicle/source_model.zip \
  --canonical-assets crash-work/assets/yaris-vehicle \
  --quadrature crash-work/reports/shell-surface-quadrature-2.json \
  --part-id 2000157 \
  --output crash-work/reports/yaris-part-2000157-readiness-repeat.json
```

E2a retains known source attachment inventory separately: six nodal-rigid groups
touch 20 selected nodes and 56 external nodes across five neighboring parts;
tied-contact master-set membership remains an unresolved pairing scope. These
are pinned audit facts in the v1 report, not implemented constraints. Full closure,
added masses, source mass/lumping/rotary equivalence and all mechanics remain
blocked; successful readiness reports still say `simulation_ready=false`.
See [the detailed source-part gate](../../planning/YARIS_SOURCE_PART_READINESS.md).

E2b's first typed inventory passed 15 new cases plus the unchanged 35 E1/E2a
cases. Add `--typed-attachments` to
the E2a invocation to request the explicit v2 envelope. The v1 report and its
historical inventory remain nested unchanged; a separate typed scope records
literal list sets, complete nodal-rigid cards, tied part-set candidates and
source-verified one-hop incidence. Optional/default constraint mechanics,
unreferenced general/additive sets, all other attachment families and actual
tie pairing remain unresolved. Referenced unknown operators fail. This path
does not load neighboring part geometry, expand source transforms or change
mechanics capacities. Tests are in `tests/test_source_attachments.py` and take
the same qualified quadrature argument/environment setting as the mass suite.

The [original v2 report](../../crash-work/reports/yaris-part-2000157-attachments-1.json)
verified all six groups, 76 members (20 selected / 56 external), five neighbor
PIDs, and source/canonical coverage of 173 union nodes / 182 incident elements.
It passed in 5.266 s using one CPU and 102.41 MiB sampled peak RSS. The report
uses 1,031,631 of the existing 1,048,576-byte cap; broader inventories must not
silently overrun this format. The
[retained checkpoint](../../crash-work/checkpoints/e2b-attachments-1/manifest.json)
binds the source, tests and guard reports. Subsequent independent source review
found no blocker within the literal one-hop scope and reverified all retained
hashes. No GPU, source mechanics or attachment load transfer was executed.
