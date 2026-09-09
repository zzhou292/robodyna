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
has `simulation_ready=false`; attachments and geometry/mass readiness are
separate unfinished gates.

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

The next separate gate must join the source geometry and attachment closure to
an independent mass/center-of-mass/inertia ledger. Native T3s, warpage, plasticity,
controls, contact and coupled dynamics still require explicit qualification.
