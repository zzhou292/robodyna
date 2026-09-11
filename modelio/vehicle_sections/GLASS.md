`VehicleSectionResolution` V2 is an explicit source-resolution extension for the
original MAT123 glass declarations. Generate it with the existing
`tools/compile_vehicle_section_resolution.py --include-glass` command. Without
that flag the compiler retains V1 output; the historical `VehicleSourcePlan`,
raw source, canonical arrays and strict V1/V2/V3 component readers stay unchanged.

V2 retains all original rows and the V1 constant-failure declaration inventory.
Its separate `glass_declarations` object has schema
`robo-dyna.vehicle-glass-declarations.v1` and policy
`original_mat123_numint1_native_tab1_placement_v1`. Every glass row retains its
complete original PART, SECTION and material blocks/cards, with typed physical
coefficients and separate source NUMINT/NLOC and resolved native policy. The
reader checks exact source associations, card field names, blank flags, numerical
bits, unit conversion, ordering and coverage before publishing an immutable handle.

The selected MAT123 branch has explicit LCSS0/LCSR0, blank inline points,
C/P/VP/EPSTHIN/EPSMAJ, positive FAIL, analytic SIGY/ETAN and NUMINT1. Pinned
OpenRadioss `a62b27e6baa555d222a580d6218867d0be4d70b5` converts this to LAW44 and
constant TAB1. The named application policy preserves original NUMINT1 through
the qualified native AnyPoint/IFAIL_SH1 caller instead of the converter's
hardcoded IFAIL_SH2. It resolves blank C/P/VP to FilteredZeroC, C0/P1/VP2 and
10000/s filtering; it does not write those defaults into original cards. The
generic material accessor exposes physical/analytic coefficients and cards;
its unused generic rate fields are canonical zeros. `native_material()` supplies
the explicit qualified rate parameters. `native_parent()` supplies the distinct
TAB1 policy and (-.3,0,.3)/FAIL table at complete original family indices.

Source NLOC -1/0/+1 resolves to Bottom/Centered/TopReferencePlane. Blank NLOC
remains a distinct source declaration with centered native placement. The
separately named policy records native IPOS4/0/3, including that the original
SECTION_SHELL converter does not itself read NLOC. Placement is held per part
and passed into the existing native reference input identity. Original nodes
are copied unchanged. Native QEPH offset inertia and unchanged TYPE1 T3 inertia
come from the existing TL startup producer; no mass is redistributed or summed.
Optional contact projection is explicitly false.

The expected original increment is 9 parts / 14,210 shells, including 8,502
noncentered windshield shells (PIDs 2000023 and 2000523). All 349,645 parents
remain: 340,292 declarations become available for native reference assessment,
with 9,353 still unresolved. These are expected source counts until the owning
original-source gate records its result. Neither source resolution nor a valid
local reference admits a complete runtime owner, contact eligibility, rigid
part, one-point midlayer, QBAT formulation or connected vehicle load path.

The unchanged 4 MiB sidecar / 512 MiB source and 768 MiB resolved-reference
profiles preflight all additions. The sidecar's 16x parse/storage allowance
covers its added typed blocks; complete parent/native inputs and per-part values
remain explicitly charged. The table cap covers constant-failure tables plus
the conservatively per-part glass material/section rows. The shared canonical
backing is retained once. Forecasts describe payload allowances, not allocator
overhead or process RSS. Reads and native-reference preparation publish only
complete immutable results, so failed attempts retain prior handles.

Small author gates are five new Python tests, three C++ glass field tests and
the existing resolution field/Python tests. They cover original SI rounding,
three placement signs, native parameter preparation, raw blank/NUMINT/NLOC
identity, malformed/late fields and retry. Full-source tests are authored but
run only in the root's serialized lane:

- Configure `modelio/vehicle_sections` with
  `ROBO_DYNA_VEHICLE_GLASS_ACTUAL_TESTS=ON`; target
  `robo_dyna_vehicle_glass_source_check`, CTest `vehicle_glass_source`.
- Configure `case/vehicle_startup` with
  `ROBO_DYNA_VEHICLE_REFERENCE_GLASS_TESTS=ON`; target
  `robo_dyna_vehicle_glass_reference_check`, CTest `vehicle_glass_reference`.
- Both require explicit `ROBO_DYNA_VEHICLE_{CANONICAL,SCOPE,DECLARATIONS,RESOLUTION,
  GLASS_RESOLUTION}` paths and `ROBO_DYNA_VEHICLE_GLASS_SHA256` for the create-only
  V2 artifact. `RESOLUTION` is the prior frozen V1 output2. Use the existing
  `Chrono_DIR`, `ROBO_DYNA_TL_ROOT` and full-shell tests-off configuration.

The original reference test compares every prior supported reference field,
checks every new source input and native result, and reports the full forecast
before construction. It retains two assessments for parity, so its process cap
must cover that test total. No native Fortran, CUDA or solver is linked here.
