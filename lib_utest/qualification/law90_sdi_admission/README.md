# Native MAT057 → LAW90 SDI admission observation

This qualification-only executable links all five unchanged native reader core
libraries and the complete native `PrintOption.cpp` writer at
`a62b27e6baa555d222a580d6218867d0be4d70b5`. The harness supplies only the writer's
current-model lookup. It invokes real `DynakeyReadModel`, `DynaToRad::CallConvert`,
`PrintEntity` with its native `FF_D00_2026`, and `RadiossblkReadModel`. No handwritten
LAW90 export, mock reader, app source admission, solver or new production dependency
is provided.

Three tiny cases retain the authenticated original PART/SECTION/MAT/curve blocks.
The baseline preserves every blank HU/SHAPE/DAMP/FAIL field. The two controls alter
only HU to1 or source DAMP to.2, respectively. The explicit `*CONTROL_UNITS` wrapper
matches canonical mm/s/tonne metadata; it is labeled test context, not an original
source card. No geometry or full original source parser is needed.

Every direct and exported/re-read field retains availability, effective native
missing-field zero, binary64 bits, and length/mass/time dimension exponents. KCON's
source stiffness descriptor and target pressure descriptor are different; copying
raw20000 is not a physical stiffness conversion. The target pressure interpreted
in mm/s/Mg is20GPa. Unit scaling executes exact HM_GET_FLOATV509:513 (complete
source authenticated), in mass/length/time order, with explicit canonical factors.
This is its scalar conversion region, not an invocation of the complete Starter
unit-table importer. Missing-field zeros match `cpp_get_floatv.cpp` / native SDI
bridge; they are never presented as stored field values.

The output feeds the existing independent HM_READ_MAT90/LAW90_UPD oracle. It uses
native raw curve ordinates and dimensioned Fscale, preserving the reader's actual
scale/subtraction ordering. It records all33 prepared values, including the
pre-default Hys classifier, single-curve Ismooth/ISRATE0 and10/3 history/cursors.
The checker does **not** force blank HU to1. `result.json` explicitly distinguishes
whether each actual path matches the currently qualified positive-Hys/IFLAG2
material profile. A successful observation test with that flag false leaves
original source admission open. The explicit HU1 control must yieldIFLAG2; changing
source DAMP must leave every target/preparation value unchanged.

## Dependencies and immutable receipts

`openradioss-law90-sdi-1/reader-source-manifest.json` pins3,118 complete official
reader/CFG/message/license files (39,503,459 B), each with Git blob and SHA256.
The complete enclosing tree pin is above. `external-source-manifest.json` records
the official v70 ZIP selected by pinned EXTLIB_VERSION, SHA256
`93a6301baf9a116ac3ba10dc212b5c38dd3e6a88319d1d6aecbec8d68f3c449b`,45,913,558 B.
Only ExprTk/header/license/readme/version records are extracted (1,890,229 B).
The CMake qualifier deliberately uses installed Boost1.74 and records BOOST_VERSION
in each observation; this differs from the native CMake's unused Boost1.70 path.
There is no claim of an executed stock OpenRadioss distribution.

CMake authenticates and stages its own copies, then reuses all five native module
CMake definitions. Native sources/CFG and force/preparation production are unchanged.
Full source identity, original fixture identity and host tests were author-only;
all C/C++ native library compilation and Fortran execution belong to root.

## Root gate

From the workspace root, inside the normal shared heavy guard:

```sh
cmake -S crash-work/worktrees/law90-sdi-admission/lib_utest/qualification/law90_sdi_admission \
  -B crash-work/build/law90-sdi-root-1 -DCMAKE_BUILD_TYPE=Release \
  -DTL_LAW90_SDI_CACHE="$PWD/crash-work/deps/openradioss-law90-sdi-1" \
  -DTL_LAW90_SDI_UNITS="$PWD/crash-work/deps/openradioss-type45-joints-1/original/starter/source/devtools/hm_reader/hm_get_floatv.F" \
  -DTL_LAW90_RADIATOR_FIXTURE="$PWD/crash-work/reports/yaris-radiator-geometry-1"
cmake --build crash-work/build/law90-sdi-root-1 -j4
ctest --test-dir crash-work/build/law90-sdi-root-1 --output-on-failure
```

Owning CTest names: `law90_sdi_source_identity`, `law90_sdi_direct_export_read`,
reused `law90_native_identity`. The source fixture's four host controls run with
`python3 -B FixtureTest.py` in this directory. This external reader integration is
an opt-in CMake qualifier, not a Bazel production dependency. No CUDA gate applies.

The native test creates `observations/` once, keeping each exact `.rad`, `.json`,
and `.log`, plus aggregate `result.json`. Preserve/move this directory before a
rerun; never silently overwrite earlier failed native evidence. Failure before a
complete result leaves the direct logs/exports for diagnosis and makes CTest fail.
