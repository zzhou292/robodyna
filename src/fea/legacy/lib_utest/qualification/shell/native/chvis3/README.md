# Exact native CHVIS3 qualification oracle

This small library compiles the original OpenRadioss `CHVIS3`, `constant_mod`,
`element_mod`, and include files from revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`. Their arithmetic and notices are
unchanged. `original/source-manifest.json` records official paths, git blob IDs,
SHA256 values and retrieval provenance. `verify_sources.py` pins that manifest
and checks all 21 original files before configuration and each library build.
The original AGPLv3-or-later notice and complete license remain in
`original/LICENSE.md`. The historical staging utility remains outside this maintained copy; all
required originals are present and builds perform no source fetches.

`NativeHourglass.F` is a separate fixture context adapter using the **same exact
COMMON includes** as CHVIS3. No COMMON layout was recreated by hand, and no
generated numerical variant exists. `HourglassNative.cpp` validates the public
inputs, serializes calls to that native context, and publishes completed finite
outputs. `HourglassNative.h` specifies the C ABI used by the CUDA comparison
fixture. This private numerical oracle does not add a second production solver
or bypass TL-FEA's modular element/material/timestepper boundaries.

The admitted oracle scope is at most 16 active planar Q4s, ISMSTR=1 or 2,
IHBE=1, NPT=3, with supplied local geometry and synthetic material/stabilization
parameters. Source CHVIS3 does not take NPT; its selection for the NPT3/IHBE1
case comes from the inspected caller. Geometry validity and the distinction
between the uniform and corrected modes remain the caller's responsibility.
Material evolution, physical stress resultants, plastic hourglass caps,
activity/deletion, mesh contact, and timestep stiffness are excluded.

The source context is explicit:

- `CPP_mach=CPP_linux964` selects the original MVSIZ=129 and lock type. The
  original r8 `my_real.inc` defines binary64; the original `NIXC=7` module
  declaration is used. Native arrays with MVSIZ dimensions are padded; active
  iteration is JFT=1..n. HOUR has its actual native leading dimension passed as
  NEL=MVSIZ, then is packed/unpacked to the public component-major n stride.
- HVISC, HVLIN and HELAS are the three public batch controls. NODADT, IDT1SH,
  IDTMINS and NADMESH are zero, excluding the native stiffness branch.
- OFF=1 is the active material multiplier; it matches the CUDA K3 clipped
  active multiplier after K1's initialized OFF=2. This does not qualify the
  production OFF/state-transition protocol.
- NPSAV=8, one private reporting part per active element, NUMELS=NFT=0, and
  zeroed PARTSAV/EANI/EHOUR give explicit bounded reporting arrays. Their three
  independent native work accumulations must agree. They are fixture storage,
  not guessed default engine initialization.
- All original COMMON symbols actually read by this admitted branch are set
  on each call. Unused COMMON storage is retained from the exact includes.
  The library mutex prevents concurrent calls from racing on that storage.

Public arrays use `array[component*n+element]`. Velocities, angular velocities,
forces and couples have 12 planes in node0 xyz through node3 xyz order. HOUR
has five planes, and the field enum defines the 20 geometry/material planes.
Each output needs the documented capacity and must be disjoint from other
outputs and borrowed inputs. The public shim checks null pointers, finite
values, physical scalar domains and batch/mode bounds; as a C pointer API it
cannot prove the caller's allocated capacity. It stages all native outputs
before publication, leaving caller outputs unchanged on detected failure.

Native H/B are positive internal terms. The public force/couple outputs are
their negatives, matching K3's restoring RHS assembly; the fourth translation
node is reconstructed by equilibrium as in K3. Work is the native per-element
increment. On this active scope it equals minus dt times restoring nodal work.
HOUR1..3 are accumulated force histories; HOUR4..5 are overwritten rotational
moments. The work uses the updated endpoint force, so it is not generally the
change of recoverable elastic energy. Elastic unloading can have negative work.
The rotational quadratic coefficient contains HELAS*H3 and the exact native
constant 0.072169; it must not be relabeled as an HVISC-only term or replaced by
a nearby irrational constant.

Root supplies the isolated GNU Fortran compiler, configures/builds this target,
and runs tests under the shared workstation guard. The library target is
`chvis3_native`; it can be linked from the new CUDA fixture by `add_subdirectory`.
There is no automatic compiler installation or source download during CMake.
The CTest `chvis3_native_analytic` invokes `test_native.py` and writes
`native-tests.json` in its build directory. The six CPU tests cover 80 isolated
modal parameter/sign subcases, retained history/zero response, load/unload/reload,
two distinct element strides, corrected affine preservation and its difference
from the uniform branch, and failure/retry publication. Coefficient reductions
are source-derived; null-mode geometry, resultant balance, nodal work and affine
invariants provide separate independent checks.

Source staging is complete. Build and runtime acceptance are recorded by the
parent's resource/test reports; this source preparation alone is not a passing
S2b or vehicle-simulation gate. The frozen S1/S2a fixture is unchanged.

This directory is a byte-preserving relocation of the native source, C ABI,
context adapter, verifier, tests and CMake into TL-FEA qualification sources.
Only this README changes. Parent CMake enables it explicitly; default operator
builds do not require Fortran. See [the qualification README](../../README.md).
The separately staged phase oracle can reuse this exact original hierarchy and
Fortran module directory. If both native wrappers share a process, their COMMON
symbols require serial coordination across wrappers; separate per-wrapper
mutexes are not a global concurrent-call guarantee.
