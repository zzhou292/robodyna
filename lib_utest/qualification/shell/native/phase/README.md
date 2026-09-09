# Native shell phase qualification oracle

This small CPU library runs unchanged pinned `CCOOR3`, `CDERI3`, `CCOEF3`,
`CDEFO3`, `CCURV3`, and `CSTRA3` arithmetic with exact original includes.
The native/chvis3 target supplies its already verified `constant_mod`,
`element_mod`, and remaining include files. `original/source-manifest.json`
records exact bytes, SHA256, git blob IDs, source URLs, and retrieval provenance;
`verify_sources.py` pins that manifest and checks every retained original file.
Original notices and `original/LICENSE.md` are retained. The wrapper is new
qualification code; no upstream arithmetic or COMMON layout is rewritten.

The admitted scope is one or two active, convex, planar Q4 elements with a
caller-supplied proper orthonormal frame, `ISMSTR=1/2`, `ITHK=0/1`, `IHBE=1`,
`ISHFRAM=0`, `ISTRAIN=1`, `IEPSDOT=0`, `NPT=3`, and the ordinary explicit
`IMPL_S=0` branch. Section shear selection is `ISH=0`, so `FSH` is constant.
The wrapper executes each element separately because native CCOEF3 assumes
one material per vector batch. This allows distinct element materials without
changing the source. No material update, frame builder, inertia, director
integration, hourglass force, or production solver is implemented here.

`ShellPhaseNative.h` defines the C ABI and all array conventions. Inputs are
borrowed for the duration of the synchronous call. Accepted OFF/SMSTR/GSTR are
read-only; complete trial outputs are staged before publication. The caller
owns commit/discard and all longer-lived history. Native GSTR ordering is
`xx, yy, engineering xy, yz, xz, kxx, kyy, kxy`; the two transverse entries
must not be swapped when comparing K2. The diagnostic normal angular-velocity
projection is adapter code; tangential projections come from native CCURV3.

The phase accepts current positions and the carried velocities over the
completed interval, with positive completed `dt` passed to native `DT1`.
Prescribed trajectories must choose their staggering explicitly. In particular,
the secant velocity `(X_n-X_{n-1})/dt` reproduces the inspected ordinary native
position update. It differs from instantaneous `omega cross X_n`. Retained
finite-step rigid-rotation strain is measured and convergence tested; finite
rotation objectivity is not assumed from a single zero-velocity test.

Initialize the reference with a zero-velocity call at X0 before a deformed
ISMSTR1 step. Native CDERI3 turns OFF1 into OFF2 and retains six local reference
coordinates; ISMSTR2 refreshes OFF1 geometry, but inherited OFF2 still freezes
it. CCOOR3 clips the accepted activity to one before CDERI3 changes OFFG.
THK0 is the section thickness for ITHK0, and current physical thickness for
ITHK1. The wrapper does not update physical thickness itself.

Input rejection includes nonfinite values, nonproper frames, nonplanarity,
nonconvex/degenerate current or frozen geometry, and near-zero denominators
in the selected native quarter-step correction. Caller buffers must have the
documented sizes and nonaliasing lifetime; raw C pointers cannot prove these.
Invalid input or a detected invalid native result leaves the entire batch
unpublished. The libraries share original Fortran COMMON blocks, so callers
must serialize phase and CHVIS3 calls with each other. Each wrapper's private
mutex only serializes calls to that one API; these are test-only libraries.

CNVEC3 uses `ELBUFDEF_MOD` and unconditionally calls CORTDIR3 for material
direction handling. The supplied-frame `shell_phase_native` target does not
link that closure. The distinct current-frame target below now preserves it
in full; no callee is stubbed.
An independently chosen analytical frame can test rigid affine fields; the
CUDA K1 frame can also be supplied for a cross-implementation phase comparison.
The latter does not independently validate K1 frame construction.

The opt-in surrounding qualification CMake enables native targets and adds
this subdirectory. `shell_phase_native_analytic` invokes the standard-library
Python/ctypes tests and writes `phase-native-tests.json` in the build directory.
Tests and their runtime results remain separate: staging this source alone is
not a qualification pass. The root coordinator owns bounded builds and runs.

## Separate exact current-frame context

`shell_frame_native` exposes `crash_shell_frame_native` in
`ShellFrameNative.h`. It computes at most two current frames by compiling full
unchanged CNVEC3 and CORTDIR3, with exact `ELBUFDEF_MOD` and `precision_mod`.
`original/frame-source-manifest.json` pins these four additional source files;
the original eleven-file phase manifest is unchanged. The verifier checks both.
All frame Fortran sources define `MYREAL8`, and the adapter checks `WP` and the
native constant kind against `C_DOUBLE` before entering the original routines.

The context fixes `ISHFRAM=0`, `IREP=0`, `IDRAPE=0`, `IGTYP=1`, and one layer.
The exact CORTDIR3 body reads IDRAPE and then has only IREP1..4 branches; for
the admitted case it accesses no layer pointers and performs no writes.
Consequently the wrapper can use the real native ELBUF type, explicitly set
its read selector, null unused layer pointers, and check complete direction
buffers against sentinels after the unchanged call. This is an evidenced
inactive branch, not an invented COMMON/type layout or a no-op replacement.

Input geometry is checked independently using scaled edges for finite values,
planarity and convexity. Postconditions require a finite proper orthonormal
frame and unchanged direction buffers. Both elements' outputs remain unchanged
on any failure, including nonfinite arithmetic inside the donor. Calls to
frame, phase and CHVIS3 remain externally serialized across their shared COMMON
state. The original phase API still accepts a supplied frame; neither API
silently changes that contract or evolves a director/history state.

`shell_frame_native_analytic` runs seven tiny standard-library CPU tests. They
compare the symmetric frame with an independent angular-bisector construction,
proper rotations, distorted polygons, scale/translation covariance and distinct
batch permutations; exercise invalid geometry and failure/retry publication;
and compose the actual native frame with the existing native phase oracle.
The finite rigid secant path retains the earlier source-derived residual as a
strict diagnostic. A frame-context pass does not qualify production objectivity,
prestress transport, native startup, or a Yaris formulation mapping.
The JSON report explicitly records `production_objectivity_qualified=false`,
`native_startup_qualified=false`, and `yaris_formulation_mapping_qualified=false`
independently of whether these seven context tests pass.
