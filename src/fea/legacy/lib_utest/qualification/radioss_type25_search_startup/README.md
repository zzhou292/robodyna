# Search startup qualification

The owning target is C++ plus independently extracted native Fortran. It retains
complete original I7REMNODE_INIT, get_list_remnode, I25REMNOR and margin_reduction
routines; original I25BUC_VOX1 margin, machine.inc/default-reader multiplier,
I25XSAVE extent and initial ICONT allocation blocks are mechanically copied.
Selected INACTI5's no-write flag branch is source pinned. No production numerical
helper supplies expected values. Qualification allocator wrappers only own local
scratch and a checked worst-case G*S native output allocation.

Native preprocessing is explicitly one-thread: OpenMP is not enabled. The
serial scratch behavior is meaningful and is tested with mixed expanding and
nonexpanding rows. InvariantNoExpansion has a separate exact bound and rejection
test; it does not claim the original mixed-worker algorithm is deterministic.
The reference's limits are4096 nodes/secondaries and512 expanded mains; these
bounds do not raise or redefine production capacities.

Twelve host groups cover full fields/bit patterns, nonempty and mixed Q4/T3
removal order, inversion, inclusive distance thresholds, expanded G2/G4 margin
branch, REAL4/default node-count thresholds, ICURV branch/SI lengths, complete
cap failure/retry, malformed/unknown contributor policy, aliases/overflow and a
native nonprogress case. The latter deliberately does not execute the native
infinite loop. A restored-activity control compares to the original oracle.
The actual cycle0 wall fixture compares its produced margin and removal/extent
values, using captured coefficient/gap inputs as qualification evidence only.
It does not use the later cycle216 transient normal fields as startup truth.

The CXX-only production consumer inherits precise flags and links no Fortran,
GTest or CUDA. There is no artificial GPU test: this is the actual host startup
path. CMake owns the numerical host target and three source checks; Bazel owns
consumer/source checks, not a substitute native numerical gate.

Run only in the assigned bounded lane, with a fresh build directory:

```sh
cmake -S lib_utest/qualification/radioss_type25_search_startup -B BUILD_DIR \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local
cmake --build BUILD_DIR -j4
ctest --test-dir BUILD_DIR --output-on-failure
```

Source authoring/generation is not a numerical pass. Matched accepted-step
mechanics and end-to-end performance require separate physical composition.
