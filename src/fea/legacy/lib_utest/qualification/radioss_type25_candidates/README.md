# Native candidate qualification

The owning CMake project tests the production candidate values and numerical GPU
inventory. Enable TYPE25_CANDIDATE_CUDA for actual hardware tests; GNU C++/Fortran
and NVIDIA CUDA preserve the pinned precision controls. Native code is exclusively
an independent test dependency. Host and CUDA consumer targets link production
without the native oracle. Bazel supplies the header gate and production runtime
build; numerical native/hardware gates remain in this CMake project.

native/original retains untouched I25PEN3, I25COR3T and I25TRIVOX from OpenRadioss
a62b27e6. prepare.py checks hashes, extracts exact TRIVOX local bound/pair/screen
chunks and wraps whole PEN3/COR3T. COR3T has an explicit Fortran module interface;
its fixture COMMON DT1 is serial-only. Zero remote module arrays are unreachable
in the admitted local wrapper. No wrapper reads unused STIF/ITYP. The independent
inventory oracle exhaustively enumerates supplied source roles, own/removal/domain
exclusions, then invokes native screen/COR3T/PEN3. It does not execute the whole
native voxel traversal or produce original removal lists.

Cases retain the earlier2064-case T3 cohort-composition experiment, then add7,232
Q4/type/symmetry rows. The production host gate checks all9,296 PEN3 results across
four native cohort modes; actual CUDA checks the same rows. Whole COR3T gap bits
and symmetry agree on9,296 moving rows, with all32,768 five-node ICODT combinations
additionally checked on host. The original type25_pen3_cohort_probe executable
remains an experiment, not the acceptance test for this production module.

Inventory tests cover complete mixed native membership, source occurrence order,
secondary CSR, multiple scene updates, dense tasks, exact task/pair capacity,
inactive/domain-clipped NaNs, direct SI views, empty roles/activity, source/arena
alias checks, stale/discard/retry lifetime, a preserved separate accepted owner,
and CUDA poison without CPU retry. No force, history, native scheduling or physical
publication parity follows from these candidate-only gates.
