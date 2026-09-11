# LAW90 solid18 reference and current total-strain values

Pure selected LAW90/solid18/engine17, IINT2, ICP0, ISMSTR10, JCVT1 values.
This is not material force/history, source admission or a resident participant.
The qualified LAW90 point and old LAW36 adhesive public contracts are unchanged.
The actual source SDI KCON/default export gate remains a separate obligation.

The reference retains G_JAC_I10, eight PIJ72 and SGSAVINI's original-source
21 world coordinate differences. SGSAVINI precedes the later H8 orientation
correction. The native constructor uses the original A_GAUSS/W_GAUSS tables,
full selected ELBUF_INI size branches, actual zero-length local JAC allocation,
full S8ZJAC_IC/S8ZPIJ_IC/S8ZJAC_I3/S8E_PIJ, and the qualified native
SVALUE0/SMASS3 producer. SMASS3 gives equal source-slot mass values after
integrated-density reduction; the eight Gauss volumes generally differ.
No rotational J is assigned. All 72 coefficients are compared, including the
startup nu0 BIJ and local-to-global phases. Runtime ISELECT1 reads first24.

Current geometry uses exact native selected ISELECT1/ISEL_V0, immutable saved
world displacement, full S8EDEFOT3/SORDEFT3 and S8ESELECSHT. It retains separate
F-I, B-I tensor shear and corrected engineering-rate observations. The native
S8EDEFO3 gate asserts VOL/VOL0DP/EINT unchanged and SDV/spin zero for this branch.
There is no FAC selection, excluded BIJ addition, or synthetic material history.
Native starter/engine symbols and COMMON blocks have distinct private names;
real leading dimensions remain 512 for starter and the selected engine MVSIZ.
Complete enclosing sources and copied/reused source paths are SHA/Git-blob
pinned. Thirty byte-identical donors (1,071,068 bytes) are referenced from
existing qualification owners instead of copied again. Context modules supply only selected storage and fail on excluded calls.
The oracle does not consume production reference coefficients or derivatives.

Host-only convenience entry points stage locally. Host/device scratch entry
points require caller-owned disjoint scratch and expose only their status plus
unpublished staged result. A caller publishes after success; failed scratch is
not an accepted result. Measured binary64 sizes: ReferenceCoefficients 4,856 B,
Reference 6,392 B, ReferenceScratch 16,384 B, Kinematics 11,000 B and
KinematicsScratch 12,488 B. The CUDA fixture keeps both scratches in explicit
device packets; no global stack-limit request or hidden per-thread large packet.
These are pure value sizes, not a resident memory forecast.

The original fixture is read from `yaris-radiator-geometry-1` at build time:
manifest SHA256 `c5adae0a61384bb7b57c88ed113798cf5b0fbaa8758fd48f93e15cc61f65f94a`.
All 12 bounded little-endian arrays, exact source/local eight-slot joins and
raw-mm-to-SI bits are checked before build-only hex-float arrays are generated.
It retains all 1,345 original PID2000063 cells and 2,904 nodes. The manifest
records the source archive hash but explicitly does not claim the exporter
opened the archive container; root separately verified source composition.
No durable duplicate generated population header or new source parser exists.

Root qualification:

```sh
cmake -S lib_utest/qualification/law90_solid18_reference -B BUILD \
  -DTL_LAW90_SOLID18_NATIVE=ON -DTL_LAW90_SOLID18_CUDA=ON \
  -DTL_LAW90_RADIATOR_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-radiator-geometry-1 \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD -j2
ctest --test-dir BUILD --output-on-failure
```

Selectors: `law90_solid18_reference_host` (6 functions),
`law90_solid18_reference_native` (4),
`law90_solid18_reference_sourcenative` (3 original full-population functions),
`law90_solid18_reference_cuda` (2), and three identities
`law90_solid18_owning_identity`, `law90_solid18_native_identity`,
`law90_solid18_original_identity`; imported `solid18_reference_native_identity`
remains separately registered. `law90_solid18_fixture_reader` checks bounded
roundtrip, source-byte/size/manifest rejection and unchanged output on failure.
Native/CUDA/all-original numerical tests are
root-owned and pending at the author freeze.

Affected owning gates: existing `solid18_reference`, `solid18_force`,
`solid_force_startup` and `solid_resident` constructor/scratch checks when
compiling shared geometry/rate helpers. No material leaf changed.
Bazel targets: `//lib_src/elements/solid18/total_strain:reference_kinematics`,
`//lib_utest/qualification/law90_solid18_reference:host_check`,
`//lib_src/elements/solid18:reference`, `//lib_src/elements/solid18:force`,
`//lib_utest/qualification/solid18_reference:solid18_reference_host` and
`//lib_utest/qualification/solid18_force:solid18_force_host`.

Author checks: 5 new host and 4 affected force functions pass, plus the isolated
shared reference extraction's 4 existing functions. CMake host build, native
C++/CUDA-shaped host syntax and fixture/native source preparation pass within
1 CPU/512 MiB. The first host analytical test incorrectly assumed identity cube
axes; its preserved failed report is superseded by the source-supported native
Y/Z/X frame expectations. No production equation or tolerance was changed for
that fixture correction. No native/GPU execution is claimed by author checks.

Working-unit higher-mode follow-up: root tests2 passed all four native tiny,
both CUDA and all 2,690 original current packets after correcting two wrapper
frame mappings. No production math changed. The remaining original reference
working-mm comparison first failed at EID2191748, index51 (higher_mode[3].x):
actual 3.7499673921637111e-08 m, converted native 3.749967424937495e-08 m,
difference -3.2773783930456593e-16 m, old bound 2.2934403987715874e-16 m,
higher-mode group scale 0.00012441679077936085 m. Same-unit SI/native passed.

`WorkingModeBound.h` applies only to the twelve higher modes when comparing
working-unit outputs converted to SI. Each is seven signed additions of local
coordinates. Let u=epsilon(binary64)/2, gamma7=7u/(1-7u), a=SI local coordinates,
b=individually converted working local coordinates, and S the exact signed sum.
The forward bound is |S(a)-S(b)| + gamma7*sum|a| + (gamma7+u)/(1-u)*sum|b|
+ u/(1-u)*|b_mode|, plus the independently computed long-double witness rounding.
The terms cover native addition, individual coordinate conversion and final
mode conversion separately. Inputs are finite normal original-model values;
this is an arithmetic roundoff witness, not a fitted material/geometry tolerance.
All checked local coordinates and every other packet group retain their prior
comparators; same-unit checks are unchanged. A new host function passes 384
cancellation cases and perturbation negatives. The new original-source function
records the exact native witness and checks all twelve mode perturbations plus
density rejection; its execution is root-owned.
