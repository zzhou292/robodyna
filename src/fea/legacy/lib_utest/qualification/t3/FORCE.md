# T3 prescribed force/history port contract

Source implementation staged; **no force-port build or numerical execution is
claimed**. The force stage follows the reviewed
[design](../../../../../planning/T3_FORCE_PORT_IMPLEMENTATION.md), retained
native R3 (`f605e70`) and passed startup/rates (`f5ceb29`). All tests, arithmetic
and budgets below are fixed before the first force-stage execution.

`elements/t3/T3Force.h` returns a staged `ForceTrial`: unchanged38 prescribed
kinematics observables,26 proposed native history values,18 positive world
force/couple components and10 diagnostic values. Exact reference-input bits,
base time and next nonoverflowing sample are checked. Caller acceptance is an
explicit value copy. There is no owner, clock, CUDA API, allocation, Fortran
runtime, h=0 force initializer or implicit accepted-history change in production.

## Source ownership and operation order

The donor remains `a62b27e6baa555d222a580d6218867d0be4d70b5`.
`force-source-manifest.json` records original Git blobs, selected ranges,
native wrappers, shared dependencies and the reviewed live build registrations.
`verify_force_sources.py` checks those records and calls the existing startup/
native closure verifier. Prior checkpoints, native originals, numerical R1/R2
sources/tests and the startup README remain unchanged.

| Owning header | Retained source expressions |
| --- | --- |
| `T3History{Data,}.h` | Native T3 history adapter: FOR5/FOR_G5/MOM3/STRA8/THK/EINT2/EPSD/activity plus reference and stamp. |
| `T3Material.h` | PM elastic coefficients via shared LAW1, selected C3COEF3 t²/current-volume/shear-factor/GS/offset. |
| `T3Law1.h` | C3STRA3 raw*(h/A), C3FORC3:553–564 EPSD, selected MULAWGLC old/new work, THK and DM around SIGEPS01G. |
| `T3StiffnessDiagnostics.h` | C3DT3 NODADT1, IGTYP1, native index7 length/STI/STIR/DTEL. |
| `T3ForceProjection.h` | C3SROTO3 IFRAM_OLD1 copy, C3FINT3 force/couple polynomial and C3FCUM3/C3MCUM3 world projection. |
| `T3Force.h` | Same current GeometryWork, material preparation before rates, save kinematics, LAW1, stiffness, projection and final publication. |

T3 reuses its qualified current geometry, quarter-step rates and common fixed
math types. `materials/ShellElasticLaw1.h` is unchanged and supplies only PM
coefficients and centered point stress/MOM arithmetic. No QEPH packing,
stabilization, hourglass, timestep or projection code enters this formulation.
The source branch is ISH3N2/IFRAM_OLD1/IRESP2/IGTYP1/ISMSTR-1/ITHK0/IDRIL0,
NPTTOT0/ISTRAIN1/IEPSDOT0, centered active1 LAW1, GEO37=0/GEO38=5/6/GEO199=0.

History units are FOR/FOR_G/MOM in Pa, STRA first5 dimensionless/last3 in1/m,
THK in m, signed EINT in J, EPSD in1/s. Physical moment per length is t_eff²*MOM.
There is no HOURG or EVIS. Force thickness stays immutable reference t while
reported THK evolves. Old **total** FOR contributes to work before restoring
FOR_G; instantaneous DM affects total XX/YY/XY and vanishes on hold. EPSD uses
base reported THK and is overwritten. Native work is not a potential or a
general nonnegative dissipation measure. Internal loads are positive N/N*m;
complete native C3UPDT3 subtracts them from a seeded nodal RHS.

The public raw T3 order is already XX/YY/XY/YZ/ZX/KXX/KYY/KXY. C3STRA3 uses
`dx=raw*(h/A)`, preserving its factor order, rather than normalized-rate*h or
QEPH's XZ/YZ swap. Kinematics retains pre-strain/pre-C3DT3 values. C3FINT3
keeps native node3 subtraction order and nonuniform transverse-shear couple
terms; C3MCUM3 uses exactly two local components. The selected C3SROTO3 copy
does not invent a history-frame rotation or objective-integration result.

## Resolved constant and intermediate map

All reached constants are MYREAL8. Exact integers0,1,2,3,4 and half/quarter
are representable directly; native ratios retain binary64 division. The
source EP10/EP20 are successive integer powers of10, exact through10²⁰;
the equivalent products used in T3 constants yield the same binary64 values.
The source EP30=EP20*EP10 performs its native rounded product before reciprocal.

| Quantity | Retained expression / frozen binary64 |
| --- | --- |
| EP20 and shared `1e20` | `0x1.5af1d78b58c40p+66`, identical. |
| EM20, shared sound-speed density floor | `1/EP20 = 0x1.79ca10c924223p-67`. |
| EM30, reported-thickness floor | `1/(EP20*EP10) = 0x1.4484bfeebc29fp-100`. |
| ONEP414 | `(1+4/10)+1/100+4/1000 = 0x1.69fbe76c8b439p+0`; never sqrt(2). |
| DM | `ZEP01+FIVEEM3 = 1/100+5/1000 = 0x1.eb851eb851eb8p-7`. |
| THIRD / FOURTH / ONE_OVER_9 / ONE_OVER_12 | `1/3`, `1/4`, `1/9`, `1/12`. |
| FOUR_OVER_3 / FIVE_OVER_6 | `4/3`, `5/6`. |

Static assertions bind shared EM20/EM30 expressions; host tests independently
check the fixed hexadecimal values. Decimal1.414 currently has the same bit
pattern, but source expressions are retained for provenance and evaluation
order. No unsupported raw REAL32 noninteger literal is reached in this branch.

The shared coefficient operations are exactly G=E/(2*(1+nu)), A11=E/(1-nu*nu),
A12=nu*A11 and c=sqrt(E/max(rho,EM20)), then the centered SIGEPS01G additions.
T3 still owns the native `THK08=t*(1/12+0*0)` input and subsequent THK update.
C3COEF3 retains `FAC1=2*(1+nu)*t²`, `D=(5/6)*A+FAC1`, and
`SHF=(5/6)*(1-0+0*FAC1/D)`, GS=G*SHF. This preserves the reached denominator
and rejects unrepresentable FAC1/D instead of simplifying away `0*infinity`.

No new geometric tolerance band is introduced. The already qualified64epsilon
FP64 geometry/Y3/ACOS exclusions remain unchanged. Additional arithmetic
admission requires finite positive t², current t*A, FAC1, D, SHF, GS and
published stiffness/speed/dt; these are explicit representability conditions.
Finite positive subnormal values are not reclassified as normal. Reported
THKN must be finite and>=EM30 before the native MAX expression; no clamp or
repair admits a thinner state. The original `h/max(h*h,EM20)` floor remains
active where selected. All nonfinite proposed histories/forces/work reject
before output publication. These conditions are not a trajectory-stability
bound or native cutoff acceptance parity.

## Fixed tests and numerical budgets

Nine host and three actual CUDA functions are defined. Each backend's full
parity matrix has **870 configurations**:864 from2scales×2shapes×3world poses×
18signed-column locations×2signs×2history states, plus6tiny translated cases
(3poses×2history states). The word signed describes column amplitude, not a
replacement local frame. IDs above2⁵³ are preserved as uint64 values.

Independent host/device physical-mode coverage is64 configurations:
2scales×2shapes×8modes×2signs, with plane-stress/t³ bending, THK, EPSD and
EINT oracles. Additional host functions cover fixed resultants/all18 virtual
power columns, load/hold/reverse/hold, changed current area, cyclic/reversed
orders and covariance, static/drilling/finite-rigid distinction, native
stiffness/scatter and rollback. CUDA additionally runs the four-step nonzero
history sequence, repeated evaluations, changed current area and aliases.

`Power` supplies actual current coordinates to a **test-local copy** passed to
the retained independent rate-map oracle. It never reinitializes the mechanical
reference or edits the native oracle. The oracle's dt0 linearized virtual
variation holds resultants fixed; production h0 remains invalid. An independent
physical calculation complements parity so constitutive-scale floors cannot
alone qualify tiny work signals.

Native/host/device comparisons retain `2e-12*(dimension+abs(native))` with
L=max current node0-relative distance, t reference thickness, E supplied
modulus, c independent sound speed: stress E; MOM E*t/L; STRA1 or1/L; THK t;
EINT/increment membrane E*t*L² and bending E*t³; force E*t*L; couple/STIR
E*t*L²; STI E*t; DTEL L/c; speed c; DM/SHF1; GS E; EPSD1/h. Existing38
kinematics budgets remain unchanged. Covariance uses the prior2e-11 coefficient.
These are dimensional roundoff comparisons, not accuracy at arbitrary tiny loads.

Independent native R3 material, physical moment, fixed-resultant power and
work checks retain `absolute+2e-10*max(abs(actual),abs(expected))`, absolute
2e-11 in the field units and **2e-22J for work**. Stored-EINT differences add
only the frozen `64epsilon*(abs(base_EINT)+abs(proposed_EINT))` allowance.
No numerical tolerance is inferred from a port execution.

Failure coverage includes13 malformed/identity/current-geometry/material-coefficient/
late-thickness/late-force cases, all26 NaN history slots, negative THK/EPSD/activity and clean
retry on both backends. Late-force geometry has maximum edge960m, safely
inside the existing cutoff, so rejection cannot be mistaken for an endpoint
geometry guard. The host also checks signed-zero reference bits; device
history preparation runs the actual HD preparation operation. Aliased base/
output success and late failure are explicit. Failed outputs/base/reference/
interval compare bytes; successful repeated results compare actual fields.

CUDA owns one reusable packet<=8KiB and one thread/block; no per-interval
allocation, device-limit changes or missing-GPU skips. Root records actual
packet bytes, registers/stack, guarded RSS/device use and time separately.
One-thread qualification makes no batching/performance claim.

Owning build option is `TL_T3_ENABLE_FORCE`, defaultOFF, with targets
`t3_force_port_check` and `t3_force_port_cuda_check` (latter requires existing
`TL_T3_ENABLE_CUDA`). `T3Force.cmake` is a fragment of the one existing entry
point. CPU flags stay no-fast-math/ffp-contract-off; CUDA fmadfalse, precise
divide/sqrt, ftzfalse and same host flags. Native R3 remains a private test
dependency. Root serializes all runs and rechecks13 prior port plus22 native
functions along with12new force functions. Shared LAW1 and its qualified QEPH
source remain unchanged. Resident integration and source-material behavior
require subsequent gates.
