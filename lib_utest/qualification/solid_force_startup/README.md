# Constructor-only solid force initialization

The three typed `InitializeForce` functions construct native TT0 fields from
exact prepared reference positions and a supplied uniform world translation.
They accept no previous History, create virgin inputs internally, execute the
selected material/geometry/viscosity/hourglass arithmetic once and stage the
complete result before publication. Output time and sample index are zero.
This is initial material/force evaluation, not a completed zero-duration interval
or an arbitrary positive step with its stamp reset. Rest is zero translation.

`InitializeHistory` remains the existing virgin-value constructor. All ordinary
`EvaluateForce` functions retain their prior strict positive interval contract.
The existing LAW36 `Update`/`UpdateCaller` dt0 guard and LAW42 positive-step guard
also remain. New material `Initialize`/`InitializeCaller` functions generate their
own virgin inputs and accept only zero dt. They share the same internal arithmetic
as ordinary updates; no separate constitutive or three-family force solver was added.

Native initialization may compute density, bulk pressure, rates, stress and
energy roundoff fields at the actual reference geometry. These are retained as
initial values, without claiming completed-interval energy or time semantics.
The family source permutation, LAW36 eight independent points, HEPH global JAC,
S6Z six-node profile and sole material/hourglass energy field stay unchanged.
The active-only rubber cutoff still rejects atomically.

The typed `solids::PrepareNodalStiffness` overloads consume actual force results.
SCUMU3 applies FOURTH to raw STI for eight-node solids; Solid18 raw STI is the
completed native point sum. S6CUMU3 applies THIRD for six actual wedge slots.
This coefficient is added at every original slot, with zero rotational increment.
It is not recomputed from nodal mass, a mean timestep or shell activity witnesses.
Native spring XKM/XKR remain TYPE13's separate R2LEN3 inputs; both producers feed
STIFN/STIFR before the existing CIN transfer. No owner or resident is introduced.

Source pin: `a62b27e6baa555d222a580d6218867d0be4d70b5`. Existing complete family,
material and reference oracles remain unchanged and independent of production.
The new stiffness packet authenticates complete SCUMU3/S6CUMU3 donors by byte
count, SHA256 and Git blob, then executes their exact scale loops with the
existing native CONSTANT_MOD. It qualifies scalar scaling, not whole-model
native scatter order or a full CIN owner. Source order is explicit in the later
resident contract. Cached CONTRL:1125–1133 seeds TT/DT1/DT2 zero; RESOL:2715 copies
DT2 into DT1 before normal force calls. No invented native skip-material branch
is used. First-step evidence is the actual independent family packet at dt0.

The owning gate contains four new host functions plus 21 unchanged host
functions, four native functions (three complete families and exact stiffness),
three all-original source functions (908/1309/195 parents), unchanged native
family regression functions and one actual CUDA function. Each native family
constructs and carries its own initial fields into four positive intervals.
The CUDA kernel owns its three histories, checks late input rejection and exact
retry, and compares complete named output fields against independent host native
packets. No padded struct success comparisons or native history seeded from TL.

Author validation: host gates and C++ syntax only; native/CUDA/source execution
belongs to the scheduled root gate. No new tolerance is supplied here.

```sh
cmake -S lib_utest/qualification/solid_force_startup -B build/solid-force-startup \
  -DCMAKE_BUILD_TYPE=Release -DTL_SOLID_FORCE_STARTUP_NATIVE=ON \
  -DTL_SOLID_FORCE_STARTUP_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build build/solid-force-startup --target solid_force_startup_host \
  solid_force_startup_legacy_host solid_force_startup_native \
  solid_force_startup_legacy_native solid_force_startup_cuda -j1
ctest --test-dir build/solid-force-startup --output-on-failure -j1
```

Bazel owners: `//lib_src/elements/solids:force_stiffness` and
`//lib_utest/qualification/solid_force_startup:host_check`; three narrowly visible
test-value libraries reuse existing fixture owners. Engine/source reference files
remain qualification-only.
