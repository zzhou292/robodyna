# TYPE25 source spring

`Type25Model` owns immutable, fully resolved startup inputs and exact endpoint
identity. `Type25Frame` implements the finite-length, two-node frame operations
from OpenRadioss R4BUF3/R4EVEC3. `Type25Math` adds finite-offset deformation,
four linear force channels, coupled force failure, endpoint wrench scatter and
the unscaled native elementary timestep. These are stateless host/device value
operations. No spring history participates in a nodal step or publication yet.

The input subset is Ileng=0, no third node, no coordinate randomization, no
sensor, no preload/INISPRI, no curves, no rate-dependent failure and no mass
scaling. The app resolves source defaults. TL receives positive SI property
mass, isotropic inertia, four stiffnesses, nonnegative damping coefficients,
signed force/couple limits, positive coupled-failure weights and exponents.
The frame rejects zero/degenerate length instead of choosing an unqualified
fallback. Global X/Y are explicit default skew seeds; a caller may supply the
same pair transformed to another global frame.

Native RMASS gives each endpoint `0.5 M` and `0.5 J`. These source-property
contributions retain their own labels and source IDs. They are not physical
shell inertia, added drilling inertia, or a new mass owner. The combined nodal
binding must add each declared contribution once. RINIT3 uses XL=1 for Ileng=0;
the finite endpoint distance affects frame and force arms, not these M/J values.

`Model::Matches` compares every ordered source/property/connection field,
including binary64 bits, source working units and reference-frame seeds.
Admission budgets are excluded from physical identity. Handles copy without
allocation, keep their source usable when moved, and cannot be assigned. A
prepared handle cannot be initialized again. Failed startup publishes nothing.
Counts have hard bounds of 1024 connections, 64 properties and 2048 global
nodes. The default host payload cap is 4 MiB (maximum opt-in 16 MiB). Complete
active arrays and reserved allocator/control metadata are charged before any
borrowed range is read. This budget is an ownership contract, not process RSS.
Shared endpoints across distinct springs remain separate contributions; source
node ID and exact reference position must agree at each repeated global index.

`SourceUnits` is mandatory even though public values are SI. It preserves the
working-unit interpretation of native bare numerical regularizers. The source
pin is `a62b27e6baa555d222a580d6218867d0be4d70b5`; complete original files and
Git-blob/SHA256 identities are in `lib_utest/qualification/type25/native`.
Frame code is a scalar/vector adaptation, not a call to a native solver.

Owning CMake target: `tl_type25_model`, included through `Type25Model.cmake`.
Bazel owner: `//lib_src/elements/type25:type25_model`. Standalone host checks
are configured from `lib_utest/qualification/type25`. `TYPE25_NATIVE_CHECKS=ON`
adds the compiled Fortran source operations; pass an explicit Fortran compiler
when using the workspace's existing local toolchain. `TYPE25_CUDA_CHECKS=ON`
adds two small pure-math CUDA checks, scheduled separately by the coordinator.
The math owner is `tl_type25_math` / `//lib_src/elements/type25:type25_math`.
See `SOURCE_CONTRACT.md` for the exact source subset, operation and phase limits.
