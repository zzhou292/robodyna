# Mapped wall assembly input validation

This changes only the pre-point mapped assembly validator. The previous full loaded case measured approximately 1.41 s per complete wall assembly and .336 s per candidate evaluation. No isolated validator timing was available. The source-based choice is the one-thread pass over all 359,785 compact surface rows, dereferencing current owner inverse mass/fixed fields and epoch-zero coordinates even for zero-contact rows. The result of this change must be measured on the identical loaded gate; no speedup is claimed by the author.

`AssemblyValidation.h` extracts the exact old header, one-row mass/role checks, and epoch-zero geometry checks. The complete serial `ValidateAssembly` wrapper retains its order and writes the inverse before checking geometry. `AssemblyValidation.cuh` supplies four small kernels before the unchanged point evaluation:

1. Reset the same control and summary, then check the original header.
2. Validate independent compact rows. The sole atomic operation selects an integer `2*compact + kind`, where mass is kind 0 and geometry is kind 1. Global domain IDs may be nonmonotonic. Within each row the inverse is read first, mass/fixed/root checks keep their exact short-circuit order, and geometry is consumed only after successful mass checks at epoch zero.
3. Copy the exact original inverse prefix. The failing row is excluded for a mass error and included for a geometry error. Header failure copies nothing. The copy re-reads the already consumed inverse from the immutable actual owner view; it requires no staging array or arithmetic.
4. Publish the same control/points-admitted result, then reset the integer scratch before parent evaluation.

The actual prepared source provides complete arrays and authenticated immutable accepted views throughout this stream sequence. This private scheduling helper does not add admission for arbitrary incomplete device pointers. Later-epoch geometry is not consumed, and header rejection touches no nodal source.

The existing `Summary.parent_failure` word has nonoverlapping validation and parent-arbitration lifetimes. It is initialized before any read and reset before the later parent consumer. Summary/Sidecar/Layout sizes, all arena bytes, host staging, source binding and allocation counts are unchanged. `side.inverse` retains the complete old partial state on all reported numerical failures. No floating atomics, force reordering, mass substitution, timestep change or new public diagnostic is introduced.

`Response`, point/parent arithmetic, six-channel scatter and STI addition, activity capture, accepted-base copy, candidate work/removal, diagnostics and publication retain the exact original bodies and call order. The strict legacy contact class is unchanged. Existing bounded fixed-tree observer paths also remain unchanged.

## Qualification

`reference/Kernels.cuh`, `Operations.cu`, and `Layout.h` retain complete baseline `cc7d08f` sources. `FrozenValidation.h` is the complete original validator with only its test namespace/include and host/device annotation changed. The source gate checks exact helper extraction, the serial wrapper's inverse publication point, all later operation bodies and the complete host assembly sequence except the single replaced launch. It also executes the existing observer/interval/scatter identity checks; their prior receipts remain retained.

Three tiny host functions compare the staged schedule to the frozen validator. Two CUDA functions run the actual kernels against that complete frozen validator, covering block boundaries/max-grid stride, sparse descending domain IDs, ordinary/rigid zero inverse and signed zero, first-row/within-row/header errors, exact partial inverse prefix, stale scratch and same-allocation retry. The new actual all-family owner function injects mass and epoch-zero position faults, restores the test-only mutation, and verifies unchanged accepted history/selector, output rejection, retry and stable allocations. The existing full physical wall tests continue to own force/stiffness rollback, activity/removal and common publication.

The author may run only the small host functions and source/syntax checks under one CPU/512 MiB. Actual NVCC/CUDA and the original loaded test remain root-owned.

Root focused commands, inside the workstation guard:

```sh
cmake -S lib_utest/qualification/mapped_wall_assembly_inputs -B <new-build> -DMAPPED_WALL_ASSEMBLY_INPUTS_CUDA=ON -DMAPPED_WALL_ASSEMBLY_INPUTS_OWNER=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <new-build> --parallel 1 --target mapped_wall_assembly_inputs_host mapped_wall_assembly_inputs_cuda mapped_wall_assembly_inputs_owner
ctest --test-dir <new-build> --output-on-failure -R '^mapped_wall_assembly_inputs_'
```

Expected 3 host + 2 packet CUDA + 1 actual-owner CUDA functions, plus source identity. Owning Bazel targets `//lib_utest/qualification/mapped_wall_assembly_inputs:host` and `:cuda`; the existing all-family owner fixture remains CMake-owned. Production ownership is `//lib_src/collision:nodal_wall_mapped`.

Affected regressions: physical_mesh_wall, mapped_wall_evaluation, mapped_wall_observers, mapped_wall_interval, mapped_wall_scatter, legacy surface/nodal wall APIs, and the identical full loaded vehicle case. Record inclusive wall assembly time and exact force/penetration/potential/work/activity outputs, without calling the result an isolated validator timing.
