# Native core bindings

These optional targets generate and compile the retained Python and C# core
interfaces against one shared native mechanics backend. They do not establish
independent FEA/MBD libraries or new CUDA coverage.

The initial profile is Linux x86_64, CPython 3.10, SWIG 4.0.2, mechanical FEA
enabled and NumPy disabled. SDK repositories require explicit local paths:

```text
--repo_env=ROBODYNA_SWIG_ROOT=/absolute/path/to/swig-r0
--repo_env=ROBODYNA_PYTHON_EXECUTABLE=/usr/bin/python3.10
```

Repositories validate and expose existing tools/headers; they never download or
install packages. The SWIG SDK receipt pins the downloaded Ubuntu package and
executable. Python admission records its interpreter, ABI and header hashes.
Nested tool launch removes inherited runfiles/Python search settings so each
declared executable uses its own runfiles rather than its caller's bundle.

## Ownership

- `declaration_views.json` lists seven explicit Body/Mesh/System-family views.
  Each declaration action authenticates its canonical header and separately pinned
  inverse recipe. Registry checks keep Bazel/CMake dependencies and pins aligned.
- `:python_generated` and `:csharp_generated` use the actual retained core
  interfaces. Outputs include wrapper C++, director headers, proxies and
  diagnostics. The generated declaration view is not a native header provider.
- `:native_core` owns `librobodyna_core.so`, linked from the retained native
  aggregate. It contains the single class-factory and body implementation.
- `:python_core` produces `_core.so`; `:csharp_core` produces `libchrono.so`.
  Both use `dynamic_deps` on `:native_core`, so they do not each link a static
  copy of the mechanics engine.
- `:python_fea` produces `_fea.so` against that same backend. Its runtime test
  exercises core/FEA object exchange, shared lifetime and 1,000 actual coupled
  spring steps, including constraint, analytic-displacement and reaction checks.

The public C++ body header defines `robodyna::mbd::RbBody`; the old header exposes
the compatibility alias. SWIG's declaration view preserves the established
binding names and parser type identities. It is derived from current canonical
source through a reviewed inverse transformation, not a second maintained class.
It rejects ordinary C++ inclusion and rejects stale or edited source/ledger pins.

## Qualification

Use the workspace guard and the current **two-worker trial** for generated
wrappers. Native implementation prerequisites build first with four workers;
the wrapper phase remains capped at two within the same eight-CPU/16-GiB guard.
Observed single-worker batch peaks were5.130 GiB for NumPy/plot/CAD,4.884 GiB for
baseline Python and3.655 GiB for the largest managed run. Preserve receipts and
return an affected batch to one worker if its guard demands it. These observations
do not establish that the new parallel policy has passed every profile yet.
The root agent owns serialized builds and tests.

```text
//tools/bindings:declaration_view_test
//tools/bindings:tool_environment_test
//build_defs/bindings:python_core
//build_defs/bindings:csharp_core
//tests/bindings:runtime_tests
```

Generation comparison uses the genuine pre-inertia `26ef28a9` baseline: 949 public
Python AST entries excluding documentation strings and 610 exact C# files.
Generation passing is not compilation/runtime evidence. The Python runtime test
loads the declared CPython extension and exercises constructors, copies, derived
casts, shared containers, child ownership and inertia operations. The ELF test
requires the one shared core and rejects duplicated out-of-line body/factory
definitions in either wrapper.

The inherited Python marker parent getter constructs shared ownership from a raw
pointer. That existing issue is excluded from the safe runtime probe and must be
investigated separately; native C++ parent rebinding is covered by frozen archive
tests. C# native wrapper compilation and proxy comparison alone do not qualify a
managed runtime. A separate Mono 6.8/net472-reference admission now passes:
`crash-work/reports/robodyna-managed-core-build-1.json` records the guarded
18.873-second compilation/test gate. The unchanged original C# build-system
demo ran 501 steps to 5.01 seconds, with finite crank motion, the expected
0.01-second clock and exactly one observed `librobodyna_core.so`.
Input-admission and failed-process receipt tests passed in the same gate.
This qualifies the headless core example, not optional managed modules,
Windows .NET Framework, GUI execution or CUDA mechanics. See
[`examples/csharp/README.md`](../../examples/csharp/README.md) and
[`MONO_SDK.md`](../sdk/MONO_SDK.md). The Mono SDK is extracted in the workspace;
these rules do not install a global managed runtime.

The subsequent optional managed gate passed all 13 exposed original assemblies
and six tests: `crash-work/reports/robodyna-managed-optional-build-3.json`
(32.244 seconds under the guard). Roslyn 3.11/C# 7.3 compiles the original local
functions against the same Mono/net472 ABI, with an explicitly declared real
XML LINQ runtime assembly. The unchanged ELF gate verifies one implementation
owner. Actual managed calls cross core, Vehicle, postprocess and VSG boundaries;
the coupon advances the core and checks shared lifetimes and loaded owners
without initializing a window or CUDA device. Four original Sensor/ROS/OpenCRG
examples and full GUI demo executions remain pending. The copied compiler-only
local-function/XML-doc compile-and-execute gate passed separately.

Construct finite-element meshes through the FEA module. The historical core-only
Mesh proxy lacks the qualified FE node argument descriptor for `AddNode`; comparing
the original and current generated functions confirmed identical metadata. This
is not a new rename defect. The runtime test uses `fea.ChMesh()` and still crosses
the core System boundary through `AddMesh`/`GetMeshes` and the explicit FE casts.

Retained CMake targets use the same declaration generator with explicit SWIG
include directories and dependencies. The SWIG parent resolves the owned source
root for both language subdirectories; the contract is a configure dependency so
updated reviewed pins cannot leave stale command arguments behind.

The reusable checkout probe is `//tools/bindings:cmake_probe`, also runnable as:

```sh
python3 tools/bindings/cmake_probe.py \
  --repo /absolute/path/to/robodyna \
  --sdk /absolute/path/to/swig-r0 \
  --eigen /absolute/path/to/eigen-source \
  --ninja /absolute/path/to/qualified/ninja \
  --output /absolute/path/to/new-probe-directory
```

Run it through the shared workstation guard with two CPUs, 4 GiB RSS, at least
32 GiB available RAM and a 300-second timeout. Output is create-only. The current
profile uses the installed `/usr/bin/cmake` and CPython 3.10; no packages are
installed. It exercises both-language and true C#-only configuration without a
supplied Robodyna root variable, checks declared generator dependencies and runs
the exact emitted SWIG commands. It runs the owning SWIG install script locally
to check interface/view/receipt packaging, without installing unbuilt child
libraries. A copied-contract negative test verifies automatic reconfiguration,
rejection of an incorrect ledger pin and recovery after restoring that pin.

The retained C# recipe currently declares its robot wrapper unconditionally, so
this probe enables the existing robot-model target as a configuration prerequisite.
It does not compile that target or the native core. Generation is executed from
configured commands rather than building the full Ninja wrapper target, whose
inherited order-only dependency would compile the whole core. This smoke is
separate from the qualified native wrapper compilation and runtime tests.
