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

- `:body_declarations` authenticates canonical body source and the separately
  pinned migration ledger, producing SWIG-only declarations and a receipt.
- `:python_generated` and `:csharp_generated` use the actual retained core
  interfaces. Outputs include wrapper C++, director headers, proxies and
  diagnostics. The generated declaration view is not a native header provider.
- `:native_core` owns `librobodyna_core.so`, linked from the retained native
  aggregate. It contains the single class-factory and body implementation.
- `:python_core` produces `_core.so`; `:csharp_core` produces `libchrono.so`.
  Both use `dynamic_deps` on `:native_core`, so they do not each link a static
  copy of the mechanics engine.

The public C++ body header defines `robodyna::mbd::RbBody`; the old header exposes
the compatibility alias. SWIG's declaration view preserves the established
binding names and parser type identities. It is derived from current canonical
source through a reviewed inverse transformation, not a second maintained class.
It rejects ordinary C++ inclusion and rejects stale or edited source/ledger pins.

## Qualification

Use the workspace guard and **one compiler job** for the large generated wrapper
translation units; the normal four-worker allowance does not apply to this phase.
The current wrapper qualification guard is 16 GiB sampled RSS. The root agent
owns serialized builds and tests.

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
tests. C# native wrapper compilation and proxy comparison do not qualify a managed
C# runtime. No managed runtime SDK is installed by these rules.

Retained CMake targets use the same declaration generator with explicit SWIG
include directories and dependencies. The SWIG parent resolves the owned source
root for both language subdirectories; the contract is a configure dependency so
updated reviewed pins cannot leave stale command arguments behind.
