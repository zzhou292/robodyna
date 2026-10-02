# Managed C# SDK admission

`@mono_sdk` uses an explicitly supplied `ROBODYNA_MONO_ROOT`. The current
Linux x86_64 runtime is Mono 6.8.0.105 from 13 official Ubuntu22.04 packages,
with the actual4.7.2 reference assemblies. The first core example passed with
Mono's compiler, but three other original examples use C#7 local functions.
A real probe confirmed that this mcs compiler cannot parse them even with its
7.2 option. Current compilation therefore uses the separately pinned Microsoft
Roslyn 3.11 compiler and one explicit C#7.3 language profile; the Mono runtime and
reference assemblies are unchanged. No original demo is rewritten to fit C#6.
This is a Mono runtime qualification, not a claim that Windows .NET Framework
or every modern .NET runtime was tested.

The packages total30,405,896 compressed bytes. Their versions, URLs and SHA256
values are recorded in `mono_pins.json`. Workspace extraction executes no
installer scripts, package hooks, certificate updates or global registrations.
The original 26-file compiler/BCL/reference/native-helper subset preserves vendor
copyright files. Optional framework APIs outside this subset require explicit
additional SDK inputs and tests.
The Roslyn profile additionally declares 134 existing Mono runtime facade
assemblies (755,200 bytes) from the same pinned mono-devel package.

The repository rule verifies every selected file, then checks both runtime and
compiler versions. Bazel actions and launchers use declared runfiles and explicit
Mono paths. They remove inherited Mono/Python/native-library search overrides;
they do not fall back to an installed SDK or restore NuGet packages. SDK files
are addressed relative to the runfiles SDK root, not an embedded workstation
path. Linux glibc/libgcc/zlib/Kerberos dependencies remain system prerequisites.
Tool admission alone does not establish compiled managed/native interoperability.

The public output names are `Robodyna.Managed.dll` and
`Robodyna.BuildSystem.exe`. Their inherited proxy types and native P/Invoke name
`chrono` remain compatibility interfaces. The native wrapper is the existing
`libchrono.so` linked through `dynamic_deps` to the single `librobodyna_core.so`.
No CMake core, replacement equation implementation or second native backend is
introduced by managed compilation.

`@roslyn_sdk` authenticates the official Microsoft compiler package and its
adjacent dependencies. Compilation receipts identify the actual compiler hash
and language version. The historical mcs core result remains evidence for that
old profile; the new compiler requires the same real interoperability gates.
This is a pinned compatibility profile, not a claim that every future compiler
can run on this Mono version. Microsoft's package documentation recommends a
full SDK/toolchain update for long-term upgrades of older MSBuild installations;
this Bazel integration invokes the compiler directly and performs its own tests.

Owning sources still default the historical bundled wrapper to net472;
the new rules use those reference assemblies directly without running MSBuild.
Original C# source/license bytes remain unchanged. `ChronoGlobals.cs.in` is
configured with the inherited version and relative data paths; launch creates
its own output directory and an explicitly selected data link.

Current qualification targets:

```text
//tools/managed:inputs_test
//build_defs/bindings:csharp_managed_core
//examples/csharp:build_system_assembly
//examples/csharp:build_system_test
```

The native wrapper can require a large single-TU C++ compilation. Use the
current two-worker/16-GiB wrapper trial (one-worker fallback) when it is not already built. The real
managed headless test runs the original500/501-step crank/rod example through
5seconds of simulated time, observes finite moving poses and the actual mapped
native core, and checks the original pre-step JSON archive call. It does not
qualify physical restart, GUI behavior, optional bindings or GPU mechanics.

Official sources: [Mono compiler language support](https://www.mono-project.com/docs/about-mono/languages/csharp/),
[Ubuntu Mono package profile](https://packages.ubuntu.com/jammy/mono-mcs).

The actual Roslyn compiler/runtime closure now includes the separately pinned
`libmono-system-xml-linq4.0-cil` runtime package; reference assemblies are never
used as runtime implementations. `//tests/managed_compiler:local_functions_test`
passed after copying only declared SDK inputs, compiling a captured local
function with XML documentation and executing the resulting IL. The subsequent
`robodyna-managed-optional-build-3.json` gate passed all 13 exposed original
assemblies and six tests, including real headless managed/native interoperability.
That does not extend the result to the remaining Sensor/ROS/OpenCRG examples or
interactive GUI trajectories.
