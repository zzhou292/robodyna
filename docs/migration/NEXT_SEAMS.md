# Next module boundaries

These are the next dependency extractions after the neutral mechanics and generic
visualization work. Canonical class names and relocated files do not establish
independent FEA/MBD linkage. Current qualification is recorded in
[EXECUTION.md](EXECUTION.md) and [RENAME_QUALIFICATION.json](RENAME_QUALIFICATION.json).

## Completed boundaries to preserve

The generic visual split is complete. Geometry, visual values, ordinary visual
models and object services have their own native owners. FE attachment and update
definitions live in `src/fea/visualization/LegacyVisualAdapter.cpp`; generic model
update uses the retained per-model updater. The generic owners have passed actual
header/link/compile-boundary checks with FEA enabled, without concrete FE or body
implementations. Keep the focused attachment, copy, update, archive and viewer
regressions when extracting the next boundary.

The actual base body and mesh implementations now live in
`robodyna::mbd::RbBody` and `robodyna::fea::RbMesh`. The actual System/NSC/SMC
and Assembly families live in `robodyna::simulation`, with implementations in
`src/simulation/composition`. Legacy headers provide reverse aliases. These
implementations still compile once in the retained mixed backend; public `:api`
targets still depend on that aggregate. Private implementation-header providers
and the thin System forward target avoid introducing a build cycle. Do not make
those private providers depend back on the public aggregate API.

The incremental participant-service seam is implemented. `RbParticipantServices`
provides four operations: read current gravity, read the CPU assembly thread
setting, invalidate initialization plus update, and invalidate update only.
System implements them against its existing fields through a stateless service
base. Body and Mesh use these operations at their original call sites.
PhysicsItem retains only its original parent pointer; its protected nonvirtual
accessor performs a typed conversion on each lookup, with no cached second owner.
Direct setters, copy/assignment, archive attachment, removal and original
node/element `SetupInitial(System*)` hooks remain intact.

The 12 behavioral baseline cases passed before and after the change. The combined
native/product test gate passed 57/57 targets, including the actual header and
dependency-closure gates in `//tests/participant_services:service_tests`.
`//include/robodyna/mechanics:participant_services` has only its header and the
existing 16-unit foundation in its compile/link closure. This proves the service
**declaration's** boundary. The working implementation still compiles in the mixed
backend: `ChPhysicsItem.cpp` explicitly includes the full `RbSystem.h` to perform
the typed conversion. Body also retains concrete System contact/reporting calls.
Separate rebuilt CMake, binding and product qualification is tracked in the
execution/qualification records; these native tests do not establish independent
FEA/MBD execution.

Preserve the original numerical behavior and archive identities. The visual
archive still represents ordinary shapes; the existing FE-shape omissions are
not physical restart support. The service base changes System layout/vtables,
so rebuild owned consumers, including native SDK wrappers. More generally,
rebuild owned consumers when C++ layout changes;
source/archive compatibility does not promise compatibility with old binaries.

## 1. Extract the thin System owner and its storage boundary

The four-service indirection is complete; the remaining dependency is the
complete mixed System declaration used by the participant accessor. System still
contains its concrete Assembly by value and exposes protected state used by
retained subclasses. Define the storage and compatibility strategy before
assigning independent ownership to this implementation.

Preserve public virtual `SetSystem`, raw `GetSystem` identity, protected parent
writes, the distinct copy/assignment behaviors and normal removal/destruction
order. Keep the existing single clock, initialization/update flags and offsets.
Do not use a global resolver, cached alternative owner or layout-dependent cast
to disguise the full-header dependency. A forward-declared System pointer alone
does not provide the inheritance information needed for the typed conversion.

Storage changes must also preserve retained protected access and collection
addresses. `ChSystemMulticore.cpp:79–81` stores pointers to Assembly body, link and
other-item collections; later methods access their contents directly. Preserve
those addresses/lifetimes or migrate these consumers through an explicit,
qualified API change. Replacing a by-value member with a PIMPL is not sufficient
without accounting for these obligations.

Gates: the existing participant, System, Assembly, copy/archive and common-clock
coupling tests; direct/repeated attachment and removal; real gravity/residual
parity; protected-subclass and Multicore collection-address checks. Add actual
header/compile/link closure checks for the extracted **implementation**, without
the aggregate satisfying its dependencies. The current declaration-only gate
does not establish that result.

## 2. Separate Body reporting and collision services

Body's applied-force/torque queries still dispatch to virtual System methods.
The ordinary System lazily assembles its residual; Multicore overrides these
methods using its own force arrays, body indices and step size. Keep that virtual
dispatch, cache invalidation, coordinate conventions and NSC/SMC inclusion rules
when introducing a typed MBD adapter. A generic residual-offset getter is not an
equivalent replacement for all retained backends.

Body contact reporting still goes through the contact container. Collision
enable/disable also consults the current collision backend's initialization and
the collision model's implementation state. Keep these operations outside the
four-operation common environment interface, preserving their lifetimes and
ordering.

Gates: actual derived-System override dispatch; lazy applied-force evaluation;
rotated-body force/torque frames; NSC/SMC reporting and contact counts; and
enable/disable before and after collision initialization. Retain optional
Multicore behavior explicitly rather than claiming it from an ordinary System
coupon.

## 3. Keep mixed assembly above FEA and MBD

`RbAssembly` owns concrete body, link, shaft and mesh collections. Its traversal,
offset assignment, injection into the system descriptor and archive ordering are
part of current behavior. Keep mixed composition above the peer domains while
separating domain contributions at their existing assembly boundaries. Do not
replace the common assembled solve with mandatory partitioned stepping merely
to make directory dependencies appear independent.

Gates: preserve collection/iteration order, DOF and constraint offsets,
mass/force/Jacobian contributions, reactions and frozen archive bytes. Exercise
a flexible beam attached to a jointed rigid body through the same assembled
solve, including action/reaction moments and constraint work.

## 4. Separate contact algebra from domain response

`ChContactContainer` includes concrete body definitions and dynamically casts
contact participants to bodies for reporting. Tuple algebra also depends on
contactable interfaces. Extract neutral algebra only where its actual header
and link closure is neutral; keep body/FE force response and reporting in explicit
domain adapters. Reuse the current contact algorithms and solver policies.

Gates: NSC/SMC coupons, FE–FE and FE–body contact, equal/opposite force and moment,
report ordering and counts, and unchanged constraint/contact work. Do not disable
FEA, omit a contact participant or drop a reporting feature to obtain a smaller
link closure.

## 5. Finish domain-specific loads and prove independent execution

Split FE nodal loads/builders from their concrete body and motor attachment
helpers. Keep neutral frame constraints distinct from concrete coupling adapters.
Then execute genuine MBD-only and FEA-only targets without the opposite domain's
concrete implementations or a hidden path through the aggregate. Check actual
header, compile and linker ownership as well as runtime results.

The qualified CUDA vehicle backend retains FE nodes, rigid-region state and CIN
transfers in one owner and one publication. Preserve that whole backend and its
single advancement throughout this work. The renamed CPU System family is not
evidence that its execution has become CUDA, or that the two backends share a
numerical stepper. Keep their execution profiles explicit.

Complete the independent-domain gates and coupled solve regression before
expanding module extraction against [CAPABILITIES.md](CAPABILITIES.md). Retain
existing CPU and CUDA implementations until their replacements are separately
qualified. Use small focused cases and the short vehicle archive comparison;
documentation or ownership changes alone do not justify rerunning the long crash.
