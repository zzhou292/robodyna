# Joined shell plasticity materials

`ShellBatchPlasticityBinding` is the host startup catalog for an entire
authenticated `ShellBatchBinding` collection. QEPH and T3 may select different
materials, curves and sections. The existing per-parent device parameter array
supplies the existing layered material adapter; constitutive equations, history
slabs, accepted/trial selection and the nodal publication boundary are unchanged.

## Initialization

1. Prepare the native `ShellBatchBinding` from explicit QEPH/T3 collections.
2. Supply curve declarations (source curve ID and samples), material declarations
   (MID, curve ID, E, nu, density and optional native rate settings), or the explicit
   analytic LAW44 SIGY/ETAN declaration with no curve (see
   [Law44AnalyticHardening](../materials/Law44AnalyticHardening.md)), and section
   declarations (SECID, homogeneous parent thickness and NIP3).
3. Supply every parent in the application's source order with its exact source
   EID, PID, MID, SECID, native family and family index. The catalog retains this
   order and checks each EID against its authenticated native family index.
4. Call `catalog.Initialize(binding, input)`. Every declaration must be referenced;
   every parent must occur exactly once. E, nu, density and thickness must match
   the corresponding native reference bit for bit. A PID cannot assign conflicting
   materials or sections to its parents. The complete curve pool is checked before
   sample copies using subtraction against the remaining capacity.
5. Initialize each participating family with
   `batch.InitializeJoined(config, binding, catalog)`. Both families receive the
   same complete catalog even when they use different MIDs.

Source ELFORM and deck-unit conversion belong to the application import layer.
The catalog validates declared native family assignments; it does not infer an
element family from an LS-DYNA formulation number or change source connectivity.

### Explicit layered elastic/plastic declarations

`ShellBatchSectionBinding.h` provides neutral aliases for the same immutable
catalog. `InitializeSections(binding,input)` and
`InitializeSectionCatalog(binding,input,limits)` explicitly enable per-parent
`ShellSectionLaw::LayeredLaw1Nip3` alongside `LayeredLaw44Nip3`. The latter remains
the trailing material tag's default, preserving existing positional declarations.
The two original initialization APIs still reject elastic declarations.

LAW1 requires original E/nu/rho, homogeneous physical thickness and NIP3. Its
unused curve ID, rate and linear-hardening fields must have their canonical
default values; they are not silently ignored. `ElasticParameters` exposes only
the qualified elastic coefficients. `Parameters` exposes only LAW44 coefficients,
and leaves its output untouched for an elastic parent. `Law` and family `Counts`
give explicit availability without fabricated plastic strain, rate or yield
values. Elastic coefficients are prepared from owned declarations when queried
at host startup; this API does not install a per-step preparation path.

Scope equality includes the explicit mode and each law tag as well as the full
existing source inventory and ordered assignments. Copies/moves retain the same
ownership rules. Small-mode limits preserve the older binding scratch treatment;
the separately named catalog API enforces its explicit scratch budget. Appending
the tag/count metadata changes the precisely charged inline host payload; it
does not change old device layouts, numerical dispatch or curve storage.

The explicit mode is consumed by the existing joined QEPH/T3 initializer after
the mixed resident gate described in [ShellMixedSections](ShellMixedSections.md).
It does not admit failure, glass, membrane or rigid source roles, or substitute
fixed thickness for source ITHICK=1.

The catalog owns all declarations and samples after successful preparation.
Returned `Parameters` values borrow the catalog's pool for host evaluation, so
that catalog must remain alive while those values are used. Stored coefficients
hold no pointers into the pool: copying or moving a catalog therefore cannot
leave a pointer into a different object's storage. Each initialized batch owns
its own complete catalog and its resident curve pool, independently of the caller.

## Scope, capacity and publication

The catalog is immutable after preparation. All validation failures preserve its
bytes and allow retry with corrected input. `SameScope` compares the complete
native inventory, all owned declarations and curve samples, and the complete
ordered parent mapping. Equality is not granted by a hash or by the current
family's materials alone. Legacy single-material scope comparisons retain their
existing rules; legacy and collection declarations cannot be mixed in one joined
publication.

Legacy `Initialize` admits 1024 parents/2048 nodes with the existing host limits.
Passing `ShellHostBindingLimits::Vehicle()` to that API still rejects before
borrowed ranges or fixed validation storage are accessed. Vehicle catalog setup
must explicitly call:

```cpp
catalog.InitializeCatalog(binding, input, ShellPlasticityCatalogLimits::Vehicle());
```

The separate host-only limits admit up to 524288 parents and nodes, with 1024
materials and sections and an unchanged **1024-point total curve pool**. The
vehicle profile supplies 256 MiB owned and 32 MiB transient startup payload
budgets. These byte values are profile defaults, not hard ceilings; legacy
`max_owned_bytes` semantics stay unchanged. The caller may choose tighter budgets;
counts and required bytes are
validated before declarations are read or storage is allocated. The default
named limits retain 1024/2048 and 4 MiB owned admission. None of these constants
or APIs enlarge resident shell/contact arenas or admit unsupported mechanics.

`host_bytes()` includes every retained array, the complete authenticated inventory
backing, and inline storage. `startup_scratch_bytes()` additionally reports the
transient inline staging object and source-identity/seen backing needed during
initialization; retained backing is charged once to the owned budget. Shared
array control payload is reserved by the existing `BoundedStartupArray` contract;
allocator metadata, sort stack and whole-process RSS remain external resources.
Copies/moves allocate nothing and continue sharing immutable dynamic arrays;
returned table curves rebind to each object's inline pool, and analytic parameters
keep null curve pointers.

The temporary PID index maps each PID to its first input occurrence; parent
validation still runs in original source order. A conflict compares against that
previously accepted first assignment. Material/section indexes map IDs to original
declaration indexes. Thus parent startup is O(P log P + P log D), instead of the
old O(P² + P D), with original first-failure order and exact native coefficient
bit checks. Definition checks retain their bounded D≤1024 loops. Legacy-size
scratch remains inline, preserving allocation-free startup for ≤128 parents.

Device histories, accepted/trial publication, the step clock and resident material
updates are unchanged. This catalog capacity is a host preparation capability,
not evidence that a complete vehicle can be advanced or contacted on the GPU.

## Qualification

`shell_plasticity_binding_check` exercises full parent coverage/order, multiple
materials within and across families, declaration ownership through copy/move,
missing/duplicate/unreferenced IDs, native material/thickness mismatch, a late
invalid curve, exact scope comparison, and the total curve-pool boundary.
Five `ShellSectionBinding` cases additionally cover elastic/table and
elastic/analytic owned catalogs, typed availability, explicit-mode scope,
late invalid/mismatched source coefficients, legacy rejection, count preflight,
exact byte/scratch limits and successful retry after rejection.

`resident_plasticity_check` additionally exercises two actual CUDA participants
with distinct materials/curves and rate settings through load/hold/reversal,
compares against the production host adapters, and verifies that a late T3 failure
preserves previously yielded owner/shell/material histories and retries exactly.
It checks full-catalog mismatch rejection, legacy/catalog mixing rejection and
unchanged allocation accounting. These are prescribed-history integration tests;
the separate native point/section/element qualification and physical dynamics
gates remain responsible for numerical and response accuracy.

`vehicle_plasticity_catalog_small` covers explicit caps before borrowed reads,
first source-order failure, exact native bits, late failure/retry, every transient
and retained allocation failure, 1024 definitions, mixed and all-analytic ownership,
and zero curve pools. `vehicle_plasticity_catalog_source_size` uses synthetic
native geometry at 349645 parents/359785 nodes with 875 definitions, inspects every
parent mapping and parameter, and rejects/retries a last-parent error. Parent and
node counts come from the selected no-tire source inventory; the 875 synthetic
definitions exercise declaration capacity. The fixture is **not** an original
Yaris material/model import or a vehicle mechanics qualification.
