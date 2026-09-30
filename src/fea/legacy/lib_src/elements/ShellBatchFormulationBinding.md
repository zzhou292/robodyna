# Complete host formulation catalog

`ShellBatchPlasticityBinding::InitializeFormulations` and
`InitializeFormulationCatalog` own a complete QEPH/T3/QBAT material/section map.
They require a prepared `ShellBatchBinding::InitializeFormulations` inventory
with at least one QBAT parent. Either other family may be absent. The small
entry retains 1024-parent/2048-node bounds; the vehicle entry requires explicit
`ShellPlasticityCatalogLimits`. Count ceilings, the 1024-definition and total
1024-curve-point pool limits remain unchanged. All older initializers reject
QBAT before accessing declarations.

The original source section `OneThicknessPoint` with NIP=1 is shared by the
midlayer triangle and all midlayer quads under one original PID/MID/SID. The
material declaration remains LAW44, with its existing tabulated/analytic and
rate policies. `Law(T3, index)` resolves `Law44Nip1` (one actual point), while
`Law(Qbat, index)` resolves `Law44QbatFourInPlane` (four in-plane points, one
thickness point each). Neither resolved role is a valid material declaration.
QBAT requires centered, qualified NPTR2/NPTS2/NPTT1 geometry and an exactly
matching prepared A11. QEPH NIP1 and QBAT NIP3 reject.

`Counts().law44` counts parents. Its `law44_nip1` and `law44_qbat` fields are
subsets, never additional material totals. Scalar `Parameters` owns/rebinds the
same material coefficients and curve view; it does not expose a three-point
history. No runtime point state is allocated by this catalog. Query failure
preserves caller output, and absent families have zero counts and no rows.

All original parent IDs, family indices, PID/MID/SID assignments, material and
section declarations and the complete geometry inventory belong to `SameScope`.
Separate original layers sharing a physical cell remain separate source parents.
Their source EIDs must be unique, while their NIDs/coordinates and native M/J
remain under the existing complete binding authority. No topology or mass is
recomputed here.

The immutable failure binding now indexes QBAT independently. Its selected
four-point role requires `ConstantAllPoints`, positive finite D1 and canonical
absent TAB1 controls. Failure declarations retain the complete catalog source
order. The existing dynamic three-point failure state is unchanged and cannot
represent QBAT.

`ShellFormulationScope { binding, catalog, failure, optional_mass }` is a borrowed
host view. `ValidateShellFormulationScope` checks complete immutable identity,
including any supplied TYPE25 mass composition. It allocates nothing and gives
no owner/participant/publication authority. Example after preparing the complete
geometry and source declarations:

```cpp
ShellBatchPlasticityBinding catalog;
auto material_report = catalog.InitializeFormulationCatalog(
    geometry, declarations, ShellPlasticityCatalogLimits::Vehicle());
// Check material_report before using catalog.
ShellBatchFailureBinding failure;
auto failure_report = failure.Initialize(catalog, failure_rows, parent_count,
                                        ShellBatchFailureLimits::Vehicle());
// Check failure_report before validating the borrowed complete scope.
auto scope_report = ValidateShellFormulationScope(
    {&geometry, &catalog, &failure, nullptr});
```

Owned and startup scratch bytes are forecast before allocation and new-entry
borrowed range reads. QBAT lookup/seen arrays have no inline capacity and are
charged by their actual extent. On the qualified x86-64 host the catalog object
grows from 101704 to 101800 B; `ShellSectionCounts` grows from 24 to 32 B and the
private scratch header from 53392 to 53424 B. Existing `sizeof` accounting covers
these deltas; there is no extra heap allocation on a QBAT-absent path. Active
curve backing and copied immutable index handles retain their lifetime after
caller arrays expire. Default public aggregate fields retain their old values.

This increment does not remove Q/T startup's explicit QBAT rejection, alter the
T3 physical-topology guard, change material/native equations, enlarge any device
arena, or implement a QBAT participant. Original source admission and genuine
four-point common publication remain separate qualifications. The next approved
resident design is `planning/QBAT_RESIDENT_INTEGRATION.md` in the workspace.
