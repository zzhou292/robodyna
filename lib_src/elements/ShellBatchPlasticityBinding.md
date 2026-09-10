# Joined shell plasticity materials

`ShellBatchPlasticityBinding` is the host startup catalog for an entire
authenticated `ShellBatchBinding` collection. QEPH and T3 may select different
materials, curves and sections. The existing per-parent device parameter array
supplies the existing layered material adapter; constitutive equations, history
slabs, accepted/trial selection and the nodal publication boundary are unchanged.

## Initialization

1. Prepare the native `ShellBatchBinding` from explicit QEPH/T3 collections.
2. Supply curve declarations (source curve ID and samples), material declarations
   (MID, curve ID, E, nu, density and optional native rate settings), and section
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

This change admits at most the existing 128 native parents and 1024 curve points
total. It does not increase shell/contact capacity. Device allocation count and
size remain exactly those of the existing plastic path: one legacy family slab
and one optional plastic slab. Curve copies and host catalog allocations happen
only at initialization. Every step uses the existing parameter array and common
accepted/trial index, without allocation or an additional clock.

## Qualification

`shell_plasticity_binding_check` exercises full parent coverage/order, multiple
materials within and across families, declaration ownership through copy/move,
missing/duplicate/unreferenced IDs, native material/thickness mismatch, a late
invalid curve, exact scope comparison, and the total curve-pool boundary.

`resident_plasticity_check` additionally exercises two actual CUDA participants
with distinct materials/curves and rate settings through load/hold/reversal,
compares against the production host adapters, and verifies that a late T3 failure
preserves previously yielded owner/shell/material histories and retries exactly.
It checks full-catalog mismatch rejection, legacy/catalog mixing rejection and
unchanged allocation accounting. These are prescribed-history integration tests;
the separate native point/section/element qualification and physical dynamics
gates remain responsible for numerical and response accuracy.
