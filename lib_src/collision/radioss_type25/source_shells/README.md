# Ordinary-shell contact startup values

This allocation-free host startup producer supplies whole-model nodal contact
stiffness, selected primary shell stiffness and role-specific shell gaps. It
reuses the qualified coefficient leaves and BoundedArena layout. It does not
own topology, physical mass/inertia, dynamics, a solver clock or OpenRadioss code.

The admitted population is the complete ordinary TYPE1 Q4/T3 shell model,
IGAP1/ILEV1/IGAP0=0/contact ITHK25=0. Explicitly reject other families, stacks,
coatings and thickness-update profiles; source-file completeness is authenticated
by the app binder. Supply PM20 and the structural THK separately from contact
part/element/property thickness operands. Structural ITHICK is not ITHK25.

SPMD_MSIN order is Q4 then T3, ascending original element IDs within each family.
Source IDs are unique across families; real physical slots contribute once, with
T3 slot4 a repeated descriptor only. Primary selection retains its own order and
must not repeat a physical shell. Additional noncontact shells contribute to
ETNOD/count and nodal maximum thickness. ILEV1 then masks secondary shell gaps
to selected main nodes; added secondary nodes can have zero gap and nonzero K.
Main gaps use whole-model nodal maxima and a distinct cap. MainGaps distributes
them over actual oriented connectivity; opposite side expansion is a separate
startup module. Main stiffness ignores part contact-gap overrides, as the native
ordinary I25GAPM branch does.

Preflight validates profiles, physical row/order semantics and extents before
returning the exact private scratch forecast. Build also verifies borrowed ranges
and duplicate primary selection, stages every result and publishes only after all
rows succeed. Inputs must remain immutable for both calls. Scratch/output are
caller-owned host storage, pairwise disjoint and disjoint from inputs. Output
bytes in the forecast include alignment. No per-step work belongs here.

Sources: OpenRadioss a62b27e6, starter CINMAS/C3INMAS ET contributions,
SPMD_MSIN family/source ordering, ASSTIFI, I25STI3, I25GAPM and I25INI_GAP_N.
Qualification combines actual pinned Starter/Engine observations with independent
heterogeneous analytic cases, branch/ordering, cap/alias and failure-atomic tests.
This module alone does not authenticate a vehicle deck or qualify adaptive gaps.
