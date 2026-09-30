# Nodal-wall contributor unit gate

Source frozen before execution; no passing result or QEPH/contact admission is
claimed. This gate exercises the owning TL participant on the existing nodal
owner, separately from QEPH history. The model is
`reference-area-lumped-nodal-wall-v1`, with actual finite-wall queries and an
immutable host-certified motion envelope. It does not use the source app's
qualification device object or change a production selector.

The public API is `collision/NodalWallContactDevice.h`. The private host model
compiler stages all validation before publication. Startup is epoch zero with
`StaggeredHalfKickStart` and collocated initial velocity. The module supports
one/two Q4 parents, <=8 incident nodes in the existing <=64-node owner, free
XYZ/fully-fixed translation masks, and no direct couple. QEPH's current
all-free startup contract is unchanged.

Pinned 64-bit host/CUDA ABI ledger, enforced by compilation assertions.
The first compile rejected an authored size tally eight bytes too large for
Model and Storage. A standalone host declaration probe measured the sizes below;
only the byte assertions/declaration and a bounded node-count cast changed.
The failed source/build report is retained in `nodal-wall-owner-first-compile-1`.
Numerical formulas, test budgets and device layout are unchanged:

| Item | Bytes |
|---|---:|
| Immutable model, including actual wall query packet | 84,952 |
| Compact result with two parents/eight nodes and interval diagnostics | 3,984 |
| Complete device allocation, including base/candidate, shares and all-destination preflight | 99,384 |

One allocation, no per-step allocations/events; one 128-thread block. Existing
owner allocation count stays six. The complete participant cap is 256 KiB.
One negative unit temporarily allocates 48 bytes of alternate mass data; it
does not mutate the owner's immutable mass. No device stack/register/heap limit
is changed. These are declared/static quantities, not a measured runtime report.

Four host functions in `NodalWallModelTest.cpp` cover reference/area/global-ID
binding, exact source/native topology scope, invalid identity/mass/caps staged
publication, actual wall holes/outside/invalid faces, fixed penetration and
source-scale error declarations. Six actual CUDA functions in
`NodalWallOwnerTest.cu` cover:

1. Full host-oracle field comparison, one/two Q4/shared nodes, native order,
   actual wall original/flip/subdivide/reversed faces and additive force/couple
   preservation, including unused global nodes.
2. Eight accepted contact-only spring steps: first half versus later full kick,
   endpoint potential, next-force exclusion from the consumed kick, independent
   kick/drift work, wall impulse/moment and convex work-defect bounds.
3. Foreign/duplicate/phase/epoch failures, late nonfinite and finite-overflow
   all-destination preflight, exact accepted-state preservation and clean retry.
4. Candidate envelope/depth/fixed-position/time failures; stale or invalid
   readback leaves caller output untouched; fresh retry preserves fixed zero law.
5. Entering/leaving activation, actual mass mismatch and a second-parent-only
   precision failure after the first parent completed.
6. A pending invalid CUDA launch intercepted before result readback poisons the
   participant and preserves caller/accepted output; no failed-context recovery.

Numerical budgets fixed before execution:

* The inherited source tuple is kappa=4e5, depth=.25 mm, cap=.5 mm, per-parent
  force error 5e-7 N and energy error 1.2500000000000005e-12 J. Global errors are
  outward sums. No unit silently treats the parent budget as a global budget.
* Ordinary analytic fixture: kappa=16, areas .5/1 m², depth=1/32 m, cap=.5 m,
  actual inverse masses 1 kg^-1, h=1/1024 s; all-active rate <=6+roundoff s^-2.
  Its contact-only `UnitQualification` is neither BQ4 nor a coupled-shell ID.
  No contact force is declared a state-independent prescribed load.
* Host/device arithmetic comparisons preserve the inherited 2e-12 absolute-plus-
  magnitude oracle. Independent long-double work/momentum comparisons use
  `256*epsilon*sum_absolute_terms + 1e-12*physical_scale`; the fixed energy scale
  is 1/64 J and impulse/moment scales are 1/1024 in their respective SI units.
  Potential-defect and impulse enclosures use measured base/candidate
  certificates; a nominal quadrature estimate need not lie in its truth interval.
* Failure-only finite-overflow fixture: kappa=1e308, inverse mass=1,
  force/energy diagnostic budgets=1e296, and a final finite -DBL_MAX destination.
  These explicit scale settings test overflow after valid local physics; they
  do not replace or relax the source tuple. The second-parent accuracy failure
  instead uses an intentionally tighter 1e-30 J budget.

Suggested root registrations: production `tl_nodal_wall_contact_device` with
`NodalWallContactModel.cpp` and `NodalWallContactDevice.cu`; host
`utest_nodal_wall_model`; CUDA `utest_nodal_wall_owner_cuda`. Link existing
nodal owner/explicit step, nodal wall host/reference weights, Q4 parametric
reference, planar wall box and prepared query targets. Strict FP64/no fast-math,
`--fmad=false --ftz=false --prec-div=true --prec-sqrt=true`; GTest main;
RUN_SERIAL/TIMEOUT120. No shared registration is authored in this slice.
Run the unchanged raw nine-host, prepared-query six-host, applicable owner and
old Q4 contact regressions after compiling the reduction extraction.

All QEPH coupled prefix/physical case work remains blocked on BQ4 and a new
contact-qualified full-recurrence admission. The private all-active contact
bound is never added to the legacy owner `RowBounds`. Output is staged before
the existing owner/material commit; this module has no material history,
accepted clock, or post-commit CUDA operation.
