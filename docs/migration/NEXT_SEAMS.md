# Next module boundaries

These are source-audited implementation steps after the neutral mechanics gate.
They are not claims of completed FEA/MBD independence. Preserve the current
assembled solve and the qualified TL FE/rigid/CIN owner throughout.

## Remove the generic visual-model dependency on FEA implementation

The source audit found a direct dependency from `ChObj`/`ChVisualModel` to
`ChVisualShapeFEA`. This currently prevents lower-level object services from being
linked without FE mesh and element implementations.

The smallest proposed patch is an explicit per-model FEA visual updater:

1. Forward-declare `ChVisualShapeFEA` in `ChVisualModel.h`; retain the existing
   typed getters and shared-pointer container for source compatibility.
2. Move `ChVisualModel::AddShapeFEA` and `ChObj::AddVisualShapeFEA` definitions into
   the FEA visualization adapter. Preserve owner assignment and attachment order.
3. Install a private per-model updater function pointer when an FE shape is added.
   Generic model update invokes it after ordinary shapes; the adapter calls the
   existing nonvirtual FE update with its existing identity-frame argument.
4. Keep concrete FE visualization, point-to-point joint visualization, and the
   system-level visual manager in their actual higher-level owners. Remove an
   unnecessary physics-item include from generic shape code only after compiling
   the resulting narrow target.

Do not add a global optional hook or disable the FEA feature to manufacture a
standalone build. Do not virtualize the existing nonvirtual FE update method:
that would change derived-class behavior. Preserve the stored `obj` semantics.

`ChVisualModel` currently archives only its ordinary `m_shapes`; the FE shape
archive calls are commented out. Preserve that scope. Do not serialize function
pointers or claim FE visual restart. Copy/assignment must retain the updater
consistently with the shared FE shape vector; clear/re-attach must remain correct.
Adding a private pointer changes the C++ object layout, so rebuild all owned
consumers together. Source and archive compatibility are not old-binary ABI
compatibility.

Gates:

- Generic object + ordinary visual model links with FEA enabled but without any
  FE mesh, element or FE visual implementation, and without the umbrella library.
- Actual transitive header, compile-owner and linker-owner assertions pass.
- Attachment counts, owner identity, update order, sharing/copy, clear/re-attach,
  archive round trips and generic factory retention preserve current behavior.
- Before/after FE geometry, color and glyph results match on a focused coupon.
- Native viewer/value tests and accepted-archive replay/rendering remain valid.

## Continue toward standalone FEA and MBD

Resolve these independently, with a focused regression before each next step:

- `ChMesh` reads concrete `ChSystem` gravity, thread and setup services. Introduce
  the smallest shared service view without introducing another clock or state owner.
- `ChAssembly` owns concrete body/link/shaft/mesh collections and their archive
  order. Put mixed composition above the domains, preserving offsets and iteration.
- Generic contact reporting depends on concrete bodies; tuple algebra also has a
  contactable dependency back into higher-level code. Separate neutral algebra
  from domain reporting/response adapters without duplicating solvers.
- Split FE nodal loads and builders from their concrete body/motor attachment
  helpers. Keep neutral frame constraints distinct from body-specific coupling.

Final standalone gates must execute real MBD and FE cases without the opposite
domain's concrete implementations. The coupled flexible-beam/rigid-body gate
must preserve a common assembled solve, reactions, moments and constraint work.
Only then migrate further domain packages against `CAPABILITIES.md`, retaining
CPU implementations until their CUDA coverage is separately qualified.
