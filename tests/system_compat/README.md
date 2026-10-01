# System-family compatibility baseline

These tests precede the actual `ChSystem`, `ChSystemNSC`, and `ChSystemSMC`
implementation move. They use inherited public names intentionally; aliases must
preserve the same behavior afterward. They still link the native aggregate.

First qualify `//tests/system_compat:runtime_test` and build
`//tests/system_compat:write_baseline`. Root runs the producer into a new output
directory under the existing workstation guard, freezes its original bytes and
source/toolchain provenance, then selects the frozen archive/integrity tests.
The fixture directory is not populated automatically by the tests.

The producer exercises concrete NSC and SMC object tags, stored gravity, time,
step metadata, solver identity, sleeping flag, SMC settings and assembly body
identity/owner restoration in JSON, XML and binary. It records the actual
`typeid(ChSystem).name()` and verifies that its version key occurs in the JSON;
do not substitute a guessed compiler mangling during the rename.

One graph is constructed first in both producer and writer-test processes, before
other `ChObj` allocations. This preserves the actual auto-generated assembly
names without modifying protected state or normalizing archive bytes. Repeated
writes reuse that graph. Fresh-process regeneration must additionally match
before root freezes the fixture.

The scope is deliberately limited to what the existing archives store:

- System name, collision system, contact container and timestepper are not
  serialized by the current base implementation.
- SMC stiff-contact state and custom force-algorithm ownership are not restored.
- The NSC contact-container constructor alone does not initialize its bounce
  threshold. Live tests set it explicitly; archive checks do not read an unset
  restored field or invent a documented default.
- Stored `MultiStep` enum metadata does not demonstrate persistent friction;
  the inherited SMC implementation documents its fallback to `OneStep`.
- No claim is made that these object archives can physically restart a solve.

Four runtime cases cover constructor/static-creation profiles, old factory tags,
contact-method-specific container admission, and real NSC/SMC time advancement
with callback registration/removal. An existing FE-node/body constrained-spring
gate is reused for the common mixed-system clock. These are bounded CPU tests,
with one requested thread and no renderer or GPU solver.

The source audit and future rename must preserve contact law distinction, solver
selection, ownership, callback ordering, state layout and archive tags. A failure
on unchanged source is evidence to investigate, not permission to mix numerical
or persistence changes into the rename.
