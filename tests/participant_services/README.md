# Participant-service behavior baseline

All twelve cases passed before introducing the service interface. Evidence:
`crash-work/reports/robodyna-participant-services-baseline-2.json` in the outer
workspace. The initial compile failure was in test access to a private Body
override; the helper now invokes the existing public PhysicsItem virtual method.
Production visibility and behavior were unchanged.

Tests cover distinct invalidation scopes, direct owner reassignment, current
gravity for bodies/nodes/a real tetrahedron, detached copies versus assignment,
owner propagation and destruction, attached-object archive reads, and unchanged
node/element initialization hooks. They use existing public APIs and supplied
time; they introduce no second clock, cached owner or hidden state.

The same cases must pass after the service change, together with frozen archives,
coupled dynamics and source/ownership checks. Passing this baseline does not
establish independent FEA/MBD linkage.
