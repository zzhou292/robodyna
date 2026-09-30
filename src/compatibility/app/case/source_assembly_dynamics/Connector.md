# Optional TYPE25 case contribution

The case attaches the immutable source connector model and combined nodal M/J
to the same owner as its QEPH/T3 batches. `ConnectorWorkspace` contains one TL
batch and two copied result buffers. Those buffers use the existing case sample
selection; they create no independent state owner, token or clock.

Startup binds original geometry with uniform translation/zero spin and the
native zero-force cache. It checks the native property timestep before a step.
Accepted forces assemble in QEPH, T3, TYPE25, wall order. The owner advances
once; all candidate results, signed native work/activity and native timestep
bounds are checked before the common three-participant publication. Rejection
discards all trials together. Accepted connector views become visible only with
the common commit. The publication is destroyed before its borrowed connector.

Owner inverse coefficients, wall startup energy, stored kinetic/kick checks,
and optional force-stage observations use authoritative combined nodal M/J.
Connector translation/rotation subtotals are already inside ordinary-node
kinetic totals; shell physical/drilling J channels retain their original meaning.
The group replacement remains unchanged because attached endpoints are ordinary.
The existing native-dt safety fraction is also applied to every connector.
No connector channel work is treated as dissipation or added to shell plastic
work. There is no new physical-energy tolerance.

Explicit connector host/device reserves apply only when a source connector is
present. Device allocations are included in the common allocation report.
The host budget includes both copied result arrays and the declared resident
batch host reserve. No allocations occur while stepping. Optional stage timing
keeps its existing24 named stages; connector work appears in the inclusive step
time until separate connector timing channels are introduced.

The host observation gate covers independently calculated combined-M/J kick
and force-stage partitions plus corrupted channels/load/capture rejection.
The actual-case CUDA gate covers512 steps at8h with the959-shell/1093-node
source, stable allocations, complete accepted connector identity and contact;
late work/phase/wrench rejection with exact retry; and source/cap rejection
before owner allocation. These runtime tests require the root's serialized GPU
qualification; author host syntax is not runtime evidence.
