# Active nodal wall contact storage

The fixed `NodalWallDeviceResults` ABI remains 79,248 bytes. Default device
configuration still admits 128 parents, 128 compact nodes and 128 global nodes
within 512 KiB. Explicit count limits admit up to 1,024 parents and 2,048 nodes;
the hard device cap is 8 MiB. This changes storage admission only: the native
Q4 A0/4 and T3 A0/3 contact law, certified arithmetic, isotropic mass admission,
64 workers, source parent/member reductions and planar triangle query remain.

`NodalWallContactArena` lays out one device allocation containing the header,
immutable model arrays, compact status, four shares per parent, six global
scatter arrays, addition uncertainty, and both base/candidate results. Every
extent is computed before borrowed positions or mass are read. A retained host
arena constructs typed records and stages complete readback. A separate host
header rebases pointers to device addresses; the host never reads a pointer
field by dereferencing the device header. Publication follows complete model
preparation and both initialization copies. No per-step growth occurs.

Host admission includes the full retained arena and a 512 KiB reserve for
header shadows, model/layout objects and the existing finite-capacity wall
validator's temporary containers. Caller storage and allocator bookkeeping are
excluded. Default host admission is 1 MiB and its hard maximum is 16 MiB.

Large output uses `NodalWallDeviceResultView`: diagnostics, parent records,
compact node records and face IDs belong to the caller. Capacities must exactly
match the active model. All ranges must be aligned, finite, and disjoint from
one another, the view and expected diagnostics. Legacy readback rejects a large
model before reading any output or expected record. All reads finish in private
staging and diagnostics identity is checked before any caller output changes.

The synthetic capacity fixture contains exactly 804 Q4, 111 T3 and 1,030 incident
nodes. Its nodal mass and J are summed from native QEPH/T3 startup. The shared
geometry fixture also exercises host weight ownership. It is not source Yaris.
The active CUDA gate uses a real 1,030-node `FENodalState`, checks every node and
parent's analytic force/area coverage over three accepted intervals, and checks
fixed device allocation counts and no intercepted `cudaMalloc` calls during
stepping/readback. A separate extreme-load assembly gate forces
overflow at global node 1,029 after all earlier nodes were privately staged;
every physical force remains unchanged and a clean retry is exact. Readback
checks cover legacy rejection, active capacity/alias failures, stale identities
and an injected third-copy CUDA failure after two successful staging transfers,
without partial caller publication. These linker wrappers exist only in the
active CUDA qualification executable; production has no fault-injection hooks.

Targets added to the existing opt-in contact harness are
`utest_nodal_wall_arena` and `utest_nodal_wall_capacity_cuda`; the five prior
contact targets remain required. Existing host model gates now own their
prepared arena rather than copying a fixed array aggregate. This qualification
does not establish shell/contact dynamics with nodal-rigid constraints: those
must supply and qualify the constrained contact mass metric separately.
