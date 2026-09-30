# Explicit QEPH uniform-translation startup

Prospective source-only gate. Five new CUDA functions live in
`QephUniformStartupTest.cu`; no numerical execution is claimed.

`QephBatchConfig::startup` defaults to `ReferenceRest` with zero declared
velocity. That path retains its original validation/scatter arithmetic and
zero initial diagnostics. `ReferenceUniformTranslation` is an explicit
standalone `CoupledForces` declaration with a finite common world velocity in
m/s. It is forbidden for joined publication and prescribed-field batches.
The first supplied assembly must contain reference-coordinate bits, exactly
the declared velocity bits at every node, zero omega, identity quaternion and
the existing free native shared mass / total isotropic inertia. No initial
material stress, spin, independent nodal velocity field or force evaluation
at dt=0 is supported. The geometry producer is unchanged; the first numerical
qualification is restricted to the planar fixture below, not arbitrary poses.

Declared kinetic arithmetic is preflighted before allocation. The device then
validates all actual sources and successfully assembles every accepted element
before measuring initial translation kinetic energy in control scratch. Only
after successful kernel/readback synchronization does the host publish those
kinetic fields with the first numerical binding. Late failures do not publish
partial K0 or overwrite accepted diagnostics. Rest/joined paths skip the new
measurement. No slab, allocation, time advance or material/history call is added.

First uniform binding requires `AssembleAccepted(owner, view)`. Before any K0
measurement, the existing host-only owner predicate authenticates the supplied
source pointer identity; it neither dereferences expired views nor consumes CUDA
errors or grants force-destination access. No owner pointer is retained. The raw
overload rejects first uniform binding, while legacy rest and already-bound
assembly behavior are unchanged. The retained first source identity is checked
again before first joint publication. A foreign declared-speed buffer cannot
expose nonzero K0 for an actual owner at rest. Caller results remain unchanged
on failure, and a genuine owner can retry after rejected first binding.

Frozen ordinary fixture: one/two adjacent 20 mm flat Q4 cells, normal world X,
reference X=-.000375/4 m, E=200 GPa, nu=.3, rho=7890 kg/m³, t=.001648 m,
physical initial v=(8,0,0) m/s and omega=0. Steps are H0=2^-24 s and H0/2,
two intervals each. Qualified native startup supplies shared m and total J.
Zero-load flight and a separate known uniform 1 m/s² force probe check the
first h/2 kick followed by h, with every drift h. The latter uses per-node
world force m_i times the declared acceleration and no contact. The paired
group requests 24 native cell intervals in total; startup never calls native
force at dt=0.

Existing native/owner parity and work-ledger budgets are reused. Initial K0
uses 256 epsilon times the sum of absolute kinetic terms plus 1e-12 times
`.5*rho*t*Side²*8²*cell_count` joules. Independent zero regular rates use the
existing 2e-12 arithmetic factor with Speed/Side and Speed/Side² scales;
hourglass rates use Speed or Speed/Side according to their existing units.
Nothing is snapped to zero and no measured failure can widen these budgets.

The five functions cover initial nonzero K0 plus zero history/cache; actual
native/CUDA first-half/full-kick flight and known-force parity; malformed
metadata, finite-term aggregate overflow and joined-mode rejection before
allocation; late velocity/omega/q/mass/reference/scatter failures preserving
unbound output; rejected first publication / half-kick retry, foreign-source
publication rejection and CUDA readback poison preserving caller bytes.
The overflow-only fixture retains unit-size native batch geometry and
v=(4e153,0,0), whose finite individual terms overflow their aggregate; it is
not an admitted motion or altered physical tolerance.

The appended startup record is expected to add 32 bytes to the existing
config/model/storage layout (to be measured by the parent ABI probe). Control,
result slabs, owner storage and allocation counts do not change. All previous
standalone and joined regression suites remain required. This gate establishes
known-zero-internal-force startup only: no CW1 screen selection, incoming
contact, long response, mixed force feedback or vehicle admission follows.
