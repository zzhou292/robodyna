# T3 and mixed contributor unit gate

Frozen before first execution. `utest_nodal_wall_native_cuda` contains five
CUDA test functions in `NodalWallNativeTest.cu`, with host reference preparation
in `NodalWallNativeFixture.cpp`. No execution or pass is claimed here.

The ordinary tuple is κ = 16 N/m³, maximum depth .5 m, initial positive depth
1/32 m and h = 1/1024 s. Native startup uses E = 2e6 Pa, ν = .3,
ρ = 1024 kg/m³ and t = 1/32 m. The independently checked T3 mass uses the
qualified native angle weights; contact uses immutable area divided by three.
Mixed mass and **total** isotropic inertia come from `ShellBatchBinding`, in
native Q4-then-T3 order, and are checked against the existing long-double
world-edge/atan2 oracle. Physical plus area-added inertia is never added to the
native total a second time. No native material/history update occurs here.

The existing absolute budgets remain 5e-7 N per parent force and
1.2500000000000005e-12 J per parent potential. Arithmetic comparisons retain
2e-12. Independent work checks retain 256 ε times the sum of absolute terms
plus 1e-12 times 1/64 J; impulse and moment checks use the inherited 1/1024
scale in their respective units. Long-double load/energy oracles must lie
within the reported binary64 intervals without adding a test allowance.
Long-double division by three is a higher precision calculation, not an
exact-real predicate.

The first function also evaluates the source parameter tuple κ = 4e5 N/m³,
depth scale .00025 m and maximum depth .0005 m, with exactly the same accuracy
budgets. These probes stop before owner seal/advance. They supply no source
timestep or coupled-response admission. The inherited κ = 1e308 and 1e296
force/energy budgets occur only in the explicit late-overflow failure probe.

1. One and two scalene native T3 parents plus the mixed five-node shared edge
   check actual native mass/J, independent force, potential, world moment,
   surface power, exact host/device result fields, additive destination loads,
   finite face identity and the private all-active rate bound. The two T3-only
   layouts leave global node 0 fully fixed and outside every parent. In the
   mixed layout, penetrating global 0 belongs to Q4 and is absent from T3
   `{1,4,2}`: it is the decisive unused-fourth-slot regression. T3 force slot 3
   must remain exactly zero.
2. Two T3 and mixed fixtures run eight contact-only intervals. Independent
   velocity, position, kinetic change, base-force impulse and separate kick /
   drift work check the first h/2 kick and subsequent h kicks. Candidate
   evaluation leaves assembled forces and accepted state unchanged. Owner and
   contributor allocations remain unchanged throughout.
3. The T3-only vertex of the mixed patch enters and leaves contact. Certified
   pressure activation, zero-pressure release, impulse and endpoint velocity
   show that endpoint contact never enters the kick that produced it.
4. Fresh cyclic and parent-order permutations include exactly edge-on
   reference planes, an actual projected seam point, both wall diagonals,
   subdivision and reversed face order. Queries are checked against the
   original complete finite-face scan, with independent load/moment checks.
5. A late second-family accuracy failure, foreign zero inverse mass, candidate
   cap/outside/nonfinite and fixed-node motion faults preserve output/state.
   Discard and clean retry are checked against the host oracle. A late finite
   destination overflow after a preceding Q4 destination is staged preserves
   all six destination arrays, including a nonzero seeded couple.

Single T3 uses global nodes `{1,2,3}` and two T3 use `{1,2,3}`, `{2,4,3}`.
Mixed uses the retained flat `Edge5` geometry and Q4 `{0,1,2,3}`, T3 `{1,4,2}`.
All immutable references are prepared once. Edge-on probes use a 1/8 scale;
subsequent current-coordinate probes remain prescribed contact experiments.
Sorted parent IDs remain ascending independently of the supplied input order.

Storage remains exactly 99,384 bytes in one contributor allocation, with
128 workers and the same fixed stride-four private share storage. The owner
retains six allocations. The gate admits only contributor behavior for at
most two native parents and the existing owner capacity. It provides no
mixed shell feedback, full coupled recurrence, source-part dynamics,
94-parent capacity increase or vehicle performance claim.
