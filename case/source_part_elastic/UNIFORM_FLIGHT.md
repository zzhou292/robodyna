# Original source part uniform-flight qualification

This experiment reuses `SourcePartElasticCase`, its one TL nodal owner and clock,
the original source collection/binding, both shell families and the common
publication. `Experiment::ElasticPulse` remains the default. Its force shape,
evaluation order, stepping and pulse archive remain unchanged.

`UniformFlightConfig(1)` selects the original 117 nodes and 94 parents, reference
positions, identity orientations, velocity `(1,0,0)` m/s and zero angular velocity.
It uses native source density/thickness and native total isotropic inertia,
including added inertia. No pulse is constructed, allocated or launched. The
explicit startup declarations are passed to both joined TL families, which bind
the actual owner before the common publication measures initial kinetic energy.
No zero-length material interval or startup force cache is synthesized.

The case exposes measured `initial_kinetic_energy()`. Initial raw and synchronized
velocities come from the accepted owner readback. Common epoch-zero kinetic energy
is populated once; family kinetic is unavailable/zero and common base kinetic
remains zero until a completed interval exists. Subsequent energy admission uses
`K_sync + EINT0 + EINT1 + EVIS - K0`, with the existing `1e-10` J absolute floor
and the unchanged strict discrete kick-work check. The initial owner step still
uses a half kick. Raw velocities remain at the previous midpoint after each step;
synchronized fields are derived using the complete new endpoint internal force.

Uniform mode requires a finite nonzero initial velocity and neutral pulse fields:
duration/amplitude zero, spatial axis zero, direction `(0,0,1)`. Pulse mode
requires initial rest. Unknown modes and contradictory declarations are rejected
before allocation. The existing pulse archive rejects uniform mode before creating
files; this change does not introduce a moving/contact output schema.

## Frozen gate before first execution

The new tests run exactly 64 steps at `h = 2^-24` seconds (3.814697265625 us),
with all 88 QEPH and 6 T3 parents checked against their native interval oracles
at every accepted step. This is a rigid-flight qualification, not a crash demo.
Refinement factories retain the same velocity and use `h/2` or `h/4`; this patch
does not claim those longer refinements have run.

The direct rigid-translation oracle uses long-double arithmetic. Let
`r = 2e-13 * (accepted_steps + 1)`, `X` be the largest actual source-coordinate
magnitude, `V` the declared speed, and `L` the shortest source-parent edge. Frozen
per-component bounds are `r*(1+X+V*t)` m for position, `r*(1+V)` m/s for raw and
synchronized velocity, that velocity allowance divided by `L` for angular
velocity, and `r*(1+V*t/L)` for orientation. Chord/relative-displacement bounds
are `2*sqrt(3)` times the position allowance; strain, thickness-curvature and
area/thickness ratio allowances use position allowance divided by `L`.

Native reference nodal mass and total inertia are scattered independently in
long double. Their source agreement and initial energy allowance retain a
`2e-12` relative coefficient. Momentum bounds use total native mass times the
velocity allowance plus native reduction allowance. Kinetic bounds follow the
quadratic velocity/spin bounds using the same native mass and total inertia.
These are fixed qualification arithmetic allowances, not relaxed production
limits. The existing per-step native force/history comparisons and `1e-10` J
case energy admission remain independent checks.

The suite additionally rejects the first moving trial after both material
families and common kinetic diagnostics prepare, verifies accepted owner fields,
initial K0, both history slabs and work accumulators survive, then checks an exact
retry against an unfailed case and the native interval. The allocation footprint
must remain constant through all 64 accepted steps. Run the existing pulse suite
and frozen pulse scientific-byte comparison after integration as regression gates.

## Passing integration

All three new functions and the three existing pulse/native functions pass at
TL `7335646`. The 6,016 uniform-flight native cell intervals report zero measured
position, raw/synchronized velocity, angular velocity and chord error. Measured
common K0 is 0.12825446944093644 J; maximum energy residual is
2.0328790734103208e-17 J. The XML retains these measured properties in
`crash-work/reports/source-uniform-flight-tests-1.xml`.

The retained 64-step pulse prefix preserves all 21 scientific files / 695,141
bytes exactly, as well as every scientific final metric. Only elapsed time,
the 32-byte T3 startup record and their inventory metadata differ. Explicit
device allocation for the pulse case is now 819,959 B. See
`source-uniform-flight-pulse-parity-1.json` and its two archived inputs.
