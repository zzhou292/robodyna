# TYPE45 working-length comparison

This is a test-only correction. Production equations, source inputs, loads,
steps, native donors, and the base `2e-12 * max(1, abs(expected))` comparison
remain unchanged. SI profiles use exactly the previous comparison. Millimetre
profiles add a length sensitivity allowance only to local/world translational
forces and the resulting offset couples. Scalar material couples, work, all
stiffness/damping values, frames, rotations, phase and reference checks keep
their previous tolerance.

## Preserved failure and exact diagnosis

The first frozen root gate failed 39 host/native checks: 12 local forces,
24 world forces, three lever couples. All were millimetre profiles. The
original 44 geometries added 648 local/world force failures, maximum
`1.0710820674830757e-8 N`. No other compared channel failed. Reports remain at
`crash-work/reports/type45-joint-root-tests-1` and
`type45-diagnosis-{labeled-native,summary}-1`.

In the first spherical step, both implementations give
`K = 62941535.053423554 N/m`. The SI local z displacement is
`-1.7944346368363451e-7 m`; projecting and subtracting the millimetre packet,
then converting its result, gives `-1.7944346368459208e-7 m`.
The force difference is `6.4433791635565285e-11 N`. The otherwise identical SI
native packet matches the production force exactly. This is cancellation
followed by stiffness/damping amplification, not an altered material response.

## Scope and error propagation

`NativeLengthRoundoff.cpp` evaluates current and reference projections from
exact represented SI and native packets. Native coordinates are the exact
binary64 results of the same divisions used by `NativeOracle`. It uses each
implementation's observed frame; those frames still undergo the independent
original frame/reference comparisons. This is conditional sensitivity to the
checked frames, not a general forward-error proof for arbitrary rotations.

For a projection, the absolute term sum is
`S = sum(abs(R[j] * (x2[j] - x1[j])))`. A conservative
`gamma12 = 12*epsilon / (1 - 12*epsilon)` covers the three coordinate
subtractions, products, sum, dimensional conversion and the wider-precision
check's own smaller error. Both current/reference projections and their
subtraction are bounded. The observed displacements and separations must pass
these stricter operation-scale checks before a force allowance is computed.
The difference between the two precise projected packets explicitly includes
the SI-to-working conversion's loss; it is not hidden in a relative force
factor. The long-double evaluation has at least 64 significand bits on the
owning host; the witness independently uses 100 decimal digits.

For `F = K*d + C*(d-old)/dt`, the comparison propagates the current displacement
bound through `abs(K) + abs(C)/dt`, includes the actual already-checked old
history difference, the two prepared coefficient differences, and arithmetic
error proportional to the separate elastic/viscous term magnitudes. It then
propagates local force error through the frame and the literal
`0.5 * separation cross force` lever. The comparison receipt records previous
history only after the successful native comparison step; rejected trial
attempts do not call native Step or advance that receipt. Nothing is persisted
by a runtime producer.

This covers the supplied scalar, finite, non-overflowing mm/tonne/second
fixtures and their selected positive-step recurrences. It does not authorize a
new source default, owner clock, arbitrary extreme-scale data, or relaxed
mechanical admission.

## Independent controls

`NativeLengthWitnessTest.cpp` evaluates the scalar force definition at
100 decimal digits from each exact packet, separately for production/native
projection and retained old history. It checks both against their own
projection/arithmetic bound, across eight accepted steps with ordinary and
large translated coordinates. It also rejects `1e-4 N` force and `1e-6 N m`
couple corruption in every component/step. Existing device-owned failed
trial/exact-retry tests consume the same comparison helper. Host/native and
CUDA use the same complete native oracle; native inputs never use a production
reference or the comparison receipt.
