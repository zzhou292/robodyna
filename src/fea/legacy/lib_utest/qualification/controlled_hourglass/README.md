# Controlled LAW42 hourglass value qualification

This isolated leaf ports the complete selected LAW42 branch of a62b27e6 SHOUR_CTL.
It is SI, alpha2/no-Prony, OFF1, DN.1, with Poisson ratio at most the promoted
Fortran REAL literal0.48999f. Native dt0 and rawSTI0 queries are admitted.
It does not enable any vehicle profile, H24/S6 adapter, or distortion mechanics.

State carries12 force-valued FHOUR entries, distinct from legacy Pa history.
Input velocities/forces use native eight-slot local order. Projection[4][3]
means PX1..4 by H1..3. Result work is the actual native dt times modal power,
not an energy difference; internal energy remains density and STI remains raw.
Invalid/nonfinite results leave the complete output and input State unchanged.

The independent Fortran wrapper executes the unchanged complete SHOUR_CTL with
true MVSIZ129 backing, real reader/update PM slot producer, and reversible
output-only modal/work observations. Source bytes/flags come from the qualified
full H24 fixture. No production C++ expression feeds native arithmetic.
The63-double comparison packet is history12, force24, energy/STI/work3,
modal velocity12, modal force12. Every channel except signed work retains its
native-scale 128-epsilon bound with no absolute floor. Signed work is checked
against its own exact binary64 Z/X/Y ordered modal replay on both sides, plus
the derived forward-error bound in [WORK_COMPARISON.md](WORK_COMPARISON.md).
Its modal force/rate inputs remain subject to the original 128-epsilon bounds.
The earlier proposed 128-epsilon native absolute-product scale is superseded;
it was never accepted as qualification.

The fixture includes raw operand/projection basis cases; these check the leaf
binding and are not claims that arbitrary projections describe a physical cell.
The full native H24 coupon separately qualifies its actual geometry/caller.
Configure with TL_CONTROLLED_HOURGLASS_CUDA=ON for the mandatory actual-device
comparison. GPU tests fail when no device is available; none silently skip.

Fourth-mode work follows the native modal-power expression despite its different
rate/scatter normalization. It is not asserted equal to a generic nodal
force-velocity ledger; complete mechanical-energy accounting remains separate.
