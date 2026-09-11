# Selected integrated circular beam reference

This value API prepares native TYPE18 circular four-point section, reference
orientation and PMASS coefficients. The source profile is ELFORM1/CST1/QR0,
converted ISECT2/INTR2 and declared Ismstr4, stored ISMSTR0. Source IDs, original
N1/N2/N3 and working coordinates are retained. N3 establishes orientation only.
The actual four Yaris parts have RT1/RR1/RT2/RR2 blank (zero), LOCAL2 and no
section offset. Unsupported releases or profiles reject before publication.

`Reference::section()` and `native_mass()` use the declared working units.
`endpoint()` converts native per-endpoint M, total scalar J, STI/STIR and the
explicit I7STIFS=1 interface coefficient to SI once. Each endpoint receives the
same native coefficient; this slice does not scatter a global nodal array.
The named axial/section/torsional intermediate terms preserve PMASS evidence.
They are not declared to be a physical/added inertia partition. No physical J
is inferred by subtracting an estimated regularizer from native total J.

The native orientation output is PEVECI's normalized SKEW seed, not a complete
orthonormal runtime frame. Native PCOORI tests use working-unit EM20 and the
absolute `2*EM06` cross-product threshold. An absent/endpoint-aliased or nearly
collinear N3 selects the native global Y/Z fallback; original source identity
is retained. A supplied alias must have exactly matching coordinate bits.

Native PMASS recomputes section moments from the four point areas including
area squared/12. It preserves the coefficient jump, short-beam multiplier,
torsional lower bound and native df0→.01 stiffness damping adjustment. Source
working values and every intermediate must remain finite and positive where
required by this selected profile. Failure leaves the complete public result
unchanged. No constructor dt, OFF, strain history, force recurrence, source
mass closure, ledger or resident/owner admission is supplied by this API.

The independent qualification uses complete PCOORI, PEVECI and PMASS, the exact
complete DEFBEAM_SECT body, and authenticated native material/default setup.
The first actual population gate is all142 original cells, with both original
diameters and independently gathered raw-mm coordinates. Root owns native and
CUDA execution; author host tests do not establish native parity.
