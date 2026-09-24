# Zero-neutral native storage domain

Status: isolated source implementation; unbuilt and unqualified. The selected
combined vehicle source is unchanged. This slice changes storage admission only,
not arithmetic formulas, geometry, work, source identity or projection shortcuts.

## Proof against the actual helper graph

Arithmetic::Add checks IsZero(a.numerator) then IsZero(b.numerator) BEFORE choosing
a minimum exponent or shifting either operand. A zero cannot force a nonzero
coefficient onto its stored -1074 lattice. Subtract delegates to Add/Negate;
Compare delegates to Subtract/Sign. Cross/Dot and every geometric comparison use
these helpers. A source audit found no other dyadic alignment or direct exponent
comparison in Geometry or CellKernel.

Exact can retain a zero exponent; Multiply may sum zero exponents and At may
subtract sample depth from them. Those representations remain zero. Scale by
zero and cancellations likewise produce zero. No operation converts a zero
numerator back to nonzero without Add returning the other operand. Exponents
still remain in the original bounded integer range (degree<=4, depth<=53), so
leaving them unnormalized cannot overflow an int.

For nonzero input components, let Emin/Emax be the ORIGINAL unnormalized decoded
binary64 exponents, D=max_depth+1<=53 and B=53+Emax-Emin+D. Do not remove trailing
significand zero bits or ignore nonzero subnormals. A nonzero degree-k value's
stored exponent is at least k*(Emin-D): Add aligns only two nonzero values of the
same degree, Multiply adds exponents, At decreases coordinate exponents by at
most D, and scalar factors do not lower exponents. The earlier complete helper
bound4B+9 and rounded4+4/6+2 multiplication bound therefore apply unchanged.
Zero intermediates are exempt from this exponent lower bound precisely because
they cannot participate in alignment. An all-zero input has no nonzero arithmetic
and still executes ordinary degenerate-geometry classification.

The existing fixed core preserves overflow before its zero-product/zero-shift
returns. FixedIntegerPolicy::Sign and Checked latch invalidity; Add's IsZero uses
Sign, and the context clears only at pair entry. No policy/core/context helper is
modified. Existing forced-out-of-domain work0/work1/sticky-fault tests stay in the
required suite. Checked Boost's normalized zero has one zero limb and multiply
uses the scalar zero return before any resize/Karatsuba path. Newly admitted
nonzero intermediates fit8limbs and cannot allocate in the original or selected
backend. The existing version/64-bit/cutoff40 gates remain.


The degree induction also covers cancellation-created zeros: Add can retain the
lower input exponent when a-b cancels, but any later Add bypasses that zero and
any later product stays zero. The only nonzero constants entering arithmetic
are uint64 interpolation factors (<=2^53) and RegularCell's factor4 via Scale;
they preserve spatial degree and do not introduce a separate dyadic lattice.
There is no dyadic nonzero constant added to a geometric homogeneous polynomial.
Default scratch/hull values are zero and are overwritten or stay neutral. Thus
neither zero provenance nor scalar constants create a hidden lower exponent for
a nonzero degree-k term. Future nonhomogeneous helpers need a fresh audit.

## Keep storage and shortcut authority separate

ExactProjectionDomain::FromPaths remains the original stored-exponent calculation
including zero. The same scanner has a private compile-time nonzero variant,
accessible only to NativeStorageDomain. Storage initially uses the original proof;
only a supported candidate wider than125bits needs the additional zero-neutral
scan. The diagnostic report retains original `projection` and adds `storage`.
CellKernel and its relative-face, common-translation, EE and endpoint shortcuts
continue to call the original public FromPaths. Broader storage admission must
never accidentally broaden those algorithms, because additional separation
proofs could change work/status even when mathematically sound.

Both CPU and device continue to derive admission from the same actual immutable
pair. No caller flag, arbitrary public backend selector, retry, dropped pair,
cohort/work-cap change or geometry translation is introduced.

## Measured corpus provenance versus inference

Source inspection of the unchanged maintained benchmark Classes/MakeCases shows
that its1,024 wide rows at4,096pairs are exactly four synthetic families,256each.
At max_depth20 their original B values are1097(VF,coplanar,box-touch) and1107(deep
dyadic); ignoring exactzero gives75,75,75,85. The remaining3,072 Yaris-derived rows
were already admitted. These values were checked from the literal binary64
endpoint arrays; no numerical benchmark was run for this slice. Preserve the
ENTIRE original4096corpus and original limits for eventual CPU/GPU ABBA. No
vehicle eligibility proportion or speedup is inferred from this synthetic split.

## Qualification still required

Compare full wide/adaptiveBoost/fixed8 results, work and witnesses on original
zero-coordinate families, signed/allzero, cancellations, B125/B126, depth0/20/52,
subnormal and huge scales, true mixed nonzero exponent fallback, malformed and
unsupported input, sticky checked failures, worker/publication/cap/retry paths.
GPU tests must exercise newly admitted rows and preserve a genuine NONZERO wide
fallback corpus rather than deleting old fallback assertions. Existing native
math body pins remain unchanged. Requalify compiled worker stack and CUDA
resources; then time full original mixed corpus, without dropped rows or shifted
geometry. Keep this branch unselected until all gates and measurements pass.
