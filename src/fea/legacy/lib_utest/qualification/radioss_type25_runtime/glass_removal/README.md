# Genuine glass removal through the common native contact group

This small CUDA coupon retains the original Yaris material/section 2000452
(`589_doorfrontwindow`), LAW44 linear hardening, filtered-zero-C rate policy,
centered three-point section and TAB1 AnyPoint failure table. Source NUMINT=1
selects the failure rule; it does not reduce material integration to one point.
`source-2000452.json` retains the source blocks and their hashes from the existing
vehicle declaration report.

The geometry, load and contact coefficients are explicitly synthetic: one
fixed physical Q4 mesh wall, one deformable QEPH glass patch and one independent
elastic T3 patch (11 nodes). The owner advances at 150 ns with the unchanged
post-CIN structural screen and rotation limit. The fixture declares empty CIN;
the adjacent full-ledger runtime coupons qualify genuine CIN participation.

A bounded physical bending couple drives virgin glass to failure. Neither
material histories nor activity masks are overwritten. At removal at least one
thickness point must remain active, distinguishing AnyPoint from AllPoints.
The self interface uses I_DEL=1; the fixed wall uses I_DEL=0. They share one
owner and publisher, with independent authenticated contact sources.

Tests cover source staging without early publication, changed self operands,
unchanged wall operands, next-cycle search and retained-main release, three
subsequent steps, and late common rejection followed by an exact retry.
Rollback comparisons cover all owner coordinates/velocities/rotations/M/J,
both physical families' complete typed force/material/failure values, both
contact histories and all publication selectors. No raw struct padding is an
equality contract. This is a lifecycle coupon, not a vehicle trajectory oracle.

Both interfaces use the existing `initial_source::PrepareSource` and
`Transaction::GeneralInitialize` path, including genuine Starter-normal input,
fixed-wall ready-normal authentication, complete two-interface census and
source-derived geometric exclusions. This is the same initialization API used
by the vehicle. No history field is seeded by the test.

The contact coefficient range is explicitly[0,1e30], as in the existing
MovingCacheRig mechanics fixture. The reused source-admission helper defaults
KMAX to zero; carrying that admission-only default into the first version of
this coupon clamped every incoming contact coefficient to zero. The strong
wall-history and actual-force witnesses caught that fixture error. Its failed
receipts are preserved; GeneralInitialize alone did not fix that zero clamp.
The correction changes only the declared synthetic contact clamp, preserving
the original glass material/failure, geometry, physical loads and timestep.

The contact-response witness reads each interface's actual endpoint force and
couple additions separately, excluding STI-only changes. Both must be nonzero
before the removal commit. Retained glass and wall owners must also have real
nonzero saved stiffness; candidate counts alone are not a force witness.
