# Four-point QBAT force/history value

This implements the selected OpenRadioss IHBE11, TYPE1, centered, explicit
ISMSTR2, NPTR2/NPTS2/NPTT1, IDRIL0, ITHK1/ IPLAS1 recurrence. Each of the four
surface points owns one LAW44 material and constant-D1 failure history.
It is a prescribed value API, with no resident contributor, source admission,
nodal clock, mass removal or contact deletion.

`InitializeHistory` creates a virgin history associated with a prepared
reference and immutable material parameters. `PreparePrescribedHistory`
imports only a finite packet in the same selected domain, for prescribed
qualification; it does not authenticate a solver restart. Borrowed curve
storage must remain immutable and accessible on the executing host/device.
`EvaluateForce` takes endpoint positions and midpoint velocity/spin from
`qeph::PrescribedInterval`; the base time and next sample must match history.
The timestamp of new point failure is the geometry endpoint.

The shared `sections::UpdateMembraneLaw44Point` performs GS0 constitutive
update, sequential elastic/plastic physical-thickness additions, rounded
delta-PLA work and constant failure, then saves masked stress. It does not
perform parent removal or quadrature. The existing positive-GS shell entry
retains its old validation and arithmetic.

QBAT retains separate current material stress and cached FORPG stress
(including both viscosity stages). Parent removal is tested only at the fourth
surface point. Earlier points' already assembled in-plane forces remain in
that removal packet. On the next OFF0 interval the native predictor, filter
and strain operations still execute, while current forces become zero.

Running thickness changes after every material call by the native quarter
correction. CBAFORI1 uses the saved pre-point volume; CBAVISC uses the corrected
thickness. Initial aggregate volume and final shear-work thickness remain
distinct. GEO13 zero resolves to force DN=.001, while the already qualified
startup CNDLENI retains its own input semantics.

EINT[2], WPLA and EVIS are distinct native work ledgers. EVIS follows the
original expression, including its omission of the in-plane FXY product.
WPLA must not be added to EINT as another internal-work contribution.
The selected NPTT1/IDRIL0 moments are invariant zero: no mutable MOM history is
exposed; returned couples and bending EINT explicitly expose those zero
channels, and transverse/bending strain and saved-stress admission is checked.
No complete energy-balance identity or energy residual tolerance is claimed.

All output is staged locally, including history. Failure preserves output
and accepted values; a successful call may publish into a trial containing
the borrowed accepted history. Compare/serialize named fields, never padding.
The original public geometry entry still admits OFF1 only. Its private
position-only helper is shared by the explicitly checked OFF0/1 force path.

Source and qualification: `lib_utest/qualification/qbat_force/README.md`.
The incoming QEPH placement extension must retain QBAT's centered-only guard
and include placement in complete reference identity; this increment is based
on TL 22f0755, before that added field.
