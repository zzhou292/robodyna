# Physical replay camera clipping

The physical replay wrapper derives clipping from the actual initialized VSG
camera and the bounds of every archived position plus any additional wall mesh.
It changes only the perspective near/far distances, before the first render.
The camera pose, field of view, original coordinates and deformation scale 1 are
unchanged. The full-motion bounds are already scanned to choose the default
camera; storing their six doubles fits the existing scene bookkeeping reserve.

The near plane is one ten-thousandth of the smaller camera distance and scene
diagonal, reduced further to at most half the nearest box depth when the whole
box is in front. The far plane includes the furthest corner with a relative
margin. Invalid, degenerate or float-unrepresentable projections reject before
rendering. A box crossing the eye plane is reported; changing clipping cannot
make geometry behind a chosen camera visible or guarantee lateral framing.

The capture manifest records scene bounds, near/far planes, camera distance,
scene diagonal and minimum/maximum camera depths in metres. The console prints
the clipping distances too. The pure owning tests cover millimetre, centimetre
and vehicle scales, corner coverage, translated/rotated views, eye-plane
crossing and failure preservation. Actual capture qualification additionally
requires inspection of the first, middle and last frames for intended full-mesh
visibility; numeric clipping checks do not replace that review.

This is an application wrapper around the existing renderer. No Chrono or VSG
third-party source or physical archive is changed.
