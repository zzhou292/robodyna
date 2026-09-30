# Native TYPE13 startup values

`Type13Startup.h` owns the pure property/reference/coefficient startup leaf. It
does not admit a connector into a nodal owner, tie its endpoints to shells, or
advance an element. The original Yaris beam endpoints require those later steps.

The source adapter supplies resolved native working-unit values. `PropertyInput`
requires four five-point curves and six explicit channel references, `Ileng=1`,
`H=1`, `Ifail=1`, `Ifail2=0`, no sensor/rate failure, unit curve scales and zero
damping. The remaining native selected-branch scales/offsets are fixed to their
resolved unit/zero values; there is no interface for silently ignoring a
different branch. Failure limits and weighting parameters are retained as
declarations; this leaf does not evaluate failure. Translation curves use strain
and force; rotation curves use angle per native length and moment. Translational
stiffness is force, and rotational stiffness is moment times length. These are
distinct from the TYPE25 length-independent spring coefficients.

`InitializeProperty(input, property)` checks all extents before reading borrowed
curves, stages four owned curve objects, and runs the RKINI3 maximum-slope loop in
native point order for every channel. Property copies own their curve bytes;
they have no borrowed pointers or lazy initialization. No allocation occurs.
The starter `XIN <= 1e-20` floor is applied in declared working units and its
added contribution is retained separately. Negative supplied inertia is rejected
by this bounded admission scope.

`InitializeElement(property, reference_input, startup)` consumes exact native
working-unit N1/N2/N3 positions. N3 only defines orientation. The input also gives
the resolved coordinate-noise amplitude, orthonormal fallback skew, and four
explicit zero release flags. R4BUF3's N3 alignment condition divides by the
squared N3 seed length; it must not be replaced by a normalized angular test.
Its Y/X fallback branches retain the donor arithmetic. Degenerate/undefined
native frames are rejected before publication. Both original length and SI
length/endpoint positions are reported. RMASS forms `.5 * property * length`
in native arithmetic before converting its two identical endpoint M/J
contributions to SI. The reported added inertia is a subset of total native J.

Both functions publish only after every check succeeds. This is value-level
failure atomicity, not an owner transaction. Objects may contain C++ padding;
serialization and comparisons must use named fields. The API carries no source
IDs: original keyword identity, conversion/default policy, and raw-to-SI source
association remain owned by robo-dyna's modelio layer.

OpenRadioss provenance: revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`, RKINI3, R4BUF3, RMASS, and
`hm_read_prop13` retained completely under
`lib_utest/qualification/type13/native/original`. Exact extraction boundaries and
hashes are in that directory's `source-manifest.json`. These adapted operations
are AGPL-3.0-or-later; see `LICENSE.md`.
