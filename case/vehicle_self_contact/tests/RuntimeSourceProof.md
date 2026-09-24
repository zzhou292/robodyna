# Runtime source proof

`verify_runtime_sources.py APP_ROOT TL_ROOT` checks source/build wiring. It does
not replace native host, CUDA, frozen-geometry or full vehicle qualification.

Local incidence is a premise of a continuous proof. The source check follows
`LocalContact.cpp` through native masked feature discovery, strict unmasked
finite-thickness separation and bounded whole-interval topology coverage.
`Candidate.cpp` sends ordinary first witnesses and unresolved work-limited
crossings through the full policy classifier. Its exact-translation route
requires the native invariant-path certificate. `Values.cpp` rejects a local
endpoint intersection unless its result carries `CertifiedLocalTopology`.

The former check requiring `LocallyExcluded` in `Candidate.cpp` described the
removed endpoint-only shortcut. The combined cone/roster qualification exposed
that stale assertion after all three legacy frozen replays passed. Updating this
source check restores the current ownership contract; it changes no numerical
implementation, geometry, force, work limit or historical evidence.

The local-topology check accepts the original direct implementation and the
private templated implementation used for cone-first search. For the latter it
follows the public wrapper's fixed `SharedVertexOrder::ConeFirst` call into
`CertifyQuadraticLocalTopologyImpl<order>` and then bounded
`CertifyQuadraticFacetCoverageImpl<order>`. Both layouts must retain exact source
vertex/edge identity, both endpoint intersection classifications and rejection of
nonlocal intersections before whole-interval coverage. The public signature has
no runtime order selector; production callers cannot invoke the private helper
or the comparison-only old-order entry points. These are source obligations,
not a substitute for numerical equivalence and CUDA qualification.

The same check also accepts the private `RootConeSearch {Original, AffineOnly,
Full}` template, whose public cone-first wrapper defaults to `Full`. It follows
`<order, search>` through coverage while requiring both search extensions to stay
at depth/path zero in the dedicated local-topology phase: affine paths use the
existing affine extension and non-affine paths use the curved extension. Original
successes, exact shared paths, nondegeneracy and both endpoint premises precede
the additional search. All three Bernstein arm controls propose directions;
`SharedVertexAxisSeparated` must still prove opposite strict signs over the full
bounds. Neither public headers nor production callers expose private selection or
comparison entry points. Historical direct, cone-first-only and affine-only source
layouts remain supported under their original obligations.

The candidate-publication check follows either the original direct definition or
both raw/indexed wrappers into the same private `ValidateCandidatePublicationsImpl`.
Both wrappers must be exact single delegates. The indexed local lookup retains
its raw fallback and `RequiresIntersectionAdmission` decision; canonical pair
accounting, source-key identity and unresolved rejection precede publication.
The local exclusion branch must still reject every endpoint-only witness before
publishing `ExcludedLocalIntersection`. The lexical index comes from the actual
candidate discovery cohort after prepared-intersection validation. No numerical
policy, tolerance, authority or old recovery source is changed by this adaptation.
