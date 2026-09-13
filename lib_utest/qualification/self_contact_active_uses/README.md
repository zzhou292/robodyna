# Symmetric directed active feature uses

`SelfContactActiveUseBinding` is the isolated host value/profile slice named
`SymmetricDirectedVertexDualReferenceV1`. It retains S0 and fixed facets, plus
the actual execution `NodalRigidAssemblyBinding` when rigid groups are present
and the complete copied
`TiedCinAttachmentModel` range/witness roster. It adds no force, penalty,
broadphase, narrowphase, crossing algorithm, CUDA state, runtime participant,
accepted history, or second clock.

## Area and ownership policy

For fixed level `n=2^level`, each T3 has `n*n` facets and each Q4 has
`2*n*n`. T3 reference area is the certified native simplex area produced by
`PrepareT3MaterialMeasure`. Q4 uses the already named
`Q4CenterAreaContactModel`: `A=4*|Xu(0,0) x Xv(0,0)|`, evaluated and certified
at the natural center. Q4 varying material area, `area_enclosure()`, current,
chord, and render areas are not used.

One parent-local facet vertex use has facet valence `m`, dual
`D=m*A/(3*facet_count)`, and directed VF event area `D/2`. Thus one orientation
represents half of its vertex-side parent area; both orientations represent
`0.5*(A+B)`. Every operation uses the existing outward interval/certificate
helpers. Canonical vertex/edge records only deduplicate topology. Parent uses
retain independent source identity, map, area, half-thickness, support, and
activity. Coincident source layers do not weld.

Activity is a pure caller-stamped `{base,current}` byte view aligned with the
binding's deterministic parent order. `current<=base` is mandatory. An inactive
parent resolves to zero thickness/area without redistribution. The binding
stores no activity state and therefore cannot create a second acceptance clock.

## Exclusion and unresolved policy

VF excludes only direct canonical vertex/facet incidence. EE excludes only
shared endpoint incidence. PID equality and whole-parent shared-corner
adjacency are ignored. A nonincident pair from the same physical parent returns
`SameParentNeedsCurrentRegularity` with zero force area; only a later
owner-authenticated current-regularity result may turn that case into an own-
parent exclusion. Same-body exclusion requires every nonzero weighted slot
of both exact endpoint maps to be a member of one identical actual rigid group
ordinal from the exact binding retained by the physical execution. A missing
execution authority or counterfeit domain-equivalent rigid binding rejects
initialization. Different, partial, ordinary/rigid, and mixed-group maps remain
admitted. Any nonzero CIN-secondary slot is explicitly
`UnsupportedCinSecondary`; zero slots do not count, and CIN masters remain
admitted.

Static complete local tied support is checked only from actual attachment rows,
the declared master EID/nodes, and a matching copied native witness. It is
reported in the pair's independent tied result as
`CompleteLocalSupportNeedsRuntimeActivity`, with `excluded=false`; it is never
guessed as excluded. Its force-support result remains the independently required
`UnsupportedCinSecondary`. This isolated source lacks the exact runtime evidence still required: the actual
owner/attempt-authenticated per-witness `witness_activity` bytes (all relevant
rows positively active), current master-parent activity, and the accepted
no-interface-release-event admission used by `AdvanceStaggeredCin`. A future
runtime classifier must consume that evidence from the owner transaction, not
caller-fabricated flags. Partial, mismatched, and unrelated relations are not
excluded.

EE owns no line area or effective mass. Strict interior-interior minima,
EE-only penetration/crossing, geometric ties, coplanar overlap, and zero
distance all return `UnadmittedEdgeEdgeForceArea`. This topology/area slice has
no exact runtime geometry receipt capable of proving that an EE is already
covered by a VF event, so it never suppresses a nonlocal EE.

## Qualification

The host target covers independent long-double T3/Q4 formulas; flat and warped
Q4/T3/mixed dual sums at levels 0/1/2; 27 event-level combinations across all
nine level pairs per family pairing; bidirectional pressure/resultant identity;
removal/no-redistribution/reactivation; coincident layers; local, remote and
same-parent incidence; same PID; exact execution PART/plain authority,
counterfeit/source-kind collision rejection, and different/partial/mixed support;
CIN secondary/master and local/unrelated ties; every explicit EE status;
permutation/deduplication; checked overflow; all count caps; exact
one-byte-short; alias/output preservation; and retry. The source target proves
the no-mechanics/no-allocation boundary and selected arithmetic/API tokens.

Author host/source gate:

```sh
cmake -S lib_utest/qualification/self_contact_active_uses \
  -B /tmp/self-contact-active-uses -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/self-contact-active-uses --parallel 1
ctest --test-dir /tmp/self-contact-active-uses --output-on-failure
```

Bazel wiring is `//lib_utest/qualification/self_contact_active_uses:host_check`
and `:source_check`. Bazel, owning root composition, actual selected-source
level inventories, native/CUDA/NVCC, and runtime owner/CIN witness-activity
tests are root-only gates and are not author evidence.
