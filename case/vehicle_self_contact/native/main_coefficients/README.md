# Selected V5 shell main coefficient source

This startup source handle derives all selected shell main coefficients from the
complete retained physical assembly. It keeps declared contact-parent identity
separate from the physical shell supplying INCOQ3 material/thickness operands.
No captured coefficient, owner, normal, volume or role table enters production.

The input is a Ready `CorrectedNodalSource`, its shared `OriginalSelection`, the
authenticated original coordinate member and the authenticated combine member.
The corrected handle supplies closed import/property/material authority only;
its later global nodal K is never substituted for main K. Outputs remain native
units, explicitly tagged by `provenance().units`.

`CoefficientValuesReady` means complete K. `certificate().owners_complete` is a
separate requirement for any later owner-sensitive binding. Every exact best
DX/ST candidate is retained. Unknown same-supergroup order may still publish
bit-identical K with unresolved ownership; no arbitrary EID becomes IELEM_M.
Known ownership is external EID/PID plus the actual physical parent index, not a
fabricated absolute native IXC/IXTG row. This product supplies neither solid
contact faces nor gaps, friction, erosion/removal, interface/runtime admission.

The source phase is after shell CGRHEAD/CGRTAILS and rebuilt BUILD_CNEL, before
INITIA. It is distinct from HM_SET's earlier whole-model surface suppression.
Native incidence is corner-major. Q4 chooses the first equal positive DX/ST;
T3 chooses the last. Equal-card candidates with preserved distinct material IDs
can be ranked between native supergroups only after the closed pre-MID control
certificate. Same-group graph reordering remains unresolved.

That certificate authenticates combine bytes/block hashes through the existing
source reader. No optional CONTROL_TIMESTEP card is admitted: the converter
only creates /AMS for optional IMSCLOptFlag1/2/3; HM_READ_SMS explicitly starts
ISMS=0 and sets it only while reading /AMS. It does not alter mass scaling or
any timestep. Independent MAT_ADD/thermal/adaptive/XFEM modifiers disable the
group certificate. Compared PART controls require an authenticated blank
EOSID/HGID/GRAV/ADPOPT/TMID tail; material/property control cards must match
apart from IDs and NLOC. Equal node sets share the node-based rigid coverage;
intrinsic rigid material grouping is excluded. No PID-specific rule exists.

Existing source-native coordinate, V5 reader raw8, membership, unsigned surface
ordering and resolved-shell topology producers are reused. Every coated primary
is checked against native INSOL3D's orientation permutation. Nonidentity rejects
without publication; partners/tags are never rebuilt. The admitted unprojected
source route is independent of mechanical NLOC. Finite negative pre-INITIA PENTA
volume is preserved through the qualified signed coating scalar; no ABS or later
mechanics reorientation is applied. Complete source defaults establish FILLSOL1,
element override0 and scale1. Material PM32/PM107 come from the qualified typed
post-UPDMAT accessor with separate effective property control.

The immutable handle retains the shared OriginalSelection and corrected source
authorities, one output topology arena, one expanded K vector,
source/owner bindings and the full candidate roster. Forecast charges shared
corrected backing, temporary packing, DOMs, sorted keys, copied metadata and
chunked meaningful-field hashing before construction. Failure publishes no
handle. Native topology warnings remain available through `topology_report()`.

Qualification is host startup only. `tests/native` copies complete original
INCOQ3 plus exact BUILD_CNEL Q4/T3 incidence blocks and original MY_ORDERS. It is
serial, bounded and never linked by production. Existing independent ordinary
and coated scalar oracles supply numerical expectations. The optional actual
V5 target must be run separately through a guarded forecast and create-only
whole-source command; it is never an automatic CTest. No executed pass is
implied by these instructions.


## Optional mixed-face support queries

`SupportQuery` adds a bounded Q4 incidence index for triangular solid-face
queries. It reuses the original positive DX/ST ranking and corner/material-group
winner certificate. A matching T3 remains the I25GAPM owner even when native
INCOQ3 also finds a thicker Q4. Without a T3, all Q4s containing the three query
nodes are considered; no exact-Q4-key shortcut is used. Empty winners explicitly
mean no physical shell support. The original selected-shell wrapper, source
scope, allocation path and forecast remain unchanged.

The index borrows immutable Inputs/Packed for its lifetime. It does not detect
mutation through other aliases or authenticate caller-made value fixtures.
Complete source preparation remains the factory's responsibility. New tests
reuse the complete native INCOQ3/CNEL oracle and preserve the old fifteen groups.
This slice does not resolve solid support, post-GAPM internal/erosion flags,
final main K, normals or runtime admission.


## Solid membership query

`SolidSupportQuery` consumes complete reader-phase raw8 rows and the complete
initial emitted-solid flags. Native TAGELEMS makes repeated raw incidences one
matching solid; the index preserves that set while leaving all raw rows intact.
For one/two matches, source EID uniqueness proves the maximum-ID NEL and the
TYPE25 flag-collapse result independent of representative traversal. With more
than two, every match remains reported, but the first-two owner pair is explicitly
unavailable as `NeedsNativeReaderOrder`. Empty membership is separate from an
invented zero-valued support.

This source query does not compute K/gaps or publish a post-GAPM topology source.
Independent whole INSOL3D/NORMA1D/CNEL coupons observe the actual owner pair,
warning, primary orientation and no-match preservation channels. A later complete
source composition must retain pre-shell internal count even if INCOQ3 overrides
the final owner, and must derive the final IDEL_SOLID after that native phase.
