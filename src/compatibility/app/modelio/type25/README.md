# Original TYPE25 source model

`VehicleType25Source::Prepare(census, domain, declaration, limits)` admits the
named `OriginalDefaultSpotweldsV1` profile and constructs the existing immutable
TL TYPE25 model for all 2,828 original welds. The source census already retains
the authenticated WID/N1/N2 cards, original block and line identities. Every
record must remain default-only. The model's row order is exactly that source
order, and the retained census provides the original cards for each row.

`ResolvedSpotweldProperty` is the single shared blank-default property producer
used by this adapter and the existing component adapter. Its original arithmetic
order and t/mm/s unit conversions are unchanged. The caller supplies an explicit
generated property ID, which must not equal an original section ID. This is a
declared generated property, not an inferred original material or registry entry.

The domain must contain every current census-union node and match the original
source-instance identity. Its order may differ. Every supplied domain node must
have the exact canonical NID and represented SI coordinate bits; no SI/working
round trip is used. Additional original nodes are allowed because the current
union has unresolved source obligations. Their presence does not authorize their
coefficient or DOF roles, and 372,343 is not fixed as the final vehicle count.

The adapter resolves endpoints with the domain's existing index, then calls
`type25::Model::Initialize` with its explicit Vehicle limits. TL owns frame,
initial-history and endpoint M/J construction. Source preparation rejects any
actual bad reference with its original WID; it does not fill missing values or
change native thresholds. The returned object retains source, domain and native
model backing; temporary input arrays are released after the model copies them.
No mass subtotal is accumulated and no state owner, force contributor or clock
is created.

Preflight charges the greater of the prior census phase and the current retained
source bound plus domain, canonical decode, input staging and native model
reservation. Retired census decoder workspace is not double-counted as retained
payload. The original conservative rigid-source reservation remains explicit;
no unmeasured RSS claim is substituted. Default app admission remains 512 MiB,
with TL's existing 16 MiB Vehicle model cap.

Tiny tests cover exact default values, endpoint coefficients, reordered and
superset domains, missing endpoints, optional fields and signed-zero rejection.
The owning original tests reuse the census fixture and exercise every original
model row, endpoint/reference value, exact-cap retry, late coordinate failure,
source lifetime and a larger declared domain. The existing three component
spotweld tests are available as a separate regression target.

Configure `modelio/type25` using the usual Chrono/TL paths, with
`ROBO_DYNA_VEHICLE_TYPE25_ACTUAL_TESTS=ON` and the same four
`ROBO_DYNA_PHYSICAL_{CANONICAL,SCOPE,DECLARATIONS,TYPE13_DECLARATION}` paths as the
census. Add `ROBO_DYNA_SOURCE_BRACKET_INVENTORY` for the existing seven-part
fixture. Targets are `robo_dyna_vehicle_type25_fields_check`,
`robo_dyna_vehicle_type25_source_check` and, when enabled,
`robo_dyna_type25_component_regression_check`. Root owns original-source execution.
