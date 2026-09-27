# Original solid-control source authority

This source-only utility extracts existing CONTACT_INTERIOR membership and
native single-MID property sharing. It does not enable controlled mechanics,
produce material slots/contact coefficients, own a physical model or run a clock.
Raw inputs are authenticated by existing canonical/member/block readers; there
is no new keyword parser or PID/family table.

`DirectSource` owns direct requested parts, original counts and source card/set
evidence plus a shared immutable canonical handle. It does not resolve sharing
or require a TYPE25/SOFT1 interface. Existing contact seed keeps its SOFT1
preflight at the original call boundary, then adapts retained counts. Unsupported
shared-property clones therefore do not become new early seed failures.

`EffectiveSource` is a separate stage over that same declaration authority and
an authenticated ImportContext. It owns all original/unselected PART rows,
source/native property identities, distinct direct/effective flags, source block
member/line/hash/original-ID/offset provenance and generator census. Controlled
multi-MID sharing returns NeedsNativePropertyMapping; unrequested clone identity
remains zero and disabled. Common ID offsets are proved, mixed offsets reject.
Known source-generator closure remains enforced, while the nonzero TYPE25
requirement stays in the legacy contact adapter. Equivalent independently owned
canonical clones are accepted through authenticated identity; the extra backing
reservation is explicit, not charged as newly allocated own payload.

Before:
  case seed parsed direct sets and retained counts;
  case correction parsed PART/SECTION/includes and resolved effective sharing;
  structural mechanics had no reusable control authority.

After:
  canonical/member source -> modelio DirectSource -> case seed retained counts;
  same DirectSource + import context -> modelio EffectiveSource;
  case correction consumes its rows with unchanged numeric/material/order code;
  future structural builder can consume the same source without depending on a
  completed physical model or corrected contact result.

All string/vector evidence is owned; no ImportMembers string_view escapes.
The legacy public part-control copy coexists with the shared effective result
and is accounted. Direct retained admission is4MiB; effective retained admission
is8MiB, with another8MiB in consumer forecast for extraction/result coexistence.
Existing parser working reservations remain. Forecast values increase explicitly
by20MiB across both stages; hard source/workstation caps stay unchanged. Measured
owned payload includes string/vector capacities and conservative allocator slack;
shared canonical/import backing is separately accounted by callers. Retained
admission occurs before immutable publication, with no partial source returned.

`source-extraction.json` proves unchanged relation/offset/keyword leaves and case
digest bytes plus seven untouched numeric/digest sources. Focused tests cover
actual source-set parsing without self-contact, direct-before-sharing failure
semantics, borrowed member destruction, unsupported/nested declarations, sharing
and capacity-aware accounting. The full consumer test additionally covers exact
and one-byte-short retained caps, independent canonical clones and retained
source lifetime. Qualification passed at executable source5c128776851cf031757e7e6cb74fa9a09ffcc54e:
12focused GTests plus the consumer and5actual source tests. The actual test
checks4980solid/3686controlled/6656affected/922part counts. All3015440corrected-K
bytes and every old source/pre-correction/property/material/certificate digest
match the preserved qualified artifact. The sole sourceJSONdifference is the
explicit20MiB forecast increase; hard caps are unchanged. Exact/one-byte-short
retained budgets and independent canonical clone/lifetime checks passed.
Workspace receipt: `crash-work/reports/solid-control-source-qualified-1.json`.
No vehicle binary was promoted and no IC1 force/history/STI profile was enabled.
