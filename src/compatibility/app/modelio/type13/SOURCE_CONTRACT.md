# Original TYPE13 startup loading

The bounded path loads all 4,442 original beam spotwelds of PID/MID/SECID 2000486
from the 2010 Yaris source. It initializes one owned normalized TYPE13 property
and each native reference/endpoint coefficient. It creates no nodal owner,
constraint, tie, recurrence, contact or accepted interval. The 7,493 physical
endpoints do not directly share retained shell nodes; their original tied
interface still needs actual projection/coupling. NID 2000001 is an orientation
node only. The total reference union is 7,494 nodes.

`modelio.type13_source` reuses the original metadata/card parser, bounded source
scanner and canonical array reader. It requires the frozen ZIP/key/README and
canonical identities, preserves complete PART/SECTION_BEAM/MAT_SPOTWELD blocks,
and retains the original working-coordinate doubles and active source row text.
Each raw-to-SI multiplication must match the existing canonical bytes. All four
release fields are original blanks (resolved zero), LOCAL is supplied 2, and N3
is supplied 2000001. The complete original source ledger is checked after the
bounded scan. Python contains no converter curve or stiffness formulas.

`SourceType13::Read(path, expected_identity, limits)` authenticates the complete
compiled declaration before parsing. `ReadProperty` verifies raw cards, typed
field names/blank masks/values and exact source block identities. The C++
`ConvertProperty` leaf alone implements the selected original converter scalar
expressions. `ReadGeometry` verifies the node union, full beam source order and
both compact original-selection hashes, then calls qualified TL startup. Its
immutable shared `Data` owns source text, converted values and all startup
results; copying a handle does not duplicate or borrow mutable source storage.
Initialization failures cannot replace an existing handle.

`ReadLimits` bounds JSON bytes (8 MiB), nodes/beams (8192 each), and host startup
payload (128 MiB). Preflight precedes reading/DOM allocation and uses the existing
VehicleSourcePlan pattern: checked 16× encoded bytes plus typed-array, string,
identity-buffer and staging allowances. This is a conservative simultaneous
payload budget, not an allocator/RSS prediction. Final vector/string capacities
are counted before publication; inline string capacities are conservatively
counted again. Source count/byte caps do not guarantee every combination fits.
The result exposes `startup_budget_bytes` and `owned_payload_bytes`.

The original property has density 7.8e-9 t/mm³, E = 50000 N/mm², nu = .3, SIGY = 300 N/mm²,
ET = 5000 N/mm², TS1=TS2=5 mm and blank TT1/TT2. The app explicitly resolves blank
inner diameters to zero for the selected CST1 circular-section conversion.
Four owned five-point curves feed six channels: axial, shared shear, torsion,
shared bending. ET is used directly by this converter; no LAW44 ETAN mapping is
introduced. The exact converter pi is 3.14159265359. Declared TFAIL = 1e20 is retained
but its converter sensor code is commented out; it is not an active time cutoff.
EFAIL = 2e20 supplies the six signed limits. NRR/NRS/NRT/MRR/MSS/MTT/NF and material
DT are absent in this selected scope. This increment does not execute failure.

Native policy is Ileng = 1, H = 1, Ifail = 1, Ifail2 = 0, unit curve scales, alpha = 1, beta = 2,
zero damping, no sensor and no rate-dependent failure. The original deck lacks
CONTROL_UNITS. The app therefore declares the original README's t/mm/s units;
it does not claim that the unmodified converter's missing-unit error/default
behavior supplied those units. Complete pinned converter/default donors are
retained in `reference/original`, revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`. The active helper is
`convertprops.cxx:2149-2362`; blank GetValue behavior is in `convertutils.h`;
the selected radioss2018 property CFG supplies the resolved controls. No generic
TYPE13 unit-default table is used by this helper. Adapted converter code is
AGPL-3.0-or-later; see LICENSE.md and the exact source manifest.

The original selection hashes are generated from the authenticated source and
are checked with shared BoundedArrayIO little-endian encoding and SHA256:

* Nodes, ascending NID: ID/line/blank mask (u64), three native and three SI
  binary64 bit patterns, zero TC/RC (u64), raw active-text byte length (u64), then
  those ASCII bytes. 1,139,040 bytes, SHA256
  `d1d51ae7bfcbcfa687f7ba14b21072e3de0b09abb3a7ca43bab390bae50f387a`.
* Beams, canonical source order: EID/line/canonical index, all 10 original integer
  fields, blank mask, raw active-text byte length (all u64), then those bytes.
  888,400 bytes, SHA256
  `6768583917785f5b7a26602a963341a817405b61fa9ad7dd8ba18676607a2635`.

These identities reject coherently rewritten raw/native/SI coordinates or
plausibly reordered provenance even if a caller rehashes the JSON. The compiler
does not modify the frozen canonical arrays.
