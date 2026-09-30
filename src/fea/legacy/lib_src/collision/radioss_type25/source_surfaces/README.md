# Initial solid and shell source surfaces

This bounded host value producer covers the selected initial native
CREATE_SURFACE_FROM_ELEMENT stage, with complete provided early reader-order
H8/declared-PENTA6 and Q4/T3 tables. The selected caller context is explicitly
NADMESH=0 and NUMELTRIA=0; the future source binder must prove these controls,
not infer them solely from these value types. It does not create a physical owner, parse a
deck, resolve later mechanical support, compute K/gaps, classify coatings,
deduplicate I25SURFI keys, create shell partners or admit erosion/runtime.

PART selection derives every matching physical table row in reader order;
SOLID selection is explicitly sorted/unique and contains no shell clause.
Unknown/duplicate clause metadata, absent phase, unsupported higher order,
invalid raw topology and incomplete ranges reject. Source EIDs are preserved;
physical table order is explicit input, never inferred from EID or mechanical
family vectors. A shipping caller still needs genuine source authority or the
separately reviewed suppression-order certificate.

Exterior processing preserves original raw face numbers and repeated PENTA
slots. Native incidence is corner-major, including repeated solid occurrences.
The internal test first checks all target nodes against a candidate solid, then
uses the donor's distinct-node-count and rotated-edge criterion. It does not
substitute a graphics same-face test. The first matching complete-model Q4/T3
shell controls whether selected-part membership suppresses an emitted face.
ALL skips internal-solid filtering but retains shell suppression. EXT1/EXT2
have the same face path here; their separate shell-edge behavior is outside
this module. Optional shell-normal reversal follows SHELL_SURFACE_BUFFER.

Faces retain kind, original EID/PID, family-local reader row, raw solid face
ordinal and original buffer ordinal. Output order matches the source's stable
five-word sort (four node IDs then ELEM). Native raw roles1/3/7 remain unresolved.
The emitted-solid flags correspond to original surface records, not every
selected solid or the complete nodal coefficient contribution ledger.

Preflight reads descriptors only and reserves the complete-table upper bound;
it does not claim the exact selected face count. Build admits every input/output
range and alias before private typed construction, validates every physical row,
then publishes only a complete success. Output and Snapshot are unchanged on
failure. No truncation, coordinate welding, atomics, device state or per-step
work appears here. Native oracle code is a qualification-only dependency.

NativeRaw8 is a separate explicit reader-brick value profile. It retains all
eight source occurrences even for collapsed edges; the original face compaction,
corner-major incidence, internal test and shell suppression consume them without
changes. Hex8 still requires eight distinct nodes and DeclaredPenta6 still
requires its exact repeated-slot representation. No source PID/material dispatch,
coordinate repair, PENTA remapping, face deletion shortcut or physical geometry
admission is introduced. The caller must supply an authentic native raw8 packet.
