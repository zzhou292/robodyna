# QEPH rejected-candidate diagnostics

This optional diagnostic retains one rejected QEPH element's exact incoming
values. It is not a checkpoint, physical restart, replacement integrator, or
permission to accept a rejected interval. TL-FEA authenticates and copies the
owner/attempt/source values and owns the existing-operator replay. The app
owns capture lifetime, source attribution, encoding and companion publication.

Enable explicitly with `ROBO_NATIVE_QEPH_REJECTION_CAPTURE=1` on the native
preview runner. The default is disabled. The dynamics preflight charges the
owned capture record, TL's complete 64 KiB failure-copy/replay reservation, and
an additional 1 MiB bounded serialization reservation. The normal successful
step performs no diagnostic transfer, source lookup or array capture.

The failed QEPH result is intercepted before Require throws and the dynamics
owner discards the attempt. Capture failure never changes that original result.
A retained value record survives discard. Only after ordinary run-loop prefix
closure does the app create the `qeph-rejection` companion. Capture status and
export status are separate. An unavailable/unsupported capture can export its
metadata, but never an invented input packet. A partial export is preserved.

Codec responsibilities are separated:

- `WordArchive.h`: bounded integer and exact IEEE binary64 word encoding.
- `MetadataFields.h`: original report and accepted/candidate identities.
- `GeometryFields.h`: complete native reference, accepted force/history and
  exact gathered candidate positions, velocities and angular velocities.
- `MaterialFields.h`: material coefficients, curve arrays and active tagged
  section/failure history. No inactive union bytes are serialized.
- `Codec.cpp`: versioned field order and exact consumption.
- `Export.cpp`: existing bounded artifact IO, hashes, decode/readback and TL
  replay outcome. JSON contains descriptors and integer/status/source context;
  numerical input values are little-endian uint64 IEEE bit patterns. NaN
  payloads and signed zeros never pass through JSON number conversion.

History reconstruction uses its public accepted-value constructor and the
complete captured ReferenceData. An unprepared or mismatched incoming history
rejects; it is not repaired. Native input capture has the same prerequisite.

The companion contains `failure.json`, `metadata.words.bin` when available,
and `input.words.bin` only for a complete authenticated capture. Its manifest
describes requested and captured source IDs/law, original batch/element/node
statuses and diagnostic status. Host replay success means the existing element
operator could run on those incoming values; the separate reproduction flag
reports whether its rejection matches the original device result. It does not
validate a full vehicle restart or crash trajectory.

Focused host tests cover exact word roundtrips, nonfinite/signed-zero inputs,
tagged failure states, malformed data, original-error preservation, source and
attempt mismatch, source attribution, readback and create-only publication.
A separate live coupon must verify pre-discard capture against an actual
rejected owner attempt; host policy tests alone make no such claim.
