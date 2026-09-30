# Original source part into a placed finite mesh wall

`MeshWallConfig(refinement)` and `MeshWallSettings(config)` select the frozen
workspace experiment in `planning/SOURCE_PART_WALL_PILOT.md`: the original
117-node, 94-parent elastic source part translates at `(1,0,0)` m/s toward its
authenticated canonical triangle mesh, placed with a 0.5 mm leading gap.
The setup owns the native-mass penalty certificate, actual placed wall, finite
coverage rectangle, hard penetration cap and contact-only step guard. Original
density/thickness and native total inertia remain unchanged. This is experimental
elastic midsurface contact with zero friction/offset, not full-vehicle admission.

Use the existing case's wall overload of `Initialize`, then the same `Step` and
`Capture` methods. `wall_setup()` exposes immutable placement/certificates;
`accepted_contact()` exposes only the case-owned copied result of a committed
interval and is null at epoch zero. `wall_metrics()` exposes the accepted ledger.
Initial rendering uses actual owner fields and certified separated setup metadata.
No zero-time candidate or fabricated contact result is created.

The transaction is one owner trial, QEPH assembly, T3 assembly, contact assembly,
one drift, both material candidates, contact candidate/readback, common kinetic
measurement, all observation gates, and one owner/two-history commit. Contact
assembles last so its addition uncertainty includes the existing force vector.
The optional contact state owns reusable result caches and no second solver or
clock. Rejection discards every trial. Only fixed-size non-failing copies follow
the sole commit; contact retains its `PreparedCandidate` association alongside
the actual accepted owner stamp. Capture never reads current contributor scratch.

The observer adds each unique contact-node force once before reconstructing
endpoint velocity. It keeps the raw midpoint fields and checks carried kinetic
work using the actual half kick, contact work and addition roundoff. Physical
admission is `abs(Ksync + native_work + Uc - K0) + uncertainty <= .05*K0 + 1e-10 J`;
the uncertainty includes certified contact potential, force-to-synchronized-K
propagation and arithmetic. Native work minus its arithmetic uncertainty must
remain at least `-.01*K0`. The convex contact defect has separate lower/upper
bounds, not a symmetric quadratic allowance. Existing native and finite-wall
geometric/step checks remain active.

Cumulative carried wall impulse is checked against carried linear momentum.
Angular momentum is reported using endpoint positions and raw carried v/omega
at their actual velocity time; cumulative signed wall-kick moments carry their
own outward error. Those angular observations are not an additional qualified
angular acceptance gate. Positive node force defines the first/last contact
epochs and contact-interval count. No rebound completion is inferred here.

The new native test target runs one fixed 8,448-H prefix with the actual gap,
all 94 native shell histories, independent host wall forces at each base,
per-parent/node force/potential certificates, and expected onset bracket
8388/8389. It then saves a clean next candidate, rejects a last-node finite-mesh
fault after both material candidates and a late observer fault, and checks exact
retry without a second long prefix. Existing pulse/flight tests and the frozen
pulse scientific-byte comparison remain regression gates. Full h/h2/h4 impact,
rebound qualification and the new wall archive/video are separate next gates.

The integrated onset/rollback gate passes in workspace report
`source-part-wall-engine-native-2.xml`: 794,206 native cell intervals including
the clean retry, with peak prefix penetration 3.4323045e-6 m and physical-energy
error upper bound 1.1360483e-8 J. The earlier T3 projection-resolution failure is
preserved; [the layered qualification note](T3_TRAJECTORY_QUALIFICATION.md)
explains the reviewed harness change without changing production equations or
physical bounds. Existing pulse/flight regressions pass, and the pulse archive
retains 21 scientific files / 695,141 bytes exactly.
