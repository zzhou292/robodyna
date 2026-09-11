# Original adhesive and rubber solid source

`VehicleSolidSource::Prepare` consumes the shared authenticated canonical source,
the original main-member bytes and the named
`OriginalAdhesive18RubberHephS6zV1` policy. It retains all 908 adhesive cells and
all 1,504 rubber cells in original record order. The remaining 12,822 original
solid records remain in the canonical backing with an explicit outside count.
Their absence from this selected producer is not a complete vehicle exclusion
or mass-effect decision.

The source profile is deliberately separate from the immutable shell V1 plan.
No original card, shell role, owner domain, nodal coefficient or runtime history
is replaced. PART/SECTION/MATERIAL/curve/HOURGLASS blocks use the existing
authenticated source-card reader. Selected element cards are checked against
the exact canonical EID/PID/eight-node rows and source lines. Native input uses
the represented canonical SI positions directly, preserving signed zero. There
is no SI-to-working-to-SI reconstruction.

| Source | Explicit resolution |
| --- | --- |
| PID2000977, MAT024, ELFORM2, LCSS2100010, blank C/P/LCSR and VP0 | LAW36, Isolid18/JHBE17, 2x2x2 points, ICPRE2/ISMSTR2/JCVT1; original eight-point curve, SIGY retained as an unused table-branch field |
| PIDs2000477--2000484, MAT007, blank ELFORM, HGID2000017/IHQ2/QM.1 | Original converter outcome Isolid1 remains declared; named demo mapping selects 1,309 HEPH24 bricks and 195 S6Z wedges, LAW42 alpha2/nu.463, total strain10 |
| Original rubber raw `[A,B,C,D,E,E,F,F]` | Existing qualified `MapCollapsedTopEdges`: raw slots1,2,5,4,3,7, then each native reference's own orientation processing |

Rubber density retains `source_rho * 1e12` binary64 rounding. The LAW42
coefficient producer receives `source_mu * 1e6`, nu.463 and the native positive
default tension cutoff `1e20 * 1e6` Pa. No original hourglass coefficient is
relabeled as a HEPH damping parameter. HEPH references explicitly request the
global total-strain Jacobian with original millimetre threshold scaling. The
separately exposed S6Z force profile selects the qualified
`Law42MaterialSoundSpeedV1` stabilization repair and DN.1. This differs from
the original converter's Isolid1 element selection; it is not a legacy LS-DYNA
trajectory equivalence claim.

The native sources and independently qualified values are owned in TL:

- `qualification/solid18_reference` and `solid18_force`: original adhesive
  source receipt and resolved S8EFORC3 profile; `solid_law36_point` owns its curve.
- `qualification/solid_common/source_fixture`: complete original rubber
  source/array identities. `solid24_reference` and `solid6z_reference` own all
  source geometry/mass and topology gates.
- `qualification/solid_law42_caller/native/original/starter/source/materials/mat/mat042/hm_read_mat42.F:153`
  supplies the EP20 cutoff. `solid24_force` and `solid6z_force` own the selected
  force profiles and the explicit S6Z stabilization repair.
- All donors use OpenRadioss `a62b27e6baa555d222a580d6218867d0be4d70b5`.
  `planning/YARIS_RUBBER_SOURCE_PROFILE.md` records the approved demo decision.

The production adapter does not include any qualification fixture. Tests compare
every original typed input and all named native reference/mass values with the
existing independently authenticated fixtures. Material curves remain owned by
the immutable handle; the later `solids::Model` deep-copies/deduplicates them by
MID and maps endpoints through its supplied physical domain.

`Preflight` runs before member hashing, selected allocations or native reference
preparation. Its 512 MiB profile reserves canonical backing once, metadata DOMs,
the borrowed member, bounded source evidence, decoded arrays and their decoder
temporaries, selected rows and all typed reference buffers. This is a conservative
simultaneous startup bound, not measured RSS. `owned_payload_bytes` counts the
new handle's retained payload and a control reserve; it excludes the separately
retained canonical backing and allocator overhead. The final handle is published
only after all declarations, geometry and native references pass. Native failures
identify the original EID and status.

Configure the owning host target with explicit `Chrono_DIR` and
`ROBO_DYNA_TL_ROOT`. Small fields/geometry checks are always enabled. Root's
original gate adds `ROBO_DYNA_VEHICLE_SOLID_ACTUAL_TESTS=ON`,
`ROBO_DYNA_VEHICLE_CANONICAL=<canonical directory>` and
`ROBO_DYNA_VEHICLE_SCOPE=<scope10 report>`; it reuses the existing source fixture
driver to authenticate/extract the original main member. No GPU is needed for
this source/reference gate. Neither this gate nor positive native masses admit
the complete physical ledger, missing producers, rigid/tied DOFs, contact or a
vehicle trajectory.

## Explicit extended rubber source policy

`OriginalAdhesive18ExtendedRubberHephS6zV2` adds exactly four source parts to V1:

| Original PID | Source cells | Selected HEPH / S6Z | Literal source density t/mm³ |
| --- | ---: | ---: | ---: |
| 2000017 | 297 | 226 / 71 | 1.9800e-9 |
| 2000393 | 72 | 72 / 0 | 1.9800e-9 |
| 2000509 | 234 | 192 / 42 | 1.9990e-9 |
| 2000521 | 234 | 192 / 42 | 1.9990e-9 |

It retains 13 parts and 3,249 cells: 908 adhesive solid18, 1,991 HEPH and
350 S6Z. The other 11,985 source solids remain explicitly outside. No production
case factory selects V2 by default. The four additions use the same MAT007
LAW42 preparation, original blank ELFORM and PART HG2000017 override, and
explicit HEPH/S6Z case mapping as V1. There is no new force law or material
substitution. The main and auxiliary source cards remain immutable.

`SourcePolicy.h` owns only the explicit PID selection and expected counts.
`Preflight`, `ReadDeclarations`, `ReadPart`, `ReadGeometry` and the single
`PrepareReferences` append path are shared. Default limits remain512MiB and
4,096 parents; both policies fit the same conservative default reservation.
The actual owned payload is counted after successful preparation. Lowering the
parent cap below3,249 rejects V2 before source/member preparation.

The independent audit receipt is
`crash-work/reports/yaris-remaining-metal-rubber-source-1.json`, SHA256
`751a2834dd7c31d9fe84633589ab6c17ce430d7417b43beb305f121038cbb2e5`.
Tests retain only four source counts/ordered-record hashes from that receipt,
not another full geometry fixture or source parser. They verify all added raw
EIDs, source order, units, geometry, material provenance, canonical coverage,
typed point/slot mapping, old2,412-reference bit parity and exact cap/retry.
The optional native tests call the existing complete solid24/solid6z reference
oracles for all682/155 new references. Their numerical predicates/tolerances
remain the existing owning TL predicates. Those tests compare SI packets;
they do not claim a new original-working-coordinate round-trip equivalence.

Enable the existing V1 tests plus `ROBO_DYNA_VEHICLE_SOLID_EXTENDED_TESTS=ON`
for the new `vehicle_solid_extended_source` CTest. Adding
`ROBO_DYNA_VEHICLE_SOLID_EXTENDED_NATIVE=ON` enables GNU Fortran and the two
existing TL native libraries, producing `vehicle_solid24_extended_native` and
`vehicle_solid6z_extended_native`. Source/member fixtures and
`ROBO_DYNA_TL_ROOT` remain the same explicit inputs as V1. No CUDA is required.
These original/native runs belong to the root qualification lane.

This slice does not restore metallic/foam/beam paths, original antiroll joints,
groups or masses, nor CONTACT_INTERIOR. A later full constructor must rebuild
the same PhysicalScope/domain/ledger/rigid/CIN identities from the selected
source and qualify those relations before selecting a new demo profile. The
existing physical factories' V1 call sites are unchanged. It is not a complete
connected-source or full-vehicle runtime admission.

## Explicit rear metal source policy

`OriginalAdhesive18ExtendedRubberRearLaw44V3` retains V2 and adds the 210
PID2000016 rear-bar cells and 96 PID2000392 rear-tube cells. The result is
15 parts and 3,555 solids, with 11,679 original solids outside this producer.
`RearMaterial.cpp` reads the actual MAT024 declarations and shared 46-point
curve2100270 through the existing authenticated source reader. It prepares
LAW44 with E50/200 GPa, nu.3, original density, C8000/s, P8 and the qualified
VP0 filtered-rate branch with its native 10000/s cutoff. Curve stress converts
from original MPa to Pa; plastic strain and the original seconds remain unchanged.
The immutable source handle owns both curve arrays and prepared materials.

The 306 new references use the qualified solid18 LAW44 ICP1/ISMSTR2 profile.
All eight original slots are retained, including the 109 repeated-pair H8
records. They are not passed through the rubber S6Z topology conversion.
Existing reference initialization, row packing and resource accounting are
shared; the V3 forecast additionally reserves LAW44 references. The existing
4,096-parent and 512 MiB source caps cover this profile.

Enable `ROBO_DYNA_VEHICLE_SOLID_REAR_TESTS=ON` alongside the actual and extended
source tests to run `vehicle_rear_solid_source`. It checks every added source
slot and represented SI position, all 46 curve values, qualified material
parameters, positive per-slot mass, shared lifetime, unchanged V2 records and
reference values, rejected material branches and exact resource bounds.
Root's `rear-solid-source-root-tests-1` passed all 19 functions across four
CTest targets. The source adapter adds no default vehicle selection or runtime
history: physical domain, complete original rigid groups, coefficient ledger
and resident state must be composed and tested before these parts enter a run.
