# Original tied slave classification read set

`TiedClassificationContext` retains the same canonical backing through the
original tied declaration, auxiliary evidence and `RigidPartSource`. Its named
policy proves input conditions for the observed original slaves. It does not
publish a whole-model IKINE inventory or native rigid registration order.

All main and auxiliary rigid memberships, MAT_RIGID PART memberships and extra
nodes must avoid every original tied slave. Plain nodal groups retain their raw
card: PNODE is column 3 (zero based), and the converter adds it to the slave set
while creating a separate main node. Nondefault other options remain unsupported
in this profile. Every original joint N1 through N5 is checked, even when regular
TYPE45 conversion uses fewer nodes. Late slave intersection, source TC/RC,
missing cards, unknown consumed constraints or an incomplete keyword census
reject preparation.

The authenticated complete main-member canonical importer admits only original
eight-slot solid records (six or eight distinct nodes), with no tetra10 source
topology. Auxiliary evidence now also retains its ELEMENT_SOLID block; the
context checks all ten fixed fields and rejects a high-order tail. This proves
the source topology prerequisite for absent midpoint tags, not solid-force
admission. Geometry remains in its shared canonical backing.

## Explicit original wall replacement

Auxiliary `ReplaceWithMeshWall` replaces the six primitive wall cards. The
additional required `ReplaceWholeOriginalWallWithMeshWall` policy authenticates
the complete small `wall.key` member and retains all its blocks, node IDs and
shell IDs. It checks complete shell/part/node associations and rejects an
observed slave in that assembly. This separately disposes of the original shell
MAT_RIGID body and its generated main-node role.

The original member is 10,604 bytes, SHA256
`ef02a4701b37d27cec81b1f9a02ab555f55ac61f68b070e8b0c18dc23b1d5155`:
PID/SID/MID 1001, EIDs 1001–1046, NIDs 1001–1062. No original mass is reassigned
and no new mesh-wall owner is instantiated by this receipt.

## Native read set and provenance

The source policy is pinned to OpenRadioss
`a62b27e6baa555d222a580d6218867d0be4d70b5`. Complete ITAGSL2, KININI, KINSET and
CHECKRBY registration excerpts are independently qualified in TL
`qualification/tied_shell_classification`. Added projection tests compare both
rigid kinds and different overlapping master registration orders against native
execution. Late rigid, cyclic, section100, RBE2/RBE3 and second-interface controls
establish when the projection is invalid.

| Source/caller | Read-set consequence |
| --- | --- |
| `convertrigids.cxx:53–149`, `constrained_nodal_rigid_body.cfg:360–361` | Fresh main node; complete original slave set plus PNODE; selected ordinary options checked. |
| MAT_RIGID/extras/merges in the same converter | Qualified source topology supplies complete original membership; disjointness does not require mass/centroid realization. |
| `convertconstrainedjoints.cxx:1578–1704` | Original regular cylindrical/revolute/spherical joints become TYPE45 springs; all original N1..N5 are conservatively checked. |
| `convertconstrainedinterpolations.cxx:42–82` | Source interpolation generates RBE3; absent under the complete census and rejected if introduced. |
| `convertbcs.cxx`, `convertnodes.cxx` | No original boundary imposed/SPC roles; original slave TC/RC explicitly zero. NODE conversion does not invent an imposed condition. |
| `convertcrosssections.cxx` | Cross-section roles are rejected; material SECTION_SHELL numbers are not native imposed-section100/101 roles. |
| `convertelements.cxx` | Admitted structural shapes introduce no RBE/original-node alias; auxiliary accelerometers produce ACCEL/SKEW/ADMAS. |
| `inintr2.F:145–184`, complete `iniend.F:294–331` | TYPE2/ILEV28 INIEND realizes coefficients without changing IKINE; original converted contacts have no TYPE12 tagging prelude. |
| Complete ITAGSL2 | Complete compact NSV/MSR roles still reach the existing classifier in order, including global role tags and shared ITF mutations. |

Source acquisition inventories are under
`crash-work/deps/openradioss-tied-interface-1/`, including
`source-manifest-classification-5.json` (SHA256
`a5b7edf15c33127a4d7b237629bb9fb96f2f50a884f4ffd0b0dc439a952c4711`),
plus earlier pinned rigid/type13 converter inventories. Each acquisition checked
official Git blob and SHA256 identity. No native formula is duplicated here.

Later KINCHK effects, physical coefficient realization, applicable node M/J
producers and runtime CIN/PEN branch admission remain separate obligations. A
source read-set result does not close them. PEN is not forced to zero.

## Limits and checks

Defaults are 512 MiB, 16 files, 16,384 blocks, 65,536 observed slaves and a 64 KiB
wall-member cap. Preflight charges the rigid source API's inclusive startup
reservation plus distinct tied/auxiliary payload, metadata parser/index storage
and wall draft. This conservatively reserves retired rigid scratch; canonical
backing is charged once. Own payload excludes allocator/control-block overhead
and RSS.

Five new source functions and five old auxiliary functions form the host gate.
Original context/classifier tests are in
`case/vehicle_startup/tied_classification`; full-source execution remains an
explicit root-owned gate.
