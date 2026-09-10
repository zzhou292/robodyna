# Source-native rotation domain (explicit opt-in)

`Config::rotation_domain = RotationDomain::NativeShellGeometryV1`, selected in
this pilot by `--native-rotation-domain`, guards the rotation of every source
shell's native geometric frame and each native nodal normal against its original
source geometry. The existing `deformation.maximum_rotation` bounds both measures
and every rigid-group member's quaternion angle. The pilot keeps this bound at
1 rad. The qualified policy cap is 1.5 rad; larger declarations fail startup.
The default `NodalQuaternion` policy retains the original all-node quaternion
comparison and archive bytes.

The actual node quaternion remains finite/unit, is advanced by the unchanged
owner, and remains in every output along with the all-node maximum. The existing
per-step rotation increment bound remains active. No angular velocity, native
couple, inertia, material history, contact law, source geometry, clock, or other
deformation/timestep guard changes. In the selected ordinary isotropic shell
recurrence, native rates/forces consume endpoint positions and carried velocity
and spin, not an accumulated nodal quaternion material director. The geometric
measurement is a domain guard, not an energy or coupled stability certificate.

`NativeRotationReferences` holds only immutable observation references: all
804 QEPH and 111 T3 source EIDs, original native frames/normals and the exact
76 rigid-group member flags over 1030 nodes. It reuses the qualified native
geometry operations at original source coordinates during host startup. T3's
three normals are its current frame's positive third axis; QEPH retains its
individual warped-parent local normals. It owns no state/history or mass.
All storage is charged before CUDA startup. Measurements allocate nothing during
Step, consume the actual candidate force packets already read from the two
batches, and publish diagnostics only at the sole common commit.

Evidence supporting this scope:

- Frozen native qualification `70e7f7bf738eb087053816cc908da35a6f81399a`
  tests QEPH/T3 superposed rotations through 1.647949 rad and source-warped QEPH
  localized spin through 1.904297 rad at h and h/2, with independent native
  material/force recurrence. This supports the separate 1.5 rad geometric cap.
- Actual unmodified 8h trace `reports/source-assembly-spin-prefix-1.jsonl`
  ends at accepted epoch 4304, 0.5130767822265625 ms, stopped by the 1 rad nodal
  quaternion proxy. Its SHA256 is
  `c579d677004cff35308ecbc0e1bb9ab6c962352d0853646dcccc5ab07e3fd383`.
- `reports/source-assembly-spin-native-analysis-2.json` compares 1078 paired
  actual transitions for EIDs 2214871/2214872 at shared ordinary NID 2181592.
  Maximum native/port differences are 4.55e-13 N force, 1.76e-14 N m couple,
  1.20e-7 Pa material stress and 1.74e-18 plastic strain. This checks the actual
  recurrence and source policy; agreement alone does not prove harmless spin.
- That node reaches 7812.69 rad/s with 99.2045% added isotropic inertia.
  Per-parent frozen-state own-normal removal is nearly null in native force,
  while actual tangential carried power reaches 5055.21 W. The incident normals
  differ. A common-node spin therefore remains mechanically consequential;
  per-parent removal is a diagnostic what-if, not a compatible new trajectory.

Config gains optional `rotation_domain` with exact keys `policy`,
`native_qualification_commit`, `maximum_supported_rotation_rad`. Accepted frame
and final diagnostics gain optional `native_rotation_domain` with exact keys
`maximum_frame_rotation_rad`, `maximum_nodal_normal_rotation_rad`,
`maximum_rigid_member_rotation_rad`. Initial values are zero. Existing
`maximum_rotation_rad` and CSV columns still describe actual all-node quaternion
angle (0..pi in the opt-in policy); they are not relabeled geometric measures.
The replay reader independently checks policy, metadata, metric bounds and saved
node/group quaternion maxima. Reader changes are separately owned.

The host tests cover complete original source inventory and native warped normals,
proper-frame/normal measures, late nonfinite and alias failures, qualification
cap, explicit CLI parsing and legacy serialization parity. Actual GPU tests
separately compare 64 contact intervals' state/history/contact and stable device
allocation, plus private-copy guard faults, accepted rollback and exact retry.
Those tests do not publish a fabricated >1 rad nodal candidate. A longer actual
run with this policy is still required to assess the next response/domain limit;
no longer horizon or convergence claim follows from startup alone.
