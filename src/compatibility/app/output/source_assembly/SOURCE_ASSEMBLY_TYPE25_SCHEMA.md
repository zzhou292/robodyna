# Optional source TYPE25 connector output

The seven-part CLI selection retains the authenticated original bracket and
spotweld (959 shells, 1093 nodes, one WID 2101297, six nodal rigid groups).
`SourceAssemblyConnectorInputFields.cpp` and `SourceAssemblyConnectorFrameFields.cpp`
own the optional input and accepted-frame extensions. The six-part selection
omits every connector key and preserves its previous field/CSV representation.
The production writer only consumes the live case's accepted result view and the
same accepted common-publication capture. `FrameView` remains a formatting-only
host qualification seam, not an acceptance authority.

`input.connectors.kind` is `openradioss_type25_linear_finite_offset_v1` and its
`policy` is `openradioss_tonne_millimetre_second_direct_import`. The explicitly
selected source import uses tonne/mm/s conversion `[1000,0.001,1]`, generated
property ID 383348001108, native M=0.001 kg, scalar J=1e-8 kg m²,
axial/shear K=1e8 N/m, torsion/bending K=1000 N m/rad and zero damping.
All four stiffness/damping/failure channels, original WID/NIDs/card line numbers,
generated property mapping, global node order, immutable reference positions,
reference transverse axis/length and both endpoint contributions are serialized.
The four channel order is axial, shear, torsion, bending. Translational channel
forces/limits use N and rotational ones N m; displacement uses m and rotation rad.

`input.native_nodes` and `part_native_ledger` remain shell-only. Each row in
`input.connectors.endpoint_contributions` declares its complete column order,
source association, endpoint M/J and authoritative total node M/J. Accumulate the
endpoint contributions in source connection/local endpoint order onto the shell
ledger exactly once. The native primary/group properties retain shell-only M/J:
all connector endpoints are proven outside the internal rigid groups at startup.
No frame or reader should add connector M/J to those aggregate properties.

Each frame's optional `connectors` contains the same `kind`, a complete accepted
`diagnostics` identity and source-ordered `elements`. An element records native
history (transverse axis, displacement, rotation, local force/couple, four signed
work channels, failure criterion and active flag), current and midpoint frames,
both endpoint force/couple wrenches, critical dt and elementary stiffnesses.
The diagnostic carries source/owner/configuration/qualification, accepted/base
epoch, attempt, position/base/velocity/base-velocity/kick times, validity, completed
interval/assembled flags, element/activity/failure counts, signed work totals and
last-interval increments, RHS kick/drift work and minimum native dt. At epoch zero
there is no completed interval and no fabricated attempt, work or force cache.
Native failure can retain the just-computed force at the failing sample; the next
OFF=0 evaluation clears it. Inactive status alone does not mean that the same
sample's force or accumulated signed work is zero.

Common shell diagnostics add `connector_kinetic_columns` =
`translation_J,rotation_J` and `base_connector_kinetic_J` / `connector_kinetic_J`.
`motion.before`, `motion.after` and nonnull `force_stage_kinetic` also add
`connector_kinetic_J`. These are explicit subtotals already contained in native
ordinary-node translation/rotation, not extra energy to add. The shell physical
and added scalar-inertia channels stay separate. At initialization common base K
is zero; current K and motion.after contain the actual initial connector K.
Stored and force-stage phase labels retain their existing, distinct meanings.
Neither is a global physical energy balance or a dissipated-energy assertion.

The configuration's `connector_work_scope` states that existing
`native_internal_work_J`, final metrics and the 39-column accepted-interval CSV
retain shell plus stabilization work. TYPE25 signed work is reported separately
in the sampled connector frame. Its increments belong only to that frame's last
accepted interval; sparse frame increments must not be summed as a complete work
history. This extension introduces no new files or ledger columns.

`connector_storage_limits` records the actual case contributor budgets.
The output format admits at most 128 connectors (the pinned selection has one),
within the unchanged 8 MiB configuration/frame reservations and 2 GiB total cap.
The worst-token encoding fixture includes all 1093 nodes, 959 layered parents,
128 connector rows and the enabled force-stage record. Forecasting uses the
selected authenticated inventory byte size; the complete inventory and existing
file hashes remain mandatory before a bundle is complete. Writer failure cannot
change the accepted solver state, and an incomplete writer cannot publish a
complete manifest. The original nodal spin trace is restricted to nodes without
connector contributions until that diagnostic has an explicit connector ledger.

Qualification owners: host `output/source_assembly` with optional
`ROBO_DYNA_SOURCE_BRACKET_INVENTORY`; actual writer
`output/source_assembly/wall_artifacts` with the same explicit fixture. The latter
checks real accepted output after a rejected candidate and exact retry. GPU
execution is a separately scheduled gate, not implied by host formatting tests.
