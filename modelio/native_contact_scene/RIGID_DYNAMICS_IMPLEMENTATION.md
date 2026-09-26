# Source-bound rigid TYPE25 scene integration

The independent reference is sealed in `native-coupled-scene-engine-qualified-4.json`:
1,001 force evaluations at300ns,18positive FOR3 packets/54lanes,23geometry cohorts,
one search, both rigid-force passes and recovery on every cycle. This is reference
observation only. The raw JSONL and all earlier failed observations remain immutable.
Debugger timing is excluded from speed claims. No new native reference is needed.

Implementation uses app97647e8 plus qualified v3 source5a03622 in isolated
`native-rigid-scene-dynamics`, and the qualified TL delivery containing the
accepted-owner mass policy. No new physical owner, timestep or force law is added.

1. Extend the existing strict DeclaredSource reader with only the rigid v3 schema.
   Authenticate original/export coupling fields and recompute primaryID/centroid,
   member roster and converter placeholders from the physical mesh. Retain these
   as reference provenance, never append primary19 to physical mesh/domain.
2. PhysicalSource retains its ShellBatchBinding, actual M/J ledger and source
   LAW44 declaration. Construct genuine PartTopology(part2,members10..18), then
   existing NodalRigidPartAssemblyModel and NodalRigidAssemblyBinding. The PART
   aggregate already retains native1e-20 primary regularizers in its generated
   aggregate; do not add them to physical nodal mass. Native member order is
   source-ID order. Compare source-derived startup center/frame/inertias to the
   sealed Starter and Engine data, including finite wall masses.
3. Declare patch execution RigidSkin from authentic complete RBODY coverage/OFF;
   wall remains layered LAW44. A role-specific internal catalog key separates
   covered/uncovered parents sharing the raw material and section. It is an
   execution key, not a new material card. Raw material/thickness/parent IDs stay
   in DeclaredSource and archive identity. Keep genuine reference M/J/K/gaps.
4. Existing PackOwner gets actual Part membership flags. Use its current rigid
   group packing/forecast and explicit VehicleAssembly limits. Preserve actual
   wall fixed masks, projected initial velocity, empty CIN raw M/J store and
   sole physical publisher. Descriptive forecast stamps must include actual
   rigid-group metadata. No fake witness or alternate constraint roster.
5. Existing MovingContactSource keeps all12 primary shells and genuine Starter
   topology/current-normal stages. Opt into AcceptedOwnerCoefficients only for
   this declared coupled profile; v1/v2 keep the old static policy unchanged.
   Do not invent same-body exclusion:20same-body pairs pass identity/CSR but
   none reach native STO in this geometry. Preserve strict real search predicates.
6. NativeSceneDynamics reuses its existing assemble-contact→advance→candidate→
   common publication code. No group force/recovery code is copied into app.
   Accepted snapshot/archive uses the existing one-owner capture path, with
   explicit v3 source schema and actual coupled profile metadata.

Qualification expectations are generated only from the sealed Engine4 JSONL,
never consumed by production source creation. Map node ordinals through ITAB,
candidate/history rows through NSV→ITAB, and main ordinals through recorded
connectivity/MSEGLO. Primary19 maps only to the typed rigid group, not physical
node18 or any archive node.

Compare contact force/history at evaluation n to MAINF return n; after TL commit
compare physical X/V/rawM to native MAINF entry n+1, and accepted spin to that
cycle's RGBODFP entry. RGBODV changes A/AR, not X/V/VR; its return is an acceleration
phase for the next integration. Existing captured `matches_last_main_fields` on
return records is inherited ENTRY metadata, not endpoint authority; derived
review recomputes all relationships from raw fields. Compare source/discrete
features, ICONT, phase stamps, offset and loss transitions exactly. Keep original
contact/state budgets, including source-unit associations; investigate any new
rigid association residual instead of adding event-specific tolerances.

Owning gates: strict v3 parser/tamper rejection; genuine startup ledger/group/
RigidSkin/source correspondence; full1,000 CUDA intervals against the recorded
physical/contact sequence; positive-force discard/retry preserving all accepted
physical/group/history/normal/mass state; old fixed/moving1,000 regressions. Use
existing qualification-only read access for group/M/J fields. No run until root
returns the single heavy lane and reviews frozen source/guard proposals.

CIN/tied coupling stays a separate next declaration. Keep Irem_i2=1 where actual
pair-specific CSR permits contact; no blanket slave or same-body deletion, and
no diagnostic setting3 unless a separately declared case requires it. Full V5
and matched four-worker performance remain outside this small rigid gate.
