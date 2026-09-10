The source wall component prepares contact for the complete original 117-node,
94-parent part. The geometry module supplies immutable reference areas. The
caller supplies the actual native shell binding and its inverse masses; contact
does not select a structural mass, inertia, shell element, step or clock.

`SourcePartWallSetup` is immutable host preparation. It checks the original
source/material inventory, initial stamp, inverse mass bits, every assembled
unique-node area floor, and the actual common initial kinetic energy supplied
by the case. The measured value must fall inside an independent outward native
mass and declared uniform velocity enclosure. The energy budget and upward
penalty coefficient use that enclosure, with a closed lower potential proof.
The certificate retains the measured value, velocity declaration and fixed dt.

The original canonical mesh stays unchanged. `PlacedCanonicalWall` copies it
with one declared X translation. Setup certifies the actual represented gap
and the whole source Y/Z envelope against that finite mesh, including the
existing exposed-edge and hole checks. Both projected coverage X endpoints
equal the placed wall X. Source attachments remain unapplied.

`SourcePartWallContact` composes setup with TL's `NodalWallContactDevice`. Its
step guard uses the contributor's owning assembled all-active stiffness rate,
including the actual native inverse masses and rounded contact coefficients.
This is only a contact guard; the case must qualify the combined shell/contact
recurrence and enforce all native candidate envelopes separately.

The case calls `AssembleAccepted` inside its existing additive force phase and
`EvaluateCandidate` before its single owner/material commit. Candidate results
are copied by value from private fixed-capacity staging only after evaluation
and readback succeed. The case owns their accepted cache and may publish it
only after the same owner and both native histories commit. `DiscardTrial`
invalidates contact scratch, with no clock, owner or history mutation. There is
no archive or renderer state in this component.

Include `SourcePartWallContact.cmake` after the native source collection and TL
nodal owner exist. Optional `SourcePartWallContactChecks.cmake` adds three host
setup tests and three CUDA contributor tests. The latter check one separated
contact-only unit interval, copied source identity, deliberate rejected trial
faults and clean retry; they do not admit a full impact or coupled shell step.
The setup and runtime failure paths preserve published objects and caller
diagnostics/results. All six functions pass in the bounded integration build,
including exact scientific agreement after rejection/retry. Enable their owning
targets with `ROBO_DYNA_SOURCE_PART_WALL_CONTACT=ON` alongside the existing source
collection and TL nodal owner options. Evidence and measured properties are in
`crash-work/reports/source-part-wall-contact-{build,tests}-1.*` and the matching
two XML files under `source-part-wall-contact-tests-1/`. A coupled impact still
requires the case-level onset, energy, penetration and refinement gates.
