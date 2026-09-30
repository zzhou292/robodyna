# Rigid groups with joined shells and native wall forces

This bounded R3b gate couples the qualified rigid owner to one native QEPH
parent, one native T3 parent and a finite two-triangle wall. It reuses the
existing shared-edge shell fixture and its complete native M/J union. The
three-member source group spans both families; two ordinary nodes remain.
Reference motion may be rest or an explicit uniform translation.

The production extension admits only a complete actual-owner group descriptor.
The raw grouped assembly/candidate overloads reject. Live-owner source binding
is required even at rest, and candidate evaluation authenticates the common
owner token. Grouped shells require `CoupledForces` and joined publication.
The owner mass tag remains unspecified, all native coefficient/fixity/reference
checks remain active, and the legacy translation PSD proof remains closed.
Contact's native-mass rate is a local diagnostic. Common publication still
reports its existing raw nodal kinetic sum; these tests do not interpret it as
aggregate group kinetic energy or a coupled step stability proof.

`native_coefficient_admission_check` checks complete/exact descriptor admission,
including overflow-sized metadata, and rejection by the old translation proof
before borrowed mass reads. `rigid_shell_contact_check` adds eight CUDA tests:

- Grouped standalone/prescribed/partial startup rejects before source reads.
- Rest and moving raw/forged source rejection preserves diagnostics and loads.
- Actual native mass, native inertia, fixity and reference mismatches still fail.
- A legitimate same-count different source cannot replace the configured owner,
  even if a fabricated config and view agree with each other.
- Foreign tokens/pointers and output aliases reject; a fresh transaction retries.
- Four coupled steps transfer wall force through the group, gather constrained
  shell endpoints, agree with the existing host element force arithmetic,
  preserve accepted values until one common commit and retain allocation counts.
- A late final-node contact geometry fault after both histories and common
  diagnostics are prepared rolls everything back; retry matches a clean owner.
- The ordinary advance and each standalone shell commit cannot bypass the
  dedicated group advance and joined publication.

Configure the standalone CMake directory with the same admitted CUDA precise
arithmetic flags as the production modules. Targets reuse the production owner,
joined shell and finite-wall libraries; no donor runtime or new state owner is
introduced. The pure host target also has a Bazel owner. Root must serialize
actual CUDA/build qualification under the workstation guard.

Author checks (2026-09-10): two host tests pass from a direct bounded g++ build,
maximum child RSS 179,232 KiB. Five production host translation units pass syntax
checks (168,448 KiB). Contact and the three focused CUDA translation units pass
host syntax surrogates with launches stripped only in `/tmp` (231,492 KiB peak);
this is not CUDA compile/runtime evidence. All author checks used one affinity
CPU and a 512 MiB address-space cap. The actual grouped-source free-flight gate and focused R3b runtime now pass.
Root qualification: all two host and eight CUDA functions pass in
`rigid-shell-contact-tests-3`. The first retained runtime attempt exposed a
fixture using physical X extent in the projected-box API; the next exposed
a penetration fault mislabeled as a coverage fault. The fixture now projects
X to the wall and injects a final-node Y coverage violation. No production
checks or tolerances were relaxed. Plastic-law numerical correctness and
aggregate energy observations retain their separate qualification owners.
