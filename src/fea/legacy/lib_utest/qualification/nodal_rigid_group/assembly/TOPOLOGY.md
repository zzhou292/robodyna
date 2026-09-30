# Original rigid PART topology

`NodalRigidPartTopology` owns literal source PART, extra-node SET and merge
records. It preserves the supplied order, checks the complete expected member
inventory, checks disjointness against every supplied plain-rigid member, and
then publishes immutable original and resolved-root views. It performs no
coordinate or mass calculation and has no owner, timestep or commit API.

This version admits unmerged bodies and disjoint one-child Iflag=2 merge pairs.
It rejects missing/duplicate PART or extra SET identities, repeated original
members, missing/extra expected members, overlap with existing nodal-rigid
members, self-merges, cycles, chains and multiple children. Its root-member
order is parent PART, parent extra set, child PART, child extra set; the original
PART/extra rows remain available independently for the native raw-value seam.
Root records count the original primaries without creating generated node IDs.

The independent limits are 1,024 PARTs, 16,384 original members and 16,384 other
rigid members; callers may lower them and the default 8 MiB startup payload
budget. These are topology limits only. They do not enlarge native group or
owner admission. All borrowed array counts and address extents pass preflight
before member scans. Exact persistent array extents and all five temporary
identity indexes plus merge-role arrays are included in startup admission.
The index utility's existing shared-control reserve is included; allocator heap
metadata is outside the payload convention, as in the other startup models.
Failures leave the model empty and retryable; successful models are immutable
and retain no caller pointers.

The source fixture comes from two already authenticated reports; their SHA-256
identities, original archive/member identities and generated fixture hash are
in `topology_fixture/source-manifest.json`. The verifier can reproduce the
literal fixture using `--audit REPORT --scope REPORT`, and its normal owning
build checks the frozen fixture hash. It contains all original 22 PARTs, 20
extra sets, two merges, 5,452 members, and 7,539 members of the 759 existing
nodal-rigid groups. The resulting 20 root sizes and 22 primary count are checked
against the independently retained resolved-source census. The 54 auxiliary
members are included as source IDs; no mass or rotational inertia is inferred.

Five host functions pass under one CPU/512 MiB, including the complete literal
census, source lifetime, exact ordering, late coverage/overlap rejection,
unsupported merge networks, count/address/byte bounds and retry. XML evidence:
`/tmp/nodal-rigid-topology-author-check.xml`. Root owning commands:

```sh
cmake --build BUILD --parallel 1 --target nodal_rigid_part_topology_check
ctest --test-dir BUILD -R '^nodal_rigid_part_topology_check$' --output-on-failure -j1
bazel test //lib_utest/qualification/nodal_rigid_group/assembly:nodal_rigid_part_topology_check
```

Remaining admission work is deliberately separate: authenticate literal source
options through the app, resolve converter centroid/member ordering, assemble
every original coefficient producer (including auxiliary nodes), compute the
22 raw bodies and two native merges, finalize only 20 roots, and extend the
actual owner with matching source identity and supported membership limits.
The full vehicle's joints, discrete elements and other source load paths are
not released by this topology model.


Root integration is TL `72f1873`. The owning CMake and Bazel executables each
pass all five functions, and the fixture regenerates exactly from authenticated
scope-10 and the rigid source audit. Reports:
`crash-work/reports/rigid-part-topology-{tests,fixture,bazel-tests}-1`.
This validates source topology, not assembled coefficients or CUDA owner admission.
