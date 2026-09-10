# Native triangle and mixed wall-contact results

All five new CUDA functions and 41 existing functions passed on their first
execution. The production change admits exact Q4/4 and T3/3 parents and uses
each parent's arity in four loops. The existing contact law, finite-wall query,
sorted reduction, staged additive assembly and owner transaction are retained.

The new tests use actual native startup mass and total rotary inertia for
one/two scalene T3 parents and a shared-edge Q4/T3 union. Independent mass,
force, potential, moment and power checks pass alongside exact host/device
certificates. The unused fourth triangle slot produces no node-zero spring.
Sixteen accepted contact-only intervals check half/full kicks and separate
kick/drift work. Activation/release, late second-parent precision and finite
additive overflow, invalid mass, candidate geometry faults and clean retry pass.
The distinct physical and failure tuples remain those frozen in
[the test contract](NODAL_WALL_NATIVE_TESTS.md).

Contact storage is exactly **99,384 bytes in one allocation**, unchanged across
steps; the existing owner retains six allocations. No material history or
native shell force is advanced by the new contact-only fixtures. Current caps
remain two parents, eight incident nodes and 64 owner nodes.

| Guarded execution | Result |
|---|---|
| `nodal-wall-native-tests-1` | New native contact 5; existing host law 9, prepared query 6, host model 4 and Q4 CUDA owner 6: all 30 pass |
| `nodal-wall-native-cw0-tests-1` | Existing QEPH batch 9, short coupled shell 3 and shell/wall transaction 4: all 16 pass |
| `nodal-wall-native-build-1`, `nodal-wall-native-cw0-build-1` | CMake builds pass with one worker |
| `nodal-wall-native-bazel-1` | Both production host/device contact archives pass, six build actions |

Reports and GTest XML are in `crash-work/reports/` in the parent workspace.
The largest sampled process-group RSS was 591,523,840 bytes during Bazel;
all RAM, CPU-affinity and GPU guards passed. Short-run memory sampling is
supplemented by explicit device-allocation assertions, not treated as an exact
driver-memory measurement.

The pre-execution [input map](native-mixed-source-map.json) records 111 inputs,
SHA-256 `b5d33ce6d830c2a03c1d83fa315f0b5f8164e2106b86366fb55e5792310b88d6`.
It delegates historical qualification evidence to pinned checkpoints and is
not a hermetic toolchain image. The create-only `nodal-wall-native-1` checkpoint
retains current sources, the actual test binaries and all execution evidence.

This qualifies the bounded contact contributor. Mixed shell force feedback,
incoming-impact stability/startup, source materials/attachments, connected
vehicle capacity and a Yaris rendering remain open.
