# Vehicle-sized immutable wall weights

The explicit `NodalWallWeightLimits::Vehicle()` profile admits up to524,288
parents/global nodes,128MiB retained weights and32MiB startup scratch. Legacy
defaults, custom legacy ceilings and allocation-free128-entry startup retain
their former behavior. This is host reference-area preparation; it does not
raise the prescribed result or CUDA contact contributor limits, add folding
contact, or authenticate original full-vehicle geometry.

`NodalWallWeightStartup` reuses the neutral `SourceIdentityIndex` extracted
without changing the existing shell-index API or data layout. Parent identity
sorting is unchanged. Feature lookup replaces quadratic duplicate scans;
adjacent sorted faces identify duplicate element/face pairs. Validation and
total-area accumulation retain the old first-failing-parent order.

Node shares accumulate in the same sorted parent/local-node order as before.
Interleaving independent node sums removes the node-by-parent scan. A small
startup flag records failed node arithmetic; the ascending-node final pass
preserves the old first-failure order and compacts into already-consumed slots.
No contributor force arithmetic, uncertainty bound or tolerance changes.

The four owning host functions check exact bit parity with the existing915-
parent mixed Q4/T3 fixture, reversed input, immutable copies, duplicate/reference
failure order, explicit-profile and byte rejection before poisoned borrowed
reads, and complete349,645-parent/359,785-node capacity. The large fixture uses
synthetic native triangle references and high64-bit IDs, with all global nodes
incident; these are original count shapes only. Every area/share/source mapping
and late failure/retry is checked. Payload is53,571,320B retained and5,954,305B
startup scratch; root report `vehicle-wall-weights-tests-2.json` records194,523,136B
peak sampled RSS in0.503s. The initial failed report retains a test-fixture
element/feature aggregate-initialization error; named source fields corrected it.

Configure this directory with installed GTest/CUDA headers. It executes no GPU
work. `vehicle_wall_weights_check` is the complete host target; the source-size
case is also registered as `//lib_utest:vehicle_wall_weights_source_size`.
The existing contact harness supplies the separate legacy prescribed, weights,
model and resident regression gates. Run every job with the workstation guard.
