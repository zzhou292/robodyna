# Active host wall weights and strided resident evaluation

The explicit NodalWallWeightLimits overload admits up to 1024 prepared native
Q4/T3 parents and 2048 global nodes with a separate owned-byte budget. Legacy
initialization retains128-node/parent admission and allocation-free inline
storage. Both paths reuse the same certified area, parent sorting and unique
node reduction. Shared expanded storage uses the existing BoundedStartupArray;
replacement stages all values and copied/moved handles retain valid backing.
Contact area never substitutes for structural mass.

Three new host tests cover a synthetic804-Q4/111-T3/1030-global-node inventory,
independent area certificates, reordered parent inputs, ownership lifetime,
move/copy/replacement, byte/count rejection before borrowed reads, a malformed
last parent, and explicit rejection by old result/device consumers. This is
host startup, not larger resident contact admission.

The existing device remains471864 bytes with128-node/parent caps. Its64-worker
block strides over all compact nodes, resets and copies; ordered status, parent
and physical-node reductions are unchanged. All five affected test groups pass,
including native structural mass, host force agreement, base/candidate work and
last128th-destination overflow with exact retry. The initial run retained a
failure from an old test asserting128 workers; the assertion now requires fewer
workers than nodes, so it verifies strided coverage instead of the old launch.

Evidence: assembly-host-contact-tests-2, the full39-group integrated source-part
suite assembly-host-integration-tests-1, and assembly-host-owning-bazel-1.
Independent review found no blocking defect. Active device arenas, larger
resident results and measured complete-assembly throughput remain subsequent
work; no wall law, source geometry, physical recurrence or history was changed.
