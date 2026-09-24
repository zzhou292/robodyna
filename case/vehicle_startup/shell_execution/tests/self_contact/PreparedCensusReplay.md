The prepared census replay classifies frozen candidate geometry on the CPU. It does not initialize CUDA, advance a solver, or turn diagnostic certificates into an accepted physical interval. Its library links the existing capture target to reuse `FrozenSameRigidExclusions`; CUDA library linkage does not imply GPU execution.

Run the CLI through the workspace's serialized host guard:

```
robo_dyna_prepared_census_replay_tool census.json EXPECTED_MANIFEST_SHA256 absent-report.jsonl
```

The SHA-256 argument pins the complete manifest. The reader validates every inventoried shard, codec hash and ABI, phase, count partition, family ordering and source-pair ordering. Shards must be regular files in the manifest directory. Inputs must stay immutable during replay, as for the existing artifact readers. Input caps are 1 MiB for metadata, 64 MiB per shard including the codec header, 128 pairs per shard, and 2 GiB for the complete archive. Only one shard is decoded at a time. The codec header count is checked before its pair-vector allocation.

Every frozen pair runs the existing `CertifyQuadraticFacetPolicyCoverage` with the captured duration and work/depth limits. Actual captured owners are used unchanged; matching same-rigid endpoint support provides exclusions through the existing helper. No owners are synthesized by the replay tool. Local topology uses the solver's continuous local proof and its distinct `CertifiedLocalIntersection` result. For support or local exclusions, a separate ledger-only result preserves information that the policy wrapper can replace. The reported policy work counts that proof's work, not total CPU effort from diagnostic reevaluation.

The JSONL report contains every noncertified pair with source keys, both proof results, work/depth flags, dyadic cells, witnesses, and ownership information. The final summary counts all pairs, including certified ones. It also reports thickness-persistent rows omitted by the source census; those rows receive no continuous geometry qualification from this replay. Future nonlinear captures retain these rows. Exit zero and `complete:true` mean the diagnostic traversal completed; they do not mean all pairs certified or the physical run passed. An interrupted/failed traversal retains its partial report without a final complete summary. Existing report files are never replaced; the report has a separate 2 GiB cap.

`MissingAcceptedOwner` or `PotentialContact` can accompany a depth-exhaustion flag. `PossibleGeometricCrossing` is not proof of penetration. Intersection/owner witnesses can refer to earlier or topologically local cells. Frozen certificate ordinals index the subset; `accepted_source_order` is the original live ledger identity. Linear baseline depth zero means unknown. The earlier pinned affine and nonlinear fixture tests remain unchanged.

Failure captures use the same binary pair codec and its existing baseline
status/work/depth fields. New failure-manifest v2 adds the explicit boolean
`baseline_observed`; v1 artifacts remain readable with that flag false, including
historical default statuses that were never measured. The updated reader validates
version, flag type/uniqueness and every original common field. Older strict v1
readers reject v2 metadata rather than guessing its meaning. No old fixture is
rewritten. Human scope strings do not drive the flag.

When the terminal nonlinear observer supplies a result, the stored baseline is
its observed combined production result: work includes initial-root plus coverage
work, depth is their maximum, and coverage used the remaining production budgets.
Standalone diagnostic replay continues to use the configured limits; it is not an
exact production-budget replay. Absent baseline values retain the exact old
unreported labels. Changing provenance labels never changes numerical replay
comparison or authorizes a physical step.
