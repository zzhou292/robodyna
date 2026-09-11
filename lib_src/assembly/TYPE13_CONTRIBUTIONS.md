# Qualified TYPE13 values mapped to a declared node domain

`Type13NodeContributions` retains the already qualified immutable TYPE13 Model
and NodalNodeDomain. Its two records per connection contain the unchanged
`Model::Endpoint` result: source element/property/node IDs, owner index, mass,
native total isotropic J and the numerical added-J attribution. Record fields
also retain model-connection index and endpoint0/1. No structure padding is an
identity or persistence encoding.

The API explicitly names `PreparedEndpointSI` and `ModelThenEndpoint`. It adds
no sum, sort, inverse M/J, mutable coefficient override, clock or owner. Native
total J already contains its numerical floor; added J must not be added again.
Raw declared inertia and all working-unit/reference/curve values remain in the
retained model. No physical-J channel is invented by rounded subtraction.

Source instance and global extent must agree with the domain. Each declared
owner mapping must match its exact source NID. Endpoint positions compare the
existing prepared `Startup.reference.position_m[0/1]` bits with domain positions.
An orientation N3 receives no coefficient or TYPE13 DOF requirement, even when
another producer makes that source node physical. A mapped N3 must also match
the domain. The current reference stores only two prepared SI endpoint
positions; N3 consistency uses the same existing one-way `fixed3::Scale` on
its retained native coordinates. An unindexed N3 that appears in the domain is
checked for the same source-coordinate consistency and stays unindexed in this
model. Absent N3 remains reference evidence only. No SI/native round trip occurs.

Complete record counts and payload are checked before model node reads. The
bounded arena holds exactly two records per connection. Payload includes the
adapter handle/implementation, records and all retained model/domain backing;
embedded handles are counted once, with existing 64-byte shared-control
reserves. Already released model construction scratch is not reallocated here.
Adapter startup and retained payload are equal. Caller inputs, allocator
bookkeeping and ordinary stack frames are outside this deterministic budget;
it is not process RSS. Hard limits are 8,192 connections, 524,288 domain nodes and
256 MiB. Failed startup preserves an empty, retryable handle; copies share the
complete immutable lifetime and Matches compares model/domain identity.

Strict ShellBatchBinding and NodalMassBinding remain unchanged. This adapter
does not prove complete source coverage or independent/dependent/absent DOFs.
The selected TYPE13 property/force profile consumes endpoint translations and
rotations; a missing N3 contribution does not make its global DOFs absent.

The full native composition has a separate required order: point-mass reading,
solid and quadrilateral shell terms, truss/beam terms, one combined spring stream,
then T3 terms, before raw rigid-body initialization. Springs use the converted
native element-ID sort, followed by N1/N2. TYPE13 preserves the original EID;
converted constrained-spotweld TYPE25 IDs are generated separately from WIDs.
The current legacy shell-then-TYPE25 total cannot simply receive a TYPE13
subtotal and be called this native result. SI termwise addition also need not
match native-unit summation followed by one SI conversion. The approved future typed composer uses a named deterministic SI order over
unchanged per-element values, with independent high-precision error bounds and
native small controls. Full native bitwise summation and TYPE25 ID allocator
reproduction are not required. Exact source/count/zero semantics and all later
non-additive rigid/constraint/DOF phases remain required. The
workspace plan `planning/TYPE13_NODE_CONTRIBUTIONS.md` records authenticated
native paths, ordering, inertia semantics and the later DOF interface.

Host qualification includes shared endpoints, descending model EID order,
numerical floor attribution, interleaved extras, mapped/unindexed N3, signed
zeros, late identity/position failure and retry, exact complete cap, lifetime
and full model identity. An optional original 4,442-element gate checks all 8,884
records, all 7,493 endpoint NIDs, original SI coordinate bits, untouched N3 and
exact cap/retry. Its source fixture is shared with the old native model test;
the native model test body and its oracle calls are unchanged. Original mapping
parity is not a combined native-sum or full vehicle owner receipt.

```sh
cmake -S lib_utest/qualification/type13_contributions -B BUILD_DIR -DCMAKE_BUILD_TYPE=Release -DTYPE13_CONTRIBUTIONS_ORIGINAL=ON
cmake --build BUILD_DIR --target type13_contribution_check type13_contribution_original_check -j1
ctest --test-dir BUILD_DIR --output-on-failure
```

Production CMake target: `tl_type13_node_contributions`. Bazel production target:
`//lib_src/assembly:type13_node_contributions`; tests are in
`//lib_utest/qualification/type13_contributions`, with the original target manual.
Production links no native oracle, engine, Fortran or CUDA runtime.
