# Beam18 model, snapshot and V5 coefficient qualification

This slice owns no force equations or runtime owner. It uses the qualified
beam18 Reference/Force material contract and the existing complete native
PCOORI/PEVECI/PMASS preparation oracle. Native donor source and original 142-beam
geometry/curve authentication are reused from their owning qualifiers.

Host controls cover immutable curve lifetime and MID pooling, original source
order, N3 exclusion, exact working-unit M/J, independent domain identity,
changed MID and late EID/position failure, exact cap/one-byte-short retry, shared
backing discounts, ordered mixed coefficient addition, and source output aliases.
The V5 test appends after both V3 and V4 solid profiles while keeping old entry
points closed. Existing nodal coefficient, extended coefficient and extended
ledger host suites are included by default.

The optional native gate checks two working-unit profiles and all 142 original
beams, all four original materials with the authenticated 46-point curve, 284
endpoint records and 146 unique endpoint NIDs. It compares the immutable snapshot
and final ledger scatter against independent native endpoint M/J, not a second
beam-mass implementation. Only native/reference comparison uses the existing
3e-13 coefficient bound; production association and snapshot checks are exact.
It records actual model/ledger payload forecasts. Original orientation N3 remains
one extra source node, with no endpoint contribution. No native/GPU execution is
claimed by the author; root owns those gates.

Root commands, under the normal serialized resource guard:

```sh
cmake -S lib_utest/qualification/beam18_model -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DBEAM18_MODEL_NATIVE=ON \
  -DBEAM18_MODEL_SOURCE_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-beam18-geometry-1
cmake --build <build> -j4
ctest --test-dir <build> --output-on-failure -j1
```

Owning Bazel targets:

```sh
bazel build //lib_src/elements/beam18:model \
  //lib_src/assembly:beam18_node_contributions \
  //lib_src/assembly:nodal_coefficient_ledger \
  //lib_utest/qualification/beam18_model:host \
  //lib_utest/qualification/beam18_model:output
```

The owning manifests retain prior shared-file records and identify additive
Model/build/V5 changes. Reference, point, force and native source bytes are
unchanged. This slice does not admit full-case physics, joints or a beam resident.

Author gate: 39 host functions pass (10 new; 29 existing), plus two source
identity CTest entries in `beam18-model-author-tests-3.json`; individual XML is
under `beam18-model-author-functions-3/`. The original-fixture native test passes
C++ syntax only in `beam18-model-author-syntax-1.json`, using root's already
authenticated generated header. Builds/tests used one CPU and a 512 MiB RSS cap.
The first test compile exposed a missing default ConstView initializer; a later
new test needed an explicitly const pointer list. Both are fixed and failed
build reports retained. The first complete identity run exposed dependent
receipt changes; prior records are preserved and their owners pass after the
narrow refresh. No author native/NVCC/GPU/Bazel/full-source execution occurred.
