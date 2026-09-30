# Owning mapped TYPE25 qualification

Configure from a frozen TL checkout:

```sh
cmake -S lib_utest/qualification/type25_mapped -B /tmp/tl-type25-mapped-root-1 \
  -DCMAKE_BUILD_TYPE=Release -DTYPE25_MAPPED_NATIVE=ON -DTYPE25_MAPPED_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build /tmp/tl-type25-mapped-root-1 --parallel 1
ctest --test-dir /tmp/tl-type25-mapped-root-1 --output-on-failure
```

Root schedules native/CUDA and build resources. Targets are
`type25_mapped_host_test` (5 new + 3 existing host functions),
`type25_mapped_native_test` (4 functions), and
`type25_mapped_cuda_test` (4 functions). Two source identity CTests authenticate
reused donors and exact coefficient extracts. The Bazel host owner is
`//lib_utest/qualification/type25_mapped:type25_mapped_host_check`.
Native/CUDA parity uses owning CMake, not a second native dependency in production.

The CUDA fixture uses the existing complete physical/CIN fixture and a shared
zero-damping TYPE25 model. It measures actual force/couple/STI/STIR before adding
explicit prescribed test contributions for the other families. One test peer
claims the same publication source, performs every candidate check, commits the
only FENodalState, and swaps the existing TYPE25 slabs. Last-parent geometry
failure and later-contributor rejection both preserve accepted history and all
owner kinematics/current/saved CIN coefficients. Repeating the attempt must
produce bit-identical complete TYPE25 fields.

The source unit conversion enclosure in `NativeValues.cpp` is the existing
TYPE25 frame/force comparison budget; no production tolerance is added. Native
nodal stiffness is computed from independent property/length/MS/IN packets,
including positive-damping and native rigid-primary negative/profile controls.
The production zero-damping case must remain independent of those nodal inputs.

The 2,828 / 372,435 layout check is a scalar extent test. Original source and
full-domain runtime admission remain the separate app integration gate.
