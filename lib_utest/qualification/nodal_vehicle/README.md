# Explicit large nodal-owner admission

This gate qualifies the conventional physical-node owner only. It does not admit
vehicle shell/material/contact/connector/rigid capacities or a full-vehicle model.
The 393,165-node fixture uses the original total vehicle node count with synthetic
prescribed motion/loads; it is not the actual source geometry. The current full
shell source union is separately counted as 368,177 nodes. The 524,288-node test
exercises the new owner hard limit, including sparse loads beyond all old bounds.

Existing callers retain `max_nodes=2048` and `max_device_bytes=1 MiB`. Large
callers must explicitly set both, up to 524,288 nodes and 256 MiB. `max_nodes`
is appended to NodalStateConfig: existing positional aggregate initializers keep
their field meanings and acquire the original default count limit. This source
compatibility does not promise an unchanged C++ struct binary ABI; rebuild owners
and callers together. Limits do not allocate their maximum: all arrays retain
the actual node count. No stepping allocation, selector, clock or arithmetic
change is introduced. The optional rigid owner still admits only 64 disjoint
plain groups, at most 256 members each, with its existing initialization scope.

The checked layout forecasts the same separate cudaMalloc buffers, with no
invented alignment padding between allocations. Extended nodal payload is
411 bytes/node plus the existing control. Its two state slabs each contain
19 doubles/node; groups append 18 doubles/group to each slab plus immutable
metadata. Optional acceleration capture adds 48 bytes/(node+group) to transient
scratch and reuses the already larger host state staging. At the maximum count,
baseline nodal device payload is 205.5 MiB and host staging is 76 MiB plus masks;
the complete allowed current group/capture combination fits below 256 MiB.
The serial owner execution remains deliberate for this capacity increment.

Three pure host tests cover exact separate-allocation size at odd and large
counts, group/capture accounting, insufficient caps, and overflow preserving
all output regions. Two actual CUDA tests cover:

- 524,288-node sparse load, first half kick/later full kick, final free-node
  translation/spin and adjacent fixed-node force/couple reactions; complete
  accepted/prepared fields, active allocation slope, full-count membership query,
  last-node arithmetic rejection, late quaternion readback failure, insufficient
  output capacity, unchanged output/token/accepted state, and bit-exact retry.
- 393,165-node explicit count/byte admission, default and excessive-cap rejection,
  late initial quaternion rejection, no partial owner publication, and retry.

Author evidence: all three host tests pass in
`/tmp/tl-nodal-vehicle-host-1`; report `/tmp/tl-nodal-vehicle-host-tests-1.xml`.
Owning host configure/build/test and the CUDA-test host C++ syntax check used
one CPU3/512 MiB virtual cap; build peak 176,676 KiB. No author GPU build or run.
Root independently reviews and owns actual CUDA and affected old regression gates.

Root's standalone qualification commands, run under its shared resource guard:

```sh
cmake -S lib_utest/qualification/nodal_vehicle -B /tmp/tl-nodal-vehicle-cuda-1 \
  -DCMAKE_BUILD_TYPE=Release -DTL_NODAL_VEHICLE_ENABLE_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120 -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
cmake --build /tmp/tl-nodal-vehicle-cuda-1 --target nodal_vehicle_layout_check nodal_vehicle_owner_check -j 4
ctest --test-dir /tmp/tl-nodal-vehicle-cuda-1 --output-on-failure
```

Owning Bazel targets are
`//lib_utest/qualification/nodal_vehicle:nodal_vehicle_layout_check` and
`//lib_utest/qualification/nodal_vehicle:nodal_vehicle_owner_check`.
Retain the existing nodal step/temporal/capacity, rigid owner and prepared
snapshot/capture regressions; their 2,048-node fixtures keep their old constants.
Only the old oversized-budget rejection assertion now targets the new explicit
256 MiB hard cap: larger budgets are an intentional new capability.
