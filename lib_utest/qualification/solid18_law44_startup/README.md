# Rear LAW44 time-zero force and external scratch

`law44::InitializeForce` takes only the prepared reference, material and common
initial translation. It constructs virgin history internally and evaluates the
selected native force/material caller at TT0, DT1=DT2=0, sample0. The result is an
initial force/state cache, with no completed-interval time or energy claim.
Ordinary point `Update` and element `EvaluateForce` still require positive dt.
Point `Initialize` likewise accepts no prior history and requires exactly zero dt.

The unchanged material arithmetic lives behind those separate entries. The
constructor retains actual current-geometry roundoff, SRHO3 density/AMU, eight
strain/PLA/filter/cursor states, native SSP/ET, viscosity, storage and work, the
ZEP3 pressure split, and all eight original force slots. Repeated source nodes
are not coalesced. Initial density and mass authority remain the qualified
reference; no source mass, scalar inertia or stiffness formula changes here.
The raw point-summed stiffness is still the complete SCUMU3 input, before its
qualified per-slot FOURTH multiplication.

`law44::detail::ForceScratch` provides one caller-owned unpublished workspace
for a future worker. It reuses the existing staged force body and history
writer. Inputs must be disjoint from scratch; rejection may leave scratch
partial. Public value output remains staged, while a future owner must publish
scratch only after success. Constructor reuse clears all prior material state;
each successful force calculation also restores excluded I_SH0 shear/cross
fields to zero. No arena, per-step allocation, resident or model is added.

The native ordinary C ABIs retain their positive-step guards. New constructor
entries construct virgin point/element inputs and call the same native bodies.
The element constructor receives independent native starter volumes, initial
density, saved frame coordinates and original-slot mapping, never TL history.
Complete CONTRL and RESOL donors are reused from the existing rigid qualifier;
their time-zero and first force-call schedule is authenticated in the manifest.
The previously qualified material arithmetic, complete native point body and
complete native force body retain explicit unchanged-region SHA256 checks.

The host gate compares external scratch against the value entry over 400
intervals and alternating topologies, deliberately dirties excluded fields,
and checks late point failure/retry, other-worker independence, ordinary dt0
rejection and constructor output atomicity. New independent native tests check
the material TT0 branch, both topologies at rest/common velocity followed by
32 carried intervals, and all 306 original cells followed by three intervals.
New CUDA tests exercise the external scratch constructor, 32 intervals with
late point/phase/initial-velocity rejection and retry, and all 306 constructors
plus their first interval. Existing native point/rear and CUDA gates run in the
same owning project. Comparison rules are reused unchanged.

The author runs only bounded host, source-identity and C++/CUDA-shaped syntax
checks. Native, NVCC, CUDA and original geometry execution remain root gates.

```sh
cmake -S lib_utest/qualification/solid18_law44_startup -B <new-build> \
  -DCMAKE_BUILD_TYPE=Release -DREAR18_STARTUP_NATIVE=ON \
  -DREAR18_STARTUP_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DREAR18_SOURCE_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-rear-metal-geometry-1
cmake --build <new-build> -j2
ctest --test-dir <new-build> --output-on-failure -j1
```

Bazel owners: `//lib_utest/qualification/solid18_law44_startup:host_check`,
`//lib_utest/qualification/solid18_law44_force:host_check`,
`//lib_utest/qualification/solid_law44_point:host_check`.

Author evidence: the host CMake project passes all five CTests (four new
startup/scratch functions, seven existing point functions, five existing rear
functions and source identities). The bounded configure/build/test took 5.00 s
with 245,292 KiB peak child RSS. Three native C++ units and the CUDA-shaped
original-source branches pass syntax checks. No native/GPU execution is claimed.
