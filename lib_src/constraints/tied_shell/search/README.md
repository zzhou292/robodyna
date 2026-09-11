# Bounded geometric startup search

`AssessSearch` accepts complete, already packed IRECT/NSV arrays in explicit
original working length units. It creates a private existing GPU `Broadphase`,
with masters in mesh 0 and repeated-point slaves in mesh 1, self pairs disabled
and no neighbor map. Each master uses the qualified native MAX-diagonal box
radius rounded outward by `nextafter`. The existing GPU directed bound arithmetic
and inclusive box test provide a conservative candidate set on any chosen axis.
The driver then restores `(NSV, IRECT)` order, rejects duplicate pairs, excludes
physical own nodes and applies the exact original-unit native box filter. It
calls the shared native MIN-diagonal projection/ordered choice values without
copying their equations. Repeated T3 slots retain native order.

The result contains every slave. Unmatched rows remain unmatched; a singular or
otherwise rejected force patch remains the selected geometric association with
its separate numerical status. `ordered_master` is one-based IRECT rank. Outside
warnings and native selection distance are projection diagnostics. No I2TID3
compaction/warnings, classifier, coefficients, constraint owner or mechanics
clock are produced. This first profile explicitly requires zero secondary
shell incidence/thickness, established by the source producer.

All allocations and publication are private to a synchronous call. Failure
preserves the prior complete result. Hard count and impossible fixed-byte caps
reject before borrowed payload reads or CUDA operations. CUDA scratch is queried
and admitted before any allocation. The reported byte forecast includes both
host mesh copies, native bounds, radius arrays, complete output rows, all GPU
mesh/AABB/sort/scan storage, actual CUB scratch and bounded pair capacity. The
capacity is a reservation, not a request to allocate unused pairs. Borrowed
source/input storage and any caller-retained previous result are separate.
Numbers describe payload, not allocator metadata, CUDA driver/context growth or
process RSS. The application additionally charges its retained source exactly
once and its typed input staging. Maximum defaults are 128 MiB device and
512 MiB internal host payload; the app can lower these limits.

The source-shaped count test uses 194,622 working nodes, 171,813 masters and
11,165 secondaries without allocating those arrays. Actual CUB sizes and full
source candidate counts require the owning GPU gate. Host tests do not establish
GPU conservativeness or full original-source completeness; the independent
native bucket qualification compares those separately.

Root qualification passes all eight driver functions, including actual CUDA,
`tied-search-driver-root-tests-1`. The app's complete source gate also passes:
all11165 selected ranks, per-node exact-box counts and projections match an
independent native traversal over171813 masters. Both pipelines enumerate31104
pairs; there are no unmatched nodes, outside flags or rejected selected patches.
Driver payload forecast is100087022 B host /106955790 B device; actual CUB
sort/scan scratch is2237695 /1791 B. The owning Bazel driver target passes
`qbat-catalog-search-driver-bazel-build-1`. This is startup mapping; native
finalization, classification and mechanics remain separate.

Standalone qualification:

```
cmake -S lib_utest/qualification/tied_search_driver -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DTL_TIED_SEARCH_DRIVER_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> --parallel 1
ctest --test-dir <build> --output-on-failure
```

`tl_tied_search_driver_values` builds the budget/order helpers without compiling
CUDA. `tl_tied_search_driver` owns the actual synchronous GPU call. The six host
functions and two CUDA functions have separate executables/CTest entries.
