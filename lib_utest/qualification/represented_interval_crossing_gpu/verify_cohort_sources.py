#!/usr/bin/env python3
"""Source-only checks for optional native numerical-cohort ownership and order."""
from pathlib import Path
import json
import re

here = Path(__file__).resolve().parent
root = here.parents[2]
collision = root / "lib_src/collision"
native = (collision / "RepresentedIntervalCrossing.cpp").read_text()
lease = (collision / "represented_interval_crossing/DeviceExecution.h").read_text()
admission = (collision / "represented_interval_crossing/CohortAdmission.h").read_text()
directory = collision / "represented_interval_crossing/native_device"
workspace = (directory / "Workspace.cpp").read_text()
cohort = (directory / "Cohort.cpp").read_text()
transport = (directory / "Transport.cpp").read_text()
storage = (directory / "Workspace.h").read_text()
public = (collision / "RepresentedIntervalCrossingGpu.h").read_text()
layout = (directory / "Layout.cpp").read_text()

assert "numeric_cohort_pairs = 0" in public
assert "MaximumNumericCohortPairs = 4096" in (directory / "KernelTypes.h").read_text()
assert "AuthenticatedNumericCohort(const AuthenticatedNumericCohort&) = delete" in lease
assert lease.count("ConstructionKey() noexcept {}") == 2
assert lease.count("static_assert(!std::is_aggregate_v<ConstructionKey>)") == 2
assert "std::optional<AuthenticatedNumericCohort> cohort_" in lease
assert "bool prefetch_disjoint = false" in native
assert "roster.prefetch_disjoint = detail::NumericCohortRanges" in native
assert "return storage.DisjointFromOwned(data, bytes)" in native
assert "roster.prefetch_disjoint ? roster.ordered_pairs : nullptr" in native
assert "roster.prefetch_disjoint ? roster.slice_capacity : 0" in native
assert "owned(paths, path_bytes) && owned(pairs, pair_bytes)" in admission
assert "disjoint(paths, path_bytes, pairs, pair_bytes)" in admission
assert "Bytes(paths, path_count" in admission and "Bytes(pairs, pair_count" in admission
assert native.index("roster.authenticated = true") < native.index("scene.cohort_.emplace")
assert "(cohort_capacity / scene.slice_capacity_) * scene.slice_capacity_" in native
assert "if (cohort_capacity < pair_count)" in native
assert "return work.scene_.cohort_ ? ExecuteCohort(work) : ExecuteSlice(work)" in workspace
assert "CanonicalizePair(" in native and "CanonicalizePair(" in cohort
assert "cohort.computed_ = true" in cohort
assert cohort.index("Submit(work, jobs)") < cohort.index("cohort.computed_ = true")
assert cohort.index("cache_[first + i].result.key, work.pairs()[i].key") < cohort.index("work.status()[i].complete = true")
assert "if (!cached.device_complete)" in cohort
assert "++report_.host_pairs" in cohort and "++report_.consumed_device_pairs" in cohort
assert "FaultScope(work);\n    return DeviceFailure(error)" in transport
assert "failure.input_pair = work.pairs()[ordinal - work.first_ordinal_].input_pair" in transport
for forbidden in ("AuthenticatedScene*", "AuthenticatedNumericCohort*", "RepresentedTrianglePath*", "cohort_ready"):
    assert forbidden not in storage, forbidden
assert "host.Append<CachedPair>(limits.numeric_cohort_pairs" in layout
assert "forecast.numeric_cache_host_bytes = next.host_cache.bytes" in layout
for production in (workspace, cohort, transport):
    assert "cudaDeviceSetLimit" not in production
    assert "GTest" not in production
assert "CacheIsOptInAndComposesExactBoundedPayloads" in (here / "NumericCohortHostTest.cpp").read_text()
tests = (here / "NumericCohortCudaTest.cpp").read_text()
assert len(re.findall(r"TEST\(NativeGpuNumericCohortCuda,", tests)) == 9
assert "fault_cohort_begin" in tests and "LateNativeWorkFailureSuppressesPrefetchedLaterSlices" in tests
for name in ("NumericCohortHostTest.cpp", "NumericCohortCudaTest.cpp", "NumericCohortCases.h", "CopyFault.cpp"):
    assert name in (here / "CMakeLists.txt").read_text()
    assert name in (here / "BUILD.bazel").read_text()
print(json.dumps({"status": "passed", "scope": "private numerical cache and unchanged native publication slices",
                  "numerical_execution": False}))
