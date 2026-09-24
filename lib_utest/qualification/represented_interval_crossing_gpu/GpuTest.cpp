// SPDX-License-Identifier: AGPL-3.0-or-later
#include "GpuFixture.h"
#include <cmath>

namespace native_gpu_test {
TEST(NativeGpuCuda, MixedExactAndFallbackResultsMatchEveryNativeFieldAcrossWorkersAndCalls) {
  Streams streams;
  for (unsigned workers : {1u, 32u, 128u}) {
    auto limits = Limits(); limits.device_workers = workers;
    auto cpu = fixture::Owner(limits.native); c::RepresentedIntervalCrossingGpu gpu;
    ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
    auto cases = Mixed(); const auto expected_device = DevicePairs(cases, limits.native.max_depth);
    ASSERT_GT(expected_device, 0u); ASSERT_LT(expected_device, cases.pairs.size());
    for (unsigned repeat = 0; repeat < 3; ++repeat) {
      const auto result = Compare(cpu, gpu, cases, streams.first);
      ASSERT_EQ(result.native.status, S::Ok);
      EXPECT_EQ(result.device.status, D::Ok);
      EXPECT_EQ(result.device.device_pairs, expected_device);
      EXPECT_EQ(result.device.host_pairs, cases.pairs.size() - expected_device);
      EXPECT_EQ(result.device.batches, 1u);
      std::reverse(cases.pairs.begin(), cases.pairs.end());
      for (auto& pair : cases.pairs) std::swap(pair.first, pair.second);
    }
  }
}
TEST(NativeGpuCuda, MultipleBlocksAndGridStrideReuseEveryWorkerOn257CanonicalJobs) {
  Streams streams; auto limits = Limits();
  limits.native.max_paths = 600; limits.native.max_input_pairs = 300;
  limits.native.max_results = 300; limits.native.max_total_work = 300 * 64;
  limits.device_workers = 128;
  auto cpu = fixture::Owner(limits.native); c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  Cases cases;
  const std::array<c::Vec3, 3> collapsed{{{2,2,2},{2,2,2},{2,2,2}}};
  for (unsigned row = 0; row < 257; ++row) {
    if (row % 4 == 0) Add(cases, Positive(), Positive(), Positive());
    else if (row % 4 == 1) Add(cases, Positive(), Positive(3), Positive(3));
    else if (row % 4 == 2) Add(cases, Positive(), collapsed, collapsed);
    else Add(cases, Positive(), Positive(3), Positive(1.5));
  }
  ASSERT_EQ(DevicePairs(cases, limits.native.max_depth), 257u);
  for (unsigned repeat = 0; repeat < 3; ++repeat) {
    const auto current = Compare(cpu, gpu, cases, streams.first);
    ASSERT_EQ(current.native.status, S::Ok);
    EXPECT_EQ(current.device.device_pairs, 257u); EXPECT_EQ(current.device.host_pairs, 0u);
    EXPECT_EQ(current.device.batches, 1u); ASSERT_EQ(gpu.results().count, 257u);
    std::reverse(cases.pairs.begin(), cases.pairs.end());
    for (auto& pair : cases.pairs) std::swap(pair.first, pair.second);
  }
}
TEST(NativeGpuCuda, ExactStorageBoundaryAndExtremeCoordinatesSelectAuthenticDeviceOrHostRows) {
  Streams streams;
  for (unsigned depth : {0u, 20u, 52u}) {
    auto limits = Limits(); limits.native.max_depth = depth;
    auto cpu = fixture::Owner(limits.native); c::RepresentedIntervalCrossingGpu gpu;
    ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
    for (unsigned bits : {125u, 126u}) {
      // Reuse NativeStorageTest's exact stored-exponent threshold construction.
      auto triangle = Positive(); triangle[0].z = std::ldexp(1., 55 + int(depth + 1) - int(bits));
      Cases cases; Add(cases, triangle, triangle, triangle);
      const auto domain = c::represented_interval_crossing::NativeStorageDomain::FromPaths(
          cases.paths[0], cases.paths[1], depth);
      ASSERT_EQ(domain.report().projection.coordinate_bits, bits);
      const auto result = Compare(cpu, gpu, cases, streams.first);
      ASSERT_EQ(result.native.status, S::Ok); ASSERT_EQ(result.device.status, D::Ok);
      EXPECT_EQ(result.device.device_pairs, bits == 125u ? 1u : 0u);
      EXPECT_EQ(result.device.host_pairs, bits == 125u ? 0u : 1u);
    }
  }
  auto limits = Limits(); auto cpu = fixture::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  for (int exponent : {-1070, -1000, 700, 1020}) {
    auto a = Positive(), b = Positive(3);
    for (auto* triangle : {&a, &b}) for (auto& point : *triangle) {
      point.x = std::ldexp(point.x, exponent); point.y = std::ldexp(point.y, exponent);
      point.z = std::ldexp(point.z, exponent);
    }
    Cases cases; Add(cases, a, b, b);
    ASSERT_EQ(DevicePairs(cases, limits.native.max_depth), 1u);
    const auto result = Compare(cpu, gpu, cases, streams.first);
    ASSERT_EQ(result.native.status, S::Ok); EXPECT_EQ(result.device.device_pairs, 1u);
  }
  for (double zero : {0., -0.}) {
    auto a = Positive(), b = Positive(3); a[0].z = zero;
    Cases cases; Add(cases, a, b, b);
    const auto domain=c::represented_interval_crossing::NativeStorageDomain::FromPaths(
        cases.paths[0],cases.paths[1],limits.native.max_depth).report();
    ASSERT_GT(domain.projection.coordinate_bits,125u);
    ASSERT_LE(domain.storage.coordinate_bits,125u);
    ASSERT_EQ(DevicePairs(cases, limits.native.max_depth), 1u);
    const auto result = Compare(cpu, gpu, cases, streams.first);
    ASSERT_EQ(result.native.status, S::Ok);
    EXPECT_EQ(result.device.device_pairs, 1u);EXPECT_EQ(result.device.host_pairs, 0u);
  }
}
TEST(NativeGpuCuda, CanonicalDedupAndSameSourceIdentityAreAuthenticatedByOriginalFrontend) {
  Streams streams; auto limits = Limits();
  auto cpu = fixture::Owner(limits.native); c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  Cases cases; Add(cases, Positive(), Positive(), Positive());
  cases.paths[1].vertices[0].key = cases.paths[0].vertices[0].key;
  cases.paths[1] = fixture::Permute(cases.paths[1], {2,0,1});
  cases.pairs.push_back({1,0}); cases.pairs.push_back({0,1});
  const auto result = Compare(cpu, gpu, cases, streams.first);
  ASSERT_EQ(result.native.status, S::Ok); EXPECT_EQ(result.native.unique_pairs, 1u);
  EXPECT_EQ(result.device.device_pairs, 1u); EXPECT_EQ(result.device.host_pairs, 0u);
  EXPECT_EQ(gpu.results().count, 1u);
}
TEST(NativeGpuCuda, WideAndUnsupportedInputsUseHostBeforeAnyDeviceLaunch) {
  Streams streams; auto limits = Limits(); auto cpu = fixture::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu; ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  Cases cases; Add(cases, fixture::BaseTriangle(), fixture::BaseTriangle(1), fixture::BaseTriangle(1));
  RequireNonzeroWideStorage(cases.paths);
  const auto wide=c::represented_interval_crossing::NativeStorageDomain::FromPaths(
      cases.paths[0],cases.paths[1],limits.native.max_depth).report();
  ASSERT_GT(wide.storage.coordinate_bits,125u);
  ASSERT_FALSE(wide.eligible);
  for(const auto& path:cases.paths)for(const auto& vertex:path.vertices)for(const auto point:vertex.endpoint)
    ASSERT_TRUE(point.x!=0 && point.y!=0 && point.z!=0);
  Add(cases, Positive(), Positive(), Positive(), c::RepresentedMotion::RigidArc);
  const auto result = Compare(cpu, gpu, cases, streams.first);
  ASSERT_EQ(result.native.status, S::Ok); EXPECT_EQ(result.device.status, D::Ok);
  EXPECT_EQ(result.device.device_pairs, 0u); EXPECT_EQ(result.device.host_pairs, 2u);
  EXPECT_EQ(result.device.batches, 0u);
}
TEST(NativeGpuCuda, EmptyAfterNonemptyAndMalformedNullInputsKeepNativePublicationSemantics) {
  Streams streams; auto limits = Limits(); auto cpu = fixture::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu; ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  Cases cases; Add(cases, Positive(), Positive(), Positive());
  ASSERT_EQ(Compare(cpu, gpu, cases, streams.first).native.status, S::Ok);
  const auto expected = cpu.Certify(nullptr, 0, nullptr, 0);
  const auto empty = gpu.Certify(nullptr, 0, nullptr, 0, streams.first);
  fixture::SameNativeReport(empty.native, expected); fixture::SameView(gpu.results(), cpu.results());
  ASSERT_TRUE(gpu.results().complete); EXPECT_EQ(gpu.results().count, 0u);
  EXPECT_EQ(empty.device.device_pairs, 0u); EXPECT_EQ(empty.device.host_pairs, 0u); EXPECT_EQ(empty.device.batches, 0u);
  ASSERT_EQ(Compare(cpu, gpu, cases, streams.first).native.status, S::Ok);
  for (unsigned mutation = 0; mutation < 3; ++mutation) {
    const auto before = gpu.results(); const auto values = Copy(before);
    const auto* paths = mutation == 0 ? nullptr : cases.paths.data();
    const std::size_t path_count = mutation == 1 ? 0 : cases.paths.size();
    const auto* pairs = mutation == 2 ? nullptr : cases.pairs.data();
    const auto original = cpu.Certify(paths, path_count, pairs, 1);
    const auto current = gpu.Certify(paths, path_count, pairs, 1, streams.first);
    fixture::SameNativeReport(current.native, original); EXPECT_EQ(current.device.status, D::NotInvoked);
    Preserved(before, values, gpu.results());
  }
  EXPECT_EQ(Compare(cpu, gpu, cases, streams.first).native.status, S::Ok);
}
TEST(NativeGpuCuda, InvalidUnusedPathOrSharedTrajectoryRejectsBeforeDeviceAndPreservesOutput) {
  Streams streams; auto limits = Limits(); auto cpu = fixture::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu; ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  Cases cases; Add(cases, Positive(), Positive(), Positive());
  ASSERT_EQ(Compare(cpu, gpu, cases, streams.first).native.status, S::Ok);
  const auto before = gpu.results(); const auto values = Copy(before);
  for (unsigned mutation = 0; mutation < 2; ++mutation) {
    auto invalid = cases;
    invalid.paths.push_back(fixture::Static(999, Positive(), 9999));
    if (!mutation) invalid.paths.back().vertices[2].endpoint[1].z = NAN;
    else {
      invalid.paths.back().vertices[0].key = invalid.paths[0].vertices[0].key;
      invalid.paths.back().vertices[0].endpoint[1].x += 1;
      invalid.paths.back() = fixture::Permute(invalid.paths.back(), {0,1,2});
    }
    const auto original = cpu.Certify(invalid.paths.data(), invalid.paths.size(), invalid.pairs.data(), 1);
    const auto current = gpu.Certify(invalid.paths.data(), invalid.paths.size(), invalid.pairs.data(), 1, streams.first);
    fixture::SameNativeReport(current.native, original); EXPECT_EQ(current.device.status, D::NotInvoked);
    Preserved(before, values, gpu.results());
  }
  EXPECT_EQ(Compare(cpu, gpu, cases, streams.first).native.status, S::Ok);
}
TEST(NativeGpuCuda, TotalWorkAndResultCapsRetainCanonicalFirstFailureAndLastPublication) {
  Streams streams;
  for (bool result_cap : {false, true}) {
    auto limits = Limits();
    if (result_cap) limits.native.max_results = 1; else limits.native.max_total_work = 1;
    auto cpu = fixture::Owner(limits.native); c::RepresentedIntervalCrossingGpu gpu;
    ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
    Cases small; Add(small, Positive(), Positive(), Positive());
    ASSERT_EQ(Compare(cpu, gpu, small, streams.first).native.status, S::Ok);
    const auto before = gpu.results(); const auto values = Copy(before);
    auto larger = small; Add(larger, Positive(), Positive(3), Positive(3));
    const auto original = cpu.Certify(larger.paths.data(), larger.paths.size(), larger.pairs.data(), 2);
    const auto current = gpu.Certify(larger.paths.data(), larger.paths.size(), larger.pairs.data(), 2, streams.first);
    fixture::SameNativeReport(current.native, original); EXPECT_EQ(current.native.status, S::ResourceLimit);
    Preserved(before, values, gpu.results());
    if (result_cap) EXPECT_EQ(current.device.status, D::NotInvoked);
    EXPECT_EQ(Compare(cpu, gpu, small, streams.first).native.status, S::Ok);
  }
}
TEST(NativeGpuCuda, StreamAndOwnedPublicationAliasesRejectWithoutPoisoningValidRetry) {
  Streams streams; auto limits = Limits(); c::RepresentedIntervalCrossingGpu gpu;
  EXPECT_EQ(gpu.Initialize(limits, nullptr).native.status, S::InvalidInput);
  EXPECT_FALSE(gpu.initialized());
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  auto cpu = fixture::Owner(limits.native); Cases cases; Add(cases, Positive(), Positive(), Positive());
  ASSERT_EQ(Compare(cpu, gpu, cases, streams.first).native.status, S::Ok);
  const auto before = gpu.results(); const auto values = Copy(before);
  EXPECT_EQ(gpu.Certify(cases.paths.data(), 2, cases.pairs.data(), 1, streams.other).native.status, S::InvalidInput);
  Preserved(before, values, gpu.results());
  EXPECT_EQ(gpu.Certify(cases.paths.data(), 2,
      reinterpret_cast<const c::RepresentedTrianglePair*>(before.data), 1, streams.first).native.status, S::InvalidInput);
  Preserved(before, values, gpu.results());
  EXPECT_EQ(Compare(cpu, gpu, cases, streams.first).native.status, S::Ok);
}
}  // namespace native_gpu_test
