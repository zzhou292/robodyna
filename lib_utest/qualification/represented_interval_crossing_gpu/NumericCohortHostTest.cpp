// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NumericCohortCases.h"
#include "lib_src/collision/represented_interval_crossing/CohortAdmission.h"
namespace native_gpu_cohort_test {
TEST(NativeGpuNumericCohortValues, CacheIsOptInAndComposesExactBoundedPayloads) {
  const auto source = Roster(257);
  auto limits = Limits(source, 256, 0);
  EXPECT_EQ(c::RepresentedIntervalGpuLimits{}.numeric_cohort_pairs, 0u);
  const auto before = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  ASSERT_EQ(before.report.native.status, S::Ok);
  EXPECT_EQ(before.forecast.numeric_cache_host_bytes, 0u);
  limits.numeric_cohort_pairs = 4096;
  const auto after = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  ASSERT_EQ(after.report.native.status, S::Ok);
  EXPECT_EQ(after.forecast.numeric_cohort_pairs, 4096u);
  EXPECT_GT(after.forecast.numeric_cache_host_bytes, 4096u * sizeof(c::RepresentedIntervalResult));
  EXPECT_GT(after.forecast.workspace_host_bytes, before.forecast.workspace_host_bytes);
  EXPECT_GT(after.forecast.device_bytes, before.forecast.device_bytes);
  EXPECT_EQ(after.forecast.native.owned_host_bytes, before.forecast.native.owned_host_bytes);
  EXPECT_EQ(after.forecast.native.pair_capacity, 256u);
  EXPECT_EQ(after.forecast.native.result_capacity, 256u);
  auto tight = limits;
  tight.max_device_bytes = after.forecast.device_bytes;
  tight.max_workspace_host_bytes = after.forecast.startup_host_bytes - after.forecast.native.startup_host_bytes;
  ASSERT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(tight).report.native.status, S::Ok);
  --tight.max_device_bytes;
  EXPECT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(tight).report.native.status, S::ResourceLimit);
  ++tight.max_device_bytes; --tight.max_workspace_host_bytes;
  EXPECT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(tight).report.native.status, S::ResourceLimit);
}
TEST(NativeGpuNumericCohortValues, CohortCapCannotRaiseNativeInputResultOrWorkLimits) {
  const auto source = Roster(5);
  auto limits = Limits(source, 2, 4097);
  const auto invalid = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  EXPECT_EQ(invalid.report.native.status, S::InvalidInput);
  EXPECT_EQ(invalid.report.device.status, D::InvalidInput);
  limits.numeric_cohort_pairs = 4096;
  limits.native.max_work_per_pair = 0;
  const auto expected = c::RepresentedIntervalCrossing::Preflight(limits.native);
  const auto actual = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  test::SameNativeReport(expected.report, actual.report.native);
  EXPECT_EQ(actual.report.device.status, D::NotInvoked);
}
TEST(NativeGpuNumericCohortValues, WiderRangeProofCanFailWhileOriginalSlicesRemainAdmissible) {
  const auto source = Roster(9);
  // Actual typed arrays; the owned-range model marks a later suffix as owned.
  // No pointer punning, forged scene or caller-provided execution flag is used.
  const auto disjoint = [](const void* a, std::size_t an, const void* b, std::size_t bn) {
    if (!an || !bn) return true;
    const auto x = reinterpret_cast<std::uintptr_t>(a), y = reinterpret_cast<std::uintptr_t>(b);
    return a && b && an <= UINTPTR_MAX - x && bn <= UINTPTR_MAX - y &&
        (x + an <= y || y + bn <= x);
  };
  const auto owned = [&](const void* p, std::size_t bytes) {
    return disjoint(p, bytes, source.pairs.data() + 4, 2 * sizeof(source.pairs[0]));
  };
  const auto check = [&](std::size_t offset, std::size_t count) {
    return batch::detail::NumericCohortRanges(source.paths.data(), source.paths.size(),
        source.pairs.data() + offset, count, owned, disjoint);
  };
  EXPECT_FALSE(check(0, source.pairs.size()));
  EXPECT_TRUE(check(0, 2));
  EXPECT_TRUE(check(2, 2));
  EXPECT_FALSE(check(4, 2));
  unsigned calls = 0;
  const auto count_owned = [&](const void*, std::size_t) { ++calls; return true; };
  EXPECT_FALSE(batch::detail::NumericCohortRanges(source.paths.data(), SIZE_MAX,
      source.pairs.data(), 1, count_owned, disjoint));
  EXPECT_EQ(calls, 0u);
}
}  // namespace native_gpu_cohort_test
