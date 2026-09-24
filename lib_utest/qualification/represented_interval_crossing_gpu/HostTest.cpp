// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
namespace native_gpu_test {
TEST(NativeGpuForecast, UninitializedFacadeDoesNotPublishOrAllocate) {
  c::RepresentedIntervalCrossingGpu owner;
  EXPECT_FALSE(owner.initialized()); EXPECT_FALSE(owner.results().complete);
  EXPECT_EQ(owner.forecast().owned_host_bytes, 0u);
  const auto result = owner.Certify(nullptr, 0, nullptr, 0, nullptr);
  EXPECT_EQ(result.native.status, S::NotInitialized);
  EXPECT_EQ(result.device.status, D::NotInvoked);
}
TEST(NativeGpuForecast, NativeInvalidLimitsRetainNativeDiagnosticBeforeDeviceAdmission) {
  auto limits = Limits(); limits.native.max_depth = 53;
  const auto old = c::RepresentedIntervalCrossing::Preflight(limits.native);
  const auto current = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  fixture::SameNativeReport(current.report.native, old.report);
  EXPECT_EQ(current.report.device.status, D::NotInvoked);
  EXPECT_EQ(current.forecast.device_bytes, 0u);
}
TEST(NativeGpuForecast, ExplicitWorkerAndExactPayloadCapsHaveNoHiddenGrowth) {
  auto limits = Limits();
  const auto cpu = c::RepresentedIntervalCrossing::Preflight(limits.native);
  const auto expected = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  ASSERT_EQ(expected.report.native.status, S::Ok);
  ASSERT_EQ(expected.report.device.status, D::Ok);
  EXPECT_EQ(expected.forecast.native.owned_host_bytes, cpu.forecast.owned_host_bytes);
  EXPECT_EQ(expected.forecast.owned_host_bytes,
      cpu.forecast.owned_host_bytes + expected.forecast.workspace_host_bytes);
  EXPECT_GE(expected.forecast.startup_host_bytes, expected.forecast.owned_host_bytes);
  EXPECT_GT(expected.forecast.device_scratch_bytes, 0u);
  EXPECT_GT(expected.forecast.device_dfs_bytes, 0u);
  auto one = limits; one.device_workers = 1;
  const auto single = c::RepresentedIntervalCrossingGpu::Preflight(one);
  ASSERT_EQ(single.report.native.status, S::Ok);
  EXPECT_EQ(expected.forecast.device_scratch_bytes, limits.device_workers * single.forecast.device_scratch_bytes);
  EXPECT_EQ(expected.forecast.device_dfs_bytes, limits.device_workers * single.forecast.device_dfs_bytes);
  auto tight = limits;
  tight.max_device_bytes = expected.forecast.device_bytes;
  tight.max_workspace_host_bytes = expected.forecast.startup_host_bytes - cpu.forecast.startup_host_bytes;
  EXPECT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(tight).report.native.status, S::Ok);
  --tight.max_device_bytes;
  EXPECT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(tight).report.native.status, S::ResourceLimit);
  ++tight.max_device_bytes; --tight.max_workspace_host_bytes;
  EXPECT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(tight).report.native.status, S::ResourceLimit);
  limits.device_workers = 0;
  const auto invalid = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  EXPECT_EQ(invalid.report.native.status, S::InvalidInput);
  EXPECT_EQ(invalid.report.device.status, D::InvalidInput);
}
}  // namespace native_gpu_test
