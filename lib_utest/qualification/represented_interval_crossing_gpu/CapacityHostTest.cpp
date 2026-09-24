// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
namespace native_gpu_test {
TEST(NativeGpuCapacity, DefaultStays128AndWiderScratchIsFullyForecast) {
  auto limits = Limits();
  EXPECT_EQ(limits.device_workers, 128u);
  limits.device_workers = 1;
  const auto single = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  ASSERT_EQ(single.report.native.status, S::Ok);
  for (unsigned workers : {128u, 512u, 2048u, 4096u}) {
    limits.device_workers = workers;
    const auto wider = c::RepresentedIntervalCrossingGpu::Preflight(limits);
    ASSERT_EQ(wider.report.native.status, S::Ok);
    EXPECT_EQ(wider.forecast.device_workers, workers);
    EXPECT_EQ(wider.forecast.device_scratch_bytes, workers * single.forecast.device_scratch_bytes);
    EXPECT_EQ(wider.forecast.device_dfs_bytes, workers * single.forecast.device_dfs_bytes);
    EXPECT_EQ(wider.forecast.workspace_host_bytes, single.forecast.workspace_host_bytes);
    EXPECT_EQ(wider.forecast.native.owned_host_bytes, single.forecast.native.owned_host_bytes);
    auto tight = limits;
    tight.max_device_bytes = wider.forecast.device_bytes;
    ASSERT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(tight).report.native.status, S::Ok);
    --tight.max_device_bytes;
    EXPECT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(tight).report.native.status, S::ResourceLimit);
  }
}
TEST(NativeGpuCapacity, MaxWorkerAndCombinedArenaCapsRejectBeforeAllocation) {
  auto limits = Limits();
  limits.device_workers = 4097;
  const auto too_many = c::RepresentedIntervalCrossingGpu::Preflight(limits);
  EXPECT_EQ(too_many.report.native.status, S::InvalidInput);
  EXPECT_EQ(too_many.report.device.status, D::InvalidInput);
  limits.device_workers = 4096;
  limits.max_device_bytes = 1;
  EXPECT_EQ(c::RepresentedIntervalCrossingGpu::Preflight(limits).report.native.status, S::ResourceLimit);
}
}  // namespace native_gpu_test
