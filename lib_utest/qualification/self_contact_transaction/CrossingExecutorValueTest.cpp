// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/self_contact_transaction/CrossingExecutor.h"
#include "lib_src/collision/SelfContactTransactionTypes.h"
#include <gtest/gtest.h>
namespace {
namespace c = tlfea::contact;
namespace sct = c::self_contact_transaction;
TEST(CrossingExecutorValues, DefaultCpuIgnoresUnusedGpuOptionsAndKeepsOriginalForecast) {
  c::SelfContactTransactionConfig config;
  c::SelfContactTransactionLimits limits;
  EXPECT_FALSE(config.enable_cuda_native_crossing);
  EXPECT_EQ(config.native_crossing_device_workers, 128u);
  EXPECT_EQ(config.native_crossing_numeric_cohort_pairs, 0u);
  config.native_crossing_device_workers = 0;
  config.native_crossing_numeric_cohort_pairs = SIZE_MAX;
  limits.max_device_bytes = 0;
  const auto original = c::RepresentedIntervalCrossing::Preflight(limits.crossing);
  const auto actual = sct::CrossingExecutor::Preflight(config, limits);
  ASSERT_EQ(actual.report.status, c::RepresentedIntervalStatus::Ok);
  EXPECT_EQ(actual.owned_host_bytes, original.forecast.owned_host_bytes);
  EXPECT_EQ(actual.startup_host_bytes, original.forecast.startup_host_bytes);
  EXPECT_EQ(actual.device_bytes, 0u);
  EXPECT_EQ(actual.device_allocations, 0u);
  EXPECT_EQ(actual.device_status, c::RepresentedIntervalDeviceStatus::NotInvoked);
}
TEST(CrossingExecutorValues, OneMapperRetainsAllNativeBudgetsAndExplicitExecutionOptions) {
  c::SelfContactTransactionConfig config;
  c::SelfContactTransactionLimits limits;
  config.enable_cuda_native_crossing = true;
  config.native_crossing_device_workers = 4096;
  config.native_crossing_numeric_cohort_pairs = 4096;
  limits.crossing.max_paths = 97;
  limits.crossing.max_input_pairs = 23;
  limits.crossing.max_results = 19;
  limits.crossing.max_work_per_pair = 101;
  limits.crossing.max_total_work = 202;
  limits.crossing.max_depth = 7;
  limits.crossing.worker_count = 3;
  const auto mapped = sct::CrossingGpuLimits(config, limits);
#define FIELD(name) EXPECT_EQ(mapped.native.name, limits.crossing.name)
  FIELD(max_paths); FIELD(max_input_pairs); FIELD(max_results);
  FIELD(max_work_per_pair); FIELD(max_total_work); FIELD(max_depth);
  FIELD(worker_count); FIELD(max_host_bytes);
#undef FIELD
  EXPECT_EQ(mapped.device_workers, 4096u);
  EXPECT_EQ(mapped.numeric_cohort_pairs, 4096u);
  EXPECT_EQ(mapped.max_device_bytes, limits.max_device_bytes);
  EXPECT_EQ(mapped.max_workspace_host_bytes, limits.max_host_bytes);
}
TEST(CrossingExecutorValues, ForecastCountsOneFallbackPoolAndOnlySubtractsEmbeddedHandle) {
  c::SelfContactTransactionConfig config;
  c::SelfContactTransactionLimits limits;
  config.enable_cuda_native_crossing = true;
  config.native_crossing_numeric_cohort_pairs = 4096;
  const auto raw = c::RepresentedIntervalCrossingGpu::Preflight(sct::CrossingGpuLimits(config, limits));
  ASSERT_EQ(raw.report.native.status, c::RepresentedIntervalStatus::Ok);
  const auto actual = sct::CrossingExecutor::Preflight(config, limits);
  ASSERT_EQ(actual.report.status, c::RepresentedIntervalStatus::Ok);
  EXPECT_EQ(actual.owned_host_bytes + sizeof(c::RepresentedIntervalCrossingGpu), raw.forecast.owned_host_bytes);
  EXPECT_EQ(actual.startup_host_bytes + sizeof(c::RepresentedIntervalCrossingGpu), raw.forecast.startup_host_bytes);
  EXPECT_EQ(actual.native.owned_host_bytes, raw.forecast.native.owned_host_bytes);
  EXPECT_EQ(actual.device_bytes, raw.forecast.device_bytes);
  EXPECT_EQ(actual.device_allocations, 1u);
  EXPECT_EQ(actual.device_status, c::RepresentedIntervalDeviceStatus::Ok);
}
TEST(CrossingExecutorValues, ExactWorkspaceCapsAreAdmittedAndOneByteShortIsRejected) {
  c::SelfContactTransactionConfig config;
  c::SelfContactTransactionLimits limits;
  config.enable_cuda_native_crossing = true;
  const auto raw = c::RepresentedIntervalCrossingGpu::Preflight(sct::CrossingGpuLimits(config, limits));
  ASSERT_EQ(raw.report.native.status, c::RepresentedIntervalStatus::Ok);
  limits.max_host_bytes = raw.forecast.startup_host_bytes - raw.forecast.native.startup_host_bytes;
  limits.max_device_bytes = raw.forecast.device_bytes;
  ASSERT_EQ(sct::CrossingExecutor::Preflight(config, limits).report.status, c::RepresentedIntervalStatus::Ok);
  auto short_cap = limits;
  --short_cap.max_host_bytes;
  EXPECT_EQ(sct::CrossingExecutor::Preflight(config, short_cap).report.status, c::RepresentedIntervalStatus::ResourceLimit);
  short_cap = limits;
  --short_cap.max_device_bytes;
  EXPECT_EQ(sct::CrossingExecutor::Preflight(config, short_cap).report.status, c::RepresentedIntervalStatus::ResourceLimit);
  config.native_crossing_numeric_cohort_pairs = 4097;
  EXPECT_EQ(sct::CrossingExecutor::Preflight(config, limits).report.status, c::RepresentedIntervalStatus::InvalidInput);
}
TEST(CrossingExecutorValues, DeviceFailureScopeDoesNotOverwriteNativeFailureIdentity) {
  c::SelfContactTransactionReport actual;
  actual.status = c::SelfContactTransactionStatus::CrossingFailure;
  actual.crossing_status = c::RepresentedIntervalStatus::ResourceLimit;
  actual.pair = 11;
  actual.message = "native work limit";
  c::RepresentedIntervalDeviceReport device;
  device.fault_cohort_begin = 4096;
  device.fault_cohort_count = 17;
  device.fault_pair_ordinal = 4102;
  for (auto status : {c::RepresentedIntervalDeviceStatus::NotInvoked, c::RepresentedIntervalDeviceStatus::Ok}) {
    device.status = status;
    sct::DescribeCrossingDeviceFailure(device, actual);
    EXPECT_EQ(actual.crossing_device_status, c::RepresentedIntervalDeviceStatus::NotInvoked);
    EXPECT_EQ(actual.crossing_fault_cohort_begin, SIZE_MAX);
  }
  device.status = c::RepresentedIntervalDeviceStatus::DeviceFailure;
  sct::DescribeCrossingDeviceFailure(device, actual);
  EXPECT_EQ(actual.crossing_device_status, device.status);
  EXPECT_EQ(actual.crossing_fault_cohort_begin, 4096u);
  EXPECT_EQ(actual.crossing_fault_cohort_count, 17u);
  EXPECT_EQ(actual.crossing_fault_pair_ordinal, 4102u);
  EXPECT_EQ(actual.crossing_status, c::RepresentedIntervalStatus::ResourceLimit);
  EXPECT_EQ(actual.pair, 11u);
  EXPECT_STREQ(actual.message, "native work limit");
}
}  // namespace
