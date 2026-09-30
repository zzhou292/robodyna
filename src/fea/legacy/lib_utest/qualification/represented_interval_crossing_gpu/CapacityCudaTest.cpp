// SPDX-License-Identifier: AGPL-3.0-or-later
#include "GpuFixture.h"
namespace native_gpu_test {
TEST(NativeGpuCapacityCuda, WidePoolsAnd4097JobsPreserveEveryFieldAndWorkCount) {
  Streams streams;
  Cases cases;
  for (unsigned i = 0; i < 4097; ++i) {
    if (i % 3 == 0) Add(cases, Positive(), Positive(), Positive());
    else if (i % 3 == 1) Add(cases, Positive(), Positive(3), Positive(3));
    else Add(cases, Positive(), Positive(3), Positive(1));
  }
  ASSERT_EQ(DevicePairs(cases, Limits().native.max_depth), cases.pairs.size());
  for (unsigned workers : {512u, 2048u, 4096u}) {
    SCOPED_TRACE(workers);
    auto limits = Limits();
    limits.native.max_paths = cases.paths.size();
    limits.native.max_input_pairs = limits.native.max_results = cases.pairs.size();
    limits.device_workers = workers;
    // Keep original work/depth budgets; every represented fixture needs one
    // visit. Only independent capacity and scheduling width change here.
    auto cpu = fixture::Owner(limits.native);
    c::RepresentedIntervalCrossingGpu gpu;
    ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
    for (unsigned repeat = 0; repeat < 2; ++repeat) {
      const auto result = Compare(cpu, gpu, cases, streams.first);
      ASSERT_EQ(result.native.status, S::Ok);
      EXPECT_EQ(result.native.work, cases.pairs.size());
      EXPECT_EQ(result.device.device_pairs, cases.pairs.size());
      EXPECT_EQ(result.device.host_pairs, 0u);
      EXPECT_EQ(result.device.batches, 1u);
      EXPECT_EQ(result.device.scene_uploads, 1u);
      std::reverse(cases.pairs.begin(), cases.pairs.end());
      for (auto& pair : cases.pairs) std::swap(pair.first, pair.second);
    }
  }
}
}  // namespace native_gpu_test
