#include "lib_src/solvers/NodalUniformMotionObserver.h"
#include <gtest/gtest.h>
namespace tl::fea {
TEST(NodalUniformMotionCapacity, CompleteLimitsAreDescriptorOnlyAndExact) {
  for(const auto n:{std::size_t{1},std::size_t{129},MaxActiveNodalStateNodes}) {
    const auto f=NodalUniformMotionObserver::Preflight(n);
    ASSERT_EQ(f.report.status,NodalStatus::Ok);
    EXPECT_GE(f.device_bytes,3*n*sizeof(double));
    EXPECT_GT(f.blocks,0);
    NodalUniformMotionLimits limits;limits.max_host_bytes=f.host_bytes;limits.max_device_bytes=f.device_bytes;
    EXPECT_EQ(NodalUniformMotionObserver::Preflight(n,limits).report.status,NodalStatus::Ok);
    --limits.max_device_bytes;
    EXPECT_EQ(NodalUniformMotionObserver::Preflight(n,limits).report.status,NodalStatus::ResourceLimit);
    limits.max_device_bytes=f.device_bytes;--limits.max_host_bytes;
    EXPECT_EQ(NodalUniformMotionObserver::Preflight(n,limits).report.status,NodalStatus::ResourceLimit);
  }
  EXPECT_EQ(NodalUniformMotionObserver::Preflight(0).report.status,NodalStatus::ResourceLimit);
  EXPECT_EQ(NodalUniformMotionObserver::Preflight(SIZE_MAX).report.status,NodalStatus::ResourceLimit);
}
}
