#include "ObservationFixture.h"
#include <gtest/gtest.h>
#include "ObservationTransfer.h"
namespace crash::cases::vehicle_dynamics::motion_test {
TEST(VehicleMotionObservation, ActualOldCpuObserverAndGpuSummaryMatchAcrossGridStrideBoundaries) {
  int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
  for(const auto n:{std::size_t{1},std::size_t{129},std::size_t{131073},tl::fea::MaxActiveNodalStateNodes}) {
    SCOPED_TRACE(n);
    Fixture f(n);
    const auto allocation=f.owner.allocations();
    motion_transfer_probe::Begin();
    const auto old=f.Old();
    const auto old_transfer=motion_transfer_probe::End();
    motion_transfer_probe::Begin();
    const auto next=f.New();
    const auto new_transfer=motion_transfer_probe::End();
    EXPECT_EQ(old_transfer.host_bytes,19*n*sizeof(double));
    EXPECT_EQ(old_transfer.host_reads,1u);
    EXPECT_EQ(new_transfer.host_reads,1u);
    EXPECT_LE(new_transfer.host_bytes,128u);
    EXPECT_EQ(old_transfer.device_allocations,0u);
    EXPECT_EQ(new_transfer.device_allocations,0u);
    EXPECT_TRUE(Same(old,next));
    if(n>1)EXPECT_GT(next.maximum_velocity_error,0);
    if(n>1)EXPECT_GT(next.maximum_orientation_error,0);
    EXPECT_TRUE(Same(f.New(),next));
    EXPECT_EQ(f.owner.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(f.owner.accepted().epoch,0);
  }
}
}
