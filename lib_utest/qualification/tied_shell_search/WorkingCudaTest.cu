// SPDX-License-Identifier: MIT
#include "WorkingFixture.h"
namespace tied_search_test {
struct WorkingPacket {
  ts::CandidateProjection projection;
  ts::NativeSearchBounds bounds;
  ts::SearchChoice choice;
  ts::Status status=ts::Status::Success;
  bool within=false;
};
__global__ void WorkingDevice(ts::WorkingSearchInput input,ts::WorkingSearchBoundsInput bounds,WorkingPacket* out) {
  auto next=*out;
  next.status=ts::ProjectCandidate(input,next.projection);
  if(next.status==ts::Status::Success) next.status=ts::PrepareSearchBounds(bounds,next.bounds);
  if(next.status==ts::Status::Success)
    next.status=ts::WithinWorkingSearchBounds(next.bounds,input.geometry.secondary_position,next.within);
  if(next.status==ts::Status::Success) next.status=ts::ConsiderCandidate(input,7,next.choice);
  if(next.status==ts::Status::Success) *out=next;
  else out->status=next.status;
}
TEST(TiedSearchWorkingCuda, OriginalWorkingPacketMatchesNativeAndRetriesAfterLateInvalidGeometry) {
  WorkingPacket* device=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(*device)),cudaSuccess);
  for(unsigned shape=0;shape<3;++shape) {
    const auto input=OriginalWorkingShape(shape);
    WorkingPacket initial,actual;
    ASSERT_EQ(cudaMemcpy(device,&initial,sizeof(initial),cudaMemcpyHostToDevice),cudaSuccess);
    WorkingDevice<<<1,1>>>(input,WorkingBounds(input),device);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(actual.status,ts::Status::Success);
    CompareWorking(input,actual.projection,actual.bounds,actual.within);
    NativeChoice native;Native(input,7,native);
    EXPECT_EQ(actual.choice.matched,native.selected!=0);
    if(actual.choice.matched) EXPECT_EQ(actual.choice.ordered_master,native.selected);
    const auto before=actual;
    auto bad=WorkingBounds(input);bad.master_position[3].z=std::numeric_limits<double>::quiet_NaN();
    WorkingDevice<<<1,1>>>(input,bad,device);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_EQ(actual.status,ts::Status::InvalidInput);
    Compare(actual.projection,before.projection);
    EXPECT_EQ(BoundValues(actual.bounds),BoundValues(before.bounds));
    EXPECT_EQ(actual.within,before.within);
    WorkingDevice<<<1,1>>>(input,WorkingBounds(input),device);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_EQ(actual.status,ts::Status::Success);
    CompareWorking(input,actual.projection,actual.bounds,actual.within);
    ASSERT_FALSE(HasFailure());
  }
  EXPECT_EQ(cudaFree(device),cudaSuccess);
}
} // namespace tied_search_test
