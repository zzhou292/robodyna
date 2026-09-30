// SPDX-License-Identifier: MIT
#include "BoundsFixture.h"
namespace tied_search_test {
struct BoundsPacket {
  ts::NativeSearchBounds bounds;
  ts::Status status=ts::Status::Success;
  bool within=false;
};
__global__ void DeviceBounds(ts::SearchBoundsInput in,ts::Vec3 point,BoundsPacket* out) {
  auto staged=*out;
  staged.status=ts::PrepareSearchBounds(in,staged.bounds);
  if(staged.status==ts::Status::Success)
    staged.status=ts::WithinSearchBounds(staged.bounds,point,staged.within);
  if(staged.status==ts::Status::Success) *out=staged;
  else out->status=staged.status;
}
TEST(TiedSearchBoundsCuda, NativeBoundsAndRejectedAttemptRetryOnDevice) {
  BoundsPacket* device=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(*device)),cudaSuccess);
  for(unsigned shape=0;shape<3;++shape) for(double maximum:{0.,.03}) {
    const auto input=BoundsInput(Shape(shape),maximum);
    for(const auto point:{ts::Vec3{0,0,0},ts::Vec3{0,0,.2}}) {
      BoundsPacket empty,actual;
      ASSERT_EQ(cudaMemcpy(device,&empty,sizeof(empty),cudaMemcpyHostToDevice),cudaSuccess);
      DeviceBounds<<<1,1>>>(input,point,device);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(actual.status,ts::Status::Success);
      const auto native=NativeBounds(input,point);
      const auto values=BoundValues(actual.bounds);
      for(unsigned i=0;i<values.size();++i) EXPECT_DOUBLE_EQ(values[i],native[i]);
      EXPECT_EQ(actual.within,native[7]==1);
      const auto before=actual;
      auto bad=input;bad.master_position_m[3].z=std::numeric_limits<double>::quiet_NaN();
      DeviceBounds<<<1,1>>>(bad,point,device);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
      EXPECT_EQ(actual.status,ts::Status::InvalidInput);
      EXPECT_EQ(BoundValues(actual.bounds),BoundValues(before.bounds));
      EXPECT_EQ(actual.within,before.within);
      DeviceBounds<<<1,1>>>(input,point,device);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
      EXPECT_EQ(actual.status,ts::Status::Success);
      EXPECT_EQ(BoundValues(actual.bounds),BoundValues(before.bounds));
      EXPECT_EQ(actual.within,before.within);
      ASSERT_FALSE(HasFailure());
    }
  }
  EXPECT_EQ(cudaFree(device),cudaSuccess);
}
} // namespace tied_search_test
