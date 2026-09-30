#include "Tab1NativeSupport.h"
#include <cuda_runtime.h>
#include <limits>
namespace tab1_test {
namespace {
struct PointPacket {
  fail::Tab1ConstantTable table;
  fail::Tab1ConstantFailureHistory history;
  fail::Tab1ConstantFailureResult result;
  Status status=Status::InvalidParameters;
};
struct PointDevice { PointPacket* p=nullptr; ~PointDevice(){if(p)cudaFree(p);} };
__global__ void AdvanceFailurePoint(PointPacket* p,fail::Tab1ConstantFailureInput in) {
  p->status=fail::UpdateTab1ConstantFailure(p->table,p->history,in,p->result);
  if(p->status==Status::Ok)p->history=p->result.history;
}
}
TEST(Tab1Cuda, DeviceOwnedPointTinyIncrementUncappedDamageDisplayAndCacheMatchNative) {
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  PointDevice device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.p),sizeof(PointPacket)),cudaSuccess);
  PointPacket packet;
  packet.table=Table();
  auto native=NativeHistory(packet.history);
  ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
  for(unsigned step=0;step<8;++step) {
    fail::Tab1ConstantFailureInput in;
    in.plastic_strain_increment=step==0?std::nextafter(0.,1.):step<4?.001:.03;
    in.native_evaluation_time_s=(step+1)*1.e-7;
    const double magnitude=step==0?.5e-20:step==1?1.e-20:
        step==2?std::nextafter(1.e-20,1.):30e6;
    in.current_stress[0]=(step%2?1.:-1.)*magnitude;
    in.current_stress[1]=step==3?30e6:0.;
    double stress[5]{in.current_stress[0],in.current_stress[1],0,0,0};
    std::array<double,6> expected{};
    tab1_native_point(packet.table.triaxiality,packet.table.failure_strain,native.data(),1,
        in.plastic_strain_increment,in.native_evaluation_time_s,stress,expected.data());
    AdvanceFailurePoint<<<1,1>>>(device.p,in);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.status,Status::Ok);
    Close(packet.history.damage,expected[0],0.);
    EXPECT_EQ(packet.history.failure_time_s,expected[1]);
    EXPECT_EQ(packet.history.point_active,expected[2]==1.);
    Close(packet.history.maximum_damage,expected[3],0.);
    EXPECT_EQ(packet.history.table_segment,expected[4]);
    std::copy_n(expected.begin(),5,native.begin());
  }
  EXPECT_GT(packet.history.damage,1.); EXPECT_EQ(packet.history.maximum_damage,1.);
  const auto h=Bytes(packet.history);
  const auto r=Bytes(packet.result);
  fail::Tab1ConstantFailureInput bad;
  bad.native_evaluation_time_s=9.e-7;
  bad.current_stress[2]=std::numeric_limits<double>::quiet_NaN();
  AdvanceFailurePoint<<<1,1>>>(device.p,bad);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(packet.status,Status::InvalidIncrement);
  EXPECT_EQ(Bytes(packet.history),h); EXPECT_EQ(Bytes(packet.result),r);
}
} // namespace tab1_test
