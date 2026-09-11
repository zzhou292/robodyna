#include "Fixture.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <cstring>
namespace qbat_test {
struct DevicePacket {
  qb::Reference reference;
  qb::Geometry geometry;
  qb::Status status;
};
__global__ void Startup(qb::ReferenceInput input,DevicePacket* packet) {
  packet->status=qb::InitializeReference(input,packet->reference);
}
__global__ void CurrentKernel(qb::CurrentInput input,DevicePacket* packet) {
  packet->status=qb::EvaluateGeometry(packet->reference,input,packet->geometry);
}
class QbatDevice : public ::testing::Test {
 protected:
  DevicePacket* device=nullptr;
  void SetUp() override {
    int count=0;
    ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
    ASSERT_GT(count,0);
    ASSERT_EQ(cudaMalloc(&device,sizeof(DevicePacket)),cudaSuccess);
    const DevicePacket initial{};
    ASSERT_EQ(cudaMemcpy(device,&initial,sizeof(initial),cudaMemcpyHostToDevice),cudaSuccess);
  }
  void TearDown() override { if(device) EXPECT_EQ(cudaFree(device),cudaSuccess); }
  DevicePacket Copy() {
    DevicePacket result;
    EXPECT_EQ(cudaMemcpy(&result,device,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
    return result;
  }
};
template<std::size_t N>
void Agreement(const std::array<double,N>& a,const std::array<double,N>& b) {
  for(std::size_t i=0;i<N;++i) {
    EXPECT_NEAR(a[i],b[i],3e-12*std::max(std::abs(a[i]),std::abs(b[i]))+1e-24) << i;
  }
}
TEST_F(QbatDevice, StartupAnd96PrescribedCurrentPackets) {
  const auto input=Fixture();
  qb::Reference host;
  ASSERT_EQ(qb::InitializeReference(input,host),qb::Status::kSuccess);
  Startup<<<1,1>>>(input,device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  auto actual=Copy();
  ASSERT_EQ(actual.status,qb::Status::kSuccess);
  Agreement(Values(actual.reference),Values(host));
  for(unsigned step=0;step<96;++step) {
    SCOPED_TRACE(step);
    auto current=Current(input);
    for(auto& x:current.position_m) {
      x.y+=.02*std::sin(.1*step)*x.x;
      x=Transform(x,.019*step,.0002*step);
    }
    qb::Geometry expected;
    ASSERT_EQ(qb::EvaluateGeometry(host,current,expected),qb::Status::kSuccess);
    CurrentKernel<<<1,1>>>(current,device);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    actual=Copy();
    ASSERT_EQ(actual.status,qb::Status::kSuccess);
    Agreement(Values(actual.geometry),Values(expected));
  }
}
TEST_F(QbatDevice, StartupFailureGeometryFailureAndExactDeviceRetry) {
  auto input=Fixture();
  Startup<<<1,1>>>(input,device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(Copy().status,qb::Status::kSuccess);
  auto current=Current(input);
  CurrentKernel<<<1,1>>>(current,device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  const auto before=Copy();
  ASSERT_EQ(before.status,qb::Status::kSuccess);
  input.initial_a11_pa=1.7976931348623157e308;
  input.quadrilateral.thickness=1;
  Startup<<<1,1>>>(input,device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  auto after=Copy();
  ASSERT_EQ(after.status,qb::Status::kNonfiniteResult);
  EXPECT_EQ(Values(after.reference),Values(before.reference));
  current.position_m[3]=current.position_m[1];
  CurrentKernel<<<1,1>>>(current,device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  after=Copy();
  ASSERT_NE(after.status,qb::Status::kSuccess);
  EXPECT_EQ(Values(after.geometry),Values(before.geometry));
  current=Current(Fixture());
  CurrentKernel<<<1,1>>>(current,device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  after=Copy();
  ASSERT_EQ(after.status,qb::Status::kSuccess);
  EXPECT_EQ(Values(after.geometry),Values(before.geometry));
}
} // namespace qbat_test
