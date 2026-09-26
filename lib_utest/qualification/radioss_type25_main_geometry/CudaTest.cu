// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <stdexcept>
namespace main_geometry_test {
namespace {
struct Record { n::CoefficientStatus status=n::CoefficientStatus::InvalidInput;n::NativeExteriorMainGeometryResult value; };
__global__ void Kernel(const n::NativeExteriorMainGeometryInput* input,Record* output,std::size_t count,bool reverse) {
  for(std::size_t t=blockIdx.x*blockDim.x+threadIdx.x;t<count;t+=blockDim.x*gridDim.x) {
    const auto i=reverse?count-1-t:t;
    auto next=output[i];next.status=n::EvaluateNativeExteriorMainGeometry(input[i],&next.value);output[i]=next;
  }
}
void Check(cudaError_t status) { if(status!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(status)); }
class MainGeometryCuda:public type25_friction_test::PacketCuda<> {
 protected:
  std::vector<Record> Run(const std::vector<n::NativeExteriorMainGeometryInput>& values,unsigned width,bool reverse) {
    static_assert(sizeof(n::NativeExteriorMainGeometryInput)<=RowBytes&&sizeof(Record)<=RowBytes);
    if(values.empty()||values.size()>Capacity)throw std::invalid_argument("Geometry fixture capacity");
    std::vector<Record> result(values.size());for(auto& row:result)row.value=Sentinel();
    std::vector<n::NativeExteriorMainGeometryInput> unchanged(values.size());
    type25_friction_test::Drain drain{stream};
    Check(cudaMemcpyAsync(input,values.data(),values.size()*sizeof(values[0]),cudaMemcpyHostToDevice,stream));
    Check(cudaMemcpyAsync(output,result.data(),result.size()*sizeof(Record),cudaMemcpyHostToDevice,stream));
    Kernel<<<3,width,0,stream>>>(static_cast<const n::NativeExteriorMainGeometryInput*>(input),
      static_cast<Record*>(output),values.size(),reverse);
    Check(cudaGetLastError());
    Check(cudaMemcpyAsync(result.data(),output,result.size()*sizeof(Record),cudaMemcpyDeviceToHost,stream));
    Check(cudaMemcpyAsync(unchanged.data(),input,values.size()*sizeof(values[0]),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));
    EXPECT_EQ(std::memcmp(values.data(),unchanged.data(),values.size()*sizeof(values[0])),0);
    return result;
  }
};
TEST_F(MainGeometryCuda, NativeFullBitsAndPrimaryPermutationsAcrossLaunchOrders) {
  auto packets=Cases();
  for(unsigned width:{1u,7u,32u,128u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(width);
    SCOPED_TRACE(reverse);
    std::reverse(packets.begin(),packets.end());
    const auto values=Run(packets,width,reverse);
    for(std::size_t i=0;i<packets.size();++i) {
      SCOPED_TRACE(i);
      ASSERT_EQ(values[i].status,n::CoefficientStatus::Ok);
      n::NativeExteriorMainGeometryResult host;
      ASSERT_EQ(n::EvaluateNativeExteriorMainGeometry(packets[i],&host),n::CoefficientStatus::Ok);
      Same(values[i].value,host);Same(values[i].value,Oracle(packets[i]));
    }
  }
}
TEST_F(MainGeometryCuda, InvalidAndOverflowPacketsRetainAllOutputsThenRetry) {
  const auto packets=Invalid();
  for(unsigned width:{1u,7u,128u}) {
    SCOPED_TRACE(width);
    const auto values=Run(packets,width,true);
    for(unsigned i=0;i<packets.size();++i) {
      EXPECT_EQ(values[i].status,InvalidStatus(i));Same(values[i].value,Sentinel());
    }
    const auto retry=Run({Basic(),Basic(true)},width,false);
    for(unsigned i=0;i<2;++i) {
      ASSERT_EQ(retry[i].status,n::CoefficientStatus::Ok);
      Same(retry[i].value,Oracle(Basic(i!=0)));
    }
  }
}
} // namespace
} // namespace main_geometry_test
