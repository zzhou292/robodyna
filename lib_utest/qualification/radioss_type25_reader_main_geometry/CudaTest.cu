// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <stdexcept>
namespace reader_main_geometry_test {
namespace {
struct Record {
  n::CoefficientStatus status=n::CoefficientStatus::InvalidInput;
  n::NativeInternalMainGeometryResult value;
  n::CoefficientStatus volume_status=n::CoefficientStatus::InvalidInput;
  double volume=71.;
};
__global__ void Kernel(const n::NativeExteriorMainGeometryInput* input,Record* output,std::size_t count,bool reverse) {
  for(std::size_t t=blockIdx.x*blockDim.x+threadIdx.x;t<count;t+=blockDim.x*gridDim.x) {
    const auto i=reverse?count-1-t:t;auto next=output[i];
    next.status=n::EvaluateNativeInternalMainGeometry(input[i],&next.value);
    next.volume_status=n::EvaluateNativeEightSlotReaderVolume(input[i].solid_raw,&next.volume);
    output[i]=next;
  }
}
void Check(cudaError_t status) {if(status!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(status));}
class ReaderGeometryCuda:public type25_friction_test::PacketCuda<> {
 protected:
  std::vector<Record> Run(const std::vector<n::NativeExteriorMainGeometryInput>& values,unsigned width,bool reverse) {
    static_assert(sizeof(n::NativeExteriorMainGeometryInput)<=RowBytes&&sizeof(Record)<=RowBytes);
    if(values.empty()||values.size()>Capacity)throw std::invalid_argument("Reader geometry fixture capacity");
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
TEST_F(ReaderGeometryCuda, CompleteNativeInternalAndSecondVolumeBitsAcrossOrders) {
  auto packets=Cases();
  for(unsigned width:{1u,7u,32u,128u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(width);
    SCOPED_TRACE(reverse);
    std::reverse(packets.begin(),packets.end());const auto values=Run(packets,width,reverse);
    for(std::size_t i=0;i<packets.size();++i) {
      SCOPED_TRACE(i);
      ASSERT_EQ(values[i].status,n::CoefficientStatus::Ok);
      ASSERT_EQ(values[i].volume_status,n::CoefficientStatus::Ok);
      Same(values[i].value,InternalOracle(packets[i]).value);
      Exact(values[i].volume,VolumeOracle(packets[i].solid_raw));
    }
  }
}
TEST_F(ReaderGeometryCuda, RejectedConsumedInputsPreserveOutputsThenEarlyReturnAndRetry) {
  const auto packets=Invalid();
  for(unsigned width:{1u,7u,128u}) {
    SCOPED_TRACE(width);
    const auto values=Run(packets,width,true);
    for(unsigned i=0;i<packets.size();++i) {
      EXPECT_EQ(values[i].status,main_geometry_test::InvalidStatus(i));Same(values[i].value,Sentinel());
      if(i==3||i==5) {EXPECT_NE(values[i].volume_status,n::CoefficientStatus::Ok);Exact(values[i].volume,71.);}
    }
    auto early=Basic();for(auto& x:early.solid_raw)x={std::numeric_limits<double>::max(),0,0};
    const auto retry=Run({Basic(),Basic(true),early},width,false);
    for(unsigned i=0;i<3;++i) {
      ASSERT_EQ(retry[i].status,n::CoefficientStatus::Ok);
      Same(retry[i].value,InternalOracle(i==2?early:Basic(i==1)).value);
    }
  }
}
}
}
