// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <algorithm>
#include <stdexcept>
namespace type25_contribution_test {
namespace {
TL_MATH_HOST_DEVICE n::CoefficientStatus Leaf(const n::NativeSolidNodalInput& in,n::NativeSolidNodalShares* out) {
  return n::EvaluateNativeSolidNodalShares(in,out);
}
TL_MATH_HOST_DEVICE n::CoefficientStatus Leaf(const n::NativeSpringNodalInput& in,n::NativeScalarCoefficient* out) {
  return n::EvaluateNativeSpringNodalCoefficient(in,out);
}
template<class V> struct Record {n::CoefficientStatus status=n::CoefficientStatus::InvalidInput;V value;};
template<class P,class V> __global__ void Kernel(const P* input,Record<V>* output,std::size_t count,bool reverse) {
  for(std::size_t thread=blockIdx.x*blockDim.x+threadIdx.x;thread<count;thread+=blockDim.x*gridDim.x) {
    const auto i=reverse?count-1-thread:thread;
    auto next=output[i];next.status=Leaf(input[i],&next.value);output[i]=next;
  }
}
void Check(cudaError_t status){if(status!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(status));}
class ContributionCuda:public type25_friction_test::PacketCuda<> {
 protected:
  template<class P,class V> std::vector<Record<V>> RunPackets(const std::vector<P>& packets,V seed,unsigned threads,bool reverse) {
    static_assert(sizeof(P)<=RowBytes&&sizeof(Record<V>)<=RowBytes);
    if(packets.empty()||packets.size()>Capacity)throw std::invalid_argument("Contribution CUDA fixture capacity");
    std::vector<Record<V>> result(packets.size());for(auto& row:result)row.value=seed;
    std::vector<P> unchanged(packets.size());
    type25_friction_test::Drain drain{stream};
    Check(cudaMemcpyAsync(input,packets.data(),packets.size()*sizeof(P),cudaMemcpyHostToDevice,stream));
    Check(cudaMemcpyAsync(output,result.data(),result.size()*sizeof(Record<V>),cudaMemcpyHostToDevice,stream));
    Kernel<<<3,threads,0,stream>>>(static_cast<const P*>(input),static_cast<Record<V>*>(output),packets.size(),reverse);
    Check(cudaGetLastError());
    Check(cudaMemcpyAsync(result.data(),output,result.size()*sizeof(Record<V>),cudaMemcpyDeviceToHost,stream));
    Check(cudaMemcpyAsync(unchanged.data(),input,unchanged.size()*sizeof(P),cudaMemcpyDeviceToHost,stream));
    Check(cudaStreamSynchronize(stream));
    EXPECT_EQ(std::memcmp(packets.data(),unchanged.data(),packets.size()*sizeof(P)),0);
    return result;
  }
};
TEST_F(ContributionCuda, SolidRawMaskAndAllDefinedNativeBitsMatchEveryLaunch) {
  auto packets=SolidCases();
  for(unsigned width:{1u,7u,32u,128u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(width);
    SCOPED_TRACE(reverse);
    std::reverse(packets.begin(),packets.end());
    const auto values=RunPackets(packets,n::NativeSolidNodalShares{73.,91.,0x35},width,reverse);
    for(std::size_t i=0;i<packets.size();++i) {
      SCOPED_TRACE(i);
      ASSERT_EQ(values[i].status,n::CoefficientStatus::Ok);
      n::NativeSolidNodalShares host;
      ASSERT_EQ(n::EvaluateNativeSolidNodalShares(packets[i],&host),n::CoefficientStatus::Ok);
      Same(values[i].value,host);CompareSolid(packets[i],values[i].value);
    }
  }
}
TEST_F(ContributionCuda, SpringNativeMaxZeroTiesFloorsAndUnreadChannelsMatchEveryLaunch) {
  auto packets=SpringCases();
  for(unsigned width:{1u,7u,32u,128u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(width);
    SCOPED_TRACE(reverse);
    std::reverse(packets.begin(),packets.end());
    const auto values=RunPackets(packets,n::NativeScalarCoefficient{73.},width,reverse);
    for(std::size_t i=0;i<packets.size();++i) {
      SCOPED_TRACE(i);
      ASSERT_EQ(values[i].status,n::CoefficientStatus::Ok);
      n::NativeScalarCoefficient host;
      ASSERT_EQ(n::EvaluateNativeSpringNodalCoefficient(packets[i],&host),n::CoefficientStatus::Ok);
      Exact(values[i].value.value,host.value);Exact(values[i].value.value,ReferenceSpring(packets[i]));
    }
  }
}
TEST_F(ContributionCuda, RejectedNumericsAndProfilesPreserveOutputsAndRetry) {
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  std::vector<n::NativeSolidNodalInput> solids{{n::SolidNodalKind::Unspecified,nan,nan,nan},
    {n::SolidNodalKind::Hex8,nan,1,5},{n::SolidNodalKind::Hex8,0,std::numeric_limits<double>::max(),2}};
  const n::NativeSolidNodalShares seed{71.,-19.,0x53};
  const auto failed=RunPackets(solids,seed,7,true);
  const n::CoefficientStatus statuses[]{n::CoefficientStatus::UnsupportedProfile,n::CoefficientStatus::InvalidInput,n::CoefficientStatus::NonfiniteResult};
  for(unsigned i=0;i<3;++i){EXPECT_EQ(failed[i].status,statuses[i]);Same(failed[i].value,seed);}
  std::vector<n::NativeSpringNodalInput> springs(3,Spring());
  springs[0].interface_initialization=0;for(auto& x:springs[0].translation)x={nan,nan};springs[0].geometric_length=nan;
  springs[1].translation[0].slope=nan;
  springs[2].translation[0]={std::numeric_limits<double>::max(),2};
  const auto spring_failed=RunPackets(springs,n::NativeScalarCoefficient{71.},128,false);
  for(unsigned i=0;i<3;++i){EXPECT_EQ(spring_failed[i].status,statuses[i]);Exact(spring_failed[i].value.value,71.);}
  const auto retry=RunPackets(std::vector<n::NativeSpringNodalInput>{Spring()},n::NativeScalarCoefficient{71.},1,false);
  ASSERT_EQ(retry[0].status,n::CoefficientStatus::Ok);
  Exact(retry[0].value.value,ReferenceSpring(Spring()));
}
}
} // namespace type25_contribution_test
