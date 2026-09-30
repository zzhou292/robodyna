// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "lib_src/elements/type13/resident/Storage.h"
#include "lib_src/elements/type25/Type25BatchStorage.h"
#include "lib_src/elements/mapped_connector/Kernels.cuh"
#include "Serial13.cuh"
#include "Serial25.cuh"
namespace connector_test {
template<class Packet> class Device {
 public:
  Device() { EXPECT_EQ(cudaMallocManaged(&data,sizeof(Packet)),cudaSuccess);if(data)new(data)Packet{}; }
  ~Device() { if(data)cudaFree(data); }
  Packet* data=nullptr;
};
void Assemble(Packet13& p,bool serial,bool initial=false) {
  auto view=p.input.View();
  if(serial)a::serial_reference::LaunchAssembly(&p.storage,0,view,p.input.Cin(),initial,true);
  else a::LaunchAssembly(&p.storage,0,view,p.input.Cin(),initial,true);
}
void Assemble(Packet25& p,bool serial,bool initial=false) {
  auto view=p.input.View();
  if(serial)b::serial_reference::LaunchMappedAssembly(&p.storage,&p.storage.slab[0],view,p.input.Cin(),initial);
  else b::LaunchMappedAssembly(&p.storage,&p.storage.slab[0],view,p.input.Cin(),initial);
}
template<class P> void SameControl(const P& actual,const P& expected) {
  const auto& a=actual.storage.control;const auto& b=expected.storage.control;
  EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.element_status,b.element_status);
  EXPECT_EQ(a.element,b.element);EXPECT_EQ(a.node,b.node);
  EXPECT_EQ(actual.input.result.status,expected.input.result.status);
  EXPECT_EQ(actual.input.result.node,expected.input.result.node);
}
template<class P> void SuccessCases() {
  Device<P> actual,reference;ASSERT_NE(actual.data,nullptr);ASSERT_NE(reference.data,nullptr);
  for(bool initial:{false,true}) for(unsigned mask=0;mask<32;++mask) {
    Prepare(*actual.data);Prepare(*reference.data);
    for(unsigned p=0;p<Parents;++p)if(mask&(1u<<p)){Remove(*actual.data,p);Remove(*reference.data,p);}
    const auto saved=actual.data->values[2];
    Assemble(*actual.data,false,initial);Assemble(*reference.data,true,initial);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    EXPECT_EQ(actual.data->storage.control.status,decltype(actual.data->storage.control.status)::Success);
    SameControl(*actual.data,*reference.data);CompareValues(actual.data->input,reference.data->input);
    EXPECT_EQ(std::memcmp(&saved,&actual.data->values[2],sizeof(saved)),0);
  }
}
template<class P> void FaultCases() {
  Device<P> actual,reference;ASSERT_NE(actual.data,nullptr);ASSERT_NE(reference.data,nullptr);
  for(unsigned fault=0;fault<12;++fault) {
    Prepare(*actual.data);Prepare(*reference.data);
    for(auto* p:{actual.data,reference.data}) {
      if(fault==0) { BadCoefficient(*p,0);p->input.orientation[4*4]=0; }
      if(fault==1) p->values[3].endpoints[0].force_N.x=std::numeric_limits<double>::quiet_NaN();
      if(fault==2) { p->values[0].endpoints[0].force_N.x=std::numeric_limits<double>::max();p->input.value[0][0]=std::numeric_limits<double>::max();p->input.fixed[4]=1; }
      if(fault==3) p->input.value[6][5]=std::numeric_limits<double>::max(),p->input.value[6][0]=std::numeric_limits<double>::quiet_NaN();
      if(fault==4) p->input.inverse[3]=-1;
      if(fault==5) p->input.rotation[4]=0;
      if(fault==6) p->input.bounds.sealed=true;
      if(fault==7) p->input.result.attempt=2;
      if(fault==8) p->input.orientation[4*0]=p->input.orientation[4*3]=0;
      if(fault==9) { LargeCoefficient(*p,0);p->input.value[6][0]=std::numeric_limits<double>::max(); }
      if(fault==10) p->elements[3].nodes[1]=p->elements[3].nodes[0];
      if(fault==11) p->input.position[3*4]=.1;
    }
    const auto before=actual.data->input;
    Assemble(*actual.data,false,fault==11);Assemble(*reference.data,true,fault==11);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    EXPECT_NE(actual.data->storage.control.status,decltype(actual.data->storage.control.status)::Success);
    SameControl(*actual.data,*reference.data);CompareValues(actual.data->input,before);
    Prepare(*actual.data);Prepare(*reference.data);
    actual.data->input.value[2][0]=reference.data->input.value[2][0]=.375;
    Assemble(*actual.data,false);Assemble(*reference.data,true);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);SameControl(*actual.data,*reference.data);
    CompareValues(actual.data->input,reference.data->input);
  }
}
TEST(ConnectorMappedAssemblyCuda,Type13AllChannelsAndRemovalMasksMatchFrozenSerialBits) { SuccessCases<Packet13>(); }
TEST(ConnectorMappedAssemblyCuda,Type25AllChannelsAndRemovalMasksMatchFrozenSerialBits) { SuccessCases<Packet25>(); }
TEST(ConnectorMappedAssemblyCuda,Type13FirstFailureRollbackAndRetryMatchFrozenSerial) { FaultCases<Packet13>(); }
TEST(ConnectorMappedAssemblyCuda,Type25FirstFailureRollbackAndRetryMatchFrozenSerial) { FaultCases<Packet25>(); }
TEST(ConnectorMappedAssemblyCuda,GridSchedulesPreserveEveryDestinationBit) {
  Device<Packet13> actual,reference;
  ASSERT_NE(actual.data,nullptr);ASSERT_NE(reference.data,nullptr);
  cudaStream_t stream;ASSERT_EQ(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking),cudaSuccess);
  for(unsigned threads:{32,64,128,256}) for(unsigned blocks:{1,3,17}) {
    Prepare(*actual.data);Prepare(*reference.data);
    actual.data->input.stream=stream;
    auto& p=*actual.data;const auto view=p.input.View();const auto cin=p.input.Cin();
    mc::Begin<a::AssemblyFamily><<<1,1,0,stream>>>(&p.storage,view);
    mc::PrepareParents<a::AssemblyFamily><<<blocks,threads,0,stream>>>(&p.storage,0,view,cin,false);
    mc::GatherNodes<a::AssemblyFamily><<<blocks,threads,0,stream>>>(&p.storage,0,view,cin);
    mc::Finish<a::AssemblyFamily><<<1,1,0,stream>>>(&p.storage,view);
    mc::Publish<a::AssemblyFamily><<<blocks,threads,0,stream>>>(&p.storage,view,cin);
    Assemble(*reference.data,true);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    SameControl(p,*reference.data);CompareValues(p.input,reference.data->input);
  }
  EXPECT_EQ(cudaStreamDestroy(stream),cudaSuccess);
}
} // namespace connector_test
