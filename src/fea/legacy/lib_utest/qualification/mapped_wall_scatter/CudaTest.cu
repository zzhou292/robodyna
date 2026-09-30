#include "Fixture.h"
#include "FrozenScatter.h"
#include "lib_src/collision/nodal_wall_mapped/Scatter.cuh"
#include "lib_src/collision/NodalWallContactKernels.cuh"
namespace wall_scatter_test {
struct Device {
  Packet* packet=nullptr;
  Device() {EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&packet),sizeof(Packet)),cudaSuccess);}
  ~Device() {cudaFree(packet);}
};
__global__ void Serial(d::Storage* s,m::Sidecar side,fe::NodalAssemblyView view,fe::NodalCinAssemblyView cin) {
  if(s->control.status!=c::NodalWallDeviceStatus::Ok) return;
  if(wall_scatter_frozen::StageStiffness(*s,side,cin) && wall_scatter_frozen::Scatter(*s,view,false)) {
    wall_scatter_frozen::PublishScatter(*s,view);
    for(unsigned i=0;i<s->model.node_count;++i) cin.translational_stiffness[s->model.nodes[i].node]=side.stiffness[i];
  }
}
__global__ void Legacy(d::Storage* s,fe::NodalAssemblyView view,bool frozen) {
  if(frozen) wall_scatter_frozen::Scatter(*s,view);
  else d::Scatter(*s,view);
}
void Launch(Packet& p,bool serial) {
  if(serial) Serial<<<1,1>>>(&p.storage,p.Side(),p.View(),p.Cin());
  else m::parallel::Scatter(&p.storage,p.Side(),p.View(),p.Cin(),Nodes,nullptr);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
}
TEST(MappedWallScatterCuda, MultipleBlocksSixChannelsSignedZerosAndCancellationMatchFrozenBits) {
  Device a,b;
  ASSERT_NE(a.packet,nullptr);ASSERT_NE(b.packet,nullptr);
  a.packet->Reset();b.packet->Reset();
  const auto before=a.packet->base[Nodes-1].force.value;
  Launch(*a.packet,false);Launch(*b.packet,true);
  ASSERT_EQ(a.packet->storage.control.status,c::NodalWallDeviceStatus::Ok);
  Same(*a.packet,*b.packet);
  EXPECT_EQ(a.packet->base[Nodes-1].force.value,before);
  for(unsigned n=0;n<Globals;++n) {
    bool incident=false;
    for(unsigned i=0;i<Nodes;++i) incident=incident||a.packet->nodes[i].node==n;
    if(!incident) for(unsigned channel=0;channel<6;++channel)
      EXPECT_EQ(Bits(a.packet->forces[channel*Globals+n]),Bits(b.packet->forces[channel*Globals+n]));
  }
}
TEST(MappedWallScatterCuda, EarliestAndLateFailuresPreserveAllDestinationsThenRetry) {
  Device a,b;
  ASSERT_NE(a.packet,nullptr);ASSERT_NE(b.packet,nullptr);
  for(unsigned fault=0;fault<6;++fault) {
    SCOPED_TRACE(fault);
    a.packet->Reset();b.packet->Reset();
    Fault(*a.packet,fault);Fault(*b.packet,fault);
    const Destinations before(*a.packet);
    const auto base=a.packet->base[Nodes-1].force.value;
    Launch(*a.packet,false);Launch(*b.packet,true);
    ASSERT_NE(a.packet->storage.control.status,c::NodalWallDeviceStatus::Ok);
    Same(*a.packet,*b.packet);before.Unchanged(*a.packet);
    const auto compact=fault==1||fault==3?5:Nodes-1;
    EXPECT_EQ(a.packet->storage.control.node,fault==5?17:a.packet->nodes[compact].node);
    EXPECT_EQ(a.packet->base[Nodes-1].force.value,base);
    a.packet->Reset();b.packet->Reset();
    Launch(*a.packet,false);Launch(*b.packet,true);
    ASSERT_EQ(a.packet->storage.control.status,c::NodalWallDeviceStatus::Ok);
    Same(*a.packet,*b.packet);
  }
}
TEST(MappedWallScatterCuda, LegacySerialScheduleStillMatchesFrozenSuccessAndLateFailure) {
  Device a,b;
  ASSERT_NE(a.packet,nullptr);ASSERT_NE(b.packet,nullptr);
  for(unsigned fault:{6u,2u,3u}) {
    a.packet->Reset();b.packet->Reset();
    Fault(*a.packet,fault);Fault(*b.packet,fault);
    Legacy<<<1,1>>>(&a.packet->storage,a.packet->View(),false);
    Legacy<<<1,1>>>(&b.packet->storage,b.packet->View(),true);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    Same(*a.packet,*b.packet);
  }
}
} // namespace wall_scatter_test
