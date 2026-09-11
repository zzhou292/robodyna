#include "Fixture.h"
#include "FrozenScatter.h"
namespace wall_scatter_test {
void Frozen(Packet& p) {
  if(p.storage.control.status!=c::NodalWallDeviceStatus::Ok) return;
  if(wall_scatter_frozen::StageStiffness(p.storage,p.Side(),p.Cin()) &&
      wall_scatter_frozen::Scatter(p.storage,p.View(),false)) {
    wall_scatter_frozen::PublishScatter(p.storage,p.View());
    for(unsigned i=0;i<Nodes;++i) p.stiffness[p.nodes[i].node]=p.private_stiffness[i];
  }
}
void Staged(Packet& p) {
  if(p.storage.control.status!=c::NodalWallDeviceStatus::Ok) return;
  // Reverse visitation demonstrates that arbitration follows compact identity,
  // independently of the order in which disjoint private results complete.
  for(unsigned i=Nodes;i-->0;) {
    p.status[i]={};
    m::StageStiffnessNode(p.storage,p.Side(),p.Cin(),i,p.status[i]);
  }
  m::CheckScatterNodes(p.storage);
  if(p.storage.control.status!=c::NodalWallDeviceStatus::Ok) return;
  const auto staged=d::StagedForces(p.storage,p.View());
  for(unsigned i=Nodes;i-->0;) {
    p.status[i]={};
    for(unsigned channel=0;channel<6;++channel) d::CopyScatterChannel(p.storage,p.View(),i,channel);
    d::AccumulateScatterNode(p.storage,p.View(),staged,i,p.status[i]);
  }
  m::CheckScatterNodes(p.storage);
  if(p.storage.control.status!=c::NodalWallDeviceStatus::Ok) return;
  for(unsigned i=Nodes;i-->0;) {
    for(unsigned channel=0;channel<6;++channel) d::PublishScatterChannel(p.storage,p.View(),i,channel);
    p.stiffness[p.nodes[i].node]=p.private_stiffness[i];
  }
}
TEST(MappedWallScatterHost, IndependentNodeBitsAndStiffnessMatchFrozenWithCancellation) {
  Packet current,serial;
  current.Reset();serial.Reset();
  Staged(current);Frozen(serial);
  ASSERT_EQ(current.storage.control.status,c::NodalWallDeviceStatus::Ok);
  Same(current,serial);
  const auto node=current.nodes[0].node;
  EXPECT_EQ(Bits(current.forces[Globals+node]),Bits(0.)); // Native add +0 changes -0 if present.
}
TEST(MappedWallScatterHost, PhasePriorityCompactOrderCompleteRollbackAndRetry) {
  Packet current,serial;
  for(unsigned fault=0;fault<6;++fault) {
    SCOPED_TRACE(fault);
    current.Reset();serial.Reset();
    Fault(current,fault);Fault(serial,fault);
    const Destinations before(current);
    Staged(current);Frozen(serial);
    ASSERT_NE(current.storage.control.status,c::NodalWallDeviceStatus::Ok);
    Same(current,serial);before.Unchanged(current);
    const auto compact=fault==1||fault==3?5:Nodes-1;
    EXPECT_EQ(current.storage.control.node,fault==5?17:current.nodes[compact].node);
    current.Reset();serial.Reset();
    Staged(current);Frozen(serial);
    ASSERT_EQ(current.storage.control.status,c::NodalWallDeviceStatus::Ok);
    Same(current,serial);
  }
}
} // namespace wall_scatter_test
