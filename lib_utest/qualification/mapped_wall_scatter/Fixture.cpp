#include "Fixture.h"
namespace wall_scatter_test {
void Packet::Reset() {
  storage={};
  storage.model.node_count=Nodes;
  storage.model.config.owner.node_count=Globals;
  storage.model.nodes=nodes;
  storage.node_status=status;
  storage.result.nodes=result;
  storage.base.nodes=base;
  storage.staged_force=staged;
  storage.addition_error=error;
  std::fill_n(staged,6*Globals,-999.);
  std::fill_n(private_stiffness,Nodes,-888.);
  std::fill_n(error,Nodes,-777.);
  for(unsigned i=0;i<Globals;++i) {
    stiffness[i]=i%7?double(i):0.;
    rotation[i]=i%2?-0.:3.;
    for(unsigned channel=0;channel<6;++channel)
      forces[channel*Globals+i]=i%3==0?-0.:double(int(i%5)-2);
  }
  for(unsigned i=0;i<Nodes;++i) {
    const unsigned n=(73*i+101)%Globals; // Unique, sparse, deliberately nonmonotonic.
    nodes[i]={};
    nodes[i].node=n;
    result[i]={};
    result[i].node=n;
    result[i].stiffness.value=i%7?double(i+1)*.125:0.;
    if(i==0) forces[Globals+n]=-0.;
    result[i].force_world.x=i%4==0?-0.:i%4==1?-forces[n]:i%4==2?1e-25:-1e12;
    // Only the selected scalar normal force is scattered by this profile.
    result[i].force_world.y=std::numeric_limits<double>::quiet_NaN();
    result[i].force_world.z=std::numeric_limits<double>::quiet_NaN();
    base[i]={};
    base[i].force.value=double(i)+.25;
    status[i]={};
    status[i].status=c::NodalWallDeviceStatus::GeometryFailure; // Must reset stale slots.
  }
  assembly_result={};
  bounds={};
}
fe::NodalAssemblyView Packet::View() {
  fe::NodalAssemblyView view;
  view.forces={forces,forces+Globals,forces+2*Globals,forces+3*Globals,forces+4*Globals,forces+5*Globals,Globals,3};
  view.result=&assembly_result;
  view.bounds=&bounds;
  return view;
}
fe::NodalCinAssemblyView Packet::Cin() {
  fe::NodalCinAssemblyView cin;
  cin.translational_stiffness=stiffness;
  cin.rotational_stiffness=rotation;
  cin.node_count=Globals;
  return cin;
}
m::Sidecar Packet::Side() {
  m::Sidecar side;
  side.stiffness=private_stiffness;
  return side;
}
void Fault(Packet& p,unsigned fault) {
  const double bad=std::numeric_limits<double>::quiet_NaN();
  if(fault==0) p.stiffness[p.nodes[Nodes-1].node]=bad;
  if(fault==1) {
    p.stiffness[p.nodes[5].node]=-1;
    p.stiffness[p.nodes[130].node]=bad;
  }
  if(fault==2) p.forces[5*Globals+p.nodes[Nodes-1].node]=bad;
  if(fault==3) {
    p.forces[p.nodes[5].node]=std::numeric_limits<double>::max();
    p.result[5].force_world.x=std::numeric_limits<double>::max();
    p.forces[2*Globals+p.nodes[130].node]=bad;
  }
  if(fault==4) {
    p.stiffness[p.nodes[Nodes-1].node]=bad;
    p.forces[p.nodes[5].node]=bad; // Later stiffness wins over earlier force node.
  }
  if(fault==5) {
    p.storage.control.status=c::NodalWallDeviceStatus::StepTooLarge;
    p.storage.control.node=17;
  }
}
} // namespace wall_scatter_test
