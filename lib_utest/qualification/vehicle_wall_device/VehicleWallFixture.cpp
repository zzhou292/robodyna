#include "VehicleWallFixture.h"
#include <cmath>
namespace vehicle_wall_device_test {
bool Fixture::Prepare() {
  vehicle_shell_test::Fixture source(nq,nt,n);
  // Repeated small patches keep finite wall extents fixed. Counts, distinct
  // node identities and mixed connectivity are synthetic capacity evidence.
  const auto place=[](auto& parents){for(auto& p:parents)for(auto& x:p.reference.position)
    x={0,.5*std::fmod(x.x,2.)-.25,.5*x.y-.25};};
  place(source.q);place(source.t);
  fe::ShellBatchBinding binding;
  const auto b=binding.Initialize(source.input(),fe::ShellHostBindingLimits::Vehicle());
  if(b.status!=fe::ShellBindingStatus::Success)return false;
  for(std::size_t i=0;i<n;++i) {
    const auto& node=binding.nodes()[i];x[3*i]=node.position.x;x[3*i+1]=node.position.y;x[3*i+2]=node.position.z;
    inverse[i]=1/node.native.mass;inverse_j[i]=1/node.native.isotropic_inertia;q[4*i]=1;
  }
  std::vector<sc::Q4ParametricReference> quads(nq);std::vector<sc::T3MaterialMeasure> triangles(nt);
  std::vector<sc::NodalWallParentInput> input(nq+nt);
  for(std::size_t e=0;e<nq;++e) {
    sc::SurfaceQ4 p;p.parent_element_id=p.feature_id=binding.qeph_source_id(e);
    for(unsigned l=0;l<4;++l)p.nodes[l]=binding.qeph_nodes(e)[l];
    if(quads[e].Initialize(Positions(),&p,1).status!=sc::Q4ParametricStatus::Ok)return false;
    input[e]={&quads[e],0,nullptr};
  }
  for(std::size_t e=0;e<nt;++e) {
    sc::SurfaceTriangle p;p.parent_element_id=p.feature_id=binding.t3_source_id(e);
    p.interpolation=sc::SurfaceInterpolation::kLinearTriangle;
    for(unsigned l=0;l<3;++l)p.nodes[l]=binding.t3_nodes(e)[l];
    if(sc::PrepareT3MaterialMeasure(Positions(),p,&triangles[e])!=sc::SurfaceMeasureStatus::Ok)return false;
    input[nq+e]={nullptr,0,&triangles[e]};
  }
  if(weights.Initialize(n,input.data(),input.size(),sc::NodalWallWeightLimits::Vehicle()).status!=sc::NodalWallStatus::Ok||
      weights.node_count()!=n)return false;
  for(std::size_t p=0;p<weights.parent_count();++p) {
    const auto& parent=weights.parent(p);const long double share=parent.arity==4?1.L/16:1.L/24;
    for(unsigned l=0;l<parent.arity;++l)area[parent.nodes[l]]+=share;
  }
  for(std::size_t i=0;i<n;++i){x[3*i]=1./4096;v[3*i]=(static_cast<int>(i%5)-2)/64.;}
  x[3*(n-1)]=1./32;return true;
}
sc::NodalWallDeviceConfig Fixture::Config(fe::NodalStamp stamp) const {
  if(!stamp.owner_id){stamp.owner_id=77;stamp.node_count=n;stamp.fixed_dt=H;stamp.has_rotations=true;
    stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;}
  sc::NodalWallDeviceConfig c;c.owner=stamp;c.configuration_id=991;c.wall_binding_id=771;c.qualification_id=Qualification;
  c.law={0,16,.5,nodal_wall_owner_test::ForceBudget,nodal_wall_owner_test::EnergyBudget};
  c.limits=sc::NodalWallDeviceLimits::Vehicle();c.max_device_bytes=sc::MaxVehicleNodalWallDeviceBytes;
  c.max_host_bytes=sc::MaxVehicleNodalWallHostBytes;return c;
}
bool Fixture::Owner(fe::FENodalState& owner) const {
  fe::NodalStateConfig c;c.node_count=n;c.fixed_dt=H;c.max_nodes=fe::MaxActiveNodalStateNodes;
  c.max_device_bytes=fe::MaxActiveNodalStateDeviceBytes;c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto r=owner.Initialize(c,{x.data(),v.data(),w.data(),n,q.data()},inverse.data(),{fixed.data(),fixed.data(),inverse_j.data()});
  EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message;return r.status==fe::NodalStatus::Ok;
}
bool Fixture::Bind(fe::FENodalState& owner,sc::NodalWallContactDevice& contact) const {
  const auto r=contact.Initialize(Config(owner.accepted()),wall.view(),weights,Positions(),inverse.data(),fixed.data(),motion);
  EXPECT_EQ(r.status,Code::Ok)<<r.message;return r.status==Code::Ok;
}
} // namespace vehicle_wall_device_test
