#include "NodalWallCapacityFixture.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"

namespace nodal_wall_capacity_test {
namespace {
template<class T> void Material(T& in) {
  in.density=1024; in.thickness=1./32; in.young_modulus=2e6; in.poisson_ratio=.3;
}
}
bool Fixture::Prepare() {
  if(!source.Prepare() || weights.Initialize(Nodes,source.input.data(),Parents,{}).status!=sc::NodalWallStatus::Ok ||
      weights.node_count()!=Nodes) return false;
  x=source.x; v.resize(3*Nodes); omega.resize(3*Nodes); q.resize(4*Nodes);
  mass.resize(Nodes); inertia.resize(Nodes); inverse.resize(Nodes); inverse_j.resize(Nodes);
  fixed.resize(Nodes); rotation_fixed.resize(Nodes); area.resize(Nodes);
  for(unsigned p=0;p<Parents;++p) {
    const auto& parent=weights.parent(p);
    if(parent.arity==4) {
      fe::qeph::ReferenceInput in; Material(in);
      for(unsigned l=0;l<4;++l) { const auto n=parent.nodes[l]; const auto a=source.positions().at(n);
        in.position[l]={a.x,a.y,a.z}; in.node_ids[l]=1000+n; }
      fe::qeph::ReferenceData out;
      if(fe::qeph::InitializeReference(in,out)!=fe::qeph::Status::kSuccess) return false;
      for(unsigned l=0;l<4;++l) { const auto n=parent.nodes[l]; mass[n]+=out.nodal_mass[l];
        inertia[n]+=out.isotropic_inertia[l]; area[n]+=SquareArea/4; }
    } else {
      fe::t3::ReferenceInput in; Material(in);
      for(unsigned l=0;l<3;++l) { const auto n=parent.nodes[l]; const auto a=source.positions().at(n);
        in.position[l]={a.x,a.y,a.z}; in.node_ids[l]=1000+n; }
      fe::t3::ReferenceData out;
      if(fe::t3::InitializeReference(in,out)!=fe::t3::Status::kSuccess) return false;
      for(unsigned l=0;l<3;++l) { const auto n=parent.nodes[l]; mass[n]+=out.nodal_mass[l];
        inertia[n]+=out.isotropic_inertia[l]; area[n]+=SquareArea/6; }
    }
  }
  for(unsigned n=0;n<Nodes;++n) {
    if(!(mass[n]>0 && inertia[n]>0 && area[n]>0)) return false;
    inverse[n]=1/mass[n]; inverse_j[n]=1/inertia[n]; q[4*n]=1;
    x[3*n]=1./4096; v[3*n]=(static_cast<int>(n%5)-2)/64.;
  }
  x[3*(Nodes-1)]=1./32; return true;
}
sc::NodalWallDeviceConfig Fixture::Config(fe::NodalStamp stamp) const {
  if(!stamp.owner_id) { stamp.owner_id=77; stamp.node_count=Nodes; stamp.fixed_dt=Step;
    stamp.has_rotations=true; stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart; }
  sc::NodalWallDeviceConfig c; c.owner=stamp; c.configuration_id=991; c.wall_binding_id=771;
  c.qualification_id=Qualification; c.law={0,16,.5,nodal_wall_owner_test::ForceBudget,nodal_wall_owner_test::EnergyBudget};
  c.limits={1024,2048,2048}; c.max_device_bytes=sc::MaxActiveNodalWallDeviceBytes;
  c.max_host_bytes=sc::MaxNodalWallHostBytes; return c;
}
bool Fixture::Owner(fe::FENodalState& owner) const {
  fe::NodalStateConfig c; c.node_count=Nodes; c.fixed_dt=Step;
  c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto r=owner.Initialize(c,{x.data(),v.data(),omega.data(),Nodes,q.data()},inverse.data(),
      {fixed.data(),rotation_fixed.data(),inverse_j.data()});
  EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message; return r.status==fe::NodalStatus::Ok;
}
bool Fixture::Bind(fe::FENodalState& owner,sc::NodalWallContactDevice& contact) const {
  const auto r=contact.Initialize(Config(owner.accepted()),wall.view(),weights,Positions(),inverse.data(),fixed.data(),motion);
  EXPECT_EQ(r.status,Code::Ok)<<r.message; return r.status==Code::Ok;
}
} // namespace nodal_wall_capacity_test
