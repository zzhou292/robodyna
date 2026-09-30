#include "RigidShellContactFixture.h"
#include "lib_src/elements/qeph/QephForce.h"
#include "lib_src/elements/t3/T3Force.h"

namespace rigid_shell_contact_test {
bool Fixture::Prepare(std::uint64_t source,double speed) {
  auto input=shell_binding_test::Edge();
  // Reorient the existing shared-edge patch into a slightly tilted YZ plane.
  // Source node IDs and native QEPH/T3 producers remain authoritative.
  auto transform=[](tl::math::Vec3 p) { return tl::math::Vec3{.001*p.x,p.x,p.y}; };
  for(auto& p:input.qeph.position) p=transform(p);
  for(auto& p:input.t3.position) p=transform(p);
  const auto b=binding.Initialize(input);
  EXPECT_EQ(b.status,fe::ShellBindingStatus::Success)<<b.message;
  if(b.status!=fe::ShellBindingStatus::Success) return false;
  initial.n=Nodes; initial.h=H;
  for(unsigned n=0;n<Nodes;++n) {
    const auto& node=binding.nodes()[n]; const double x[]{node.position.x,node.position.y,node.position.z};
    for(unsigned a=0;a<3;++a) initial.x[3*n+a]=x[a];
    initial.v[3*n]=speed;
    initial.inverse[n]=1/node.native.mass;
    initial.inverse_inertia[n]=1/node.native.isotropic_inertia;
  }
  std::array<fe::NodalRigidGroupMember,3> members;
  const unsigned member_nodes[]{0,1,4};
  for(unsigned i=0;i<3;++i) {
    const unsigned n=member_nodes[i]; const auto& node=binding.nodes()[n]; const auto& m=node.native;
    members[i]={node.source_id,n,node.position,m.mass,m.isotropic_inertia,m.physical_inertia,m.added_inertia};
  }
  const fe::NodalRigidGroupInput group{2001,2002,members.data(),members.size()};
  const auto g=groups.Initialize({source,Nodes,&group,1,{1000,.001}});
  EXPECT_TRUE(g)<<g.message; if(!g) return false;
  sc::SurfaceQ4 quad; quad.parent_element_id=10; quad.feature_id=20;
  sc::SurfaceTriangle tri; tri.parent_element_id=11; tri.feature_id=21;
  tri.interpolation=sc::SurfaceInterpolation::kLinearTriangle;
  for(unsigned i=0;i<4;++i) quad.nodes[i]=binding.qeph_nodes()[i];
  for(unsigned i=0;i<3;++i) tri.nodes[i]=binding.t3_nodes()[i];
  const sc::VectorView x{initial.x.data(),Nodes,3,1};
  sc::Q4ParametricReference qref; sc::T3MaterialMeasure tref;
  const auto qr=qref.Initialize(x,&quad,1); EXPECT_EQ(qr.status,sc::Q4ParametricStatus::Ok);
  const auto tr=sc::PrepareT3MaterialMeasure(x,tri,&tref); EXPECT_EQ(tr,sc::SurfaceMeasureStatus::Ok);
  if(qr.status!=sc::Q4ParametricStatus::Ok||tr!=sc::SurfaceMeasureStatus::Ok) return false;
  const sc::NodalWallParentInput parents[]{{&qref,0,nullptr},{nullptr,0,&tref}};
  const auto wr=weights.Initialize(Nodes,parents,2); EXPECT_EQ(wr.status,sc::NodalWallStatus::Ok);
  wall_vertices={{{{0,-2,-2},301,401},{{0,4,-2},302,402},{{0,4,3},303,403},{{0,-2,3},304,404}}};
  // Reversed YZ winding supplies the required -X wall normal.
  wall_faces={{{{0,2,1},501,601,701},{{0,3,2},502,601,701}}};
  return wr.status==sc::NodalWallStatus::Ok;
}
fe::NodalReport Fixture::Owner(fe::FENodalState& owner) const {
  fe::NodalStateConfig config; config.node_count=Nodes; config.fixed_dt=H;
  config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  return owner.Initialize(config,{initial.x.data(),initial.v.data(),initial.omega.data(),Nodes,initial.q.data()},
      initial.inverse.data(),{initial.fixed.data(),initial.rotation_fixed.data(),initial.inverse_inertia.data()},groups);
}
q::QephBatchConfig Fixture::QConfig(fe::NodalStamp stamp) const {
  q::QephBatchConfig c; c.owner=stamp; c.element_count=1; c.configuration_id=Configuration;
  c.qualification_id=Qualification; c.usage=q::BatchUsage::CoupledForces;
  if(initial.v[0]!=0) c.startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,{initial.v[0],0,0}};
  return c;
}
t::T3BatchConfig Fixture::TConfig(fe::NodalStamp stamp) const {
  t::T3BatchConfig c; c.owner=stamp; c.element_count=1; c.configuration_id=Configuration;
  c.qualification_id=Qualification; c.usage=t::BatchUsage::CoupledForces;
  if(initial.v[0]!=0) c.startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,{initial.v[0],0,0}};
  return c;
}
sc::NodalWallDeviceConfig Fixture::WallConfig(fe::NodalStamp stamp) const {
  sc::NodalWallDeviceConfig c; c.owner=stamp; c.configuration_id=Configuration;
  c.qualification_id=Qualification; c.wall_binding_id=3001;
  c.law={0,1000,.02,1e-6,1e-8}; c.exposed_clearance=.1; return c;
}
bool Rig::Initialize(std::uint64_t source_id,double speed,bool bind) {
  if(!source.Prepare(source_id,speed)) return false;
  const auto n=source.Owner(owner); EXPECT_EQ(n.status,fe::NodalStatus::Ok)<<n.message;
  if(n.status!=fe::NodalStatus::Ok) return false;
  const auto q=qeph.InitializeJoined(source.QConfig(owner.accepted()),source.binding);
  const auto t=t3.InitializeJoined(source.TConfig(owner.accepted()),source.binding);
  EXPECT_EQ(q.status,q::BatchStatus::Success)<<q.message; EXPECT_EQ(t.status,t::BatchStatus::Success)<<t.message;
  const auto w=contact.Initialize(source.WallConfig(owner.accepted()),source.Wall(),source.weights,
      {source.initial.x.data(),Nodes,3,1},source.initial.inverse.data(),source.initial.fixed.data(),source.motion);
  EXPECT_EQ(w.status,sc::NodalWallDeviceStatus::Ok)<<w.message;
  return q.status==q::BatchStatus::Success&&t.status==t::BatchStatus::Success&&w.status==sc::NodalWallDeviceStatus::Ok&&(!bind||Bind());
}
bool Rig::Bind() {
  Prepared p; if(!Assemble(*this,p,false)) return false;
  Discard();
  const auto r=publication.Initialize(owner,qeph,t3); EXPECT_EQ(r.status,fe::ShellPublicationStatus::Success)<<r.message;
  return r.status==fe::ShellPublicationStatus::Success;
}
bool Assemble(Rig& r,Prepared& p,bool load) {
  nt::Loads loads;
  if(load) { loads.force[3*4+1]=1; loads.couple[3*4+2]=.02; }
  if(!nt::BeginLoad(r.owner,loads,p.token,p.assembly)) return false;
  EXPECT_EQ(p.assembly.mass.model,sc::TranslationMassModel::kUnspecified);
  const auto qr=r.qeph.AssembleAccepted(r.owner,p.assembly); const auto tr=r.t3.AssembleAccepted(r.owner,p.assembly);
  const auto wr=r.contact.AssembleAccepted(r.owner,p.assembly,&p.wall_base);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message; EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  EXPECT_EQ(wr.status,sc::NodalWallDeviceStatus::Ok)<<wr.message;
  return qr.status==q::BatchStatus::Success&&tr.status==t::BatchStatus::Success&&wr.status==sc::NodalWallDeviceStatus::Ok;
}
bool Advance(Rig& r,Prepared& p) {
  auto n=r.owner.SealAssembly(p.token); EXPECT_EQ(n.status,fe::NodalStatus::Ok); if(n.status!=fe::NodalStatus::Ok) return false;
  n=fe::AdvanceStaggeredRigidGroups(r.owner,p.token,
      {p.assembly.owner_id,p.assembly.accepted.base_epoch,p.assembly.attempt,H,.2,Qualification});
  EXPECT_EQ(n.status,fe::NodalStatus::Ok)<<n.message; if(n.status!=fe::NodalStatus::Ok) return false;
  n=r.owner.BorrowPrepared(p.token,&p.view); EXPECT_EQ(n.status,fe::NodalStatus::Ok); return n.status==fe::NodalStatus::Ok;
}
bool Shells(Rig& r,Prepared& p) {
  const auto qr=r.qeph.EvaluateCandidate(r.owner,p.token,p.view,&p.shells.qeph);
  const auto tr=r.t3.EvaluateCandidate(r.owner,p.token,p.view,&p.shells.t3);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message; EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  if(qr.status!=q::BatchStatus::Success||tr.status!=t::BatchStatus::Success) return false;
  EXPECT_EQ(r.qeph.CopyPreparedResults(p.shells.qeph,&p.quad,1).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.CopyPreparedResults(p.shells.t3,&p.triangle,1).status,t::BatchStatus::Success);
  fe::ShellBatchDiagnostics joined;
  const auto pub=r.publication.Prepare(r.owner,p.token,p.shells.qeph,p.shells.t3,&joined);
  if(pub.status==fe::ShellPublicationStatus::Success) p.shells=joined;
  EXPECT_EQ(pub.status,fe::ShellPublicationStatus::Success)<<pub.message;
  return pub.status==fe::ShellPublicationStatus::Success;
}
bool Prepare(Rig& r,Prepared& p,bool load) {
  if(!Assemble(r,p,load)||!Advance(r,p)||!Shells(r,p)) return false;
  const auto w=r.contact.EvaluateCandidate(r.owner,p.token,p.view,&p.wall_candidate);
  EXPECT_EQ(w.status,sc::NodalWallDeviceStatus::Ok)<<w.message; return w.status==sc::NodalWallDeviceStatus::Ok;
}
bool Commit(Rig& r,const Prepared& p,bool valid) {
  const auto result=r.publication.Commit(r.owner,p.token,p.shells,
      {p.view.owner_id,p.view.kinematics.base_epoch,p.view.attempt,Qualification,valid});
  if(valid) EXPECT_EQ(result.status,fe::ShellPublicationStatus::Success)<<result.message;
  else EXPECT_NE(result.status,fe::ShellPublicationStatus::Success);
  return result.status==fe::ShellPublicationStatus::Success;
}
nt::Snapshot ReadPrepared(const fe::NodalPreparedView& p) {
  nt::Snapshot out;
  const double* src[]{p.kinematics.position_xyz,p.kinematics.velocity_xyz,p.kinematics.angular_velocity_xyz,p.kinematics.orientation_wxyz};
  double* dst[]{out.x.data(),out.v.data(),out.omega.data(),out.q.data()};
  for(unsigned i=0;i<4;++i) EXPECT_EQ(cudaMemcpyAsync(dst[i],src[i],Nodes*(i==3?4:3)*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess); return out;
}
std::array<double,6*Nodes> ReadLoads(const fe::NodalAssemblyView& a) {
  std::array<double,6*Nodes> out{};
  const double* src[]{a.forces.force_x,a.forces.force_y,a.forces.force_z,a.forces.couple_x,a.forces.couple_y,a.forces.couple_z};
  for(unsigned i=0;i<6;++i) EXPECT_EQ(cudaMemcpyAsync(out.data()+i*Nodes,src[i],Nodes*sizeof(double),cudaMemcpyDeviceToHost,a.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess); return out;
}
fe::NodalRigidGroupSnapshot ReadGroup(fe::FENodalState& owner) {
  fe::NodalRigidGroupSnapshot group; fe::NodalStamp stamp;
  EXPECT_EQ(owner.CopyAcceptedRigidGroups({&group,1},&stamp).status,fe::NodalStatus::Ok); return group;
}
void SameGroup(const fe::NodalRigidGroupSnapshot& a,const fe::NodalRigidGroupSnapshot& b) {
  EXPECT_EQ(a.source_group_id,b.source_group_id); EXPECT_EQ(a.source_node_set_id,b.source_node_set_id);
  EXPECT_EQ(Bytes(a.state),Bytes(b.state));
}
} // namespace rigid_shell_contact_test
