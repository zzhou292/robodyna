#include "ResidentCollectionFixture.h"

namespace resident_collection_test {
namespace {
double Component(tl::math::Vec3 v,unsigned a) { return a==0?v.x:a==1?v.y:v.z; }
__global__ void AddLoads(fe::NodalAssemblyView view,const double* load) {
  for(unsigned n=0;n<view.accepted.node_count;++n) {
    view.forces.force_x[n]+=load[3*n]; view.forces.force_y[n]+=load[3*n+1];
    view.forces.force_z[n]+=load[3*n+2];
    view.forces.couple_x[n]+=load[3*Capacity+3*n]; view.forces.couple_y[n]+=load[3*Capacity+3*n+1];
    view.forces.couple_z[n]+=load[3*Capacity+3*n+2];
  }
}
static_assert(sizeof(fe::NodalAssemblyView)+sizeof(double*)<4096,"Loads are pointer-backed");
bool Endpoint(const fe::NodalPreparedView& p,Snapshot& out) {
  const auto n=p.kinematics.node_count;
  EXPECT_EQ(cudaMemcpyAsync(out.x.data(),p.kinematics.position_xyz,3*n*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(out.v.data(),p.kinematics.velocity_xyz,3*n*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(out.omega.data(),p.kinematics.angular_velocity_xyz,3*n*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(out.orientation.data(),p.kinematics.orientation_wxyz,4*n*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess);
  return !::testing::Test::HasFailure();
}
}
Rig::~Rig() { if(device_loads) cudaFree(device_loads); }
bool Rig::BuildReference() {
  const auto material=shell_binding_test::Edge();
  for(unsigned y=0;y<9;++y) for(unsigned xnode=0;xnode<13;++xnode) {
    const unsigned n=13*y+xnode; x[3*n]=xnode*.125; x[3*n+1]=y*.125; orientation[4*n]=1;
  }
  unsigned qi=0,ti=0;
  for(unsigned row=0;row<8;++row) for(unsigned col=0;col<12;++col) {
    const unsigned n=13*row+col;
    const std::array<std::size_t,4> corners{n,n+1,n+14,n+13};
    if(qi<QCount) {
      auto& cell=qinput[qi]; cell.reference=material.qeph; cell.nodes=corners;
      cell.source_parent_id=1000+qi++;
      for(unsigned local=0;local<4;++local) {
        const auto node=corners[local]; cell.reference.node_ids[local]=100+node;
        cell.reference.position[local]={x[3*node],x[3*node+1],x[3*node+2]};
      }
    } else for(unsigned half=0;half<2;++half) {
      auto& cell=tinput[ti]; cell.reference=material.t3;
      cell.nodes=half?std::array<std::size_t,3>{corners[0],corners[2],corners[3]}:
                      std::array<std::size_t,3>{corners[0],corners[1],corners[2]};
      cell.source_parent_id=(std::uint64_t{1}<<54)+2000+ti++;
      for(unsigned local=0;local<3;++local) {
        const auto node=cell.nodes[local]; cell.reference.node_ids[local]=100+node;
        cell.reference.position[local]={x[3*node],x[3*node+1],x[3*node+2]};
      }
    }
  }
  EXPECT_EQ(qi,QCount); EXPECT_EQ(ti,TCount);
  const auto r=binding.Initialize({qinput.data(),tinput.data(),QCount,TCount,Nodes});
  EXPECT_EQ(r.status,fe::ShellBindingStatus::Success)<<r.message;
  if(r.status!=fe::ShellBindingStatus::Success) return false;
  for(unsigned n=0;n<Nodes;++n) {
    inverse[n]=1/binding.nodes()[n].native.mass;
    inverse_j[n]=1/binding.nodes()[n].native.isotropic_inertia;
  }
  return !::testing::Test::HasFailure();
}
bool Rig::InitializeOwner() {
  fe::NodalStateConfig c; c.node_count=Nodes; c.fixed_dt=H;
  c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto r=owner.Initialize(c,{x.data(),zero.data(),zero.data(),Nodes,orientation.data()},inverse.data(),
      {fixed.data(),fixed.data(),inverse_j.data()});
  EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message;
  if(r.status!=fe::NodalStatus::Ok) return false;
  const auto allocated=cudaMalloc(reinterpret_cast<void**>(&device_loads),6*Capacity*sizeof(double));
  EXPECT_EQ(allocated,cudaSuccess); return allocated==cudaSuccess;
}
q::QephBatchConfig Rig::QConfig() const {
  q::QephBatchConfig c; c.owner=owner.accepted(); c.element_count=QCount;
  c.configuration_id=Configuration; c.qualification_id=Qualification; c.usage=q::BatchUsage::CoupledForces; return c;
}
t::T3BatchConfig Rig::TConfig() const {
  t::T3BatchConfig c; c.owner=owner.accepted(); c.element_count=TCount;
  c.configuration_id=Configuration; c.qualification_id=Qualification; c.usage=t::BatchUsage::CoupledForces; return c;
}
bool Rig::InitializeParticipants() {
  const auto qr=qeph.InitializeJoined(QConfig(),binding);
  const auto tr=t3.InitializeJoined(TConfig(),binding);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  return qr.status==q::BatchStatus::Success&&tr.status==t::BatchStatus::Success;
}
bool Rig::Bind() {
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  EXPECT_EQ(owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  EXPECT_EQ(qeph.AssembleAccepted(view).status,q::BatchStatus::Success);
  EXPECT_EQ(t3.AssembleAccepted(view).status,t::BatchStatus::Success);
  Discard(); if(::testing::Test::HasFailure()) return false;
  const auto r=publication.Initialize(owner,qeph,t3);
  EXPECT_EQ(r.status,fe::ShellPublicationStatus::Success)<<r.message; return r.status==fe::ShellPublicationStatus::Success;
}
bool Rig::Initialize() { return BuildReference()&&InitializeOwner()&&InitializeParticipants()&&Bind(); }
bool Read(fe::FENodalState& owner,Snapshot& out) {
  const auto r=owner.CopyAccepted(out.buffer(),&out.stamp);
  EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message; return r.status==fe::NodalStatus::Ok;
}
bool Accepted(Rig& r,Staged& out) {
  const auto qr=r.qeph.CopyAcceptedResults(r.owner.accepted(),out.qeph.data(),out.qeph.size(),&out.diagnostics.qeph);
  const auto tr=r.t3.CopyAcceptedResults(r.owner.accepted(),out.t3.data(),out.t3.size(),&out.diagnostics.t3);
  EXPECT_EQ(qr.status,q::BatchStatus::Success); EXPECT_EQ(tr.status,t::BatchStatus::Success);
  const auto cr=r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&out.diagnostics);
  EXPECT_EQ(cr.status,fe::ShellPublicationStatus::Success);
  return !::testing::Test::HasFailure();
}
Loads Pulse(const Rig& r) {
  Loads out;
  for(unsigned n=0;n<Nodes;++n) {
    out.values[3*n]=r.binding.nodes()[n].native.mass*(.25+.5*r.x[3*n]);
    out.values[3*Capacity+3*n+1]=r.binding.nodes()[n].native.isotropic_inertia*(.125+.25*r.x[3*n+1]);
  }
  return out;
}
bool Prepare(Rig& r,const Loads& load,const Staged& cache,Prepared& p) {
  p={}; fe::NodalAssemblyView view;
  EXPECT_EQ(r.owner.BeginTrial(&p.token,&view).status,fe::NodalStatus::Ok);
  EXPECT_EQ(cudaMemcpyAsync(r.device_loads,load.values.data(),sizeof(load.values),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  AddLoads<<<1,1,0,view.stream>>>(view,r.device_loads);
  EXPECT_EQ(cudaGetLastError(),cudaSuccess);
  EXPECT_EQ(r.qeph.AssembleAccepted(view).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.AssembleAccepted(view).status,t::BatchStatus::Success);
  EXPECT_EQ(cudaMemcpyAsync(p.rhs.data(),view.forces.force_x,6*Nodes*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  std::array<double,6*Capacity> expected{};
  for(unsigned n=0;n<Nodes;++n) for(unsigned a=0;a<3;++a) {
    expected[a*Nodes+n]=load.values[3*n+a]; expected[(a+3)*Nodes+n]=load.values[3*Capacity+3*n+a];
  }
  auto scatter=[&](const auto& nodes,const auto& result) {
    for(unsigned local=0;local<nodes.size();++local) for(unsigned a=0;a<3;++a) {
      expected[a*Nodes+nodes[local]]-=Component(result.internal_force[local],a);
      expected[(a+3)*Nodes+nodes[local]]-=Component(result.internal_couple[local],a);
    }
  };
  for(unsigned e=0;e<QCount;++e) scatter(r.binding.qeph_nodes(e),cache.qeph[e]);
  for(unsigned e=0;e<TCount;++e) scatter(r.binding.t3_nodes(e),cache.t3[e]);
  EXPECT_EQ(p.rhs,expected);
  if(::testing::Test::HasFailure()) { r.Discard(); return false; }
  EXPECT_EQ(r.owner.SealAssembly(p.token).status,fe::NodalStatus::Ok);
  EXPECT_EQ(fe::AdvanceStaggeredHistory(r.owner,p.token,
    {view.owner_id,view.accepted.base_epoch,view.attempt,H,1,Qualification}).status,fe::NodalStatus::Ok);
  EXPECT_EQ(r.owner.BorrowPrepared(p.token,&p.view).status,fe::NodalStatus::Ok);
  return !::testing::Test::HasFailure()&&Endpoint(p.view,p.endpoint);
}
bool Evaluate(Rig& r,const Prepared& p,Staged& out,bool t3_first) {
  auto qeph=[&] {
    EXPECT_EQ(r.qeph.EvaluateCandidate(p.view,&out.diagnostics.qeph).status,q::BatchStatus::Success);
    EXPECT_EQ(r.qeph.CopyPreparedResults(out.diagnostics.qeph,out.qeph.data(),QCount).status,q::BatchStatus::Success);
  };
  auto tri=[&] {
    EXPECT_EQ(r.t3.EvaluateCandidate(p.view,&out.diagnostics.t3).status,t::BatchStatus::Success);
    EXPECT_EQ(r.t3.CopyPreparedResults(out.diagnostics.t3,out.t3.data(),TCount).status,t::BatchStatus::Success);
  };
  if(t3_first) { tri(); qeph(); } else { qeph(); tri(); }
  if(::testing::Test::HasFailure()) return false;
  fe::ShellBatchDiagnostics common;
  const auto report=r.publication.Prepare(r.owner,p.token,out.diagnostics.qeph,out.diagnostics.t3,&common);
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;
  if(report.status!=fe::ShellPublicationStatus::Success) return false;
  out.diagnostics=common; return true;
}
bool Publish(Rig& r,const Prepared& p,const Staged& s) {
  if(::testing::Test::HasFailure()) { r.Discard(); return false; }
  const auto& d=s.diagnostics.qeph;
  const auto report=r.publication.Commit(r.owner,p.token,s.diagnostics,
      {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true});
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;
  return report.status==fe::ShellPublicationStatus::Success;
}
} // namespace resident_collection_test
