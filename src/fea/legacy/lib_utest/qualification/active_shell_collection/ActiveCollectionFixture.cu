#include "ActiveCollectionFixture.h"
#include "lib_src/elements/qeph/QephLayeredJ2.h"
#include "lib_src/elements/t3/T3LayeredJ2.h"

namespace active_shell_test {
namespace {
__global__ void AddPulse(fe::NodalAssemblyView view) {
  for(std::size_t n=threadIdx.x;n<view.accepted.node_count;n+=blockDim.x) {
    const double x=view.accepted.position_xyz[3*n];
    const double corner=x-2*floor(x/2);
    view.forces.force_x[n]+=(.004/(.5*H*H))*corner/view.mass.inverse_mass[n];
  }
}
__global__ void CollapseNode(fe::NodalPreparedView view,std::size_t target,std::size_t source) {
  for(unsigned a=0;a<3;++a)
    const_cast<double*>(view.kinematics.position_xyz)[3*target+a]=view.kinematics.position_xyz[3*source+a];
}
bool Endpoint(const fe::NodalPreparedView& v,Snapshot& out) {
  const auto bytes=3*NodeCount*sizeof(double);
  EXPECT_EQ(cudaMemcpyAsync(out.x.data(),v.kinematics.position_xyz,bytes,cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(out.v.data(),v.kinematics.velocity_xyz,bytes,cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(out.w.data(),v.kinematics.angular_velocity_xyz,bytes,cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(out.orientation.data(),v.kinematics.orientation_wxyz,4*NodeCount*sizeof(double),cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess); return !::testing::Test::HasFailure();
}
}
Rig::Rig() {
  // Reserve the final square for the final Q4 only. Other coverage/materials
  // stay assembly-sized; the final T3 alone owns the two high-ID tail nodes.
  auto relocate=[&](std::size_t e,std::size_t first) {
    auto& input=source.q[e]; input.nodes={first,first+1,first+2,first+3};
    for(unsigned n=0;n<4;++n) {
      input.reference.node_ids[n]=host_shell_test::Fixture::NodeId(input.nodes[n]);
      input.reference.position[n]=host_shell_test::Fixture::Position(input.nodes[n]);
    }
  };
  for(std::size_t e=256;e<QCount;e+=257) relocate(e,0);
  relocate(QCount-1,1024);
}
bool Rig::BuildReference() {
  const auto geometry=binding.Initialize(source.geometry(),fe::ShellHostBindingLimits{});
  EXPECT_EQ(geometry.status,fe::ShellBindingStatus::Success)<<geometry.message;
  if(geometry.status!=fe::ShellBindingStatus::Success) return false;
  const auto material=catalog.Initialize(binding,source.catalog(),fe::ShellHostBindingLimits{});
  EXPECT_EQ(material.status,fe::ShellPlasticityBindingStatus::Success)<<material.message;
  if(material.status!=fe::ShellPlasticityBindingStatus::Success) return false;
  for(std::size_t n=0;n<NodeCount;++n) {
    const auto& node=binding.nodes()[n];
    initial.x[3*n]=node.position.x; initial.x[3*n+1]=node.position.y; initial.x[3*n+2]=node.position.z;
    initial.v[3*n]=2; initial.orientation[4*n]=1;
    inverse[n]=1/node.native.mass; inverse_j[n]=1/node.native.isotropic_inertia;
  }
  fe::NodalStateConfig c; c.node_count=NodeCount; c.fixed_dt=H;
  c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto state=owner.Initialize(c,{initial.x.data(),initial.v.data(),initial.w.data(),NodeCount,initial.orientation.data()},
      inverse.data(),{fixed.data(),fixed.data(),inverse_j.data()});
  EXPECT_EQ(state.status,fe::NodalStatus::Ok)<<state.message; return state.status==fe::NodalStatus::Ok;
}
q::QephBatchConfig Rig::QConfig() const {
  q::QephBatchConfig c; c.owner=owner.accepted(); c.element_count=QCount;
  c.configuration_id=Configuration; c.qualification_id=Qualification; c.usage=q::BatchUsage::CoupledForces;
  c.storage_limits.max_parents=1024; c.storage_limits.max_nodes=2048; c.max_device_bytes=4*1024*1024;
  c.startup.kind=fe::ShellBatchStartupKind::ReferenceUniformTranslation; c.startup.uniform_velocity={2,0,0}; return c;
}
t::T3BatchConfig Rig::TConfig() const {
  t::T3BatchConfig c; c.owner=owner.accepted(); c.element_count=TCount;
  c.configuration_id=Configuration; c.qualification_id=Qualification; c.usage=t::BatchUsage::CoupledForces;
  c.storage_limits.max_parents=1024; c.storage_limits.max_nodes=2048; c.max_device_bytes=4*1024*1024;
  c.startup.kind=fe::ShellBatchStartupKind::ReferenceUniformTranslation; c.startup.uniform_velocity={2,0,0}; return c;
}
bool Rig::Initialize(bool with_plastic) {
  plastic=with_plastic; if(!BuildReference()) return false;
  const auto qr=plastic?qeph.InitializeJoined(QConfig(),binding,catalog):qeph.InitializeJoined(QConfig(),binding);
  const auto tr=plastic?t3.InitializeJoined(TConfig(),binding,catalog):t3.InitializeJoined(TConfig(),binding);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message; EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  if(qr.status!=q::BatchStatus::Success||tr.status!=t::BatchStatus::Success) return false;
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  EXPECT_EQ(owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  const auto bound_q=qeph.AssembleAccepted(owner,view);
  EXPECT_EQ(bound_q.status,q::BatchStatus::Success)<<bound_q.message;
  const auto bound_t=t3.AssembleAccepted(owner,view);
  EXPECT_EQ(bound_t.status,t::BatchStatus::Success)<<bound_t.message; Discard();
  if(::testing::Test::HasFailure()) return false;
  fe::ShellPublicationLimits limits; limits.max_nodes=2048;
  const auto joined=publication.Initialize(owner,qeph,t3,limits);
  EXPECT_EQ(joined.status,fe::ShellPublicationStatus::Success)<<joined.message; return joined.status==fe::ShellPublicationStatus::Success;
}
bool Read(Rig& r,Snapshot& out) {
  const auto result=r.owner.CopyAccepted(out.buffer(),&out.stamp);
  EXPECT_EQ(result.status,fe::NodalStatus::Ok); return result.status==fe::NodalStatus::Ok;
}
bool Accepted(Rig& r,Results& out) {
  const auto stamp=r.owner.accepted();
  EXPECT_EQ(r.qeph.CopyAcceptedResults(stamp,out.qr.data(),QCount,&out.diagnostics.qeph).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.CopyAcceptedResults(stamp,out.tr.data(),TCount,&out.diagnostics.t3).status,t::BatchStatus::Success);
  if(r.plastic) {
    EXPECT_EQ(r.qeph.CopyAcceptedSectionHistory(stamp,out.qs.data(),QCount,&out.diagnostics.qeph).status,q::BatchStatus::Success);
    EXPECT_EQ(r.t3.CopyAcceptedSectionHistory(stamp,out.ts.data(),TCount,&out.diagnostics.t3).status,t::BatchStatus::Success);
  }
  EXPECT_EQ(r.publication.CopyAcceptedDiagnostics(stamp,&out.diagnostics).status,fe::ShellPublicationStatus::Success);
  return !::testing::Test::HasFailure();
}
bool Prepare(Rig& r,Prepared& p,bool pulse) {
  fe::NodalAssemblyView view;
  EXPECT_EQ(r.owner.BeginTrial(&p.token,&view).status,fe::NodalStatus::Ok);
  if(pulse) AddPulse<<<1,64,0,view.stream>>>(view);
  EXPECT_EQ(r.qeph.AssembleAccepted(view).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.AssembleAccepted(view).status,t::BatchStatus::Success);
  EXPECT_EQ(r.owner.SealAssembly(p.token).status,fe::NodalStatus::Ok);
  EXPECT_EQ(fe::AdvanceStaggeredHistory(r.owner,p.token,
      {view.owner_id,view.accepted.base_epoch,view.attempt,H,1,Qualification}).status,fe::NodalStatus::Ok);
  EXPECT_EQ(r.owner.BorrowPrepared(p.token,&p.view).status,fe::NodalStatus::Ok);
  return !::testing::Test::HasFailure()&&Endpoint(p.view,p.endpoint);
}
bool Evaluate(Rig& r,Prepared& p,Results& out) {
  EXPECT_EQ(r.qeph.EvaluateCandidate(p.view,&out.diagnostics.qeph).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.EvaluateCandidate(p.view,&out.diagnostics.t3).status,t::BatchStatus::Success);
  if(::testing::Test::HasFailure()) return false;
  EXPECT_EQ(r.qeph.CopyPreparedResults(out.diagnostics.qeph,out.qr.data(),QCount).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.CopyPreparedResults(out.diagnostics.t3,out.tr.data(),TCount).status,t::BatchStatus::Success);
  if(r.plastic) {
    EXPECT_EQ(r.qeph.CopyPreparedSectionHistory(out.diagnostics.qeph,out.qs.data(),QCount).status,q::BatchStatus::Success);
    EXPECT_EQ(r.t3.CopyPreparedSectionHistory(out.diagnostics.t3,out.ts.data(),TCount).status,t::BatchStatus::Success);
  }
  fe::ShellBatchDiagnostics common;
  EXPECT_EQ(r.publication.Prepare(r.owner,p.token,out.diagnostics.qeph,out.diagnostics.t3,&common).status,fe::ShellPublicationStatus::Success);
  out.diagnostics=common; return !::testing::Test::HasFailure();
}
bool Publish(Rig& r,const Prepared& p,const Results& next) {
  const auto& d=next.diagnostics.qeph;
  const auto report=r.publication.Commit(r.owner,p.token,next.diagnostics,{d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true});
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message; return report.status==fe::ShellPublicationStatus::Success;
}
void Collapse(const Prepared& p,std::size_t target,std::size_t source) {
  CollapseNode<<<1,1,0,p.view.stream>>>(p.view,target,source); ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
}
void SameResults(const Results& a,const Results& b,bool plastic) {
  EXPECT_EQ(std::memcmp(a.qr.data(),b.qr.data(),QCount*sizeof(q::ForceTrial)),0);
  EXPECT_EQ(std::memcmp(a.tr.data(),b.tr.data(),TCount*sizeof(t::ForceTrial)),0);
  if(plastic) {
    EXPECT_EQ(std::memcmp(a.qs.data(),b.qs.data(),QCount*sizeof(fe::ShellBatchSectionState)),0);
    EXPECT_EQ(std::memcmp(a.ts.data(),b.ts.data(),TCount*sizeof(fe::ShellBatchSectionState)),0);
  }
}
}
