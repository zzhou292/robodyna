#include "VehicleResidentFixture.h"
namespace vehicle_resident_test {
namespace {
__global__ void Pulse(fe::NodalAssemblyView v) {
  for(std::size_t n=threadIdx.x;n<v.accepted.node_count;n+=blockDim.x){const auto x=v.accepted.position_xyz[3*n];
    const double corner=x-2*floor(x/2);v.forces.force_x[n]+=(.04/(.5*H*H))*corner/v.mass.inverse_mass[n];}
}
__global__ void CollapseNode(fe::NodalPreparedView v,std::size_t target,std::size_t source) {
  for(unsigned a=0;a<3;++a)const_cast<double*>(v.kinematics.position_xyz)[3*target+a]=v.kinematics.position_xyz[3*source+a];
}
}
bool Prepare(Rig& r,Prepared& p,bool pulse) {
  fe::NodalAssemblyView view;EXPECT_EQ(r.owner.BeginTrial(&p.token,&view).status,fe::NodalStatus::Ok);
  if(pulse)Pulse<<<1,64,0,view.stream>>>(view);
  EXPECT_EQ(r.qeph.AssembleAccepted(r.owner,view).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.AssembleAccepted(r.owner,view).status,t::BatchStatus::Success);
  EXPECT_EQ(r.owner.SealAssembly(p.token).status,fe::NodalStatus::Ok);
  EXPECT_EQ(fe::AdvanceStaggeredHistory(r.owner,p.token,{view.owner_id,view.accepted.base_epoch,view.attempt,H,1,Qualification}).status,fe::NodalStatus::Ok);
  EXPECT_EQ(r.owner.BorrowPrepared(p.token,&p.view).status,fe::NodalStatus::Ok);
  fe::NodalPreparedView copied;
  EXPECT_EQ(r.owner.CopyPrepared(p.token,p.endpoint.buffer(),&copied).status,fe::NodalStatus::Ok);
  EXPECT_TRUE(fe::trial_identity::SamePrepared(p.view,copied));return !::testing::Test::HasFailure();
}
bool Evaluate(Rig& r,Prepared& p,Results& out) {
  EXPECT_EQ(r.qeph.EvaluateCandidate(r.owner,p.token,p.view,&out.diagnostics.qeph).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.EvaluateCandidate(r.owner,p.token,p.view,&out.diagnostics.t3).status,t::BatchStatus::Success);
  if(::testing::Test::HasFailure())return false;
  EXPECT_EQ(r.qeph.CopyPreparedResults(out.diagnostics.qeph,out.qr.data(),r.nq).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.CopyPreparedResults(out.diagnostics.t3,out.tr.data(),r.nt).status,t::BatchStatus::Success);
  if(r.plastic){
    EXPECT_EQ(r.qeph.CopyPreparedSectionHistory(out.diagnostics.qeph,out.qs.data(),r.nq).status,q::BatchStatus::Success);
    EXPECT_EQ(r.t3.CopyPreparedSectionHistory(out.diagnostics.t3,out.ts.data(),r.nt).status,t::BatchStatus::Success);}
  fe::ShellBatchDiagnostics common;
  EXPECT_EQ(r.publication.Prepare(r.owner,p.token,out.diagnostics.qeph,out.diagnostics.t3,&common).status,fe::ShellPublicationStatus::Success);
  out.diagnostics=common;return !::testing::Test::HasFailure();
}
bool Publish(Rig& r,const Prepared& p,const Results& next) {
  const auto& d=next.diagnostics.qeph;const auto report=r.publication.Commit(r.owner,p.token,next.diagnostics,
    {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true});
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;return report.status==fe::ShellPublicationStatus::Success;
}
void Collapse(const Prepared& p,std::size_t target,std::size_t source) {
  CollapseNode<<<1,1,0,p.view.stream>>>(p.view,target,source);ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
}
} // namespace vehicle_resident_test
