#include "MixedShellFixture.h"
#include <iomanip>
#include <sstream>

namespace mixed_shell_test {
bool Rig::PrepareReference() {
  const auto built=binding.Initialize(input);
  EXPECT_EQ(built.status,fe::ShellBindingStatus::Success)<<built.message;
  if(built.status!=fe::ShellBindingStatus::Success) return false;
  EXPECT_EQ(binding.node_count(),Nodes);
  if(binding.node_count()!=Nodes) return false;
  initial.n=Nodes; initial.h=H;
  for(unsigned n=0;n<Nodes;++n) {
    const auto& node=binding.nodes()[n];
    initial.x[3*n]=node.position.x; initial.x[3*n+1]=node.position.y; initial.x[3*n+2]=node.position.z;
    initial.inverse[n]=1/node.native.mass; initial.inverse_inertia[n]=1/node.native.isotropic_inertia;
  }
  return true;
}
bool Rig::Initialize() {
  if(!PrepareReference()) return false;
  const auto state=initial.Initialize(owner);
  EXPECT_EQ(state.status,fe::NodalStatus::Ok)<<state.message;
  if(state.status!=fe::NodalStatus::Ok) return false;
  return InitializeParticipants();
}
bool Rig::InitializeParticipants() {
  q::QephBatchConfig qc; qc.owner=owner.accepted(); qc.element_count=1;
  qc.configuration_id=Configuration; qc.qualification_id=Qualification; qc.usage=q::BatchUsage::PrescribedFields;
  const auto qr=qeph.InitializeJoined(qc,binding);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  if(qr.status!=q::BatchStatus::Success) return false;
  t::T3BatchConfig tc; tc.owner=owner.accepted(); tc.element_count=1;
  tc.configuration_id=Configuration; tc.qualification_id=Qualification; tc.usage=t::BatchUsage::PrescribedFields;
  const auto tr=t3.InitializeJoined(tc,binding);
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  return tr.status==t::BatchStatus::Success;
}
bool Rig::Bind() {
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  const auto begin=owner.BeginTrial(&token,&view);
  EXPECT_EQ(begin.status,fe::NodalStatus::Ok);
  if(begin.status!=fe::NodalStatus::Ok) return false;
  const auto qr=qeph.AssembleAccepted(view); const auto tr=t3.AssembleAccepted(view);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  owner.Discard(); qeph.DiscardTrial(); t3.DiscardTrial();
  if(qr.status!=q::BatchStatus::Success || tr.status!=t::BatchStatus::Success) return false;
  const auto joined=publication.Initialize(qeph,t3);
  EXPECT_EQ(joined.status,fe::ShellPublicationStatus::Success)<<joined.message;
  return joined.status==fe::ShellPublicationStatus::Success;
}
Loads Schedule(const Rig& r,unsigned interval) {
  Loads result;
  if(interval>=4) { ADD_FAILURE()<<"Unknown frozen mixed schedule interval"; return result; }
  const double previous=interval?Targets[interval-1]:0,kick=interval?H:.5*H;
  const double change=(Targets[interval]-previous)/kick;
  for(unsigned n=0;n<Nodes;++n) {
    const double x=r.initial.x[3*n],y=r.initial.x[3*n+1];
    const double velocity[]{.001*(x+.25*y),.0005*(y-.5*x),.00075*(x-y)};
    const double omega[]{.002*y,.003*x,.001*(x+y)};
    for(unsigned a=0;a<3;++a) {
      result.force[3*n+a]=r.binding.nodes()[n].native.mass*change*velocity[a];
      result.couple[3*n+a]=r.binding.nodes()[n].native.isotropic_inertia*change*omega[a];
    }
  }
  return result;
}
bool Endpoint(const fe::NodalPreparedView& p,Snapshot& output) {
  Snapshot staged;
  const double* source[]{p.kinematics.position_xyz,p.kinematics.velocity_xyz,
                         p.kinematics.angular_velocity_xyz,p.kinematics.orientation_wxyz};
  double* destination[]{staged.x.data(),staged.v.data(),staged.omega.data(),staged.q.data()};
  for(unsigned field=0;field<4;++field) {
    const auto status=cudaMemcpyAsync(destination[field],source[field],(field==3?4:3)*Nodes*sizeof(double),
                                      cudaMemcpyDeviceToHost,p.stream);
    EXPECT_EQ(status,cudaSuccess); if(status!=cudaSuccess) return false;
  }
  const auto status=cudaStreamSynchronize(p.stream);
  EXPECT_EQ(status,cudaSuccess); if(status!=cudaSuccess) return false;
  output=staged; return true;
}
bool Prepare(Rig& r,const Loads& load,Prepared& output) {
  Prepared staged; staged.load=load; fe::NodalAssemblyView view;
  if(!temporal::BeginLoad(r.owner,load,staged.token,view)) { r.Discard(); return false; }
  auto report=r.owner.SealAssembly(staged.token);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok);
  if(report.status!=fe::NodalStatus::Ok) { r.Discard(); return false; }
  // PrescribedFields history transactions need their explicit scoped receipt;
  // no accepted shell cache is consumed in this dynamics-free qualification.
  report=fe::AdvanceStaggeredHistory(r.owner,staged.token,
      {view.owner_id,view.accepted.base_epoch,view.attempt,H,1.,Qualification});
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  if(report.status!=fe::NodalStatus::Ok) { r.Discard(); return false; }
  report=r.owner.BorrowPrepared(staged.token,&staged.view);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok);
  if(report.status!=fe::NodalStatus::Ok || !Endpoint(staged.view,staged.endpoint)) { r.Discard(); return false; }
  output=staged; return true;
}
bool Evaluate(Rig& r,const Prepared& p,Staged& output,bool t3_first) {
  Staged staged;
  auto qeph=[&]() {
    auto report=r.qeph.EvaluateCandidate(p.view,&staged.diagnostics.qeph);
    EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message;
    if(report.status!=q::BatchStatus::Success) return false;
    report=r.qeph.CopyPreparedResults(staged.diagnostics.qeph,&staged.qeph,1);
    EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message;
    return report.status==q::BatchStatus::Success;
  };
  auto t3=[&]() {
    auto report=r.t3.EvaluateCandidate(p.view,&staged.diagnostics.t3);
    EXPECT_EQ(report.status,t::BatchStatus::Success)<<report.message;
    if(report.status!=t::BatchStatus::Success) return false;
    report=r.t3.CopyPreparedResults(staged.diagnostics.t3,&staged.t3,1);
    EXPECT_EQ(report.status,t::BatchStatus::Success)<<report.message;
    return report.status==t::BatchStatus::Success;
  };
  if(!(t3_first?(t3()&&qeph()):(qeph()&&t3()))) { r.Discard(); return false; }
  fe::ShellBatchDiagnostics measured;
  const auto report=r.publication.Prepare(r.owner,p.token,staged.diagnostics.qeph,staged.diagnostics.t3,&measured);
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;
  if(report.status!=fe::ShellPublicationStatus::Success) { r.Discard(); return false; }
  staged.diagnostics=measured; output=staged; return true;
}
bool Accepted(Rig& r,Staged& output) {
  Staged staged;
  const auto qr=r.qeph.CopyAcceptedResults(r.owner.accepted(),&staged.qeph,1,&staged.diagnostics.qeph);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  if(qr.status!=q::BatchStatus::Success) return false;
  const auto tr=r.t3.CopyAcceptedResults(r.owner.accepted(),&staged.t3,1,&staged.diagnostics.t3);
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  if(tr.status!=t::BatchStatus::Success) return false;
  output=staged; return true;
}
fe::NodalValidationReceipt Receipt(const Staged& s) {
  const auto& d=s.diagnostics.qeph; return {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true};
}
bool Publish(Rig& r,const Prepared& p,const Staged& s) {
  Identity(r,p,s);
  if(::testing::Test::HasFailure()) { r.Discard(); return false; }
  const auto report=r.publication.Commit(r.owner,p.token,s.diagnostics,Receipt(s));
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success)<<report.message;
  return report.status==fe::ShellPublicationStatus::Success;
}
q::PrescribedInterval QephInterval(const Rig& r,const Prepared& p) {
  q::PrescribedInterval in; in.base_time=p.view.base_time; in.dt=H; in.sample_index=p.view.kinematics.base_epoch+1;
  for(unsigned i=0;i<4;++i) {
    const auto n=r.binding.qeph_nodes()[i];
    in.position_endpoint[i]={p.endpoint.x[3*n],p.endpoint.x[3*n+1],p.endpoint.x[3*n+2]};
    in.velocity_midpoint[i]={p.endpoint.v[3*n],p.endpoint.v[3*n+1],p.endpoint.v[3*n+2]};
    in.omega_midpoint[i]={p.endpoint.omega[3*n],p.endpoint.omega[3*n+1],p.endpoint.omega[3*n+2]};
  }
  return in;
}
t::PrescribedInterval T3Interval(const Rig& r,const Prepared& p) {
  t::PrescribedInterval in; in.base_time=p.view.base_time; in.dt=H; in.sample_index=p.view.kinematics.base_epoch+1;
  for(unsigned i=0;i<3;++i) {
    const auto n=r.binding.t3_nodes()[i];
    in.position[i]={p.endpoint.x[3*n],p.endpoint.x[3*n+1],p.endpoint.x[3*n+2]};
    in.velocity[i]={p.endpoint.v[3*n],p.endpoint.v[3*n+1],p.endpoint.v[3*n+2]};
    in.angular_velocity[i]={p.endpoint.omega[3*n],p.endpoint.omega[3*n+1],p.endpoint.omega[3*n+2]};
  }
  return in;
}
void Identity(const Rig& r,const Prepared& p,const Staged& s) {
  EXPECT_TRUE(s.diagnostics.valid);
  auto check=[&](const auto& d) {
    EXPECT_TRUE(d.valid); EXPECT_TRUE(d.has_completed_interval); EXPECT_FALSE(d.accepted_force_assembled);
    EXPECT_FALSE(d.kinetic_available); EXPECT_EQ(d.kinetic_translation,0); EXPECT_EQ(d.kinetic_rotation,0);
    EXPECT_EQ(d.kinetic_physical_isotropic,0); EXPECT_EQ(d.kinetic_added_isotropic,0);
    EXPECT_EQ(d.owner_id,r.owner.accepted().owner_id); EXPECT_EQ(d.configuration_id,Configuration);
    EXPECT_EQ(d.qualification_id,Qualification); EXPECT_EQ(d.base_epoch,r.owner.accepted().epoch);
    EXPECT_EQ(d.epoch,d.base_epoch+1); EXPECT_EQ(d.attempt,p.view.attempt);
    EXPECT_EQ(d.base_time,p.view.base_time); EXPECT_EQ(d.time,p.view.proposed_time);
    EXPECT_EQ(d.base_velocity_time,p.view.base_velocity_time); EXPECT_EQ(d.velocity_time,p.view.velocity_time);
    EXPECT_EQ(d.kick_dt,p.view.kick_dt); EXPECT_EQ(d.kick_dt,d.base_epoch?H:.5*H);
  };
  check(s.diagnostics.qeph); check(s.diagnostics.t3);
  EXPECT_EQ(s.diagnostics.qeph.phase,q::BatchPhase::Prepared); EXPECT_EQ(s.diagnostics.t3.phase,t::BatchPhase::Prepared);
  EXPECT_EQ(s.diagnostics.qeph.usage,q::BatchUsage::PrescribedFields); EXPECT_EQ(s.diagnostics.t3.usage,t::BatchUsage::PrescribedFields);
}
void ExactResults(const Staged& a,const Staged& b) {
  EXPECT_EQ(Bytes(a.qeph),Bytes(b.qeph)); to::Exact(a.t3,b.t3);
}
bool NativePair::Initialize(const Rig& r) {
  auto qr=qnative::Initialize(qeph_startup_test::NativeInput(r.binding.qeph_reference().input),qeph);
  EXPECT_EQ(qr,qnative::Status::kSuccess); if(qr!=qnative::Status::kSuccess) return false;
  qr=qnative::InitializeHistory(qeph,{},qeph_history);
  EXPECT_EQ(qr,qnative::Status::kSuccess); if(qr!=qnative::Status::kSuccess) return false;
  auto tr=tnative::Initialize(to::Native(r.binding.t3_reference().input),t3);
  EXPECT_EQ(tr,tnative::Status::kSuccess); if(tr!=tnative::Status::kSuccess) return false;
  tr=tnative::InitializeHistory(t3,{},t3_history);
  EXPECT_EQ(tr,tnative::Status::kSuccess); if(tr!=tnative::Status::kSuccess) return false;
  qeph_startup_test::Agreement(r.binding.qeph_reference(),qeph.data());
  to::StartupAgreement(r.binding.t3_reference(),t3);
  return true;
}
bool NativePair::Check(const Rig& r,const Prepared& p,const Staged& s,NativeTrials& output) const {
  NativeTrials staged;
  const auto qi=QephInterval(r,p); const auto ti=T3Interval(r,p);
  const auto qr=qnative::EvaluateForce(qeph,qeph_history,qeph_kinematics_test::NativeInterval(qi),staged.qeph);
  EXPECT_EQ(qr,qnative::Status::kSuccess); if(qr!=qnative::Status::kSuccess) return false;
  const auto tr=tnative::EvaluateForce(t3,t3_history,to::Native(ti),staged.t3);
  EXPECT_EQ(tr,tnative::Status::kSuccess); if(tr!=tnative::Status::kSuccess) return false;
  qo::ForceAgreement(s.qeph,staged.qeph,r.binding.qeph_reference().input,qi); qo::Balance(s.qeph,qi);
  to::Agreement(r.binding.t3_reference(),ti,s.t3,staged.t3);
  to::oracle::Check(t3,t3_history.data(),to::Native(ti),to::Native(t3,s.t3));
  to::Power(r.binding.t3_reference(),ti,s.t3);
  output=staged; return true;
}
void Property(const std::string& name,double value) {
  std::ostringstream out; out<<std::setprecision(std::numeric_limits<double>::max_digits10)<<value;
  ::testing::Test::RecordProperty(name,out.str());
}
} // namespace mixed_shell_test
