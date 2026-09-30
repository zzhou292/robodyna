#include "WallCoupledFixture.h"

namespace qeph_wall_test {
namespace {
bool ReadAssembly(const Rig& r,const fe::NodalAssemblyView& v,Loads& out) {
  const double* source[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,
                         v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  std::array<double,N> stage{};
  for(unsigned c=0;c<6;++c) {
    const auto err=cudaMemcpy(stage.data(),source[c],r.n*sizeof(double),cudaMemcpyDeviceToHost);
    EXPECT_EQ(err,cudaSuccess); if(err!=cudaSuccess) return false;
    for(unsigned n=0;n<r.n;++n) (c<3?out.force:out.couple)[3*n+c%3]=stage[n];
  }
  return true;
}
bool ReadPrepared(const Rig& r,const fe::NodalPreparedView& v,Snapshot& out) {
  const double* source[]{v.kinematics.position_xyz,v.kinematics.velocity_xyz,
                         v.kinematics.angular_velocity_xyz,v.kinematics.orientation_wxyz};
  double* destination[]{out.x.data(),out.v.data(),out.omega.data(),out.q.data()};
  for(unsigned i=0;i<4;++i) {
    const auto err=cudaMemcpy(destination[i],source[i],(i==3?4:3)*r.n*sizeof(double),cudaMemcpyDeviceToHost);
    EXPECT_EQ(err,cudaSuccess); if(err!=cudaSuccess) return false;
  }
  return true;
}
}
bool Begin(WallRig& w,Trial& output) {
  auto& r=w.shell; Trial p;
  fe::NodalAssemblyView assembly;
  auto code=r.owner.BeginTrial(&p.nodal.token,&assembly);
  EXPECT_EQ(code.status,fe::NodalStatus::Ok); if(code.status!=fe::NodalStatus::Ok) return false;
  const auto shell=w.experiment.startup.kind==q::BatchStartupKind::ReferenceRest?
    r.batch.AssembleAccepted(assembly):r.batch.AssembleAccepted(r.owner,assembly);
  EXPECT_EQ(shell.status,q::BatchStatus::Success)<<shell.message;
  if(shell.status!=q::BatchStatus::Success) { w.Discard(); return false; }
  auto contact=w.wall.AssembleAccepted(assembly,&p.contact_base);
  EXPECT_EQ(contact.status,sc::NodalWallDeviceStatus::Ok)<<contact.message;
  if(contact.status!=sc::NodalWallDeviceStatus::Ok) { w.Discard(); return false; }
  contact=w.wall.CopyResults(p.contact_base,&p.contact_base_result);
  EXPECT_EQ(contact.status,sc::NodalWallDeviceStatus::Ok)<<contact.message;
  if(contact.status!=sc::NodalWallDeviceStatus::Ok || !ReadAssembly(r,assembly,p.nodal.assembled)) {
    w.Discard(); return false;
  }
  code=r.owner.SealAssembly(p.nodal.token);
  EXPECT_EQ(code.status,fe::NodalStatus::Ok); if(code.status!=fe::NodalStatus::Ok) { w.Discard(); return false; }
  // Scoped named short recurrence receipt, never a prescribed-force declaration
  // or reuse of the free-response matrix as a contact stability theorem.
  code=fe::AdvanceStaggeredHistory(r.owner,p.nodal.token,
    {assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,r.h,w.experiment.maximum_rotation_increment,w.experiment.qualification});
  EXPECT_EQ(code.status,fe::NodalStatus::Ok)<<code.message;
  if(code.status!=fe::NodalStatus::Ok) { w.Discard(); return false; }
  code=r.owner.BorrowPrepared(p.nodal.token,&p.nodal.view);
  EXPECT_EQ(code.status,fe::NodalStatus::Ok);
  if(code.status!=fe::NodalStatus::Ok || !ReadPrepared(r,p.nodal.view,p.nodal.state)) { w.Discard(); return false; }
  output=p; return true;
}
bool Evaluate(WallRig& w,const Trial& t,Staged& output,bool contact_first) {
  Staged stage;
  auto contact=[&]() {
    auto c=w.wall.EvaluateCandidate(t.nodal.view,&stage.contact);
    EXPECT_EQ(c.status,sc::NodalWallDeviceStatus::Ok)<<c.message;
    if(c.status!=sc::NodalWallDeviceStatus::Ok) return false;
    c=w.wall.CopyResults(stage.contact,&stage.contact_result);
    EXPECT_EQ(c.status,sc::NodalWallDeviceStatus::Ok)<<c.message;
    return c.status==sc::NodalWallDeviceStatus::Ok;
  };
  auto shell=[&]() { return Candidate(w.shell,t.nodal.view,stage.shell,stage.elements); };
  if(!(contact_first?(contact()&&shell()):(shell()&&contact()))) { w.Discard(); return false; }
  output=stage; return true;
}
void Identity(const WallRig& w,const Trial& t,const Staged& s) {
  const auto& r=w.shell; const auto& p=t.nodal.view;
  EXPECT_TRUE(s.shell.valid); EXPECT_TRUE(s.shell.has_completed_interval); EXPECT_TRUE(s.shell.accepted_force_assembled);
  EXPECT_EQ(s.shell.usage,q::BatchUsage::CoupledForces); EXPECT_EQ(s.shell.phase,q::BatchPhase::Prepared);
  EXPECT_EQ(s.shell.configuration_id,w.experiment.shell_configuration); EXPECT_EQ(s.shell.qualification_id,w.experiment.qualification);
  EXPECT_EQ(s.shell.owner_id,r.owner.accepted().owner_id); EXPECT_EQ(s.shell.base_epoch,r.owner.accepted().epoch);
  EXPECT_EQ(s.shell.epoch,s.shell.base_epoch+1); EXPECT_EQ(s.shell.attempt,p.attempt);
  EXPECT_EQ(s.shell.time,p.proposed_time); EXPECT_EQ(s.shell.velocity_time,p.velocity_time);
  EXPECT_EQ(s.shell.base_time,p.base_time); EXPECT_EQ(s.shell.base_velocity_time,p.base_velocity_time);
  EXPECT_EQ(s.shell.kick_dt,p.kick_dt); EXPECT_EQ(p.kick_dt,s.shell.base_epoch?r.h:.5*r.h);
  EXPECT_TRUE(t.contact_base.valid); EXPECT_TRUE(s.contact.valid);
  EXPECT_EQ(t.contact_base.phase,sc::NodalWallDevicePhase::AcceptedBase);
  EXPECT_EQ(s.contact.phase,sc::NodalWallDevicePhase::PreparedCandidate);
  for(const auto* c:{&t.contact_base,&s.contact}) {
    EXPECT_EQ(c->owner_id,s.shell.owner_id); EXPECT_EQ(c->base_epoch,s.shell.base_epoch);
    EXPECT_EQ(c->attempt,p.attempt); EXPECT_EQ(c->qualification_id,w.experiment.qualification);
    EXPECT_EQ(c->configuration_id,w.experiment.wall_configuration); EXPECT_EQ(c->wall_binding_id,w.experiment.wall_binding);
    EXPECT_EQ(c->parent_count,r.count); EXPECT_EQ(c->node_count,r.n);
  }
  EXPECT_EQ(s.contact.time,s.shell.time); EXPECT_EQ(s.contact.velocity_time,s.shell.velocity_time);
  EXPECT_EQ(s.contact.base_time,s.shell.base_time); EXPECT_EQ(s.contact.base_velocity_time,s.shell.base_velocity_time);
  EXPECT_EQ(s.contact.kick_dt,s.shell.kick_dt); EXPECT_EQ(s.contact.scheme,fe::NodalTemporalScheme::StaggeredHalfKickStart);
  EXPECT_EQ(s.contact.velocity_phase,fe::NodalVelocityPhase::PreviousMidpoint);
}
void Agreement(const WallRig& w,const Trial& t,const Staged& s,const NativeProposal& native,
               double time,std::uint64_t epoch) {
  OwnerAgreement(w.shell,t.nodal.state,native.state);
  for(unsigned e=0;e<w.shell.count;++e) {
    SCOPED_TRACE(e);
    qeph_force_port_test::ForceAgreement(s.elements[e],native.cache[e],w.shell.element[e].reference.input,
                                       Interval(w.shell,e,t.nodal.state,time,epoch));
  }
  ContactAgreement(s.contact_result,w.Host(t.nodal.state,epoch,t.nodal.view.attempt));
}
bool Publish(WallRig& w,const Trial& t,const Staged& s,sc::NodalWallDeviceResults& published) {
  Identity(w,t,s);
  if(::testing::Test::HasFailure()) { w.Discard(); return false; }
  if(!Commit(w.shell,t.nodal.token,s.shell)) { w.wall.DiscardTrial(); return false; }
  // The only postcommit publication: fixed-size host assignment, no callbacks,
  // allocation, validation or fallible device read. Retain the interval phase.
  published=s.contact_result; w.wall.DiscardTrial(); return true;
}
} // namespace qeph_wall_test
