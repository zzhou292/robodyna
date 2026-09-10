#include "MixedMovingStartupFixture.h"

namespace mixed_moving_test {
bool MovingReference(Rig& r) {
  if(!r.PrepareReference()) return false;
  for(unsigned n=0;n<Nodes;++n) {
    r.initial.v[3*n]=Velocity.x; r.initial.v[3*n+1]=Velocity.y; r.initial.v[3*n+2]=Velocity.z;
  }
  return true;
}
q::QephBatchConfig QConfig(const Rig& r) {
  q::QephBatchConfig c; c.owner=r.owner.accepted(); c.element_count=1;
  c.configuration_id=FeedbackConfiguration; c.qualification_id=FeedbackQualification;
  c.usage=q::BatchUsage::CoupledForces; c.startup=MovingStartup(); return c;
}
t::T3BatchConfig TConfig(const Rig& r) {
  t::T3BatchConfig c; c.owner=r.owner.accepted(); c.element_count=1;
  c.configuration_id=FeedbackConfiguration; c.qualification_id=FeedbackQualification;
  c.usage=t::BatchUsage::CoupledForces; c.startup=MovingStartup(); return c;
}
bool Participants(Rig& r) {
  const auto qr=r.qeph.InitializeJoined(QConfig(r),r.binding);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  if(qr.status!=q::BatchStatus::Success) return false;
  const auto tr=r.t3.InitializeJoined(TConfig(r),r.binding);
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message; return tr.status==t::BatchStatus::Success;
}
bool BindMoving(Rig& r) {
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  EXPECT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  EXPECT_EQ(r.qeph.AssembleAccepted(r.owner,view).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.AssembleAccepted(r.owner,view).status,t::BatchStatus::Success);
  r.Discard();
  if(::testing::Test::HasFailure()) return false;
  const auto result=r.publication.Initialize(r.owner,r.qeph,r.t3);
  EXPECT_EQ(result.status,fe::ShellPublicationStatus::Success)<<result.message;
  return result.status==fe::ShellPublicationStatus::Success;
}
bool InitializeMoving(Rig& r) {
  if(!MovingReference(r)) return false;
  const auto report=r.initial.Initialize(r.owner);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok);
  return report.status==fe::NodalStatus::Ok&&Participants(r)&&BindMoving(r);
}
void InitialKinetic(const Rig& r,const Staged& cache,const fe::ShellBatchDiagnostics& common) {
  const auto truth=shell_binding_test::Truth(r.input);
  long double expected=0;
  for(unsigned n=0;n<Nodes;++n) for(unsigned a=0;a<3;++a) {
    const long double v=r.initial.v[3*n+a]; expected+=.5L*truth[n].mass*v*v;
  }
  EXPECT_TRUE(common.valid); EXPECT_GT(common.kinetic.translation,0);
  EXPECT_LE(std::abs(common.kinetic.translation-expected),256*std::numeric_limits<double>::epsilon()*std::abs(expected)+1e-18L);
  EXPECT_EQ(common.kinetic.rotation,0); EXPECT_EQ(common.kinetic.physical_isotropic,0); EXPECT_EQ(common.kinetic.added_isotropic,0);
  EXPECT_EQ(common.base_kinetic.translation,0); EXPECT_EQ(common.base_kinetic.rotation,0);
  auto check=[&](const auto& d) {
    EXPECT_FALSE(d.kinetic_available); EXPECT_EQ(d.kinetic_translation,0); EXPECT_EQ(d.kinetic_rotation,0);
    EXPECT_EQ(d.kinetic_physical_isotropic,0); EXPECT_EQ(d.kinetic_added_isotropic,0);
    EXPECT_EQ(d.epoch,0u); EXPECT_EQ(d.time,0); EXPECT_EQ(d.velocity_time,0); EXPECT_EQ(d.kick_dt,0);
    EXPECT_FALSE(d.has_completed_interval); EXPECT_FALSE(d.accepted_force_assembled);
    for(double value:d.internal_work) EXPECT_EQ(value,0);
  };
  check(cache.diagnostics.qeph); check(cache.diagnostics.t3);
  check(common.qeph); check(common.t3);
  auto zero=[&](const auto& result) {
    EXPECT_EQ(result.proposed_history.stamp().sample_index,0u); EXPECT_EQ(result.proposed_history.stamp().time,0);
    for(const auto f:result.internal_force) { EXPECT_EQ(f.x,0); EXPECT_EQ(f.y,0); EXPECT_EQ(f.z,0); }
    for(const auto m:result.internal_couple) { EXPECT_EQ(m.x,0); EXPECT_EQ(m.y,0); EXPECT_EQ(m.z,0); }
    for(double value:result.proposed_history.data().strain_curvature) EXPECT_EQ(value,0);
  };
  zero(cache.qeph); zero(cache.t3);
}
} // namespace mixed_moving_test
