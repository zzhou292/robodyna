#pragma once
#include "GroupObservationOracle.h"
namespace rigid_observation_test {
struct KickFixture {
  Fixture fixture;
  rigid_step_test::Input packet;
  rigid_step_test::Trial trial;
  std::array<Motion,Count> before{},after{};
  std::array<rigid::Wrench,Count> applied{},reaction{};
  Phase before_phase=InitialPhase(),after_phase=StoredPhase(1./1024,0);
  double h=1./1024;
  KickFixture() {
    const auto forces=rigid_step_test::Fixture(); const auto& g=*fixture.model.groups();
    packet.body={g.principal,g.center,{.3,-.2,.1},{},g.total_mass_kg,{}, {0,h/2,h}};
    for(unsigned i=0;i<Count;++i) {
      const auto& m=fixture.source[i];
      packet.member[i]={m.position,packet.body.velocity,{},forces.member[i].force,forces.member[i].couple,m.mass_kg,m.total_inertia_kg_m2};
    }
  }
  void SetTrial(const rigid_step_test::Trial& next) {
    trial=next;
    for(unsigned i=0;i<Count;++i) {
      before[i]={packet.member[i].velocity,packet.member[i].omega};
      after[i]={next.member[i].velocity,next.member[i].omega};
      applied[i]={packet.member[i].force,packet.member[i].couple};
      reaction[i]={next.member[i].reaction_force,next.member[i].reaction_couple};
    }
  }
  bool Solve() {
    rigid_step_test::Trial next; const auto status=rigid_step_test::EvaluatePacket(packet,next);
    EXPECT_EQ(status,rigid::StepStatus::Success); if(status!=rigid::StepStatus::Success) return false;
    SetTrial(next); return true;
  }
  rigid::GroupKickInput Input() const {
    const auto& b=packet.body; const auto& a=trial.primary;
    return {fixture.Metric(),before.data(),after.data(),{b.center,b.velocity,b.omega,b.previous_frame.axes},
      {a.center,a.velocity,a.omega,a.force_frame.axes},before_phase,after_phase,applied.data(),reaction.data(),b.durations.kick_dt};
  }
  void Carry(unsigned step) {
    const auto& p=trial.primary;
    packet.body.previous_frame=p.force_frame; packet.body.center=p.center; packet.body.velocity=p.velocity; packet.body.omega=p.omega;
    packet.body.durations={h,h,h}; before_phase=after_phase; after_phase=StoredPhase(before_phase.position_time+h,before_phase.position_time);
    const auto loads=rigid_step_test::Fixture(); const double scale=step<16?1:step<32?-.625:0;
    for(unsigned i=0;i<Count;++i) {
      packet.member[i].position=trial.member[i].position; packet.member[i].velocity=trial.member[i].velocity;
      packet.member[i].omega=trial.member[i].omega;
      const auto f=loads.member[i].force,c=loads.member[i].couple;
      packet.member[i].force={scale*f.x,scale*f.y,scale*f.z}; packet.member[i].couple={scale*c.x,scale*c.y,scale*c.z};
    }
  }
};
struct WorkOracle { long double native_delta=0,applied_translation=0,applied_rotation=0,reaction_translation=0,reaction_rotation=0; };
inline WorkOracle OracleWork(const rigid::GroupKickInput& input) {
  WorkOracle out;
  for(unsigned i=0;i<input.metric.member_count;++i) for(unsigned dof=0;dof<6;++dof) {
    const auto& m=input.metric.members[i]; const unsigned a=dof%3; const bool translation=dof<3;
    const long double coefficient=translation?m.mass_kg:m.total_inertia_kg_m2;
    const long double before=rigid_test::Get(translation?input.before_members[i].velocity:input.before_members[i].omega,a);
    const long double after=rigid_test::Get(translation?input.after_members[i].velocity:input.after_members[i].omega,a);
    const long double applied=rigid_test::Get(translation?input.applied[i].force:input.applied[i].couple,a);
    const long double reaction=rigid_test::Get(translation?input.reaction[i].force:input.reaction[i].couple,a);
    const long double average=(before+after)/2,dt=input.kick_dt;
    out.native_delta+=coefficient*(after-before)*average;
    (translation?out.applied_translation:out.applied_rotation)+=dt*applied*average;
    (translation?out.reaction_translation:out.reaction_rotation)+=dt*reaction*average;
  }
  return out;
}
inline void CompareWork(const rigid::GroupKickInput& input,const rigid::GroupKickObservation& output) {
  const auto before=Oracle({input.metric,input.before_members,input.before_group,input.before_phase});
  const auto after=Oracle({input.metric,input.after_members,input.after_group,input.after_phase});
  const auto work=OracleWork(input);
  Compare(output.before,before); Compare(output.after,after);
  const long double scale=std::max({before.translation+before.rotation,after.translation+after.rotation,1e-300L});
  const long double native_delta=work.native_delta,aggregate_delta=(after.translation-before.translation)+(after.rotation-before.rotation);
  EXPECT_NEAR(output.native_delta,native_delta,32*std::numeric_limits<double>::epsilon()*scale);
  EXPECT_NEAR(output.aggregate_delta,aggregate_delta,3e-12L*scale);
  EXPECT_NEAR(output.replacement_delta,aggregate_delta-native_delta,3e-12L*scale);
  EXPECT_NEAR(output.applied.translation,work.applied_translation,32*std::numeric_limits<double>::epsilon()*scale);
  EXPECT_NEAR(output.applied.rotation,work.applied_rotation,32*std::numeric_limits<double>::epsilon()*scale);
  EXPECT_NEAR(output.reaction.translation,work.reaction_translation,32*std::numeric_limits<double>::epsilon()*scale);
  EXPECT_NEAR(output.reaction.rotation,work.reaction_rotation,32*std::numeric_limits<double>::epsilon()*scale);
  EXPECT_LE(std::abs(output.native_residual),output.roundoff_budget);
  EXPECT_LE(std::abs(output.effective_residual),output.roundoff_budget);
}
} // namespace rigid_observation_test
