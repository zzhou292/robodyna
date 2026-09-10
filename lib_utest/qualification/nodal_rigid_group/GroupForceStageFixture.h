#pragma once
#include "GroupKickFixture.h"
#include "lib_src/constraints/NodalRigidForceStageKinetic.h"
namespace rigid_observation_test {
struct ForceStageFixture {
  KickFixture kick;
  std::array<rigid::ForceStageAcceleration,Count> acceleration{};
  rigid::GroupForceStageKineticInput Input() {
    for(unsigned i=0;i<Count;++i)acceleration[i]={kick.trial.member[i].acceleration,kick.trial.member[i].angular_acceleration};
    const auto& p=kick.packet.body;const auto& t=kick.trial.primary;
    return {kick.fixture.Metric(),kick.before.data(),acceleration.data(),{p.velocity,p.omega},
      {t.acceleration,t.angular_acceleration},t.force_frame.axes,
      {kick.before_phase.position_time,kick.before_phase.velocity_time,kick.before_phase.frame_time,p.durations}};
  }
};
inline Triple Collocated(Triple v,Triple a,long double half) {
  for(unsigned i=0;i<3;++i)v[i]+=a[i]*half;
  return v;
}
inline void CheckForceStageOracle(const rigid::GroupForceStageKineticInput& in,
                                  const rigid::GroupForceStageKineticObservation& out) {
  const long double half=static_cast<long double>(in.phase.durations.previous_drift_dt)/2;
  const auto v=Collocated(L(in.before_primary.velocity),L(in.primary_acceleration.translation),half);
  const auto w=Collocated(L(in.before_primary.omega),L(in.primary_acceleration.rotation),half);
  std::array<Motion,256> rounded{};
  long double translation=0,rotation=0;
  for(unsigned i=0;i<in.metric.member_count;++i) {
    const auto vm=Collocated(L(in.before_members[i].velocity),L(in.member_acceleration[i].translation),half);
    const auto wm=Collocated(L(in.before_members[i].omega),L(in.member_acceleration[i].rotation),half);
    translation+=.5L*in.metric.members[i].mass_kg*Norm(vm);
    rotation+=.5L*in.metric.members[i].total_inertia_kg_m2*Norm(wm);
    rounded[i]={{double(vm[0]),double(vm[1]),double(vm[2])},{double(wm[0]),double(wm[1]),double(wm[2])}};
  }
  const fe::NodalRigidGroupState group{in.metric.group->center,{double(v[0]),double(v[1]),double(v[2])},
    {double(w[0]),double(w[1]),double(w[2])},in.force_frame};
  const auto expected=Oracle({in.metric,rounded.data(),group,{}});
  long double body_rotation=0;
  for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)body_rotation+=.5L*w[a]*expected.tensor[3*a+b]*w[b];
  const auto body_translation=.5L*in.metric.group->total_mass_kg*Norm(v);
  Near(out.members.translation,translation);Near(out.members.native_rotation,rotation);
  Near(out.aggregate.translation,body_translation);Near(out.aggregate.rotation,body_rotation);
  for(unsigned a=0;a<3;++a) {Near(rigid_test::Get(out.collocated_primary.velocity,a),v[a]);Near(rigid_test::Get(out.collocated_primary.omega,a),w[a]);}
  rigid::GroupKineticObservation values{{},out.members,out.aggregate,out.replacement};Compare(values,expected);
  const auto scale=std::max(1e-300L,body_translation+body_rotation+translation+rotation);
  EXPECT_NEAR(out.replacement,(body_translation+body_rotation)-(translation+rotation),3e-12L*scale);
}
} // namespace rigid_observation_test
