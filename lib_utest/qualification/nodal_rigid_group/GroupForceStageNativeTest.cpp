#include "GroupForceStageFixture.h"
#include "GroupStepNativeFixture.h"
extern "C" void nodal_rigid_native_force_stage_kinetic(const int*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,const double*,const double*,const double*,double*);
namespace rigid_observation_test {
namespace {
void NativeCorrection(const rigid::GroupForceStageKineticInput& in,const rigid::GroupForceStageKineticObservation& out) {
  const int count=int(in.metric.member_count);double frame[9],j[3],w[3],ar[3];
  std::vector<double> v(3*count),omega(3*count),a(3*count),alpha(3*count),mass(count),inertia(count);
  for(unsigned axis=0;axis<3;++axis) {
    j[axis]=rigid_test::Get(in.metric.group->principal.inertia,axis);w[axis]=rigid_test::Get(in.before_primary.omega,axis);
    ar[axis]=rigid_test::Get(in.primary_acceleration.rotation,axis);
    for(unsigned b=0;b<3;++b)frame[3*axis+b]=in.force_frame.v[3*b+axis];
  }
  for(int i=0;i<count;++i) {
    mass[i]=in.metric.members[i].mass_kg;inertia[i]=in.metric.members[i].total_inertia_kg_m2;
    for(unsigned axis=0;axis<3;++axis) {
      v[3*i+axis]=rigid_test::Get(in.before_members[i].velocity,axis);omega[3*i+axis]=rigid_test::Get(in.before_members[i].omega,axis);
      a[3*i+axis]=rigid_test::Get(in.member_acceleration[i].translation,axis);alpha[3*i+axis]=rigid_test::Get(in.member_acceleration[i].rotation,axis);
    }
  }
  const double proxy=.75*in.metric.group->native_total_inertia_sum;double correction[2];
  nodal_rigid_native_force_stage_kinetic(&count,frame,j,w,v.data(),omega.data(),mass.data(),inertia.data(),&proxy,
    &in.phase.durations.previous_drift_dt,ar,a.data(),alpha.data(),correction);
  const long double primary_proxy=.5L*proxy*Norm(L(out.collocated_primary.omega));
  Near(correction[0],-static_cast<long double>(out.members.translation));
  const auto expected=static_cast<long double>(out.aggregate.rotation)-out.members.native_rotation-primary_proxy;
  EXPECT_NEAR(correction[1],expected,3e-12L*(out.aggregate.rotation+out.members.native_rotation+primary_proxy));
}
}
TEST(NodalRigidForceStageNative,ActualNativePacketsAndNonzeroAccelerationsForSixtyFourForceStages) {
  ForceStageFixture f;
  for(unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step);f.kick.SetTrial(rigid_step_test::NativePacket(f.kick.packet));
    const auto in=f.Input();rigid::GroupForceStageKineticObservation out;
    ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(in,out));NativeCorrection(in,out);CheckForceStageOracle(in,out);
    f.kick.Carry(step+1);
  }
}
TEST(NodalRigidForceStageNative,LargeNativeMemberBranchWithDenseAnisotropyAndUnequalDurations) {
  constexpr unsigned count=12;std::array<fe::NodalRigidGroupMember,count> source{};std::array<Motion,count> motion{};
  std::array<rigid::ForceStageAcceleration,count> a{};
  for(unsigned i=0;i<count;++i) {
    source[i]={100+i,i,{double(i%3),.17*double(i%4),.03*double(i*i)},1+.1*i,.003,.001,.002};
    motion[i]={{.3*i,-.2,.1},{.2,-.1*i,.7}};a[i]={{.5,-.3*i,.1},{.3*i,-.4,.8}};
  }
  fe::NodalRigidGroupInput group{300,400,source.data(),count};fe::NodalRigidGroupModel model;
  ASSERT_TRUE(model.Initialize({781,count,&group,1,{1000,.001}}));
  rigid::GroupForceStageKineticInput in{{model.groups(),source.data(),count},motion.data(),a.data(),{{.5,-.3,.2},{.2,.4,-.3}},
    {{.1,.5,-.8},{.6,-.1,.2}},Multiply(rigid_step_test::DenseFrame().axes,model.groups()[0].principal.axes),
    {.125,.0625,0,{.125,.1875,.25}}};
  rigid::GroupForceStageKineticObservation out;ASSERT_TRUE(rigid::ObserveGroupForceStageKinetic(in,out));
  NativeCorrection(in,out);CheckForceStageOracle(in,out);
}
} // namespace rigid_observation_test
