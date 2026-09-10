#include "TwoMemberFixture.h"
#include "GroupPhaseNativeFixture.h"
extern "C" void nodal_rigid_native_finite_velocity(const double*,const double*,double*,const double*);
extern "C" void nodal_rigid_native_inertia(const int*,const double*,const double*,const double*,const double*,const double*,double*);
namespace rigid_two_test {
TEST(NodalRigidTwoMemberNative,SourceShapesRetainNativeTensorAndRegularization) {
  for(double j:{1e-12,1e-6}) { SourceFixture f(j);ASSERT_TRUE(f.model.prepared());
    EXPECT_EQ(f.model.member_count(),8u);
    for(unsigned g=0;g<4;++g) {const auto& p=f.model.groups()[g];
      EXPECT_EQ(p.member_count,2u); EXPECT_EQ(p.source_group_id,GroupIds[g]);
      EXPECT_EQ(p.regularization.principal_inertia_changed,j==1e-12);
      double x[6],mass[2],inertia[2],initial[9]{},tensor[9];const int count=2;
      const double center[]{p.center.x,p.center.y,p.center.z};
      for(unsigned i=0;i<2;++i) {const auto& m=f.model.members()[2*g+i];
        EXPECT_EQ(m.source_node_id,NodeIds[g][i]);EXPECT_EQ(Bytes(m.position),Bytes(Position[g][i]));
        mass[i]=m.mass_kg;inertia[i]=m.total_inertia_kg_m2;
        for(unsigned a=0;a<3;++a)x[3*i+a]=Get(m.position,a);}
      const auto arm=rigid::detail::Subtract(p.generated_primary_position,p.center);
      const double square=arm.x*arm.x+arm.y*arm.y+arm.z*arm.z;
      for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)
        initial[3*a+b]=(a==b?p.regularization.primary_isotropic_inertia_kg_m2:0)+
          p.regularization.primary_mass_kg*((a==b?square:0)-Get(arm,a)*Get(arm,b));
      nodal_rigid_native_inertia(&count,x,center,mass,inertia,initial,tensor);
      for(unsigned i=0;i<9;++i) EXPECT_NEAR(p.raw_tensor.v[i],tensor[i],2e-20);
    }
  }
}
TEST(NodalRigidTwoMemberNative,BothThresholdsAndSourceLengthControlNativeBranch) {
  const double dt=.125;
  for(double length:{1.,.001}) for(double angle:{.0009,.0011}) for(double displacement:{.00009,.00011}) {
    const double w[]{0,0,angle/dt},arm[]{displacement/angle,0,0};double native[3];
    nodal_rigid_native_finite_velocity(w,arm,native,&dt);
    const Vec3 spin{w[0],w[1],w[2]},x{length*arm[0],0,0};
    const auto actual=rigid::two_member_detail::FiniteVelocity(spin,x,dt,length);
    for(unsigned a=0;a<3;++a) EXPECT_NEAR(Get(actual,a),length*native[a],2e-15*length);
    const bool finite=angle>.001&&displacement>.0001;
    if(finite) EXPECT_LT(actual.x,0);else EXPECT_EQ(actual.x,0);
  }
  const Vec3 w{0,0,.01/dt},x{1e-4,0,0};
  const auto si=rigid::two_member_detail::FiniteVelocity(w,x,dt,1);
  const auto source=rigid::two_member_detail::FiniteVelocity(w,x,dt,.001);
  EXPECT_EQ(si.x,0);EXPECT_LT(source.x,-1e-9); // A wrong unscaled threshold is detectable.
}
TEST(NodalRigidTwoMemberNative,FourLoadedUnloadingRecurrencesMatchNativeForSixtyFourFreshSteps) {
  SourceFixture f;ASSERT_TRUE(f.model.prepared());
  for(unsigned g=0;g<4;++g) {auto actual=f.Packet(g),native=actual;NativeSchedule schedule;
    double difference_from_large=0;
    for(unsigned step=0;step<64;++step) {
      SCOPED_TRACE(g);
      SCOPED_TRACE(step);actual.body.durations=native.body.durations=schedule.Next(1./1024);
      const double load=step<24?1.:(step<48?-.5:0.);
      for(unsigned i=0;i<2;++i) {
        actual.member[i].force=native.member[i].force={load*.01*(i?1:-1),load*.015,load*-.003};
        actual.member[i].couple=native.member[i].couple={load*2e-5,load*-3e-5,load*4e-5};
      }
      Trial result{};ASSERT_EQ(EvaluateTwoPacket(actual,result),rigid::StepStatus::Success);
      const auto expected=NativeTwoPacket(native,.001);
      { SCOPED_TRACE("same input packet");Agreement(result,NativeTwoPacket(actual,.001)); }
      { SCOPED_TRACE("independently carried trajectory");Agreement(result,expected); }
      rigid::MemberStepTrial wrong;
      ASSERT_EQ(rigid::EvaluateMemberStep(actual.body,result.primary,actual.member[0],wrong),rigid::StepStatus::Success);
      difference_from_large=std::max(difference_from_large,std::abs(wrong.velocity.x-result.member[0].velocity.x));
      Carry(result,actual);Carry(expected,native);
    }
    EXPECT_GT(difference_from_large,1e-8);
  }
}
TEST(NodalRigidTwoMemberNative,LateInvalidMemberAndUnitsDoNotPublishAndRetryExactly) {
  SourceFixture f;const auto valid=f.Packet(2);Trial expected{};
  ASSERT_EQ(EvaluateTwoPacket(valid,expected),rigid::StepStatus::Success);
  for(unsigned fault=0;fault<5;++fault) {auto in=valid;double units=.001;Trial out=expected;const auto before=Bytes(out);
    if(fault==0)in.member[1].inertia=0;
    if(fault==1)in.member[1].velocity.x=std::numeric_limits<double>::quiet_NaN();
    if(fault==2)units=0;if(fault==3)units=std::numeric_limits<double>::max();
    if(fault==4)in.member[1].velocity.y=std::numeric_limits<double>::max();
    EXPECT_NE(EvaluateTwoPacket(in,out,units),rigid::StepStatus::Success);EXPECT_EQ(Bytes(out),before);
    ASSERT_EQ(EvaluateTwoPacket(valid,out),rigid::StepStatus::Success);Agreement(out,expected,0);
  }
  fe::NodalRigidGroupModel invalid_units;
  // A tiny source mass allows the primary-J scale to remain representable;
  // the two-member displacement threshold itself must still reject overflow.
  EXPECT_FALSE(invalid_units.Initialize({781,9,f.groups.data(),4,{1e-290,1e160}}));
  EXPECT_FALSE(invalid_units.prepared());
  ASSERT_TRUE(invalid_units.Initialize({781,9,f.groups.data(),4,{1000,.001}}));
  auto one=f.groups;one[3].member_count=1;fe::NodalRigidGroupModel model;
  EXPECT_FALSE(model.Initialize({781,9,one.data(),4,{1000,.001}}));EXPECT_FALSE(model.prepared());
  ASSERT_TRUE(model.Initialize({781,9,f.groups.data(),4,{1000,.001}}));
}
TEST(NodalRigidTwoMemberNative,OverflowedTransverseNormCannotProduceFiniteApparentMotion) {
  auto body=rigid_step_test::Fixture({2,2,2}).body;body.center={};body.velocity={};
  body.omega={0,0,.005};body.applied={};
  rigid::PrimaryStepTrial primary;ASSERT_EQ(rigid::EvaluateTwoMemberPrimaryStep(body,primary),rigid::StepStatus::Success);
  rigid::MemberStepInput member{{1e200,0,0},{},{},{},{},1,.001};
  rigid::MemberStepTrial out;out.position={1,2,3};const auto before=Bytes(out);
  EXPECT_EQ(rigid::EvaluateTwoMemberStep(body,primary,member,.001,out),rigid::StepStatus::NonfiniteResult);
  EXPECT_EQ(Bytes(out),before);
}
} // namespace rigid_two_test
