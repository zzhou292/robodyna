#include "GroupKickFixture.h"
#include "GroupStepNativeFixture.h"
#include "TwoMemberFixture.h"
extern "C" void nodal_rigid_native_kinetic_correction(const int*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,double*);
namespace rigid_observation_test {
TEST(NodalRigidObservationNative,PinnedRgbcorCorrectionMatchesSeparateMemberAndAggregateChannels) {
  for(auto kind:{Fixture::Kind::Dense,Fixture::Kind::MeasurablePrimary,Fixture::Kind::CorrectedInertia}) {
    Fixture fixture(kind); rigid::GroupKineticObservation output;
    ASSERT_TRUE(rigid::ObserveGroupKinetic(fixture.Input(),output));
    const int count=Count; double frame[9],j[3],w[3],v[3*Count],omega[3*Count],mass[Count],inertia[Count];
    for(unsigned a=0;a<3;++a) {
      j[a]=rigid_test::Get(fixture.model.groups()[0].principal.inertia,a); w[a]=rigid_test::Get(fixture.state.omega,a);
      for(unsigned b=0;b<3;++b) frame[3*a+b]=fixture.state.principal_axes.v[3*b+a];
    }
    for(unsigned i=0;i<Count;++i) {
      mass[i]=fixture.source[i].mass_kg; inertia[i]=fixture.source[i].total_inertia_kg_m2;
      for(unsigned a=0;a<3;++a) { v[3*i+a]=rigid_test::Get(fixture.motion[i].velocity,a); omega[3*i+a]=rigid_test::Get(fixture.motion[i].omega,a); }
    }
    // Arbitrary positive proxy J is authored wrapper context. Native subtracts
    // it once; it must not become another physical group inertia contribution.
    const double proxy=.75*fixture.model.groups()[0].native_total_inertia_sum;
    double correction[2];
    nodal_rigid_native_kinetic_correction(&count,frame,j,w,v,omega,mass,inertia,&proxy,correction);
    Near(correction[0],-static_cast<long double>(output.members.translation));
    const long double expected_rotation=static_cast<long double>(output.aggregate.rotation)-output.members.native_rotation-
      .5L*proxy*(static_cast<long double>(w[0])*w[0]+static_cast<long double>(w[1])*w[1]+static_cast<long double>(w[2])*w[2]);
    EXPECT_NEAR(correction[1],expected_rotation,3e-12*output.aggregate.rotation);
    Compare(output,Oracle(fixture.Input()));
  }
}

TEST(NodalRigidObservationNative,NativeMemberReactionsSatisfyAllKickAndReplacementChecksForThirtyTwoSteps) {
  KickFixture fixture;
  for(unsigned step=0;step<32;++step) {
    SCOPED_TRACE(step); fixture.SetTrial(rigid_step_test::NativePacket(fixture.packet));
    rigid::GroupKickObservation output; const auto report=rigid::ObserveGroupKick(fixture.Input(),output);
    ASSERT_TRUE(report)<<int(report.status)<<" member "<<report.member<<" dof "<<report.dof<<" residual "<<report.residual<<" budget "<<report.roundoff_budget;
    CompareWork(fixture.Input(),output); fixture.Carry(step+1);
  }
}
TEST(NodalRigidObservationNative,TwoMemberStoredObservationMatchesIndependentWorldTensor) {
  rigid_two_test::SourceFixture f;
  for(unsigned g=0;g<4;++g) {
    auto packet=f.Packet(g);packet.member[1].couple={2e-4,3e-4,-4e-4};
    const auto trial=rigid_step_test::NativeTwoPacket(packet,.001);
    const Motion motion[]{{trial.member[0].velocity,trial.member[0].omega},
                          {trial.member[1].velocity,trial.member[1].omega}};
    rigid::GroupKineticInput input{{f.model.groups()+g,f.model.members()+2*g,2},motion,
      {trial.primary.center,trial.primary.velocity,trial.primary.omega,trial.primary.force_frame.axes},
      {rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame,1./1024,1./2048,0}};
    rigid::GroupKineticObservation out;ASSERT_TRUE(rigid::ObserveGroupKinetic(input,out));
    Compare(out,Oracle(input));
    EXPECT_GT(out.aggregate.principal_correction_rotation,0);
    const auto saved=rigid_step_test::Bytes(out);input.metric.member_count=1;
    EXPECT_FALSE(rigid::ObserveGroupKinetic(input,out));EXPECT_EQ(rigid_step_test::Bytes(out),saved);
  }
}
} // namespace rigid_observation_test
