// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
namespace qbat_force_test {
TEST(QbatForceNative, FourPointYieldedRotatingUnloadReloadIndependentHistories) {
  for(bool warped:{false,true}) for(double dm:{0.,.013}) {
    Fixture f(warped,dm);
    auto accepted=f.Virgin();
    NativeState native(accepted.data());
    for(unsigned step=0;step<256;++step) {
      SCOPED_TRACE(step);
      SCOPED_TRACE(warped);
      SCOPED_TRACE(dm);
      qb::ForceTrial actual;
      ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,accepted,Path(f,step),actual),
          qb::Status::kSuccess);
      native.Step(f,Path(f,step));
      CompareNative(actual,native);
      accepted=actual.proposed_history;
    }
  }
}
TEST(QbatForceNative, AllSurfaceFailureMasksCurrentForceThenRemovedHistory) {
  Fixture f;
  for(unsigned mask=0;mask<16;++mask) {
    SCOPED_TRACE(mask);
    auto values=f.Virgin().data();
    for(unsigned n=0;n<4;++n) {
      auto& p=values.point[n];
      if(mask&(1u<<n)) {
        p.failure.damage=1;p.failure.point_active=false;p.surface_active=false;
      } else {
        p.failure.damage=1-1e-8;p.material.stress[0]=10e6;
      }
    }
    values.element_active=mask!=15;
    qb::History accepted;
    ASSERT_EQ(qb::PreparePrescribedHistory(f.reference,f.material,f.failure,values,{0,0},accepted),
        qb::Status::kSuccess);
    NativeState native(values);
    for(unsigned step=0;step<3;++step) {
      qb::ForceTrial actual;
      ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,accepted,Path(f,step),actual),
          qb::Status::kSuccess);
      native.Step(f,Path(f,step));
      CompareNative(actual,native);
      EXPECT_FALSE(actual.proposed_history.data().element_active);
      accepted=actual.proposed_history;
    }
  }
}
TEST(QbatForceNative, ExplicitTableBranchAndRateOnHistoriesRemainIndependent) {
  Fixture f;
  const double strain[]{0,.2,1,3};
  const double stress[]{10e6,11e6,13e6,17e6};
  ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(250e6,.35,1000,{strain,stress,4},
      {true,8000,8,10000},f.material),mat::TabulatedShellPlasticityStatus::Ok);
  auto accepted=f.Virgin();
  NativeState native(accepted.data());
  for(unsigned step=0;step<128;++step) {
    SCOPED_TRACE(step);
    qb::ForceTrial actual;
    ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,accepted,Path(f,step),actual),
        qb::Status::kSuccess);
    native.Step(f,Path(f,step));
    CompareNative(actual,native);
    accepted=actual.proposed_history;
  }
}
TEST(QbatForceNative, WrongVelocityCorrectionDurationAndResetCacheAreDetected) {
  Fixture f;
  auto accepted=f.Virgin();
  NativeState native(accepted.data());
  for(unsigned step=0;step<16;++step) {
    qb::ForceTrial actual;
    ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,accepted,Path(f,step),actual),
        qb::Status::kSuccess);
    native.Step(f,Path(f,step));
    CompareNative(actual,native);
    accepted=actual.proposed_history;
  }
  auto prescribed=Path(f,16);
  NativeState expected=native;
  expected.Step(f,prescribed);
  NativeState wrong_duration=native;
  prescribed.dt*=2;
  wrong_duration.Step(f,prescribed);
  EXPECT_GT(std::abs(wrong_duration.output[108]-expected.output[108]),1e-5);
  NativeState wrong_frame=native;
  auto different=Path(f,16);
  for(auto& position:different.position_endpoint) position=qbat_test::Transform(position,.2);
  wrong_frame.Step(f,different);
  EXPECT_GT(std::abs(wrong_frame.output[0]-expected.output[0]),1e-3);
  // The non-affine transverse stabilization produces a force normal to the
  // flat algorithm's frame. Omitting CBAVISNP1 would erase this component.
  const double normal_force=expected.output[0]*expected.output[26]+
      expected.output[1]*expected.output[29]+expected.output[2]*expected.output[32];
  EXPECT_GT(std::abs(normal_force),1e-5);
  auto reset=f.Virgin().data();
  reset.thickness_m=accepted.data().thickness_m;
  NativeState wrong_history(reset);
  wrong_history.Step(f,Path(f,16));
  EXPECT_GT(std::abs(wrong_history.state[112]-expected.state[112]),1e-3);
}
extern "C" void law44_point_physical(int,const double*,double,double,double,double,
    const double*,const double*,const double*,double,double,double*);
TEST(QbatForceNative, CompleteGsZeroPointKeepsNativeCarriedTransverseStress) {
  const double eps[]{0,.2,1};
  const double yield[]{10e6,11e6,13e6};
  qb::Material p;
  ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(250e6,.35,1000,{eps,yield,3},p),
      mat::TabulatedShellPlasticityStatus::Ok);
  mat::TabulatedShellPlasticityHistory history;
  history.stress[3]=17;history.stress[4]=-9;
  mat::TabulatedShellPlasticityInput in;
  in.strain_increment[0]=.06;
  mat::TabulatedShellPlasticityResult actual;
  ASSERT_EQ(mat::UpdateLaw44MembranePlasticity(p,history,in,actual),mat::TabulatedShellPlasticityStatus::Ok);
  const double curve[]{0,0,0,10e6,.2,11e6,1,13e6};
  const double base[]{0,0,0,17,-9,0};
  const double rate[]{0,0,0,1,0};
  double values[13]{};
  law44_point_physical(3,curve,250e6,.35,1000,0,base,in.strain_increment,rate,.0005,.0005,values);
  for(unsigned i=0;i<5;++i) Close(actual.history.stress[i],values[i],1e-6);
  Close(actual.history.plastic_strain,values[5]);
  Close(actual.plastic_increment,values[6]);
  EXPECT_DOUBLE_EQ(values[3],17);
  EXPECT_DOUBLE_EQ(values[4],-9);
}
} // namespace qbat_force_test
