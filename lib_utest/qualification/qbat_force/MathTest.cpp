// SPDX-License-Identifier: MIT
#include "Fixture.h"
namespace qbat_force_test {
TEST(QbatForce, NamedGsZeroEntryAndLegacyPositiveGsContract) {
  Fixture f;
  mat::TabulatedShellPlasticityInput in;
  in.dt=Dt;
  in.total_strain_rate_per_s=30;
  in.strain_increment[0]=.06;
  mat::TabulatedShellPlasticityHistory h;
  h.stress[3]=17; h.stress[4]=-9;
  mat::TabulatedShellPlasticityResult output;
  const auto unchanged=Bytes(output);
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(f.material,h,in,output),
      mat::TabulatedShellPlasticityStatus::InvalidIncrement);
  EXPECT_EQ(Bytes(output),unchanged);
  ASSERT_EQ(mat::UpdateLaw44MembranePlasticity(f.material,h,in,output),
      mat::TabulatedShellPlasticityStatus::Ok);
  EXPECT_DOUBLE_EQ(output.history.stress[3],17);
  EXPECT_DOUBLE_EQ(output.history.stress[4],-9);
  auto shell=in;
  shell.transverse_shear_modulus=(5./6.)*f.material.shear_modulus;
  mat::TabulatedShellPlasticityResult legacy;
  ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(f.material,h,shell,legacy),
      mat::TabulatedShellPlasticityStatus::Ok);
  for(unsigned i=0;i<5;++i) EXPECT_DOUBLE_EQ(output.history.stress[i],legacy.history.stress[i]);
  EXPECT_DOUBLE_EQ(output.history.plastic_strain,legacy.history.plastic_strain);
  const auto saved=Bytes(output);
  in.strain_increment[4]=.01;
  EXPECT_NE(mat::UpdateLaw44MembranePlasticity(f.material,h,in,output),
      mat::TabulatedShellPlasticityStatus::Ok);
  EXPECT_EQ(Bytes(output),saved);
}
TEST(QbatForce, YieldedRotatingUnloadReloadRetainsFourHistoriesAndSeparateWork) {
  Fixture f;
  auto history=f.Virgin();
  double max_pla=0,max_difference=0,peak_thickness=0,negative_work=0;
  for(unsigned step=0;step<256;++step) {
    SCOPED_TRACE(step);
    qb::ForceTrial trial;
    ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,history,Path(f,step),trial),
        qb::Status::kSuccess);
    const auto& next=trial.proposed_history.data();
    for(const auto& point:next.point) max_pla=std::max(max_pla,point.material.plastic_strain);
    max_difference=std::max(max_difference,std::abs(next.point[0].material.plastic_strain-
        next.point[3].material.plastic_strain));
    peak_thickness=std::max(peak_thickness,std::abs(next.thickness_m-.0005));
    negative_work=std::min(negative_work,trial.diagnostics.internal_work_increment_j[0]);
    EXPECT_TRUE(next.element_active);
    EXPECT_GE(next.plastic_work_j,history.data().plastic_work_j);
    EXPECT_GE(next.numerical_viscous_work_j,history.data().numerical_viscous_work_j);
    for(unsigned p=0;p<4;++p) for(unsigned c=3;c<8;++c)
      EXPECT_DOUBLE_EQ(trial.kinematics.strain_increment[p][c],0);
    EXPECT_DOUBLE_EQ(trial.diagnostics.numerical_viscosity,.001);
    EXPECT_DOUBLE_EQ(trial.diagnostics.rotation_stiffness_nm,0);
    history=trial.proposed_history;
  }
  EXPECT_GT(max_pla,.1);
  EXPECT_GT(max_difference,1e-5);
  EXPECT_GT(peak_thickness,1e-6);
  EXPECT_LT(negative_work,0);
  EXPECT_GT(history.data().plastic_work_j,0);
  EXPECT_GT(history.data().numerical_viscous_work_j,0);
}
TEST(QbatForce, FourthSurfaceRemovalRetainsCurrentPacketThenNextZeroForce) {
  Fixture f;
  for(unsigned last=0;last<4;++last) {
    SCOPED_TRACE(last);
    auto h=NearRemoval(f,last);
    qb::ForceTrial trial;
    ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,h,Path(f,0),trial),qb::Status::kSuccess);
    ASSERT_TRUE(trial.diagnostics.removed_now);
    ASSERT_FALSE(trial.proposed_history.data().element_active);
    double current=0;
    for(auto force:trial.internal_force_n) current+=std::abs(force.x)+std::abs(force.y)+std::abs(force.z);
    EXPECT_GT(current,1e-3);
    for(const auto& point:trial.proposed_history.data().point) {
      EXPECT_FALSE(point.surface_active);
      for(double stress:point.material.stress) EXPECT_EQ(stress,0);
    }
    EXPECT_EQ(trial.proposed_history.data().point[3].force_stress_pa[0],0);
    EXPECT_DOUBLE_EQ(trial.proposed_history.data().point[last].failure.failure_time_s,Dt);
    qb::ForceTrial next;
    ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,trial.proposed_history,Path(f,1),next),
        qb::Status::kSuccess);
    for(auto force:next.internal_force_n) {
      EXPECT_EQ(force.x,0); EXPECT_EQ(force.y,0); EXPECT_EQ(force.z,0);
    }
    EXPECT_FALSE(next.diagnostics.removed_now);
    EXPECT_DOUBLE_EQ(next.proposed_history.data().thickness_m,trial.proposed_history.data().thickness_m);
  }
}
TEST(QbatForce, LateMaterialOverflowPhaseScopeAndAliasPreserveOutputThenRetry) {
  Fixture f;
  auto clean=f.Virgin();
  auto broken=clean.data();
  broken.point[3].material.stress[0]=1e308;
  qb::History imported;
  ASSERT_EQ(qb::PreparePrescribedHistory(f.reference,f.material,f.failure,broken,{0,0},imported),
      qb::Status::kSuccess);
  qb::ForceTrial output;
  ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,clean,Path(f,0),output),qb::Status::kSuccess);
  const auto saved=Bytes(output);
  const auto saved_history=Bytes(imported);
  EXPECT_NE(qb::EvaluateForce(f.reference,f.material,f.failure,imported,Path(f,0),output),qb::Status::kSuccess);
  EXPECT_EQ(Bytes(output),saved);
  EXPECT_EQ(Bytes(imported),saved_history);
  auto work_overflow=clean.data();
  work_overflow.point[3].force_stress_pa[0]=1e308;
  work_overflow.force_stress_pa[0]=.25*1e308;
  work_overflow.internal_work_j[0]=std::numeric_limits<double>::max();
  qb::History cached;
  ASSERT_EQ(qb::PreparePrescribedHistory(f.reference,f.material,f.failure,work_overflow,{0,0},cached),
      qb::Status::kSuccess);
  EXPECT_NE(qb::EvaluateForce(f.reference,f.material,f.failure,cached,Path(f,0),output),qb::Status::kSuccess);
  EXPECT_EQ(Bytes(output),saved);
  auto wrong=Path(f,0); wrong.sample_index=2;
  EXPECT_NE(qb::EvaluateForce(f.reference,f.material,f.failure,clean,wrong,output),qb::Status::kSuccess);
  EXPECT_EQ(Bytes(output),saved);
  auto changed=f.material; changed.rate.cutoff_hz=9999;
  EXPECT_NE(qb::EvaluateForce(f.reference,changed,f.failure,clean,Path(f,0),output),qb::Status::kSuccess);
  EXPECT_EQ(Bytes(output),saved);
  qb::ForceTrial expected;
  ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,output.proposed_history,Path(f,1),expected),
      qb::Status::kSuccess);
  ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,output.proposed_history,Path(f,1),output),
      qb::Status::kSuccess);
  EXPECT_EQ(StateValues(output.proposed_history.data()),StateValues(expected.proposed_history.data()));
}
} // namespace qbat_force_test
