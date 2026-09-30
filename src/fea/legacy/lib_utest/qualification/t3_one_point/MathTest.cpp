// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace t3_one_point_test {
TEST(T3OnePoint, GeneralZeroShearPreservesNarrowMembraneAndShellContracts) {
  Fixture f;
  mat::TabulatedShellPlasticityInput input;
  input.dt=Dt;
  input.total_strain_rate_per_s=21;
  input.strain_increment[0]=.06;
  input.strain_increment[3]=.13;
  input.strain_increment[4]=-.09;
  mat::TabulatedShellPlasticityHistory history;
  history.stress[3]=17;
  history.stress[4]=-9;
  mat::TabulatedShellPlasticityResult output;
  const auto saved=Bytes(output);
  EXPECT_NE(mat::UpdateLaw44ShellPlasticity(f.material,history,input,output),mat::TabulatedShellPlasticityStatus::Ok);
  EXPECT_EQ(Bytes(output),saved);
  EXPECT_NE(mat::UpdateLaw44MembranePlasticity(f.material,history,input,output),mat::TabulatedShellPlasticityStatus::Ok);
  EXPECT_EQ(Bytes(output),saved);
  ASSERT_EQ(mat::UpdateLaw44ZeroShearPlasticity(f.material,history,input,output),mat::TabulatedShellPlasticityStatus::Ok);
  EXPECT_DOUBLE_EQ(output.history.stress[3],17);
  EXPECT_DOUBLE_EQ(output.history.stress[4],-9);
  const auto current=Bytes(output);
  input.strain_increment[4]=std::numeric_limits<double>::infinity();
  EXPECT_NE(mat::UpdateLaw44ZeroShearPlasticity(f.material,history,input,output),mat::TabulatedShellPlasticityStatus::Ok);
  EXPECT_EQ(Bytes(output),current);
}
TEST(T3OnePoint, OriginalTriangleCyclesRetainEightStrainsAndCurvatureRate) {
  Fixture f;
  auto history=f.Virgin();
  double peak_pla=0,peak_thickness=0,min_work=0,curvature_rate_difference=0;
  double max_increment[8]{};
  for (unsigned step=0;step<256;++step) {
    SCOPED_TRACE(step);
    t3::OnePointForceTrial trial;
    ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,history,Path(f,step),trial),
        t3::Status::kSuccess);
    const auto& h=trial.proposed_history.shell().data();
    peak_pla=std::max(peak_pla,trial.proposed_history.point().plastic_strain);
    peak_thickness=std::max(peak_thickness,std::abs(h.thickness-.0005));
    min_work=std::min(min_work,trial.diagnostics.internal_work_increment[0]);
    double wrong[8];
    std::copy_n(trial.strain_curvature_increment,8,wrong);
    for (unsigned i=0;i<8;++i) max_increment[i]=std::max(max_increment[i],std::abs(wrong[i]));
    wrong[5]=wrong[6]=wrong[7]=0;
    const double membrane_rate=t3::one_point_detail::CallerRate(wrong,history.shell().data().thickness,Dt);
    curvature_rate_difference=std::max(curvature_rate_difference,std::abs(h.equivalent_strain_rate-membrane_rate));
    EXPECT_EQ(h.active,1);
    EXPECT_EQ(trial.diagnostics.membrane_viscosity,0);
    EXPECT_EQ(trial.diagnostics.transverse_shear_modulus,0);
    EXPECT_GT(trial.diagnostics.rotational_stiffness,0);
    EXPECT_DOUBLE_EQ(trial.diagnostics.native_sound_speed,f.material.sound_speed);
    EXPECT_EQ(h.internal_work[1],0);
    EXPECT_EQ(trial.proposed_history.point().stress[3],0);
    EXPECT_EQ(trial.proposed_history.point().stress[4],0);
    history=trial.proposed_history;
  }
  EXPECT_GT(peak_pla,.05);
  EXPECT_GT(peak_thickness,1e-6);
  EXPECT_LT(min_work,0);
  EXPECT_GT(curvature_rate_difference,1e-5);
  EXPECT_GT(history.plastic_work_j(),0);
  for (double value:max_increment) EXPECT_GT(value,1e-8);
}
TEST(T3OnePoint, SinglePointRemovalMasksCurrentForceRetainsOldWorkAndTime) {
  Fixture f;
  auto history=NearFailure(f);
  t3::OnePointForceTrial trial;
  ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,history,Path(f,0),trial),t3::Status::kSuccess);
  ASSERT_TRUE(trial.removed_now);
  EXPECT_TRUE(trial.point.failure.failed_now);
  EXPECT_EQ(trial.proposed_history.failure().failure_time_s,Dt);
  EXPECT_GT(std::abs(trial.point.current.history.stress[0]),1e3);
  EXPECT_GT(trial.diagnostics.internal_work_increment[0],0);
  EXPECT_GT(trial.plastic_work_increment_j,0);
  for (unsigned step=0;step<3;++step) {
    if (step) {
      ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,
          trial.proposed_history,Path(f,step),trial),t3::Status::kSuccess);
      EXPECT_FALSE(trial.removed_now);
      EXPECT_EQ(trial.diagnostics.internal_work_increment[0],0);
    }
    EXPECT_FALSE(trial.proposed_history.failure().point_active);
    EXPECT_EQ(trial.proposed_history.failure().failure_time_s,Dt);
    EXPECT_EQ(trial.diagnostics.translational_stiffness,0);
    EXPECT_EQ(trial.diagnostics.rotational_stiffness,0);
    EXPECT_GT(trial.diagnostics.unscaled_element_dt,0);
    for (auto force:trial.internal_force) {
      EXPECT_EQ(force.x,0); EXPECT_EQ(force.y,0); EXPECT_EQ(force.z,0);
    }
  }
}
TEST(T3OnePoint, LateFailureScopeAliasAndRetryPreserveCompleteState) {
  Fixture f;
  auto base=f.Virgin();
  t3::OnePointForceTrial output;
  ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,base,Path(f,0),output),t3::Status::kSuccess);
  const auto original=Bytes(output);
  const auto base_bytes=Bytes(base);
  auto corrupt=base.values();
  corrupt.point.stress[0]=1e308;
  corrupt.shell.stress[0]=1e308;
  corrupt.shell.material_stress[0]=1e308;
  t3::OnePointHistory huge;
  ASSERT_EQ(t3::PrepareOnePointLaw44History(f.reference,f.material,f.failure,corrupt,{0,0},huge),t3::Status::kSuccess);
  EXPECT_NE(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,huge,Path(f,0),output),t3::Status::kSuccess);
  EXPECT_EQ(Bytes(output),original);
  auto wrong=Path(f,0);
  wrong.sample_index=2;
  EXPECT_NE(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,base,wrong,output),t3::Status::kSuccess);
  EXPECT_EQ(Bytes(output),original);
  auto reference=f.reference;
  reference.input.node_ids[0]+=1;
  EXPECT_NE(t3::EvaluateOnePointLaw44Force(reference,f.material,f.failure,base,Path(f,0),output),t3::Status::kSuccess);
  EXPECT_EQ(Bytes(output),original);
  auto changed=f.material;
  changed.rate.cutoff_hz=9999;
  EXPECT_NE(t3::EvaluateOnePointLaw44Force(f.reference,changed,f.failure,base,Path(f,0),output),t3::Status::kSuccess);
  EXPECT_EQ(Bytes(output),original);
  reference=f.reference;
  reference.input.placement=tl::fea::ShellReferencePlacement::TopReferencePlane;
  EXPECT_NE(t3::InitializeOnePointLaw44History(reference,f.material,f.failure,{0,0},base),t3::Status::kSuccess);
  EXPECT_EQ(Bytes(base),base_bytes);
  t3::OnePointForceTrial expected;
  ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,output.proposed_history,Path(f,1),expected),t3::Status::kSuccess);
  ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,output.proposed_history,Path(f,1),output),t3::Status::kSuccess);
  EXPECT_EQ(StateValues(output.proposed_history.values()),StateValues(expected.proposed_history.values()));
}
} // namespace t3_one_point_test
