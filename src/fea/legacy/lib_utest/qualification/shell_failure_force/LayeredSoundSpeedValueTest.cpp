#include "FailureForceFields.h"
#include "lib_src/elements/qeph/QephLayeredLaw1.h"
#include "lib_src/elements/t3/T3LayeredLaw1.h"

namespace failure_force_test {
namespace {
template<class F> void SoundSpeedControls() {
  using ElasticHistory=std::conditional_t<std::is_same_v<F,Q>,
      q::LayeredLaw1History,t::LayeredLaw1History>;
  using ElasticTrial=std::conditional_t<std::is_same_v<F,Q>,
      q::LayeredLaw1ForceTrial,t::LayeredLaw1ForceTrial>;
  for(double nu:{0.,.3}) for(bool analytic:{false,true}) {
    SCOPED_TRACE(nu);
    SCOPED_TRACE(analytic);
    auto input=F::ReferenceValue().input;
    input.poisson_ratio=nu;
    typename F::Reference reference;
    ASSERT_EQ(InitializeReference(input,reference),F::Status::kSuccess);
    sec::PointParameters material;
    namespace mat=tl::material;
    mat::TabulatedShellPlasticityRate rate;
    rate.enabled=true;
    rate.cowper_symonds_c_per_s=0;
    rate.cowper_symonds_p=1;
    rate.cutoff_hz=100.;
    rate.policy=mat::ShellPlasticityRatePolicy::FilteredZeroC;
    const auto status=analytic?
        mat::PrepareLinearLaw44ShellPlasticity(input.young_modulus,nu,input.density,
            {220e6,1e9},rate,material):
        mat::PrepareTabulatedShellPlasticity(input.young_modulus,nu,input.density,
            {layered_failure_test::strains,layered_failure_test::yields,5},{},material);
    ASSERT_EQ(status,sec::PointStatus::Ok);
    typename F::LegacyHistory accepted;
    ASSERT_EQ(InitializeLayeredJ2History(reference,material,{},accepted),F::Status::kSuccess);
    const auto interval=Interval(reference,0);
    typename F::LegacyTrial trial;
    ASSERT_EQ(EvaluateLayeredJ2Force(reference,material,accepted,interval,trial),F::Status::kSuccess);

    mat::ShellElasticLaw1PointParameters elastic;
    ASSERT_TRUE(mat::PrepareShellElasticLaw1Point(input.young_modulus,nu,input.density,elastic));
    ElasticHistory elastic_history;
    ElasticTrial elastic_trial;
    ASSERT_EQ(InitializeLayeredLaw1History(reference,elastic,{},elastic_history),F::Status::kSuccess);
    ASSERT_EQ(EvaluateLayeredLaw1Force(reference,elastic,elastic_history,interval,elastic_trial),F::Status::kSuccess);

    const auto& d=trial.force.diagnostics;
    const auto& law1=elastic_trial.force.diagnostics;
    const double initial=std::sqrt(input.young_modulus/input.density);
    const double returned=std::sqrt((input.young_modulus/(1.-nu*nu))/input.density);
    EXPECT_DOUBLE_EQ(d.native_sound_speed,returned);
    EXPECT_DOUBLE_EQ(law1.native_sound_speed,initial);
    // C3DT3/CNDT3 retain elastic A11/G, independent of LAW44 point tangent.
    EXPECT_EQ(d.translational_stiffness,law1.translational_stiffness);
    EXPECT_EQ(d.rotational_stiffness,law1.rotational_stiffness);
    EXPECT_NEAR(d.unscaled_element_dt/law1.unscaled_element_dt,initial/returned,2e-15);
    const auto& h=trial.force.proposed_history.data();
    const auto& eh=elastic_trial.force.proposed_history.data();
    const double viscous=h.stress[0]-h.material_stress[0];
    const double elastic_viscous=eh.stress[0]-eh.material_stress[0];
    ASSERT_GT(std::abs(elastic_viscous),1.);
    EXPECT_NEAR(viscous/elastic_viscous,returned/initial,2e-12);

    // This formerly unused finite coefficient now participates in force/DT.
    const auto before=ForceValues(trial.force);
    auto bad=material;
    bad.sound_speed*=2.;
    EXPECT_NE(EvaluateLayeredJ2Force(reference,bad,accepted,interval,trial),F::Status::kSuccess);
    Exact(before,ForceValues(trial.force));
    ASSERT_EQ(EvaluateLayeredJ2Force(reference,material,accepted,interval,trial),F::Status::kSuccess);
    Exact(before,ForceValues(trial.force));
  }
}
} // namespace
TEST(ShellFailureForceValues,QephLaw44SoundSpeedPreservesLaw1StiffnessAndRejectsFalseCoefficient) {
  SoundSpeedControls<Q>();
}
TEST(ShellFailureForceValues,T3Law44SoundSpeedPreservesLaw1StiffnessAndRejectsFalseCoefficient) {
  SoundSpeedControls<T>();
}
} // namespace failure_force_test
