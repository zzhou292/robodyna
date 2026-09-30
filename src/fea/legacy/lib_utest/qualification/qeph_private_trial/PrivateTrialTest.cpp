#include "../resident_shell_tab1/HostFamily.h"
#include "lib_src/elements/qeph/QephBatchFailureSection.h"
#include <limits>

namespace {
namespace q=tl::fea::qeph;
namespace fe=tl::fea;
namespace resident=resident_tab1_test;
namespace fields=failure_force_test;

std::vector<double> ForceValues(const q::ForceTrial& value) {
  auto out=fields::ForceValues(value);
  fields::Append(out,value.kinematics.projection_metric.working_length_m);
  const auto& d=value.diagnostics;
  for(double x:{d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,
      d.stabilization_viscosity,d.translational_stiffness,d.rotational_stiffness,
      d.unscaled_element_dt,d.internal_work_increment[0],d.internal_work_increment[1],
      d.hourglass_viscous_work_increment}) fields::Append(out,x);
  return out;
}
void Same(const q::ReferenceData& reference,const q::ForceTrial& actual,const q::ForceTrial& expected) {
  fields::Exact(ForceValues(actual),ForceValues(expected));
  EXPECT_TRUE(actual.proposed_history.matches_reference(reference));
  EXPECT_EQ(actual.proposed_history.prepared(),expected.proposed_history.prepared());
  EXPECT_EQ(actual.proposed_history.stamp().sample_index,expected.proposed_history.stamp().sample_index);
  EXPECT_EQ(actual.kinematics.sample_index,expected.kinematics.sample_index);
}
void Poison(q::ForceTrial& value) {
  // Valid typed objects, deliberately stale/nonfinite output fields. Accepted
  // input is a different object and must never be read through this target.
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  for(auto& force:value.internal_force) force={nan,nan,nan};
  for(auto& couple:value.internal_couple) couple={nan,nan,nan};
  value.kinematics.area=nan;
  value.kinematics.projection_metric.working_length_m=nan;
  value.diagnostics.internal_work_increment[1]=nan;
  value.diagnostics.rotational_stiffness=nan;
}
template<class T> std::vector<unsigned char> Bytes(const T& value) {
  std::vector<unsigned char> result(sizeof(T));
  std::memcpy(result.data(),&value,sizeof(T));
  return result;
}

TEST(QephPrivateTrial,GlobalAndLegacyRecurrencesOverwritePoisonedTrialWithExactFields) {
  const auto reference=fields::Q::ReferenceValue();
  for(unsigned mode=0;mode<4;++mode) {
    SCOPED_TRACE(mode);
    q::History accepted;
    ASSERT_EQ(q::InitializeHistory(reference,{},accepted),q::Status::kSuccess);
    fe::ShellGlobalLaw1Profile profile;
    profile.thickness=fe::ShellLaw1Thickness::Accepted;
    profile.coefficient_working_length_m=mode==1?.001:mode==2?.0254:1.;
    q::ForceTrial trial;
    for(unsigned step=0;step<16;++step) {
      SCOPED_TRACE(step);
      const auto interval=fields::Interval(reference,step);
      const auto saved=Bytes(accepted);
      q::ForceTrial expected;
      const auto expected_status=mode?q::EvaluateGlobalLaw1Force(profile,reference,accepted,interval,expected):
          q::EvaluateForce(reference,accepted,interval,expected);
      ASSERT_EQ(expected_status,q::Status::kSuccess);
      Poison(trial);
      const auto actual_status=mode?q::detail::EvaluateGlobalLaw1IntoTrial(profile,reference,accepted,interval,trial):
          q::detail::EvaluateForceWithThicknessIntoTrial(reference,accepted,interval,reference.input.thickness,trial);
      ASSERT_EQ(actual_status,q::Status::kSuccess);
      Same(reference,trial,expected);
      EXPECT_EQ(saved,Bytes(accepted));
      accepted=trial.proposed_history;
    }
  }
}

TEST(QephPrivateTrial,AllResidentPoliciesPlanesFailureAndInactiveSidecarsMatchValueDispatcher) {
  for(auto plane:resident::placed::Planes) for(unsigned mask:{0u,1u,7u}) {
    SCOPED_TRACE(static_cast<unsigned>(plane));
    SCOPED_TRACE(mask);
    resident::HostFamily<resident::placed::Q> actual(plane,mask),expected(plane,mask);
    unsigned removed=0,inactive=0;
    for(unsigned step=0;step<32&&inactive<2;++step) {
      SCOPED_TRACE(step);
      const bool was_active=actual.failure->state[actual.slab][resident::Parents-1].active;
      for(unsigned parent=0;parent<resident::Parents;++parent) {
        SCOPED_TRACE(parent);
        const auto reference=actual.Reference(parent);
        const auto interval=resident::placed::Interval(reference,step);
        const auto old_history=Bytes(actual.accepted[parent]);
        const auto old_section=resident::Values(actual.mixed->plastic.section[actual.slab][parent]);
        const auto old_failure=resident::Values(actual.failure->state[actual.slab][parent]);
        ASSERT_EQ(expected.Evaluate(parent,step),q::Status::kSuccess);
        Poison(actual.candidate[parent]);
        ASSERT_EQ(q::batch_detail::EvaluateFailureSectionIntoTrial(reference,actual.accepted[parent],interval,
            *actual.mixed,*actual.failure,actual.slab,parent,actual.candidate[parent]),q::Status::kSuccess);
        Same(reference,actual.candidate[parent],expected.candidate[parent]);
        const auto target=1u-actual.slab;
        fields::Exact(resident::Values(actual.mixed->plastic.section[target][parent]),
            resident::Values(expected.mixed->plastic.section[target][parent]));
        fields::Exact(resident::Values(actual.failure->state[target][parent]),
            resident::Values(expected.failure->state[target][parent]));
        for(unsigned point=0;point<3;++point) for(unsigned component=0;component<5;++component)
          EXPECT_EQ(std::memcmp(&actual.mixed->elastic_section[target][parent].point[point].stress[component],
              &expected.mixed->elastic_section[target][parent].point[point].stress[component],sizeof(double)),0);
        EXPECT_EQ(old_history,Bytes(actual.accepted[parent]));
        fields::Exact(old_section,resident::Values(actual.mixed->plastic.section[actual.slab][parent]));
        fields::Exact(old_failure,resident::Values(actual.failure->state[actual.slab][parent]));
      }
      const auto& next=actual.failure->state[1u-actual.slab][resident::Parents-1];
      removed+=was_active&&!next.active;
      inactive+=!was_active;
      actual.Accept();
      expected.Accept();
    }
    EXPECT_EQ(removed,1u);
    EXPECT_EQ(inactive,2u);
  }
}

TEST(QephPrivateTrial,LateCoefficientFailureKeepsPublicAndAcceptedBytesThenReusesPrivateTrial) {
  const auto reference=fields::Q::ReferenceValue();
  q::History accepted;
  ASSERT_EQ(q::InitializeHistory(reference,{},accepted),q::Status::kSuccess);
  const auto interval=fields::Interval(reference,0);
  fe::ShellGlobalLaw1Profile good;
  good.thickness=fe::ShellLaw1Thickness::Accepted;
  good.coefficient_working_length_m=.001;
  q::ForceTrial expected,public_output,private_output;
  ASSERT_EQ(q::EvaluateGlobalLaw1Force(good,reference,accepted,interval,expected),q::Status::kSuccess);
  public_output=expected;
  private_output=expected;
  const auto before=Bytes(public_output),old=Bytes(accepted);
  auto invalid=good;
  invalid.coefficient_working_length_m=1e300; // Finite profile; EM20 floor yields an unrepresentable force coefficient.
  EXPECT_TRUE(fe::shell_global_law1::Valid(invalid));
  EXPECT_EQ(q::EvaluateGlobalLaw1Force(invalid,reference,accepted,interval,public_output),q::Status::kNonfiniteResult);
  EXPECT_EQ(q::detail::EvaluateGlobalLaw1IntoTrial(invalid,reference,accepted,interval,private_output),q::Status::kNonfiniteResult);
  EXPECT_GT(private_output.kinematics.area,0.); // Failed after genuine geometry was written.
  EXPECT_EQ(before,Bytes(public_output));
  EXPECT_EQ(old,Bytes(accepted));
  ASSERT_EQ(q::detail::EvaluateGlobalLaw1IntoTrial(good,reference,accepted,interval,private_output),q::Status::kSuccess);
  Same(reference,private_output,expected);
}

TEST(QephPrivateTrial,PublicGlobalOutputCanAliasItsAcceptedHistoryAcrossSuccessAndFailure) {
  const auto reference=fields::Q::ReferenceValue();
  q::ForceTrial output;
  ASSERT_EQ(q::InitializeHistory(reference,{},output.proposed_history),q::Status::kSuccess);
  fe::ShellGlobalLaw1Profile profile;
  profile.thickness=fe::ShellLaw1Thickness::Accepted;
  for(unsigned step=0;step<8;++step) {
    const auto accepted=output.proposed_history;
    auto interval=fields::Interval(reference,step);
    q::ForceTrial expected;
    ASSERT_EQ(q::EvaluateGlobalLaw1Force(profile,reference,accepted,interval,expected),q::Status::kSuccess);
    const auto bytes=Bytes(output);
    auto bad=interval;
    bad.sample_index+=1;
    EXPECT_EQ(q::EvaluateGlobalLaw1Force(profile,reference,output.proposed_history,bad,output),q::Status::kInvalidInput);
    EXPECT_EQ(bytes,Bytes(output));
    ASSERT_EQ(q::EvaluateGlobalLaw1Force(profile,reference,output.proposed_history,interval,output),q::Status::kSuccess);
    Same(reference,output,expected);
  }
}
} // namespace
