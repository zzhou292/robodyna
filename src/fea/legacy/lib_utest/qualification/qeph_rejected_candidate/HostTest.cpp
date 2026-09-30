// SPDX-License-Identifier: MIT
#include "ReplayFixture.h"
#include "lib_src/elements/qeph/rejected_candidate/Read.h"
#include <limits>
#include <gtest/gtest.h>
namespace qeph_rejected_test {
TEST(QephRejectedReplay, ParameterValuesPreserveBitsAndOnlyRebindCurveAddresses) {
  auto p=layered_failure_test::Parameters(false,true);
  p.inverse_rate_c=-0.;p.continuation=material::ShellPlasticityCurveContinuation::NativeLastSegment;
  p.linear={27.25,-0.};p.plastic_hardening_pa=71.5;
  const auto snapshot=q::CaptureMaterialParameters(p);
  double x[5]{},y[5]{};
  const auto restored=q::ReplayMaterialParameters(snapshot,x,y);
  EXPECT_EQ(restored.curve.plastic_strain,x);EXPECT_EQ(restored.curve.yield_stress_pa,y);
  EXPECT_NE(restored.curve.plastic_strain,p.curve.plastic_strain);
  EXPECT_EQ(ParameterValues(snapshot),ParameterValues(q::CaptureMaterialParameters(restored)));
  auto zero=snapshot;zero.curve_count=0;
  const auto no_curve=q::ReplayMaterialParameters(zero,x,y);
  EXPECT_EQ(no_curve.curve.plastic_strain,nullptr);EXPECT_EQ(no_curve.curve.yield_stress_pa,nullptr);
  EXPECT_EQ(ParameterValues(zero),ParameterValues(q::CaptureMaterialParameters(no_curve)));
}
TEST(QephRejectedReplay, PlainAndGlobalLaw1MatchExistingPublicForceOperators) {
  for(bool global:{false,true}) {
    SCOPED_TRACE(global);auto input=Base();
    input.mapped=global;
    if(global) {
      input.route=Route::MixedSection;input.has_mixed=true;input.law=fe::ShellSectionLaw::GlobalLaw1Npt0;
      input.global_law1={fe::ShellLaw1Thickness::Accepted,.001};
      auto values=input.accepted_force.proposed_history.data();values.thickness*=.9;
      ASSERT_EQ(q::PreparePrescribedHistory(input.element.reference,values,{input.interval.dt,1},
        input.accepted_force.proposed_history),q::Status::kSuccess);
      input.interval.base_time=input.interval.dt;input.interval.sample_index=2;
    }
    q::ForceTrial expected;
    const auto status=global?q::EvaluateGlobalLaw1Force(input.global_law1,input.element.reference,
      input.accepted_force.proposed_history,input.interval,expected):
      q::EvaluateForce(input.element.reference,input.accepted_force.proposed_history,input.interval,expected);
    ASSERT_EQ(status,q::Status::kSuccess);
    q::RejectedReplayResult actual;ASSERT_TRUE(q::ReplayRejectedCandidate(input,&actual));
    ASSERT_TRUE(actual.force_available);EXPECT_EQ(actual.operator_status,status);
    EXPECT_EQ(actual.mapped_result_checked,global);EXPECT_EQ(actual.mapped_result_valid,global);SameForce(actual.force,expected);
  }
}
TEST(QephRejectedReplay, PlasticMixedAndFailureNonePreserveAllIncomingHistoryAndWork) {
  for(unsigned route=0;route<3;++route) {
    SCOPED_TRACE(route);auto input=Plastic(route!=0);
    if(route==2){input.route=Route::FailureSection;input.has_failure=true;}
    // Advance one complete native interval, then replay a non-virgin history.
    auto p=q::ReplayMaterialParameters(input.plastic_parameters,input.curve_strain,input.curve_stress_pa);
    q::LayeredJ2History old{input.accepted_force.proposed_history,input.accepted_plastic.history};
    q::LayeredJ2ForceTrial first;
    ASSERT_EQ(q::EvaluateLayeredJ2Force(input.element.reference,p,old,input.interval,first),q::Status::kSuccess);
    fe::ShellBatchSectionState incoming;
    ASSERT_TRUE(storage::ProposedPlasticSection(input.accepted_plastic,first.proposed_section,first.section_diagnostics,
      old.shell.data().thickness,first.force.kinematics.area,incoming));
    input.accepted_force=first.force;input.accepted_plastic=incoming;
    input.interval=failure_force_test::Interval(input.element.reference,1);
    q::LayeredJ2ForceTrial expected;
    ASSERT_EQ(q::EvaluateLayeredJ2Force(input.element.reference,p,
      {first.force.proposed_history,incoming.history},input.interval,expected),q::Status::kSuccess);
    fe::ShellBatchSectionState expected_section;
    ASSERT_TRUE(storage::ProposedPlasticSection(incoming,expected.proposed_section,expected.section_diagnostics,
      first.force.proposed_history.data().thickness,expected.force.kinematics.area,expected_section));
    q::RejectedReplayResult actual;ASSERT_TRUE(q::ReplayRejectedCandidate(input,&actual));
    ASSERT_EQ(actual.operator_status,q::Status::kSuccess);SameForce(actual.force,expected.force);
    SamePlastic(actual.plastic,expected_section);
    if(route==2)SameFailure(actual.failure,input.accepted_failure);
  }
}
TEST(QephRejectedReplay, MixedElasticPreservesItsThreePointStressHistory) {
  auto input=Base();input.route=Route::MixedSection;input.has_mixed=true;
  input.law=fe::ShellSectionLaw::LayeredLaw1Nip3;
  const auto& ref=input.element.reference;
  ASSERT_TRUE(material::PrepareShellElasticLaw1Point(ref.input.young_modulus,ref.input.poisson_ratio,
    ref.input.density,input.elastic_parameters));
  q::LayeredLaw1ForceTrial first;
  ASSERT_EQ(q::EvaluateLayeredLaw1Force(ref,input.elastic_parameters,
    {input.accepted_force.proposed_history,{}},input.interval,first),q::Status::kSuccess);
  input.accepted_force=first.force;input.accepted_elastic=first.proposed_section;
  input.interval=failure_force_test::Interval(ref,1);
  q::LayeredLaw1ForceTrial expected;
  ASSERT_EQ(q::EvaluateLayeredLaw1Force(ref,input.elastic_parameters,
    {first.force.proposed_history,first.proposed_section},input.interval,expected),q::Status::kSuccess);
  q::RejectedReplayResult actual;ASSERT_TRUE(q::ReplayRejectedCandidate(input,&actual));
  ASSERT_EQ(actual.operator_status,q::Status::kSuccess);SameForce(actual.force,expected.force);
  for(unsigned p=0;p<3;++p)for(unsigned c=0;c<5;++c)
    EXPECT_EQ(Bits(actual.elastic.point[p].stress[c]),Bits(expected.proposed_section.point[p].stress[c]));
}
TEST(QephRejectedReplay, ConstantAndTab1KeepTheirActualFailureSidecars) {
  for(unsigned kind=0;kind<3;++kind) {
    SCOPED_TRACE(kind);auto input=kind==2?Tab1():Constant(kind==1?7:0);
    const auto p=q::ReplayMaterialParameters(input.plastic_parameters,input.curve_strain,input.curve_stress_pa);
    q::ForceTrial expected_force;fe::ShellBatchSectionState expected_section;fe::ShellBatchFailureState expected_failure;
    if(kind==2) {
      q::LayeredTab1ForceTrial expected;
      ASSERT_EQ(q::EvaluateLayeredTab1Force(input.element.reference,p,input.tab1_failure,
        {input.accepted_force.proposed_history,storage::Tab1FailureHistory(input.accepted_plastic,input.accepted_failure)},
        input.interval,expected),q::Status::kSuccess);
      expected_force=expected.force;expected_failure=storage::FailureState(expected.section.history);
      ASSERT_TRUE(storage::ProposedPlasticSection(input.accepted_plastic,expected.section.history.saved,
        expected.section.current.diagnostics,input.accepted_force.proposed_history.data().thickness,
        expected.force.kinematics.area,expected_section));
    } else {
      q::LayeredJ2FailureForceTrial expected;
      ASSERT_EQ(q::EvaluateLayeredJ2FailureForce(input.element.reference,p,input.constant_failure,
        {input.accepted_force.proposed_history,storage::FailureHistory(input.accepted_plastic,input.accepted_failure)},
        input.interval,expected),q::Status::kSuccess);
      expected_force=expected.force;expected_failure=storage::FailureState(expected.section.history);
      ASSERT_TRUE(storage::ProposedPlasticSection(input.accepted_plastic,expected.section.history.saved,
        expected.section.current.diagnostics,input.accepted_force.proposed_history.data().thickness,
        expected.force.kinematics.area,expected_section));
    }
    q::RejectedReplayResult actual;ASSERT_TRUE(q::ReplayRejectedCandidate(input,&actual));
    ASSERT_EQ(actual.operator_status,q::Status::kSuccess);SameForce(actual.force,expected_force);
    SamePlastic(actual.plastic,expected_section);SameFailure(actual.failure,expected_failure);
  }
}
TEST(QephRejectedReplay, RigidSkinReplaysOnlyItsExistingBookkeeping) {
  for(bool failure:{false,true}) {
    auto input=Base();input.mapped=input.has_mixed=true;input.has_failure=failure;
    input.route=Route::RigidSkin;input.law=fe::ShellSectionLaw::RigidSkin;
    q::ForceTrial expected;
    ASSERT_EQ(q::mapped::AdvanceSkin(input.element.reference,input.accepted_force,input.interval,expected),q::Status::kSuccess);
    q::RejectedReplayResult actual;ASSERT_TRUE(q::ReplayRejectedCandidate(input,&actual));
    ASSERT_EQ(actual.operator_status,q::Status::kSuccess);EXPECT_TRUE(actual.mapped_result_valid);
    SameForce(actual.force,expected);
  }
}
TEST(QephRejectedReplay, GenuineOperatorRejectionDoesNotExposePartialFailedPackets) {
  auto input=Plastic(true);
  for(auto& x:input.interval.position_endpoint)x={0,0,0};
  const auto p=q::ReplayMaterialParameters(input.plastic_parameters,input.curve_strain,input.curve_stress_pa);
  q::LayeredJ2ForceTrial expected;
  const auto status=q::EvaluateLayeredJ2Force(input.element.reference,p,
    {input.accepted_force.proposed_history,input.accepted_plastic.history},input.interval,expected);
  ASSERT_EQ(status,q::Status::kUnsupportedGeometry);
  auto actual=Sentinel();ASSERT_TRUE(q::ReplayRejectedCandidate(input,&actual));
  EXPECT_EQ(actual.operator_status,status);EXPECT_FALSE(actual.force_available);EXPECT_FALSE(actual.mapped_result_checked);
  EXPECT_FALSE(actual.force.proposed_history.prepared());
}
TEST(QephRejectedReplay, CurveCapacityIsExactAndOverflowPreservesTheCaller) {
  auto input=Plastic();
  for(unsigned i=0;i<fe::MaxShellPlasticityCurvePoints;++i) {
    input.curve_strain[i]=i*.001;input.curve_stress_pa[i]=220e6+i*1e5;
  }
  section::PointParameters parameters;
  ASSERT_EQ(material::PrepareTabulatedShellPlasticity(200e9,.3,7890,
    {input.curve_strain,input.curve_stress_pa,fe::MaxShellPlasticityCurvePoints},{},parameters),section::PointStatus::Ok);
  input.plastic_parameters=q::CaptureMaterialParameters(parameters);
  q::RejectedReplayResult valid;ASSERT_TRUE(q::ReplayRejectedCandidate(input,&valid));
  EXPECT_EQ(valid.operator_status,q::Status::kSuccess);
  input.plastic_parameters.curve_count=fe::MaxShellPlasticityCurvePoints+1;
  auto out=Sentinel();const auto before=out;
  EXPECT_FALSE(q::ReplayRejectedCandidate(input,&out));SameReplay(out,before);
}
TEST(QephRejectedReplay, MalformedRoutesAndHistoryBindingsAreFailureAtomic) {
  const auto plain=Base();const auto mixed=Plastic(true);const auto failure=Constant();
  std::vector<q::RejectedCandidateInput> bad;
  bad.push_back(plain);bad.back().route=Route::Unspecified;
  bad.push_back(plain);bad.back().route=static_cast<Route>(255);
  bad.push_back(plain);bad.back().has_mixed=true;
  bad.push_back(plain);bad.back().has_failure=true;
  bad.push_back(mixed);bad.back().has_mixed=false;
  bad.push_back(mixed);bad.back().has_failure=true;
  bad.push_back(failure);bad.back().has_failure=false;
  bad.push_back(failure);bad.back().has_mixed=false;
  bad.push_back(mixed);bad.back().route=Route::RigidSkin;bad.back().mapped=false;
  bad.push_back(mixed);bad.back().route=Route::RigidSkin;bad.back().mapped=true;
  bad.push_back(plain);bad.back().accepted_force.proposed_history={};
  bad.push_back(plain);bad.back().element.reference.input.thickness*=2;
  for(unsigned i=0;i<bad.size();++i) {
    SCOPED_TRACE(i);auto out=Sentinel();const auto before=out;
    EXPECT_FALSE(q::ReplayRejectedCandidate(bad[i],&out));SameReplay(out,before);
  }
  EXPECT_FALSE(q::ReplayRejectedCandidate(plain,nullptr));
}
TEST(QephRejectedReplay, DeclaredEnvelopeIncludesCallerStagingAndReplay) {
  const auto forecast=q::ForecastRejectedCandidateCapture();
  EXPECT_EQ(forecast.retained_host_bytes,sizeof(q::RejectedCandidateInput));
  EXPECT_LE(sizeof(q::RejectedCandidateInput),32u<<10);EXPECT_EQ(forecast.device_bytes,0u);
  EXPECT_GE(forecast.peak_host_bytes,2*sizeof(q::RejectedCandidateInput)+sizeof(q::RejectedReplayResult));
}

TEST(QephRejectedReplay, CurveRangesUseTheWholeArenaAndRejectMisalignmentOrOverflow) {
  std::array<double,2048> arena{};
  using q::rejected_detail::CurveRange;
  EXPECT_TRUE(CurveRange(arena.data()+1500,arena.data(),arena.size(),16));
  EXPECT_TRUE(CurveRange(arena.data()+2040,arena.data(),arena.size(),8));
  EXPECT_FALSE(CurveRange(arena.data()+2040,arena.data(),arena.size(),9));
  EXPECT_TRUE(CurveRange(arena.data()+1024,arena.data(),arena.size(),1024));
  EXPECT_FALSE(CurveRange(arena.data(),arena.data(),arena.size(),1025));
  EXPECT_TRUE(CurveRange(arena.data()+2048,arena.data(),arena.size(),0));
  EXPECT_FALSE(CurveRange(arena.data()+2048,arena.data(),arena.size(),1));
  const auto misaligned=reinterpret_cast<const double*>(
      reinterpret_cast<const unsigned char*>(arena.data())+1);
  EXPECT_FALSE(CurveRange(misaligned,arena.data(),arena.size(),1));
  EXPECT_FALSE(CurveRange(arena.data(),arena.data()+1,arena.size()-1,1));
  EXPECT_FALSE(CurveRange(nullptr,arena.data(),arena.size(),1));
  EXPECT_FALSE(CurveRange(arena.data(),nullptr,arena.size(),1));
  EXPECT_FALSE(CurveRange(arena.data(),arena.data(),SIZE_MAX/sizeof(double)+1,1));
}
} // namespace qeph_rejected_test
