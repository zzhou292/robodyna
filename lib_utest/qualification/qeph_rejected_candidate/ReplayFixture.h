// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/qeph/rejected_candidate/Types.h"
#include "lib_src/elements/qeph/QephBatchFailureSection.h"
#include "lib_src/elements/qeph/mapped/Result.h"
#include "lib_utest/qualification/shell_failure_force/FailureForceFields.h"
#include "lib_utest/qualification/shell_tab1_force/Tab1ForceFixture.h"
#include <array>
#include <cstring>
#include <vector>

namespace qeph_rejected_test {
namespace q=tl::fea::qeph;
namespace fe=tl::fea;
namespace material=tl::material;
namespace section=fe::sections;
namespace storage=fe::shell_batch_plasticity_detail;
using Route=q::RejectedCandidateRoute;

inline q::RejectedCandidateInput Base(const q::ReferenceData& reference=failure_force_test::Q::ReferenceValue()) {
  q::RejectedCandidateInput input;
  input.element.reference=reference;
  for(unsigned k=0;k<4;++k)input.element.nodes[k]=k;
  EXPECT_EQ(q::InitializeHistory(reference,{},input.accepted_force.proposed_history),q::Status::kSuccess);
  input.interval=failure_force_test::Interval(reference,0);
  input.route=Route::PlainForce;
  input.metadata.original={q::BatchStatus::ElementFailure,0,UINT32_MAX,q::Status::kNonfiniteResult,fe::NodalStatus::Ok};
  input.metadata.owner.owner_id=19;input.metadata.owner.node_count=4;
  input.metadata.owner.fixed_dt=input.interval.dt;
  input.metadata.accepted.owner_id=input.metadata.candidate.owner_id=19;
  input.metadata.accepted.valid=true;input.metadata.candidate.valid=false;
  input.metadata.candidate.epoch=1;input.metadata.candidate.attempt=1;
  input.metadata.candidate.time=input.interval.dt;input.metadata.candidate.kick_dt=.5*input.interval.dt;
  return input;
}
inline void Parameters(q::RejectedCandidateInput& input,const section::PointParameters& p) {
  ASSERT_LE(p.curve.count,fe::MaxShellPlasticityCurvePoints);
  input.plastic_parameters=q::CaptureMaterialParameters(p);
  for(unsigned i=0;i<p.curve.count;++i) {
    input.curve_strain[i]=p.curve.plastic_strain[i];input.curve_stress_pa[i]=p.curve.yield_stress_pa[i];
  }
}
inline q::RejectedCandidateInput Plastic(bool mixed=false) {
  auto input=Base();Parameters(input,layered_failure_test::Parameters());
  input.route=mixed?Route::MixedSection:Route::PlasticSection;
  input.has_mixed=mixed;
  if(mixed)input.law=fe::ShellSectionLaw::LayeredLaw44Nip3;
  input.accepted_plastic.cumulative_plastic_work_J=17.25;
  return input;
}
inline q::RejectedCandidateInput Constant(unsigned inactive_mask=0) {
  failure_force_test::Fixture<failure_force_test::Q> fixture(inactive_mask);
  auto input=Base(fixture.reference);
  input.accepted_force.proposed_history=fixture.accepted.shell;
  input.accepted_plastic.history=fixture.accepted.section.saved;
  input.accepted_plastic.cumulative_plastic_work_J=3.125;
  Parameters(input,fixture.material);
  input.route=Route::FailureSection;input.has_mixed=input.has_failure=true;
  input.law=fe::ShellSectionLaw::LayeredLaw44Nip3;
  input.failure_policy=fe::ShellFailurePolicy::ConstantAllPoints;
  input.constant_failure=fixture.failure;
  input.accepted_failure=storage::FailureState(fixture.accepted.section);
  return input;
}
inline q::RejectedCandidateInput Tab1() {
  tab1_force_test::Fixture<tab1_force_test::Q> fixture(1);
  auto input=Base(fixture.reference);
  input.accepted_force.proposed_history=fixture.accepted.shell;
  input.accepted_plastic.history=fixture.accepted.section.saved;
  input.accepted_plastic.cumulative_plastic_work_J=9.5;
  Parameters(input,fixture.material);
  input.route=Route::FailureSection;input.has_mixed=input.has_failure=true;
  input.law=fe::ShellSectionLaw::LayeredLaw44Nip3;
  input.failure_policy=fe::ShellFailurePolicy::Tab1AnyPoint;
  input.tab1_failure=fixture.failure;
  input.accepted_failure=storage::FailureState(fixture.accepted.section);
  return input;
}
inline std::uint64_t Bits(double value) {
  std::uint64_t result;std::memcpy(&result,&value,sizeof(result));return result;
}
inline std::vector<std::uint64_t> ParameterValues(const q::RejectedMaterialParameters& p) {
  std::vector<std::uint64_t> out{p.curve_count,static_cast<std::uint64_t>(p.rate.enabled),
    static_cast<std::uint64_t>(p.rate.policy),static_cast<std::uint64_t>(p.hardening),
    static_cast<std::uint64_t>(p.continuation)};
  for(double value:{p.young_pa,p.poisson_ratio,p.density_kg_m3,p.shear_modulus,p.a11,p.a12,p.three_g,p.sound_speed,
      p.rate.cowper_symonds_c_per_s,p.rate.cowper_symonds_p,p.rate.cutoff_hz,p.inverse_rate_c,p.inverse_rate_p,
      p.angular_cutoff_per_s,p.linear.initial_yield_pa,p.linear.tangent_modulus_pa,p.plastic_hardening_pa})
    out.push_back(Bits(value));
  return out;
}
inline std::vector<double> ForceValues(const q::ForceTrial& f) {
  auto out=failure_force_test::ForceValues(f);
  const auto& d=f.diagnostics;
  for(double value:{d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,d.stabilization_viscosity,
      d.translational_stiffness,d.rotational_stiffness,d.unscaled_element_dt,d.internal_work_increment[0],
      d.internal_work_increment[1],d.hourglass_viscous_work_increment,f.kinematics.projection_metric.working_length_m})
    out.push_back(value);
  return out;
}
inline void SameForce(const q::ForceTrial& actual,const q::ForceTrial& expected) {
  failure_force_test::Exact(ForceValues(actual),ForceValues(expected));
  EXPECT_EQ(actual.proposed_history.prepared(),expected.proposed_history.prepared());
}
inline void SamePlastic(const fe::ShellBatchSectionState& actual,const fe::ShellBatchSectionState& expected) {
  std::vector<double> a,b;
  failure_force_test::Append(a,actual.history);failure_force_test::Append(b,expected.history);
  failure_force_test::Append(a,actual.diagnostics);failure_force_test::Append(b,expected.diagnostics);
  a.push_back(actual.cumulative_plastic_work_J);b.push_back(expected.cumulative_plastic_work_J);
  failure_force_test::Exact(a,b);
}
inline void SameFailure(const fe::ShellBatchFailureState& a,const fe::ShellBatchFailureState& b) {
  ASSERT_EQ(a.policy(),b.policy());EXPECT_EQ(a.active,b.active);
  for(unsigned p=0;p<3;++p) {
    for(unsigned c=0;c<5;++c)EXPECT_EQ(Bits(a.current_force_point[p].stress[c]),Bits(b.current_force_point[p].stress[c]));
    if(a.constant_points()) {
      EXPECT_EQ(Bits(a.constant_points()[p].damage),Bits(b.constant_points()[p].damage));
      EXPECT_EQ(Bits(a.constant_points()[p].failure_time_s),Bits(b.constant_points()[p].failure_time_s));
      EXPECT_EQ(a.constant_points()[p].point_active,b.constant_points()[p].point_active);
    }
    if(a.tab1_points())tab1_test::SameFailureHistory(a.tab1_points()[p],b.tab1_points()[p]);
  }
}
inline void SameReplay(const q::RejectedReplayResult& a,const q::RejectedReplayResult& b) {
  EXPECT_EQ(a.operator_status,b.operator_status);EXPECT_EQ(a.force_available,b.force_available);
  EXPECT_EQ(a.mapped_result_checked,b.mapped_result_checked);EXPECT_EQ(a.mapped_result_valid,b.mapped_result_valid);
  SameForce(a.force,b.force);SamePlastic(a.plastic,b.plastic);SameFailure(a.failure,b.failure);
  for(unsigned p=0;p<3;++p)for(unsigned c=0;c<5;++c)
    EXPECT_EQ(Bits(a.elastic.point[p].stress[c]),Bits(b.elastic.point[p].stress[c]));
}
inline q::RejectedReplayResult Sentinel() {
  q::RejectedReplayResult out;
  out.operator_status=q::Status::kUnsupportedGeometry;out.force_available=true;
  out.mapped_result_checked=out.mapped_result_valid=true;
  out.force.diagnostics.unscaled_element_dt=-0.;
  out.force.internal_force[2]={1.25,-8.5,11};
  out.plastic.cumulative_plastic_work_J=123;
  out.elastic.point[1].stress[3]=-17;
  out.failure=fe::ShellBatchFailureState::Constant();
  out.failure.constant_points()[1].damage=.5;
  return out;
}
} // namespace qeph_rejected_test
