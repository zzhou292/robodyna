// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Types.h"
#include "../QephBatchFailureSection.h"
#include "../mapped/Result.h"
namespace tl::fea::qeph {
bool ReplayRejectedCandidate(const RejectedCandidateInput& input,RejectedReplayResult* output) noexcept {
  if(!output||input.plastic_parameters.curve_count>MaxShellPlasticityCurvePoints||
      !input.accepted_force.proposed_history.prepared()||
      !input.accepted_force.proposed_history.matches_reference(input.element.reference))return false;
  const auto route=input.route;
  switch(route) {
    case RejectedCandidateRoute::PlainForce:
    case RejectedCandidateRoute::PlasticSection:
      if(input.has_mixed||input.has_failure||input.mapped||input.law!=ShellSectionLaw::Unspecified)return false;break;
    case RejectedCandidateRoute::MixedSection:
      if(!input.has_mixed||input.has_failure)return false;break;
    case RejectedCandidateRoute::FailureSection:
      if(!input.has_mixed||!input.has_failure)return false;break;
    case RejectedCandidateRoute::RigidSkin:
      if(!input.has_mixed||!input.mapped||input.law!=ShellSectionLaw::RigidSkin)return false;break;
    default:return false;
  }
  if((route==RejectedCandidateRoute::MixedSection||route==RejectedCandidateRoute::FailureSection)&&
      input.law!=ShellSectionLaw::LayeredLaw44Nip3&&input.law!=ShellSectionLaw::LayeredLaw1Nip3&&
      input.law!=ShellSectionLaw::GlobalLaw1Npt0)return false;
  if(input.failure_policy!=ShellFailurePolicy::None&&input.failure_policy!=ShellFailurePolicy::ConstantAllPoints&&
      input.failure_policy!=ShellFailurePolicy::Tab1AnyPoint)return false;
  if((!input.has_failure&&input.failure_policy!=ShellFailurePolicy::None)||
      (input.failure_policy!=ShellFailurePolicy::None&&input.law!=ShellSectionLaw::LayeredLaw44Nip3))return false;
  auto parameters=ReplayMaterialParameters(input.plastic_parameters,input.curve_strain,input.curve_stress_pa);
  ShellBatchSectionState plastic[2]{input.accepted_plastic,{}};
  sections::ShellLayeredLaw1History elastic[2]{input.accepted_elastic,{}};
  ShellBatchFailureState failure[2]{input.accepted_failure,{}};
  auto law=input.law;auto global=input.global_law1;auto elastic_parameters=input.elastic_parameters;
  auto policy=input.failure_policy;auto constant=input.constant_failure;auto tab1=input.tab1_failure;
  shell_batch_plasticity_detail::DeviceStorage plain;
  plain.parameters=&parameters;plain.section[0]=&plastic[0];plain.section[1]=&plastic[1];
  shell_batch_plasticity_detail::MixedDeviceStorage mixed;
  mixed.plastic=plain;mixed.law=&law;mixed.global_law1=&global;mixed.elastic_parameters=&elastic_parameters;
  mixed.elastic_section[0]=&elastic[0];mixed.elastic_section[1]=&elastic[1];
  shell_batch_plasticity_detail::FailureDeviceStorage failures;
  failures.policy=&policy;failures.parameters=&constant;failures.tab1_parameters=&tab1;
  failures.state[0]=&failure[0];failures.state[1]=&failure[1];
  ForceTrial staged;Status status=Status::kInvalidInput;
  const auto& reference=input.element.reference;const auto& history=input.accepted_force.proposed_history;
  switch(route) {
    case RejectedCandidateRoute::PlainForce:
      status=detail::EvaluateForceWithThicknessIntoTrial(reference,history,input.interval,reference.input.thickness,staged);break;
    case RejectedCandidateRoute::PlasticSection:
      status=batch_detail::EvaluatePlasticSection(reference,history,input.interval,plain,0,0,staged);break;
    case RejectedCandidateRoute::MixedSection:
      status=batch_detail::EvaluateMixedSectionIntoTrial(reference,history,input.interval,mixed,0,0,staged);break;
    case RejectedCandidateRoute::FailureSection:
      status=batch_detail::EvaluateFailureSectionIntoTrial(reference,history,input.interval,mixed,failures,0,0,staged);break;
    case RejectedCandidateRoute::RigidSkin:
      status=mapped::AdvanceSkin(reference,input.accepted_force,input.interval,staged);break;
    default:return false;
  }
  RejectedReplayResult result;result.operator_status=status;
  if(status==Status::kSuccess) {
    result.force_available=true;result.force=staged;
    result.plastic=plastic[1];result.elastic=elastic[1];result.failure=failure[1];
    if(input.mapped) {
      result.mapped_result_checked=true;
      result.mapped_result_valid=mapped::ValidResult(reference,staged,
          input.interval.base_time+input.interval.dt,input.interval.sample_index,
          input.law==ShellSectionLaw::RigidSkin);
    }
  }
  *output=result;return true;
}
static_assert(2*sizeof(RejectedCandidateInput)+sizeof(RejectedCaptureReport)+
    2*sizeof(RejectedReplayResult)+4096<=ForecastRejectedCandidateCapture().peak_host_bytes,
    "Capture and replay scratch must fit the public bounded envelope");
} // namespace tl::fea::qeph
