// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
namespace tl::fea::sections {
namespace layered_j2_failure_detail {
TL_SHELL_SECTION_HD inline bool ValidHistory(
    const ShellLayeredJ2FailureHistory& h) noexcept {
  const bool all_failed=ShellNip3FailedThickness(h.failure)>=ShellNip3FailureThreshold();
  if(h.element_active==all_failed) return false;
  for(unsigned p=0;p<3;++p) {
    for(unsigned c=0;c<5;++c) {
      const double force=h.current_force_point[p].stress[c];
      const double saved=h.saved.point[p].stress[c];
      if(!tl::math::Finite(force)||!tl::math::Finite(saved)) return false;
      if(h.failure[p].point_active ? saved!=force : saved!=0) return false;
    }
  }
  return true;
}

struct ObserveFailure {
  const ConstantFailureParameters& parameters;
  const ShellLayeredJ2FailureHistory& base;
  double time;
  ShellLayeredJ2FailureResult& candidate;

  TL_SHELL_SECTION_HD PointStatus operator()(unsigned p,
      const tl::material::TabulatedShellPlasticityHistory& old,
      const tl::material::TabulatedShellPlasticityResult& point) noexcept {
    using namespace tl::material::failure;
    // Native MULAWC:2068 overwrites SIGEPS44C's increment before FAIL_JOHNSON_C.
    const double increment=point.history.plastic_strain-old.plastic_strain;
    ConstantPlasticFailureResult failure;
    if(!UpdateConstantPlasticFailure(parameters,base.failure[p],
        {increment,time,base.element_active},failure)) return PointStatus::InvalidHistory;
    candidate.constitutive_increment[p]=point.plastic_increment;
    candidate.caller_failure_increment[p]=increment;
    candidate.history.failure[p]=failure.history;
    candidate.history.saved.point[p]=point.history;
    for(unsigned c=0;c<5;++c) {
      candidate.history.current_force_point[p].stress[c]=point.history.stress[c];
      if(!failure.history.point_active)
        candidate.history.saved.point[p].stress[c]*=0.;
    }
    return PointStatus::Ok;
  }
};
} // namespace layered_j2_failure_detail

TL_SHELL_SECTION_HD inline PointStatus UpdateShellLayeredJ2Failure(
    const PointParameters& material,const ConstantFailureParameters& failure,
    const ShellLayeredJ2FailureHistory& accepted,const ShellLayeredJ2Input& input,
    double native_evaluation_time_s,ShellLayeredJ2FailureResult& output) noexcept {
  if(!ValidLayeredJ2Parameters(material)||!tl::math::Finite(failure.failure_strain)||
      !(failure.failure_strain>0)) return PointStatus::InvalidParameters;
  if(!tl::math::Finite(native_evaluation_time_s)||native_evaluation_time_s<0)
    return PointStatus::InvalidIncrement;
  if(!layered_j2_failure_detail::ValidHistory(accepted)) return PointStatus::InvalidHistory;
  ShellLayeredJ2FailureResult candidate;
  candidate.history.element_active=accepted.element_active;
  layered_j2_failure_detail::ObserveFailure observer{
      failure,accepted,native_evaluation_time_s,candidate};
  const auto status=layered_j2_detail::Update(material,accepted.saved,input,
      accepted.element_active,observer,candidate.current);
  if(status!=PointStatus::Ok) return status;
  if(accepted.element_active&&ShellNip3FailedThickness(candidate.history.failure)>=
      ShellNip3FailureThreshold()) {
    candidate.history.element_active=false;
    candidate.removed_now=true;
  }
  // Native pending OFF=0.8 is consumed before the completed section is exposed.
  if(!candidate.history.element_active) {
    for(double& value:candidate.current.material_stress) value*=0.;
    for(double& value:candidate.current.bending_stress) value*=0.;
  }
  output=candidate;
  return PointStatus::Ok;
}
} // namespace tl::fea::sections
