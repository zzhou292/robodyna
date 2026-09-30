// SPDX-License-Identifier: AGPL-3.0-or-later
// Local LAW44/TAB1 NLay1 centered NIP3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "ShellLayeredJ2Work.h"
#include "ShellLayeredFailureWork.h"
#include "lib_src/materials/failure/ShellTab1ConstantFailure.h"
namespace tl::fea::sections {
// Explicit native IFAIL_SH1, resolved PTHKF=EM06. The source adapter must
// retain its original NUMINT1 declaration and the deliberate converter override.
enum class ShellTab1ParentPolicy : unsigned char { AnyPoint };
struct ShellLayeredTab1Parameters {
  material::failure::Tab1ConstantTable table;
  ShellTab1ParentPolicy parent_policy=ShellTab1ParentPolicy::AnyPoint;
};
struct ShellLayeredTab1History {
  ShellLayeredJ2History saved;
  material::failure::Tab1ConstantFailureHistory failure[3];
  ShellFailureForcePoint current_force_point[3];
  bool element_active=true;
};
struct ShellLayeredTab1Result {
  ShellLayeredTab1History history;
  ShellLayeredJ2Result current;
  double constitutive_increment[3]{},caller_failure_increment[3]{};
  bool removed_now=false;
};
namespace layered_tab1_detail {
TL_SHELL_SECTION_HD inline double FailedThickness(const ShellLayeredTab1History& h) noexcept {
  double failed=0;
  for(unsigned p=0;p<3;++p)
    if(!h.failure[p].point_active) failed=failed+LayerForceWeight(p);
  return failed;
}
TL_SHELL_SECTION_HD inline bool ValidHistory(const ShellLayeredTab1History& h) noexcept {
  if(h.element_active==(FailedThickness(h)>=1.e-6)) return false;
  for(unsigned p=0;p<3;++p) {
    if(!material::failure::ValidTab1ConstantHistory(h.failure[p])) return false;
    for(unsigned c=0;c<5;++c) {
      const double force=h.current_force_point[p].stress[c],saved=h.saved.point[p].stress[c];
      if(!tl::math::Finite(force)||!tl::math::Finite(saved)) return false;
      if(h.failure[p].point_active?saved!=force:saved!=0) return false;
    }
  }
  return true;
}
TL_SHELL_SECTION_HD inline PointStatus PointFailureStatus(material::failure::Tab1FailureStatus s) noexcept {
  using Status=material::failure::Tab1FailureStatus;
  switch(s) {
    case Status::Ok:return PointStatus::Ok;
    case Status::InvalidParameters:return PointStatus::InvalidParameters;
    case Status::InvalidHistory:return PointStatus::InvalidHistory;
    case Status::InvalidIncrement:return PointStatus::InvalidIncrement;
    case Status::NonfiniteResult:return PointStatus::NonfiniteResult;
  }
  return PointStatus::InvalidParameters;
}
struct ObserveFailure {
  const ShellLayeredTab1Parameters& parameters;
  const ShellLayeredTab1History& base;
  double time;
  ShellLayeredTab1Result& candidate;
  TL_SHELL_SECTION_HD PointStatus operator()(unsigned p,
      const material::TabulatedShellPlasticityHistory& old,
      const material::TabulatedShellPlasticityResult& point) noexcept {
    material::failure::Tab1ConstantFailureInput input;
    for(unsigned c=0;c<3;++c) input.current_stress[c]=point.history.stress[c];
    input.plastic_strain_increment=point.history.plastic_strain-old.plastic_strain;
    input.native_evaluation_time_s=time;
    input.element_active=base.element_active;
    material::failure::Tab1ConstantFailureResult failure;
    const auto status=PointFailureStatus(material::failure::UpdateTab1ConstantFailure(
        parameters.table,base.failure[p],input,failure));
    if(status!=PointStatus::Ok) return status;
    candidate.constitutive_increment[p]=point.plastic_increment;
    candidate.caller_failure_increment[p]=input.plastic_strain_increment;
    candidate.history.failure[p]=failure.history;
    candidate.history.saved.point[p]=point.history;
    for(unsigned c=0;c<5;++c) {
      candidate.history.current_force_point[p].stress[c]=point.history.stress[c];
      if(!failure.history.point_active) candidate.history.saved.point[p].stress[c]*=0.;
    }
    return PointStatus::Ok;
  }
};
} // namespace layered_tab1_detail
// First qualified composition: analytic LAW44 and explicit filtered zero C.
// The independent TAB1 point leaf has no quadrature or hardening restriction.
TL_SHELL_SECTION_HD inline PointStatus UpdateShellLayeredTab1(
    const PointParameters& material,const ShellLayeredTab1Parameters& failure,
    const ShellLayeredTab1History& accepted,const ShellLayeredJ2Input& input,
    double native_evaluation_time_s,ShellLayeredTab1Result& output) noexcept {
  if(!ValidLayeredJ2Parameters(material)||
      material.hardening!=tl::material::ShellPlasticityHardeningKind::LinearLaw44||
      material.rate.policy!=tl::material::ShellPlasticityRatePolicy::FilteredZeroC||
      !tl::material::failure::ValidTab1ConstantTable(failure.table)||
      failure.parent_policy!=ShellTab1ParentPolicy::AnyPoint) return PointStatus::InvalidParameters;
  if(!tl::math::Finite(native_evaluation_time_s)||native_evaluation_time_s<0)
    return PointStatus::InvalidIncrement;
  if(!layered_tab1_detail::ValidHistory(accepted)) return PointStatus::InvalidHistory;
  ShellLayeredTab1Result candidate;
  candidate.history.element_active=accepted.element_active;
  layered_tab1_detail::ObserveFailure observer{failure,accepted,native_evaluation_time_s,candidate};
  const auto status=layered_j2_detail::Update(material,accepted.saved,input,
      accepted.element_active,observer,candidate.current);
  if(status!=PointStatus::Ok) return status;
  if(accepted.element_active&&layered_tab1_detail::FailedThickness(candidate.history)>=1.e-6) {
    candidate.history.element_active=false;
    candidate.removed_now=true;
  }
  if(!candidate.history.element_active) {
    for(double& value:candidate.current.material_stress) value*=0.;
    for(double& value:candidate.current.bending_stress) value*=0.;
  }
  output=candidate;
  return PointStatus::Ok;
}
} // namespace tl::fea::sections
