// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellLayeredTab1Work.h"

namespace tl::fea::sections {
inline TL_SHELL_SECTION_HD bool ValidLayeredTab1ForceParameters(
    const PointParameters& material,const ShellLayeredTab1Parameters& failure) noexcept {
  return ValidLayeredJ2Parameters(material)&&
      material.hardening==tl::material::ShellPlasticityHardeningKind::LinearLaw44&&
      material.rate.policy==tl::material::ShellPlasticityRatePolicy::FilteredZeroC&&
      tl::material::failure::ValidTab1ConstantTable(failure.table)&&
      failure.parent_policy==ShellTab1ParentPolicy::AnyPoint;
}

// The family core retains geometry, coefficient, force and publication order.
// This adapter changes only its explicitly typed constitutive/failure caller.
struct LayeredTab1ForceAdapter {
  static constexpr bool admits_failure=true;
  using Result=ShellLayeredTab1Result;
  const PointParameters& parameters;
  const ShellLayeredTab1History& accepted;
  const ShellLayeredTab1Parameters& failure;

  template<class H>
  TL_SHELL_SECTION_HD bool Matches(const H& h) const noexcept {
    return h.active==(accepted.element_active?1:0)&&
        MatchesLayeredTab1Resultants(accepted,h);
  }
  TL_SHELL_SECTION_HD PointStatus Update(const ShellLayeredJ2Input& input,
      double time,Result& result) const noexcept {
    return UpdateShellLayeredTab1(parameters,failure,accepted,input,time,result);
  }
  template<class H>
  TL_SHELL_SECTION_HD bool Apply(const Result& result,const double (&dx)[8],
      double thickness,double area,double viscosity,H& h) const noexcept {
    if(!ApplyLayeredTab1Work(result,dx,thickness,area,viscosity,h)) return false;
    h.active=result.history.element_active?1:0;
    return true;
  }
  TL_SHELL_SECTION_HD static const ShellLayeredJ2Diagnostics& Diagnostics(
      const Result& result) noexcept {
    return result.current.diagnostics;
  }
  template<class Trial>
  TL_SHELL_SECTION_HD static void Publish(const Result& result,Trial& output) noexcept {
    output.section=result;
  }
};
} // namespace tl::fea::sections
