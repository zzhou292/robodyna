// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellLayeredJ2FailureWork.h"

namespace tl::fea::sections {
struct LayeredJ2FailureForceAdapter {
  static constexpr bool admits_failure=true;
  using Result=ShellLayeredJ2FailureResult;
  const PointParameters& parameters;
  const ShellLayeredJ2FailureHistory& accepted;
  const ConstantFailureParameters& failure;
  template<class H> TL_SHELL_SECTION_HD bool Matches(const H& h) const noexcept {
    return h.active==(accepted.element_active?1:0)&&MatchesLayeredJ2FailureResultants(accepted,h);
  }
  TL_SHELL_SECTION_HD PointStatus Update(const ShellLayeredJ2Input& input,double time,Result& result) const noexcept {
    return UpdateShellLayeredJ2Failure(parameters,failure,accepted,input,time,result);
  }
  template<class H> TL_SHELL_SECTION_HD bool Apply(const Result& result,const double (&dx)[8],
      double thickness,double area,double viscosity,H& h) const noexcept {
    if(!ApplyLayeredJ2FailureWork(result,dx,thickness,area,viscosity,h)) return false;
    h.active=result.history.element_active?1:0;return true;
  }
  TL_SHELL_SECTION_HD static const ShellLayeredJ2Diagnostics& Diagnostics(const Result& r) noexcept {
    return r.current.diagnostics;
  }
  template<class T> TL_SHELL_SECTION_HD static void Publish(const Result& r,T& output) noexcept {
    output.section=r;
  }
};
} // namespace tl::fea::sections
