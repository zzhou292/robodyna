// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellLayeredJ2Work.h"

namespace tl::fea::sections {
// Shared section seam for both native family cores. Geometry, force projection
// and stabilization remain in their owning family; no extra material evaluation.
struct LayeredJ2ForceAdapter {
  static constexpr bool admits_failure=false;
  using Result=ShellLayeredJ2Result;
  const PointParameters& parameters;
  const ShellLayeredJ2History& accepted;
  template<class H> TL_SHELL_SECTION_HD bool Matches(const H& h,
      ShellReferencePlacement placement=ShellReferencePlacement::Centered) const noexcept {
    if(placement!=ShellReferencePlacement::Centered) return false;
    return MatchesLayeredJ2Resultants(accepted,h);
  }
  TL_SHELL_SECTION_HD PointStatus Update(const ShellLayeredJ2Input& input,double,Result& result) const noexcept {
    return UpdateShellLayeredJ2(parameters,accepted,input,result);
  }
  template<class H> TL_SHELL_SECTION_HD bool Apply(const Result& result,const double (&dx)[8],
      double thickness,double area,double viscosity,H& h,
      ShellReferencePlacement placement=ShellReferencePlacement::Centered) const noexcept {
    if(placement!=ShellReferencePlacement::Centered) return false;
    return ApplyLayeredJ2Work(result,dx,thickness,area,viscosity,h);
  }
  TL_SHELL_SECTION_HD static const ShellLayeredJ2Diagnostics& Diagnostics(const Result& r) noexcept {
    return r.diagnostics;
  }
  template<class T> TL_SHELL_SECTION_HD static void Publish(const Result& r,T& output) noexcept {
    output.proposed_section=r.history;output.section_diagnostics=r.diagnostics;
  }
};
} // namespace tl::fea::sections
