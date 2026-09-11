// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellLayeredJ2Failure.h"
namespace tl::fea::sections {
template<class HistoryValues>
TL_SHELL_SECTION_HD inline bool MatchesLayeredJ2FailureResultants(
    const ShellLayeredJ2FailureHistory& section,const HistoryValues& h,
    ShellReferencePlacement placement=ShellReferencePlacement::Centered) noexcept {
  if(!layered_j2_failure_detail::ValidHistory(section)) return false;
  return MatchesLayeredFailureResultants(section.current_force_point,section.element_active,h,placement);
}
template<class HistoryValues>
TL_SHELL_SECTION_HD inline bool ApplyLayeredJ2FailureWork(
    const ShellLayeredJ2FailureResult& section,const double (&dx)[8],
    double reference_thickness,double area,double viscosity,HistoryValues& output,
    ShellReferencePlacement placement=ShellReferencePlacement::Centered) noexcept {
  if(!layered_j2_failure_detail::ValidHistory(section.history)) return false;
  return ApplyLayeredFailureWork(section.current,section.history.current_force_point,
      section.history.element_active,dx,reference_thickness,area,viscosity,output,placement);
}
} // namespace tl::fea::sections
