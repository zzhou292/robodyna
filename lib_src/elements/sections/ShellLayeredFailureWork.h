// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellLayeredWork.h"
#if defined(__CUDACC__)
#define TL_FAILURE_WORK_HD __host__ __device__
#else
#define TL_FAILURE_WORK_HD
#endif

namespace tl::fea::sections {
struct ShellFailureForcePoint { double stress[5]{}; };
template<class HistoryValues>
TL_FAILURE_WORK_HD inline bool MatchesLayeredFailureResultants(
    const ShellFailureForcePoint (&points)[3],bool active,const HistoryValues& h) noexcept {
  double force[5]{},moment[3]{};
  Nip3Resultants(points,force,moment);
  if(!active) {
    for(double& value:force) value*=0.;
    for(double& value:moment) value*=0.;
  }
  for(unsigned i=0;i<5;++i) if(force[i]!=h.material_stress[i]) return false;
  for(unsigned i=0;i<3;++i) if(moment[i]!=h.bending_stress[i]) return false;
  return true;
}

// Caller owns history and publication. Failure preserves the complete target;
// a removed parent retains its old-force contribution and accumulated work.
template<class SectionResult,class HistoryValues>
TL_FAILURE_WORK_HD inline bool ApplyLayeredFailureWork(
    const SectionResult& current,const ShellFailureForcePoint (&points)[3],bool active,
    const double (&dx)[8],
    double reference_thickness,double area,double viscosity,HistoryValues& output) noexcept {
  if(!MatchesLayeredFailureResultants(points,active,current)||
      !tl::math::Finite(reference_thickness)||!(reference_thickness>0)||
      !tl::math::Finite(area)||!(area>0)||!tl::math::Finite(viscosity)||viscosity<0)
    return false;
  if(!tl::math::Finite(current.reported_thickness)||
      current.reported_thickness<1.e-30) return false;
  for(double value:dx) if(!tl::math::Finite(value)) return false;
  for(double value:output.stress) if(!tl::math::Finite(value)) return false;
  for(double value:output.bending_stress) if(!tl::math::Finite(value)) return false;
  for(double value:output.strain_curvature) if(!tl::math::Finite(value)) return false;
  for(double value:output.internal_work) if(!tl::math::Finite(value)) return false;
  auto candidate=output;
  auto work_section=current;
  // MULAWC adds viscosity to current UNMASKED FOR before multiplying by OFF.
  // Starting from the material-only masked observable changes signed zero and
  // can hide an overflowing raw sum. Reuse the original ordered reduction.
  if(!active)
    Nip3Resultants(points,work_section.material_stress,work_section.bending_stress);
  if(!ApplyLayeredSectionWork(work_section,dx,reference_thickness,area,
      viscosity,candidate,active)) return false;
  // Keep the distinct material-only observable final-parent masked as declared.
  for(unsigned i=0;i<5;++i) candidate.material_stress[i]=current.material_stress[i];
  for(double value:candidate.stress) if(!tl::math::Finite(value)) return false;
  for(double value:candidate.strain_curvature) if(!tl::math::Finite(value)) return false;
  output=candidate;
  return true;
}
} // namespace tl::fea::sections
#undef TL_FAILURE_WORK_HD
