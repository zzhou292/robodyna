// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected FAIL_TAB_C / TABLE_VINTERP, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "../../math/Quaternion.h"
#include <cstdint>
#if defined(__CUDACC__)
#define TL_TAB1_HD __host__ __device__
#else
#define TL_TAB1_HD
#endif
namespace tl::material::failure {
// Explicit constant-valued, three-abscissa native 1D table. No generalized
// damage/temperature/size/instability table is admitted by this declaration.
struct Tab1ConstantTable {
  double triaxiality[3]{};
  double failure_strain=0;
};
struct Tab1ConstantFailureHistory {
  double damage=0; // Native UVAR1 is NOT capped at Dcrit=1.
  double maximum_damage=0; // Separate native DFMAX in [0,1].
  double failure_time_s=0;
  std::uint32_t table_segment=0; // Native initial cache0, then one-based1/2.
  bool point_active=true;
};
struct Tab1ConstantFailureInput {
  double current_stress[3]{}; // Unmasked native XX, YY, XY, Pa.
  double plastic_strain_increment=0; // Actual rounded caller PLAnew-PLAold.
  double native_evaluation_time_s=0;
  bool element_active=true;
};
enum class Tab1FailureStatus : std::uint8_t {
  Ok, InvalidParameters, InvalidHistory, InvalidIncrement, NonfiniteResult
};
struct Tab1ConstantFailureResult {
  Tab1ConstantFailureHistory history;
  bool failed_now=false;
};
TL_TAB1_HD inline bool ValidTab1ConstantTable(const Tab1ConstantTable& table) noexcept {
  using tl::math::Finite;
  if(!Finite(table.failure_strain)||!(table.failure_strain>0)) return false;
  for(unsigned i=0;i<3;++i) {
    if(!Finite(table.triaxiality[i])) return false;
    if(i && (!(table.triaxiality[i]>table.triaxiality[i-1]) ||
        !Finite(table.triaxiality[i]-table.triaxiality[i-1]))) return false;
  }
  return true;
}
// Equivalent scalar specialization of native TABLE_VINTERP's bidirectional
// search. At the middle knot native selects the segment on its left. Both
// endpoints extrapolate; the weighted arithmetic is retained even for constant Y.
TL_TAB1_HD inline bool EvaluateTab1ConstantTable(const Tab1ConstantTable& table,
    double triaxiality,std::uint32_t accepted_segment,double& value,
    std::uint32_t& next_segment) noexcept {
  using tl::math::Finite;
  if(!ValidTab1ConstantTable(table)||!Finite(triaxiality)||accepted_segment>2) return false;
  const unsigned index=triaxiality<=table.triaxiality[1]?0:1;
  const double ratio=(table.triaxiality[index+1]-triaxiality)/
      (table.triaxiality[index+1]-table.triaxiality[index]);
  const double complement=1.-ratio;
  const double result=ratio*table.failure_strain+complement*table.failure_strain;
  if(!Finite(ratio)||!Finite(complement)||!Finite(result)) return false;
  value=result;
  next_segment=index+1;
  return true;
}
TL_TAB1_HD inline bool ValidTab1ConstantHistory(const Tab1ConstantFailureHistory& h) noexcept {
  using tl::math::Finite;
  return Finite(h.damage)&&h.damage>=0&&Finite(h.maximum_damage)&&
      h.maximum_damage>=0&&h.maximum_damage<=1&&Finite(h.failure_time_s)&&h.failure_time_s>=0&&
      h.table_segment<=2&&(h.point_active?h.damage<1:h.damage>=1);
}
// Local TAB1 defaults: Dcrit1, exponent1, no softening/thinning/functions,
// IFAIL_SH1. No point stress mask, parent deletion, contact or clock is owned here.
TL_TAB1_HD inline Tab1FailureStatus UpdateTab1ConstantFailure(const Tab1ConstantTable& table,
    const Tab1ConstantFailureHistory& base,const Tab1ConstantFailureInput& input,
    Tab1ConstantFailureResult& output) noexcept {
  using tl::math::Finite;
  using Status=Tab1FailureStatus;
  if(!ValidTab1ConstantTable(table)) return Status::InvalidParameters;
  if(!ValidTab1ConstantHistory(base)) return Status::InvalidHistory;
  if(!Finite(input.plastic_strain_increment)||input.plastic_strain_increment<0||
      !Finite(input.native_evaluation_time_s)||input.native_evaluation_time_s<0||
      (!base.point_active&&base.failure_time_s>input.native_evaluation_time_s)) return Status::InvalidIncrement;
  for(double stress:input.current_stress) if(!Finite(stress)) return Status::InvalidIncrement;
  Tab1ConstantFailureResult next{base,false};
  if(input.element_active&&base.point_active) {
    const double x=input.current_stress[0],y=input.current_stress[1],xy=input.current_stress[2];
    const double pressure=(1./3.)*(x+y);
    const double equivalent=::sqrt(x*x+y*y-x*y+3.*xy*xy);
    const double triaxiality=pressure/::fmax(1.e-20,equivalent);
    if(!Finite(pressure)||!Finite(equivalent)||!Finite(triaxiality)) return Status::NonfiniteResult;
    double failure_strain=0;
    if(!EvaluateTab1ConstantTable(table,triaxiality,base.table_segment,
        failure_strain,next.history.table_segment)) return Status::NonfiniteResult;
    // Keep native DP operation order for the selected exponent=1 branch.
    const double multiplier=base.damage==0?1.:1.*::pow(base.damage,1.-1./1.);
    if(failure_strain>0)
      next.history.damage=base.damage+multiplier*input.plastic_strain_increment/failure_strain;
    if(!Finite(next.history.damage)) return Status::NonfiniteResult; // Never replace native retained overflow by a clamp.
    if(next.history.damage>=1.) {
      next.history.point_active=false;
      next.history.failure_time_s=input.native_evaluation_time_s;
      next.failed_now=true;
    }
    next.history.maximum_damage=::fmin(1.,::fmax(base.maximum_damage,next.history.damage/1.));
  }
  output=next;
  return Status::Ok;
}
} // namespace tl::material::failure
#undef TL_TAB1_HD
