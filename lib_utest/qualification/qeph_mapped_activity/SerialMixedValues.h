#pragma once
#include "lib_src/elements/ShellBatchLayeredSection.h"

namespace qeph_activity_test::frozen_mixed {
using namespace tl::fea;
inline bool FiniteSection(const sections::ShellLayeredLaw1History& value) noexcept {
  for(const auto& point:value.point)for(double stress:point.stress)if(!tl::math::Finite(stress))return false;
  return true;
}
inline bool FiniteSection(const ShellBatchSectionState& value) noexcept {
  for(const auto& point:value.history.point) {
    for(double stress:point.stress)if(!tl::math::Finite(stress))return false;
    if(!tl::math::Finite(point.plastic_strain)||!tl::math::Finite(point.filtered_rate_per_s))return false;
  }
  const auto& d=value.diagnostics;
  for(double x:{d.plastic_work_density_increment,d.maximum_plastic_strain,d.mean_plastic_strain,
      d.minimum_tangent_ratio,d.mean_tangent_ratio,d.mean_yield_before_pa,d.last_point_yield_before_pa,
      value.cumulative_plastic_work_J})if(!tl::math::Finite(x))return false;
  return true;
}
} // namespace tl::fea::shell_batch_plasticity_detail
