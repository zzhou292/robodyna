#pragma once
#include "../ShellBatchOnePointSection.h"

#if defined(__CUDACC__)
#define TL_ONE_POINT_HD __host__ __device__
#else
#define TL_ONE_POINT_HD
#endif
namespace tl::fea::shell_batch_plasticity_detail {
// CUDA readback may contain a corrupted bool representation. Check bytes before
// evaluating flags as bools. Object padding is never part of this value check.
TL_ONE_POINT_HD inline bool ValidOnePointEncoding(const ShellBatchOnePointSectionState& value) noexcept {
  static_assert(sizeof(bool) == 1);
  const auto& failure = value.point.failure;
  return *reinterpret_cast<const unsigned char*>(&failure.history.point_active) <= 1 &&
      *reinterpret_cast<const unsigned char*>(&failure.failed_now) <= 1;
}
TL_ONE_POINT_HD inline bool ValidOnePointState(const ShellBatchOnePointSectionState& value,
    const material::TabulatedShellPlasticityParameters& parameters, double time) noexcept {
  using tl::math::Finite;
  if (!ValidOnePointEncoding(value) || !Finite(time) || time < 0) return false;
  const auto& point = value.point;
  const auto& current = point.current;
  const auto& saved = point.saved;
  const auto& failure = point.failure.history;
  const double nonnegative[]{current.history.plastic_strain, current.history.filtered_rate_per_s,
      current.plastic_increment, current.tangent_ratio, current.yield_before_pa,
      current.equivalent_stress_pa, current.plastic_work_density,
      point.plastic_work_increment_j, value.cumulative_plastic_work_J};
  for (double x : nonnegative) {
    if (!Finite(x) || x < 0) return false;
  }
  if (!Finite(current.elastic_thickness_strain) || !Finite(current.plastic_thickness_strain) ||
      !Finite(point.reported_thickness_m) || point.reported_thickness_m <= 0 ||
      !material::tabulated_shell_detail::HardeningDomain(parameters, saved.plastic_strain) ||
      saved.plastic_strain != current.history.plastic_strain ||
      saved.filtered_rate_per_s != current.history.filtered_rate_per_s ||
      (!parameters.rate.enabled && saved.filtered_rate_per_s != 0) ||
      point.plastic_work_increment_j > value.cumulative_plastic_work_J ||
      !Finite(failure.damage) || failure.damage < 0 || failure.damage > 1 ||
      !Finite(failure.failure_time_s) || failure.failure_time_s < 0 || failure.failure_time_s > time ||
      (failure.point_active && (failure.damage >= 1 || failure.failure_time_s != 0)) ||
      (!failure.point_active && failure.damage != 1) ||
      (point.failure.failed_now && (failure.point_active || failure.failure_time_s != time))) return false;
  const double mask = failure.point_active ? 1. : 0.;
  for (unsigned i = 0; i < 5; ++i) {
    if (!Finite(current.history.stress[i]) || !Finite(saved.stress[i]) ||
        saved.stress[i] != current.history.stress[i] * mask ||
        (i >= 3 && (current.history.stress[i] != 0 || saved.stress[i] != 0))) return false;
  }
  return true;
}
} // namespace tl::fea::shell_batch_plasticity_detail
#undef TL_ONE_POINT_HD
