// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceTypes.h"

namespace tl::fea::solid18::law44::detail {
TL_SOLID18_HD inline bool SameReference(const Reference& a, const Reference& b) noexcept {
  if (!a.prepared() || !b.prepared()) return false;
  const auto& x = a.input();
  const auto& y = b.input();
  if (!Supported(x.profile) || !Supported(y.profile) ||
      x.source_element_id != y.source_element_id || x.source_part_id != y.source_part_id ||
      x.source_section_id != y.source_section_id || x.source_material_id != y.source_material_id ||
      !SameScalar(x.density_kg_m3,y.density_kg_m3)) return false;
  for (unsigned n = 0; n < 8; ++n) {
    if (x.source_node_id[n] != y.source_node_id[n] || !SamePosition(x.position_m[n],y.position_m[n]))
      return false;
  }
  return true;
}
TL_SOLID18_HD inline bool ValidMaterial(const Reference& reference, const Material& material) noexcept {
  return reference.prepared() && Supported(reference.input().profile) &&
      point::detail::ParametersValid(material) &&
      SameScalar(reference.input().density_kg_m3,material.material.density_kg_m3);
}
TL_SOLID18_HD inline bool SameMaterial(const Material& a, const Material& b) noexcept {
  const auto& x = a.material;
  const auto& y = b.material;
  // Prepared derived fields are authenticated separately by ParametersValid.
  return a.curve.plastic_strain == b.curve.plastic_strain &&
      a.curve.yield_stress_pa == b.curve.yield_stress_pa && a.curve.count == b.curve.count &&
      x.native_units == y.native_units && SameScalar(x.young_pa,y.young_pa) &&
      SameScalar(x.poisson_ratio,y.poisson_ratio) && SameScalar(x.density_kg_m3,y.density_kg_m3) &&
      SameScalar(x.rate_c_per_s,y.rate_c_per_s) && SameScalar(x.rate_p,y.rate_p) &&
      SameScalar(x.cutoff_hz,y.cutoff_hz) && x.hardening == y.hardening &&
      SameScalar(x.analytic.a_pa,y.analytic.a_pa) && SameScalar(x.analytic.b_pa,y.analytic.b_pa) &&
      SameScalar(x.analytic.exponent,y.analytic.exponent) &&
      SameScalar(x.analytic.maximum_stress_pa,y.analytic.maximum_stress_pa) &&
      SameScalar(x.analytic.maximum_plastic_strain,y.analytic.maximum_plastic_strain);
}
TL_SOLID18_HD inline bool Nonnegative(double value) noexcept {
  return tl::math::Finite(value) && value >= 0;
}
TL_SOLID18_HD inline bool ValidHistory(const Reference& reference, const Material& material,
                                      const HistoryValues& values) noexcept {
  for (unsigned ip = 0; ip < 8; ++ip) {
    const auto& p = values.point[ip];
    if (!solid18::detail::Positive(p.density_kg_m3) || !solid18::detail::Positive(p.storage_volume_m3) ||
        !SameScalar(p.initial_volume_m3,reference.geometry().point[ip].initial_volume_m3) ||
        !tl::math::Finite(p.energy_density_j_m3) || !Nonnegative(p.plastic_work_j) ||
        !Nonnegative(p.bulk_pressure_pa) || !point::detail::HistoryValid(p.material,material)) return false;
  }
  const auto& g = values.global;
  for (double stress : g.stress_pa) if (!tl::math::Finite(stress)) return false;
  if (!solid18::detail::Positive(g.density_kg_m3) || !Nonnegative(g.plastic_strain) ||
      !Nonnegative(g.filtered_rate_per_s) || !tl::math::Finite(g.energy_density_j_m3) ||
      !Nonnegative(g.plastic_work_j) || !Nonnegative(g.bulk_pressure_pa)) return false;
  for (auto x : values.saved_local_position_m) if (!solid18::detail::Finite(x)) return false;
  return true;
}
TL_SOLID18_HD inline bool ValidInterval(const History& accepted, const PrescribedInterval& interval) noexcept {
  if (!accepted.prepared() || !solid18::detail::Positive(interval.dt_s) ||
      !SameScalar(accepted.stamp().time_s,interval.base_time_s) ||
      accepted.stamp().sample_index == UINT64_MAX ||
      interval.sample_index != accepted.stamp().sample_index+1) return false;
  const double endpoint = interval.base_time_s+interval.dt_s;
  if (!tl::math::Finite(endpoint) || endpoint <= interval.base_time_s) return false;
  const auto& ids = accepted.reference().input().source_node_id;
  for (unsigned n = 0; n < 8; ++n) {
    if (!solid18::detail::Finite(interval.position_endpoint_m[n]) ||
        !solid18::detail::Finite(interval.velocity_midpoint_m_s[n])) return false;
    for (unsigned p = 0; p < n; ++p) {
      if (ids[n] == ids[p] &&
          (!SamePosition(interval.position_endpoint_m[n],interval.position_endpoint_m[p]) ||
           !SamePosition(interval.velocity_midpoint_m_s[n],interval.velocity_midpoint_m_s[p]))) return false;
    }
  }
  return true;
}
}  // namespace tl::fea::solid18::law44::detail
