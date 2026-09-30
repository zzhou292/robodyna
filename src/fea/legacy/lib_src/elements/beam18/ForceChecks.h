// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceTypes.h"

namespace tl::fea::beam18::force_detail {
TL_BEAM18_HD inline bool MaterialValid(const Reference& reference, const Material& material) noexcept {
  if (!reference.prepared() || !point::detail::ParametersValid(material)) return false;
  const bool working = reference.input().units == WorkingUnits::TonneMillimetreSecond;
  const auto& m = material.material;
  return m.young_pa == reference.input().young * (working ? 1e6 : 1.) &&
      m.density_kg_m3 == reference.input().density * (working ? 1e12 : 1.) &&
      m.poisson_ratio == reference.input().poisson &&
      m.native_units == (working ? tl::material::law44::solid::WorkingUnits::TonneMillimetreSecond :
          tl::material::law44::solid::WorkingUnits::SI);
}
TL_BEAM18_HD inline bool SameReference(const Reference& a, const Reference& b) noexcept {
  const auto& x = a.input(); const auto& y = b.input();
  if (!a.prepared() || !b.prepared() || x.source_element_id != y.source_element_id ||
      x.source_part_id != y.source_part_id || x.source_section_id != y.source_section_id ||
      x.source_material_id != y.source_material_id || x.units != y.units || x.profile != y.profile ||
      x.radius != y.radius || x.density != y.density || x.young != y.young ||
      x.poisson != y.poisson || x.local != y.local) return false;
  for (unsigned i = 0; i < 3; ++i)
    if (x.source_node_id[i] != y.source_node_id[i] || !detail::SamePosition(x.position[i],y.position[i])) return false;
  for (unsigned i = 0; i < 4; ++i) if (x.release[i] != y.release[i]) return false;
  return true;
}
TL_BEAM18_HD inline bool SameMaterial(const Material& a, const Material& b) noexcept {
  const auto& x = a.material; const auto& y = b.material;
  return x.hardening == y.hardening && x.young_pa == y.young_pa && x.poisson_ratio == y.poisson_ratio &&
      x.density_kg_m3 == y.density_kg_m3 && x.rate_c_per_s == y.rate_c_per_s &&
      x.rate_p == y.rate_p && x.cutoff_hz == y.cutoff_hz && x.native_units == y.native_units &&
      a.curve.count == b.curve.count && a.curve.plastic_strain == b.curve.plastic_strain &&
      a.curve.yield_stress_pa == b.curve.yield_stress_pa;
}
TL_BEAM18_HD inline bool HistoryValid(const Material& material, const HistoryValues& value) noexcept {
  for (unsigned p = 0; p < 4; ++p) {
    if (!point::detail::ValidHistory(material,value.point[p])) return false;
    for (double strain : value.total_strain[p]) if (!tl::math::Finite(strain)) return false;
  }
  return tl::math::fixed3::Finite(value.section_seed) &&
      detail::Positive(tl::math::fixed3::Dot(value.section_seed,value.section_seed)) &&
      tl::math::fixed3::Finite(value.section_force_n) && tl::math::fixed3::Finite(value.section_moment_nm) &&
      tl::math::Finite(value.filtered_neutral_rate_per_s) && value.filtered_neutral_rate_per_s >= 0 &&
      tl::math::Finite(value.internal_energy_j[0]) && tl::math::Finite(value.internal_energy_j[1]) &&
      tl::math::Finite(value.plastic_work_j) && value.plastic_work_j >= 0;
}
TL_BEAM18_HD inline bool IntervalValid(const ForceHistory& history, const PrescribedInterval& in) noexcept {
  if (!history.prepared() || !detail::Positive(in.dt_s) || !tl::math::Finite(in.base_time_s) ||
      in.base_time_s != history.stamp().time_s || history.stamp().sample_index == UINT64_MAX ||
      in.sample_index != history.stamp().sample_index + 1 ||
      !tl::math::Finite(in.base_time_s + in.dt_s) || !(in.base_time_s + in.dt_s > in.base_time_s)) return false;
  for (unsigned i = 0; i < 2; ++i)
    if (!tl::math::fixed3::Finite(in.position_endpoint_m[i]) ||
        !tl::math::fixed3::Finite(in.velocity_midpoint_m_s[i]) ||
        !tl::math::fixed3::Finite(in.angular_velocity_midpoint_rad_s[i])) return false;
  return true;
}
TL_BEAM18_HD inline Section SectionSI(const Reference& reference) noexcept {
  auto value = reference.section();
  if (reference.input().units == WorkingUnits::TonneMillimetreSecond) {
    constexpr double length = .001, area = length * length, inertia = area * area;
    value.area *= area; value.inertia_x *= inertia; value.inertia_y *= inertia; value.inertia_z *= inertia;
    for (auto& p : value.point) { p.y *= length; p.z *= length; p.area *= area; }
  }
  return value;
}
} // namespace tl::fea::beam18::force_detail
