// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18Reference.h"
#include "Solid18ForceTypes.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline bool SameScalar(double a, double b) noexcept {
  const auto* x = reinterpret_cast<const unsigned char*>(&a);
  const auto* y = reinterpret_cast<const unsigned char*>(&b);
  for (unsigned i = 0; i < sizeof(double); ++i) {
    if (x[i] != y[i]) return false;
  }
  return true;
}

TL_SOLID18_HD inline bool SameVector(const Vec3& a, const Vec3& b) noexcept {
  return SameScalar(a.x,b.x) && SameScalar(a.y,b.y) && SameScalar(a.z,b.z);
}

TL_SOLID18_HD inline bool SameReference(const Reference& a, const Reference& b) noexcept {
  if (!a.prepared() || !b.prepared()) return false;
  const auto& x = a.input();
  const auto& y = b.input();
  if (!Supported(x.profile) || !Supported(y.profile) ||
      x.source_element_id != y.source_element_id || x.source_part_id != y.source_part_id ||
      x.source_section_id != y.source_section_id || x.source_material_id != y.source_material_id ||
      !SameScalar(x.density_kg_m3,y.density_kg_m3)) return false;
  for (unsigned n = 0; n < 8; ++n) {
    if (x.source_node_id[n] != y.source_node_id[n] ||
        !SameVector(x.position_m[n],y.position_m[n])) return false;
  }
  return true;
}

TL_SOLID18_HD inline bool ValidMaterial(const Reference& reference,
                                       const Material& material) noexcept {
  return reference.prepared() && Supported(reference.input().profile) &&
      tl::material::law36::detail::ParametersValid(material) &&
      SameScalar(reference.input().density_kg_m3,material.density_kg_m3);
}

TL_SOLID18_HD inline bool SameMaterial(const Material& a, const Material& b) noexcept {
  return a.curve.count == b.curve.count &&
      a.curve.plastic_strain == b.curve.plastic_strain &&
      a.curve.yield_stress_pa == b.curve.yield_stress_pa &&
      SameScalar(a.young_pa,b.young_pa) && SameScalar(a.poisson_ratio,b.poisson_ratio) &&
      SameScalar(a.density_kg_m3,b.density_kg_m3) && SameScalar(a.shear_pa,b.shear_pa) &&
      SameScalar(a.twice_shear_pa,b.twice_shear_pa) &&
      SameScalar(a.three_shear_pa,b.three_shear_pa) && SameScalar(a.bulk_pa,b.bulk_pa) &&
      SameScalar(a.sound_speed_m_s,b.sound_speed_m_s);
}

TL_SOLID18_HD inline bool ValidHistory(const Reference& reference,
                                      const HistoryValues& history) noexcept {
  for (unsigned ip = 0; ip < 8; ++ip) {
    const auto& point = history.point[ip];
    if (!Positive(point.density_kg_m3) || !Positive(point.storage_volume_m3) ||
        !SameScalar(point.initial_volume_m3,reference.geometry().point[ip].initial_volume_m3) ||
        !tl::math::Finite(point.bulk_pressure_pa) || point.bulk_pressure_pa < 0 ||
        !tl::math::Finite(point.material.internal_energy_density_j_m3) ||
        !tl::math::Finite(point.material.plastic_work_j) || point.material.plastic_work_j < 0 ||
        tl::material::law36::detail::HistoryStatus(point.material.point) !=
            tl::material::law36::Status::Ok) return false;
  }
  const auto& global = history.global;
  for (double value : global.stress_pa) {
    if (!tl::math::Finite(value)) return false;
  }
  if (!Positive(global.density_kg_m3) ||
      !tl::math::Finite(global.plastic_strain) || global.plastic_strain < 0 ||
      !tl::math::Finite(global.internal_energy_density_j_m3) ||
      !tl::math::Finite(global.plastic_work_j) || global.plastic_work_j < 0 ||
      !tl::math::Finite(global.bulk_pressure_pa) || global.bulk_pressure_pa < 0) return false;
  for (const auto& position : history.saved_local_position_m) {
    if (!Finite(position)) return false;
  }
  return true;
}

TL_SOLID18_HD inline bool ValidInterval(const History& history,
                                       const PrescribedInterval& interval) noexcept {
  if (!history.prepared() || !Positive(interval.dt_s) ||
      !SameScalar(history.stamp().time_s,interval.base_time_s) ||
      history.stamp().sample_index == UINT64_MAX ||
      interval.sample_index != history.stamp().sample_index+1) return false;
  const double endpoint = interval.base_time_s+interval.dt_s;
  if (!tl::math::Finite(endpoint) || endpoint <= interval.base_time_s) return false;
  for (unsigned n = 0; n < 8; ++n) {
    if (!Finite(interval.position_endpoint_m[n]) ||
        !Finite(interval.velocity_midpoint_m_s[n])) return false;
  }
  return true;
}
}  // namespace tl::fea::solid18::detail
