// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid6zForceTypes.h"
#include "lib_src/materials/law42/Prepare.h"
#include <limits>

namespace tl::fea::solid6z::force_detail {
namespace common = tl::fea::solid_common;
TL_BRICK_HD inline bool Valid(const ForceProfile& p) noexcept {
  return p.stabilization == StabilizationProfile::Law42MaterialSoundSpeedV1 &&
      tl::math::Finite(p.damping_coefficient) && p.damping_coefficient >= 0 &&
      p.engine_frame == 1 && p.integration_control == 2 &&
      p.compressibility_control == 1 && p.degenerate_step_control == 0;
}
TL_BRICK_HD inline bool Same(const ForceProfile& a, const ForceProfile& b) noexcept {
  return a.stabilization == b.stabilization && a.damping_coefficient == b.damping_coefficient &&
      a.engine_frame == b.engine_frame && a.integration_control == b.integration_control &&
      a.compressibility_control == b.compressibility_control &&
      a.degenerate_step_control == b.degenerate_step_control;
}
TL_BRICK_HD inline bool Same(const Material& a, const Material& b) noexcept {
  return a.mu_pa == b.mu_pa && a.poisson_ratio == b.poisson_ratio &&
      a.bulk_pa == b.bulk_pa && a.density_kg_m3 == b.density_kg_m3 &&
      a.tension_cutoff_pa == b.tension_cutoff_pa;
}
TL_BRICK_HD inline bool Same(const Reference& a, const Reference& b) noexcept {
  if (!a.prepared() || !b.prepared()) return false;
  const auto& x = a.input();
  const auto& y = b.input();
  if (x.source_element_id != y.source_element_id || x.source_part_id != y.source_part_id ||
      x.source_section_id != y.source_section_id || x.source_material_id != y.source_material_id ||
      x.density_kg_m3 != y.density_kg_m3) return false;
  const auto& p = x.profile;
  const auto& q = y.profile;
  if (p.engine_jhbe != q.engine_jhbe || p.integration_points != q.integration_points ||
      p.strain_formulation != q.strain_formulation || p.mass_distribution != q.mass_distribution ||
      p.orthotropic_frame != q.orthotropic_frame || p.thermal != q.thermal || p.ale != q.ale ||
      p.reference_shape != q.reference_shape) return false;
  for (unsigned n = 0; n < 6; ++n) {
    if (x.source_node_id[n] != y.source_node_id[n] || a.source_slot(n) != b.source_slot(n)) return false;
    for (unsigned k = 0; k < 3; ++k) {
      if (common::Component(x.position_m[n],k) != common::Component(y.position_m[n],k)) return false;
    }
  }
  return true;
}
TL_BRICK_HD inline bool Valid(const HistoryValues& value) noexcept {
  const auto& m = value.material;
  if (!common::Positive(m.density_kg_m3) || !tl::math::Finite(m.bulk_pressure_pa) ||
      m.bulk_pressure_pa < 0 || !tl::math::Finite(m.internal_energy_density_j_m3)) return false;
  for (double stress : m.stress_pa) {
    if (!tl::math::Finite(stress)) return false;
  }
  for (const auto& component : value.hourglass_stress_pa) {
    for (double stress : component) {
      if (!tl::math::Finite(stress)) return false;
    }
  }
  return true;
}
TL_BRICK_HD inline bool Valid(const HistoryStamp& stamp) noexcept {
  return tl::math::Finite(stamp.time_s) && stamp.time_s >= 0;
}
TL_BRICK_HD inline bool Valid(const PrescribedInterval& interval, HistoryStamp base) noexcept {
  const double end = interval.base_time_s+interval.dt_s;
  if (!common::Positive(interval.dt_s) || !tl::math::Finite(end) ||
      end <= interval.base_time_s || interval.base_time_s != base.time_s ||
      interval.sample_index != base.sample_index ||
      base.sample_index == std::numeric_limits<std::uint64_t>::max()) return false;
  for (unsigned n = 0; n < 6; ++n) {
    if (!common::Finite(interval.position_endpoint_m[n]) ||
        !common::Finite(interval.velocity_midpoint_m_s[n])) return false;
  }
  return true;
}
} // namespace tl::fea::solid6z::force_detail
