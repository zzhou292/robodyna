// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "PointTypes.h"

namespace tl::material::law90::point_detail {
TL_LAW90_HD inline PointStatus CheckHistory(const PreparedMaterial& material,
                                           const PointHistory& h) noexcept {
  const double values[]{h.stress_norm_pa, h.maximum_path_energy_pa,
      h.scalar_rate_s_inverse, h.path_energy_pa, h.reserved5, h.strain_norm,
      h.unloading_factor, h.effective_modulus_pa,
      h.instantaneous_quasistatic_energy_pa, h.residual_strain};
  for (double value : values)
    if (!tl::math::Finite(value)) return PointStatus::InvalidHistory;
  if (h.stress_norm_pa < 0 || h.path_energy_pa < 0 ||
      h.maximum_path_energy_pa < h.path_energy_pa || h.scalar_rate_s_inverse < 0 ||
      h.strain_norm < 0 || h.unloading_factor != 1 ||
      h.effective_modulus_pa < material.updated().young_pa ||
      h.effective_modulus_pa > material.updated().maximum_modulus_pa ||
      h.instantaneous_quasistatic_energy_pa < 0 ||
      h.residual_strain < 0 || h.residual_strain > 1)
    return PointStatus::InvalidHistory;
  for (auto cursor : h.cursor)
    if (cursor >= material.curve().count - 1) return PointStatus::InvalidCursor;
  return PointStatus::Ok;
}
TL_LAW90_HD inline PointStatus CheckResult(const PreparedMaterial& material,
                                          const PointResult& result) noexcept {
  if (CheckHistory(material, result.history) != PointStatus::Ok)
    return PointStatus::NonfiniteResult;
  for (double stress : result.cauchy_stress_pa)
    if (!tl::math::Finite(stress)) return PointStatus::NonfiniteResult;
  if (!tl::math::Finite(result.sound_speed_m_s) || result.sound_speed_m_s <= 0 ||
      !tl::math::Finite(result.scalar_rate_s_inverse) || result.scalar_rate_s_inverse < 0 ||
      !tl::math::Finite(result.tangent_factor) || result.tangent_factor < 0)
    return PointStatus::NonfiniteResult;
  return PointStatus::Ok;
}
} // namespace tl::material::law90::point_detail
