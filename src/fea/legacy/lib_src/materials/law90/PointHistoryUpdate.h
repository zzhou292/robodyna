// SPDX-License-Identifier: AGPL-3.0-or-later
// Engine SIGEPS90:370–401,662–689,807–820; OpenRadioss (C) 2026 Siemens.
#pragma once
#include "PointCurves.h"

namespace tl::material::law90::point_detail {
TL_LAW90_HD inline bool UpdateLoading(const Kinematics& kinematics,
    const CurveResponse& response, PointResult& result) noexcept {
  PointHistory& h = result.history;
  const double energy_increment = response.quasistatic_energy - h.instantaneous_quasistatic_energy_pa;
  h.instantaneous_quasistatic_energy_pa = response.quasistatic_energy;
  const double strain_increment = kinematics.strain_norm - h.strain_norm;
  const bool unloading = energy_increment < 0 && strain_increment < 0;
  result.scalar_rate_s_inverse = kinematics.scalar_rate;
  if (unloading)
    result.scalar_rate_s_inverse = detail::Minimum(result.scalar_rate_s_inverse, h.scalar_rate_s_inverse);
  h.scalar_rate_s_inverse = result.scalar_rate_s_inverse;
  return unloading;
}
TL_LAW90_HD inline void UpdatePath(const PreparedMaterial& material,
    const Kinematics& kinematics, bool unloading, CurveResponse& response,
    PointHistory& h) noexcept {
  const double delta = kinematics.strain_norm - h.strain_norm;
  h.path_energy_pa = h.path_energy_pa + .5*(response.stress_norm + h.stress_norm_pa)*delta;
  h.path_energy_pa = detail::Maximum(0, h.path_energy_pa);
  h.maximum_path_energy_pa = detail::Maximum(h.maximum_path_energy_pa, h.path_energy_pa);
  h.stress_norm_pa = response.stress_norm;
  h.strain_norm = kinematics.strain_norm;
  if (unloading && h.maximum_path_energy_pa > 0) {
    double damage = 1 - ::pow(h.path_energy_pa/h.maximum_path_energy_pa, material.reader().shape);
    damage = ::pow(damage, material.reader().alpha);
    damage = 1 - (1 - material.reader().hysteresis)*damage;
    h.unloading_factor = damage;
    for (unsigned k = 0; k < 3; ++k) response.stress[k] = damage * response.stress[k];
  }
}
TL_LAW90_HD inline void UpdateModulus(const PreparedMaterial& material,
    const Kinematics& kinematics, const CurveResponse& response,
    PointResult& result) noexcept {
  PointHistory& h = result.history;
  double residual = kinematics.strain_norm - response.stress_norm/h.effective_modulus_pa;
  residual = detail::Maximum(0, residual);
  residual = detail::Minimum(1, residual);
  const double difference = material.updated().maximum_modulus_pa - material.updated().young_pa;
  double modulus = difference*residual + material.updated().young_pa;
  modulus = detail::Minimum(material.updated().maximum_modulus_pa, modulus);
  h.residual_strain = residual;
  h.effective_modulus_pa = modulus;
  const double nu = material.reader().poisson_ratio;
  const double longitudinal = modulus*(1 - nu)/(1 + nu)/(1 - 2*nu);
  result.sound_speed_m_s = ::sqrt(longitudinal/material.reader().reference_density_kg_m3);
  result.maximum_viscosity_pa_s = 0;
}
} // namespace tl::material::law90::point_detail
