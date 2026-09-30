// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "State.h"
#include "../../ShellBatchStartup.h"

namespace tl::fea::solids::batch_detail {
template<class Array> TL_BRICK_HD inline bool FiniteValues(const Array& values) noexcept {
  for (double value : values) if (!tl::math::Finite(value)) return false;
  return true;
}
TL_BRICK_HD inline bool Positive(double value) noexcept {
  return tl::math::Finite(value) && value > 0;
}
TL_BRICK_HD inline bool Coefficients(const NodalStiffness& value,
    double raw, double dt, double factor) noexcept {
  return Positive(raw) && Positive(dt) && Positive(value.translation_n_m) &&
      value.translation_n_m == factor * raw && value.rotation_nm == 0;
}
template<unsigned Nodes> TL_BRICK_HD inline bool Forces(
    const solid18::Vec3 (&forces)[Nodes]) noexcept {
  for (const auto& force : forces) if (!tl::math::fixed3::Finite(force)) return false;
  return true;
}
TL_BRICK_HD inline bool Material42(const tl::material::law42::CallerHistory& h) noexcept {
  return FiniteValues(h.stress_pa) && Positive(h.density_kg_m3) &&
      tl::math::Finite(h.internal_energy_density_j_m3) &&
      tl::math::Finite(h.bulk_pressure_pa) && h.bulk_pressure_pa >= 0;
}
TL_BRICK_HD inline bool Caller42(const tl::material::law42::CallerResult& value) noexcept {
  const auto& p = value.point;
  const double finite[]{p.maximum_principal_stress_pa, p.minimum_principal_stress_pa,
      p.material_viscosity_pa_s, value.volume_increment_m3, value.internal_work_j};
  if (!Material42(value.history) || !FiniteValues(p.stress_pa) || !FiniteValues(finite) ||
      !FiniteValues(value.total_strain) || p.active != 1 || !Positive(p.relative_volume) ||
      !Positive(p.sound_speed_m_s) || !Positive(p.hourglass_tangent_factor) ||
      !Positive(value.average_volume_m3) || !Positive(value.unscaled_element_dt_s) ||
      !Positive(value.raw_stiffness_n_m)) return false;
  for (unsigned k = 0; k < 6; ++k)
    if (!shell_startup_detail::SameBits(value.history.stress_pa[k], p.stress_pa[k])) return false;
  return true;
}
TL_BRICK_HD inline bool SameMaterialFields(const tl::material::law42::CallerHistory& a,
    const tl::material::law42::CallerHistory& b) noexcept {
  // EINT is updated by physical stabilization after the material observation.
  using shell_startup_detail::SameBits;
  if (!SameBits(a.density_kg_m3, b.density_kg_m3) ||
      !SameBits(a.bulk_pressure_pa, b.bulk_pressure_pa)) return false;
  for (unsigned k = 0; k < 6; ++k) if (!SameBits(a.stress_pa[k], b.stress_pa[k])) return false;
  return true;
}
TL_BRICK_HD inline bool ValidResult(const Parent18& p, const solid18::Material& material,
    const State<Traits18>& state, double time, std::uint64_t epoch) noexcept {
  const auto& h = state.history;
  const auto& c = state.cache;
  const auto& d = c.diagnostics;
  const double finite[]{d.selection_factor, d.selective_poisson_ratio,
      d.internal_work_increment_j, d.plastic_work_increment_j};
  return h.prepared() && h.stamp().time_s == time && h.stamp().sample_index == epoch &&
      solid18::detail::SameReference(p.reference, h.reference()) &&
      solid18::detail::SameMaterial(material, h.material()) &&
      solid18::detail::ValidHistory(p.reference, h.data()) && Forces(c.rhs_force_n) &&
      d.selected_point < 8 && FiniteValues(finite) && d.plastic_work_increment_j >= 0 &&
      Coefficients(c.stiffness, d.raw_stiffness_n_m, d.minimum_unscaled_dt_s, .25);
}
TL_BRICK_HD inline bool ValidResult(const Parent24& p, const solid24::Material& material,
    const State<Traits24>& state, double time, std::uint64_t epoch) noexcept {
  if(!state.history.legacy()||state.cache.profile!=ResultProfile::Legacy)return false;
  const auto& h = *state.history.legacy();
  const auto& c = state.cache;
  const auto& d = c.diagnostics;
  if (!h.initialized() || h.stamp().time_s != time || h.stamp().sample_index != epoch ||
      !solid24::force_detail::SameReference(p.reference, h.reference()) ||
      !solid24::force_detail::SameMaterial(material, h.material()) ||
      !Material42(h.values().material) || !Caller42(d.material) ||
      !SameMaterialFields(h.values().material, d.material.history) || !Forces(c.rhs_force_n) ||
      !tl::math::Finite(d.stabilization_work_j) || !Positive(d.stabilization_modulus_pa) ||
      !tl::math::Finite(d.stabilization_viscosity_kg_m_s) || d.stabilization_viscosity_kg_m_s < 0 ||
      !Coefficients(c.stiffness, d.material.raw_stiffness_n_m, d.material.unscaled_element_dt_s, .25))
    return false;
  for (const auto& row : h.values().physical_hourglass) if (!FiniteValues(row)) return false;
  return true;
}
TL_BRICK_HD inline bool ValidResult(const Parent6z& p, const solid6z::Material& material,
    const State<Traits6z>& state, double time, std::uint64_t epoch) noexcept {
  const auto& h = state.history;
  const auto& c = state.cache;
  const auto& d = c.stabilization;
  const double finite[]{d.damping_kg_m_s, d.first_work_j, d.second_work_j,
      c.total_internal_work_increment_j};
  if (!h.prepared() || h.stamp().time_s != time || h.stamp().sample_index != epoch ||
      !solid6z::force_detail::Same(p.reference, h.reference()) ||
      !solid6z::force_detail::Same(p.profile, h.profile()) ||
      !solid6z::force_detail::Same(material, h.material()) ||
      !solid6z::force_detail::Valid(h.data()) || !Caller42(c.material) ||
      !SameMaterialFields(h.data().material, c.material.history) || !Forces(c.rhs_force_n) ||
      !FiniteValues(finite) || !Positive(d.effective_shear_modulus_pa) || d.damping_kg_m_s < 0 ||
      !Coefficients(c.stiffness, c.material.raw_stiffness_n_m,
          c.material.unscaled_element_dt_s, 1.0 / 3.0)) return false;
  for (const auto& row : d.modal_velocity_m_s) if (!FiniteValues(row)) return false;
  for (const auto& row : d.modal_force_n) if (!FiniteValues(row)) return false;
  return true;
}
TL_BRICK_HD inline bool ValidResult(const Parent18Law44& p,
    const solid18::law44::Material& material, const State<Traits18Law44>& state,
    double time, std::uint64_t epoch) noexcept {
  const auto& h = state.history;
  const auto& c = state.cache;
  const auto& d = c.diagnostics;
  const auto degeneracy = solid18::law44::detail::NativeDegeneracy(p.reference);
  const double finite[]{d.center_divergence_per_s, d.mean_pressure_pa,
      d.internal_work_increment_j, d.plastic_work_increment_j};
  return h.prepared() && h.stamp().time_s == time && h.stamp().sample_index == epoch &&
      solid18::law44::detail::SameReference(p.reference, h.reference()) &&
      solid18::law44::detail::SameMaterial(material, h.material()) &&
      solid18::law44::detail::ValidMaterial(p.reference, h.material()) &&
      solid18::law44::detail::ValidHistory(p.reference, material, h.data()) &&
      Forces(c.rhs_force_n) && FiniteValues(finite) && d.plastic_work_increment_j >= 0 &&
      d.native_degeneracy == degeneracy &&
      d.caller_degeneracy == (degeneracy > 0 ? degeneracy + 10 : 0) &&
      Coefficients(c.stiffness, d.raw_stiffness_n_m, d.minimum_unscaled_dt_s, .25);
}
TL_BRICK_HD inline bool ValidResult(const Parent18Law90& p,
    const solid18::total_strain::Material& material, const State<Traits18Law90>& state,
    double time, std::uint64_t epoch) noexcept {
  if(!state.history.legacy()||state.cache.profile!=ResultProfile::Legacy)return false;
  const auto& h = *state.history.legacy();
  const auto& c = state.cache;
  const auto& d = c.diagnostics;
  return h.prepared() && h.stamp().time_s == time && h.stamp().sample_index == epoch &&
      solid18::total_strain::force_detail::SameReference(p.reference, h.reference()) &&
      tl::material::law90::SamePreparedMaterial(material, h.material()) &&
      solid18::total_strain::force_detail::ValidMaterial(p.reference, h.material()) &&
      solid18::total_strain::force_detail::ValidValues(material, h.data()) &&
      Forces(c.rhs_force_n) && tl::math::Finite(d.internal_work_increment_j) &&
      Coefficients(c.stiffness, d.raw_stiffness_n_m, d.minimum_unscaled_dt_s, .25);
}
} // namespace tl::fea::solids::batch_detail
