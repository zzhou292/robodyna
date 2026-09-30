// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "State.h"
#include "lib_src/elements/beam18/Model.h"
#include "../../ShellBatchStartup.h"

namespace tl::fea::beam18::batch_detail {
TL_BEAM18_HD inline bool ValidResult(const Parent& parent, const Material& material,
    const State& state, double time, std::uint64_t epoch) noexcept {
  const auto& h = state.proposed_history;
  const auto& d = state.diagnostics;
  using shell_startup_detail::SameBits;
  if (!h.prepared() || h.stamp().time_s != time || h.stamp().sample_index != epoch ||
      !force_detail::SameReference(parent.reference, h.reference()) ||
      !force_detail::SameMaterial(material, h.material()) ||
      !force_detail::MaterialValid(parent.reference, material) ||
      !force_detail::HistoryValid(material, h.values()) ||
      !detail::Positive(state.geometry.length_m) ||
      !detail::Positive(d.translation_stiffness_n_m) || !detail::Positive(d.rotation_stiffness_nm) ||
      !detail::Positive(d.minimum_unscaled_dt_s) ||
      !tl::math::fixed3::Finite(d.damped_section_force_n) ||
      !tl::math::fixed3::Finite(d.damped_section_moment_nm) ||
      !tl::math::Finite(d.plastic_work_increment_j) || d.plastic_work_increment_j < 0)
    return false;
  for (double v : d.internal_work_increment_j) if (!tl::math::Finite(v)) return false;
  for (unsigned n = 0; n < 2; ++n)
    if (!tl::math::fixed3::Finite(state.rhs_force_n[n]) || !tl::math::fixed3::Finite(state.rhs_couple_nm[n]))
      return false;
  for (const auto axis : state.geometry.axis)
    if (!tl::math::fixed3::Finite(axis) || ::fabs(tl::math::fixed3::Dot(axis, axis) - 1.) > 1e-10)
      return false;
  if (!shell_startup_detail::SameVector(state.geometry.axis[1], h.values().section_seed)) return false;
  const auto& rate = state.rate;
  const double rates[]{rate.axial, rate.shear_y, rate.shear_z,
      rate.curvature_x, rate.curvature_y, rate.curvature_z};
  for (double v : rates) if (!tl::math::Finite(v)) return false;
  for (unsigned p = 0; p < 4; ++p) {
    const auto& point = state.point[p];
    const auto& accepted = h.values().point[p];
    if (!detail::Positive(point.yield_stress_pa) || point.tangent_factor != 1 ||
        !tl::math::Finite(point.plastic_increment) || point.plastic_increment < 0 ||
        point.history.curve_cursor != accepted.curve_cursor ||
        !SameBits(point.history.plastic_strain, accepted.plastic_strain)) return false;
    for (unsigned c = 0; c < 3; ++c)
      if (!SameBits(point.history.stress_pa[c], accepted.stress_pa[c])) return false;
  }
  return true;
}
} // namespace tl::fea::beam18::batch_detail
