// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SIGEPS44PI, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "Types.h"

namespace tl::material::law44::beam {
namespace detail {
TL_LAW44_SOLID_HD inline bool ValidHistory(const Parameters& p, const History& h) noexcept {
  if (!tl::math::Finite(h.plastic_strain) || h.plastic_strain < 0 ||
      h.plastic_strain >= solid::detail::NativeInfinity() ||
      h.curve_cursor >= p.curve.count - 1) return false;
  for (double value : h.stress_pa) if (!tl::math::Finite(value)) return false;
  return true;
}
} // namespace detail
// Closed tabulated CA0/YSCALE1, ICC1, VFLAG2 and active OFF1 branch. The caller
// owns its neutral-fiber filter, three total strains and section work. There
// is no 3D pressure, EOS, solid return mapping or independent point clock.
TL_LAW44_SOLID_HD inline Status Update(const Parameters& p, const History& accepted,
    const Input& input, Result& output) noexcept {
  if (!solid::detail::ParametersValid(p)) return Status::InvalidParameters;
  if (!detail::ValidHistory(p, accepted)) return Status::InvalidHistory;
  if (!tl::math::Finite(input.total_axial_strain) ||
      input.total_axial_strain > solid::detail::NativeInfinity() ||
      !tl::math::Finite(input.filtered_neutral_rate_per_s) ||
      input.filtered_neutral_rate_per_s < 0) return Status::InvalidInput;
  for (double value : input.strain_increment)
    if (!tl::math::Finite(value)) return Status::InvalidInput;
  Result next{};
  next.history = accepted;
  const double shear = (5. / 6.) * p.shear_pa;
  auto& stress = next.history.stress_pa;
  stress[0] = accepted.stress_pa[0] + p.material.young_pa * input.strain_increment[0];
  stress[1] = accepted.stress_pa[1] + shear * input.strain_increment[1];
  stress[2] = accepted.stress_pa[2] + shear * input.strain_increment[2];
  double yield = 0, slope = 0;
  if (!solid::detail::CurveValue(p.curve, accepted.plastic_strain,
      next.history.curve_cursor, yield, slope)) return Status::InvalidCurve;
  yield = 1. * yield;
  if (accepted.plastic_strain > 0) yield = 1. * yield;
  const double rate_factor = 1. + ::pow(input.filtered_neutral_rate_per_s *
      p.inverse_rate_c, p.inverse_rate_p);
  const double cap = p.stress_limit_pa * rate_factor;
  yield = yield * rate_factor;
  if (!tl::math::Finite(rate_factor) || !tl::math::Finite(cap) || !tl::math::Finite(yield))
    return Status::NonfiniteResult;
  yield = ::fmin(yield, cap);
  next.yield_stress_pa = yield;
  const double equivalent_square = stress[0] * stress[0] +
      3. * (stress[1] * stress[1] + stress[2] * stress[2]);
  const double yield_square = yield * yield;
  if (!tl::math::Finite(equivalent_square) || !tl::math::Finite(yield_square))
    return Status::NonfiniteResult;
  if (equivalent_square > yield_square) {
    const double equivalent = ::sqrt(equivalent_square);
    const double ratio = ::fmin(1., yield / equivalent);
    for (double& value : stress) value = value * ratio;
    next.plastic_increment = 1. * equivalent * (1. - ratio) / p.material.young_pa;
    next.history.plastic_strain = accepted.plastic_strain +
        1. * equivalent * (1. - ratio) / p.material.young_pa;
  }
  if (!detail::ValidHistory(p, next.history) ||
      !tl::math::Finite(next.plastic_increment) || next.plastic_increment < 0)
    return Status::NonfiniteResult;
  output = next;
  return Status::Ok;
}
} // namespace tl::material::law44::beam
