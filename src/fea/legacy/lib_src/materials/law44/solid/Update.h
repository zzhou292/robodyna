// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law44/solid/Rate.h"
#include "lib_src/materials/law44/solid/Tension.h"

namespace tl::material::law44::solid {
namespace detail {
TL_LAW44_SOLID_HD inline bool HistoryValid(const History& h, Curve c) noexcept {
  if (!tl::math::Finite(h.plastic_strain) || h.plastic_strain < 0 ||
      !tl::math::Finite(h.filtered_rate_per_s) || h.filtered_rate_per_s < 0 ||
      c.count < 2 || h.curve_cursor >= c.count - 1) return false;
  for (unsigned i = 0; i < 6; ++i) {
    if (!tl::math::Finite(h.stress_pa[i]) || !tl::math::Finite(h.engineering_strain[i])) return false;
  }
  return true;
}
TL_LAW44_SOLID_HD inline bool HistoryValid(const History& h, const Parameters& p) noexcept {
  if (p.material.hardening == HardeningKind::Tabulated) return HistoryValid(h, p.curve);
  if (p.material.hardening != HardeningKind::Analytic || !EmptyCurve(p.curve) ||
      h.curve_cursor != 0 || !tl::math::Finite(h.plastic_strain) || h.plastic_strain < 0 ||
      !tl::math::Finite(h.filtered_rate_per_s) || h.filtered_rate_per_s < 0) return false;
  for (unsigned i = 0; i < 6; ++i)
    if (!tl::math::Finite(h.stress_pa[i]) || !tl::math::Finite(h.engineering_strain[i])) return false;
  return true;
}
}  // namespace detail

namespace detail {
// Shared arithmetic. Only the separate virgin constructor admits zero dt.
TL_LAW44_SOLID_HD inline Status UpdateValues(const Parameters& p, const History& accepted,
    const Input& in, Result& output, bool initialization) noexcept {
  if (!detail::ParametersValid(p)) return Status::InvalidParameters;
  if (!detail::HistoryValid(accepted, p)) return Status::InvalidHistory;
  if (!tl::math::Finite(in.dt_s) || in.dt_s < 0 || (!initialization && in.dt_s == 0) ||
      !tl::math::Finite(in.relative_density) ||
      in.relative_density < -1) return Status::InvalidInput;
  for (double rate : in.engineering_rate_per_s) {
    if (!tl::math::Finite(rate)) return Status::InvalidInput;
  }
  Result r{};
  r.history = accepted;
  double increment[6]{};
  // MULAW's six independent rate*DT1 then strain += increment operations.
  for (unsigned i = 0; i < 6; ++i) {
    increment[i] = in.engineering_rate_per_s[i] * in.dt_s;
    r.history.engineering_strain[i] += increment[i];
    if (!tl::math::Finite(increment[i]) ||
        !tl::math::Finite(r.history.engineering_strain[i])) return Status::NonfiniteResult;
  }
  double rate_factor = 0, failure_factor = 0;
  if (!detail::FilterRate(p, accepted, in, r.history.filtered_rate_per_s, rate_factor) ||
      !detail::TensionFactor(r.history.engineering_strain, failure_factor)) return Status::NonfiniteResult;
  const double old_pressure = -(accepted.stress_pa[0] + accepted.stress_pa[1] +
                               accepted.stress_pa[2]) * (1. / 3.);
  const double mean_increment = (increment[0] + increment[1] + increment[2]) * (1. / 3.);
  auto& stress = r.history.stress_pa;
  for (unsigned i = 0; i < 3; ++i) {
    stress[i] = accepted.stress_pa[i] + old_pressure +
                p.twice_shear_pa * (increment[i] - mean_increment);
  }
  for (unsigned i = 3; i < 6; ++i) stress[i] = accepted.stress_pa[i] + p.shear_pa * increment[i];
  double yield = 0, slope = 0;
  const bool analytic = p.material.hardening == HardeningKind::Analytic;
  if (!analytic && !detail::CurveValue(p.curve, accepted.plastic_strain, r.history.curve_cursor,
                                      yield, slope)) return Status::InvalidCurve;
  const double cap = p.stress_limit_pa * rate_factor;  // Native ICC1.
  double hardening = p.material.young_pa;
  if (accepted.plastic_strain > 0) {
    if (analytic) {
      const auto& a = p.material.analytic;
      yield = (a.a_pa + a.b_pa * ::pow(accepted.plastic_strain, a.exponent)) * rate_factor;
      hardening = failure_factor * a.exponent * a.b_pa * rate_factor /
          ::pow(accepted.plastic_strain, 1 - a.exponent);
    } else {
      yield = yield * rate_factor;  // YSCALE1/CA0; source SIGY is not added.
      hardening = failure_factor * (slope * rate_factor);
    }
    if (!tl::math::Finite(yield) || !tl::math::Finite(cap)) return Status::NonfiniteResult;
    yield = failure_factor * ::fmin(cap, yield);
  } else {
    yield = failure_factor * (analytic ? p.material.analytic.a_pa : yield) * rate_factor;
  }
  if (accepted.plastic_strain >= p.plastic_cap_strain) {
    yield = failure_factor * cap;  // EPSGM reached, then EPMAX below.
    hardening = 0;
  }
  r.yield_stress_pa = yield;
  if (accepted.plastic_strain >= p.failure_plastic_strain) yield = 0;
  const double square = .5 * (stress[0]*stress[0] + stress[1]*stress[1] +
                             stress[2]*stress[2]) + stress[3]*stress[3] +
                        stress[4]*stress[4] + stress[5]*stress[5];
  const double equivalent = ::sqrt(3 * square);
  const double denominator = p.three_shear_pa + hardening;
  if (!tl::math::Finite(equivalent) || !tl::math::Finite(denominator) ||
      !tl::math::Finite(yield) || !tl::math::Finite(cap)) return Status::NonfiniteResult;
  const double vm_floor = ::fmax(equivalent, p.stress_floor_pa);
  double ratio = ::fmin(1., yield / vm_floor);
  r.plastic_increment = (1 - ratio) * equivalent / ::fmax(denominator, p.stress_floor_pa);
  yield = yield + r.plastic_increment * hardening;
  if (accepted.plastic_strain >= p.failure_plastic_strain) yield = 0;
  if (!tl::math::Finite(yield)) return Status::NonfiniteResult;
  ratio = ::fmin(1., yield / vm_floor);
  for (double& value : stress) value = value * ratio;
  r.history.plastic_strain += r.plastic_increment;
  const double pressure = p.bulk_pa * in.relative_density;
  for (unsigned i = 0; i < 3; ++i) stress[i] = stress[i] - pressure;
  r.sound_speed_m_s = p.sound_speed_m_s;
  r.yield_stress_pa = ::fmax(r.yield_stress_pa, yield);
  r.tangent_factor = r.plastic_increment > 0 ? hardening / (hardening + p.material.young_pa) : 1;
  if (!detail::HistoryValid(r.history, p) || !tl::math::Finite(r.plastic_increment) ||
      !tl::math::Finite(r.yield_stress_pa) || !tl::math::Finite(r.tangent_factor)) return Status::NonfiniteResult;
  output = r;
  return Status::Ok;
}
}  // namespace detail
// Closed active, convected MULAW/SIGEPS44 packet. No frame transformation,
// EOS/current-density reconstruction, energy update or dt0 interval is implied.
TL_LAW44_SOLID_HD inline Status Update(const Parameters& p, const History& accepted,
    const Input& in, Result& output) noexcept {
  return detail::UpdateValues(p,accepted,in,output,false);
}
// Native TT0 evaluation constructs its own virgin history. It is not a
// zero-duration update of an existing accepted material state.
TL_LAW44_SOLID_HD inline Status Initialize(const Parameters& p, const Input& in,
    Result& output) noexcept {
  if (in.dt_s != 0) return Status::InvalidInput;
  return detail::UpdateValues(p,History{},in,output,true);
}
}  // namespace tl::material::law44::solid
