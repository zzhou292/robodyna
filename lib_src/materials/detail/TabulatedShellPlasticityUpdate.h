// SPDX-License-Identifier: AGPL-3.0-or-later
// SIGEPS44C and MULAWC arithmetic adapted from OpenRadioss,
// Copyright (C) 2026 Siemens. See ../TabulatedShellPlasticity.md.
#pragma once
namespace tl::material {
namespace tabulated_shell_detail {
TL_TABULATED_SHELL_HD inline double EquivalentStress(const double (&s)[5]) noexcept {
  return ::sqrt(s[0]*s[0] + s[1]*s[1] - s[0]*s[1] + 3.*s[2]*s[2]);
}
} // namespace tabulated_shell_detail

TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
UpdateTabulatedShellPlasticity(const TabulatedShellPlasticityParameters& p,
    const TabulatedShellPlasticityHistory& accepted, const TabulatedShellPlasticityInput& input,
    TabulatedShellPlasticityResult& output) noexcept {
  using Status = TabulatedShellPlasticityStatus;
  if (!tl::math::Finite(p.young_pa) || p.young_pa <= 0 || !tl::math::Finite(p.poisson_ratio) ||
      p.poisson_ratio < 0 || p.poisson_ratio >= .5 || !tl::math::Finite(p.shear_modulus) ||
      p.shear_modulus <= 0 || !tl::math::Finite(p.a11) || p.a11 <= 0 ||
      !tl::math::Finite(p.a12) || p.a12 < 0 || !tl::math::Finite(p.three_g) || p.three_g <= 0)
    return Status::InvalidParameters;
  if (!tabulated_shell_detail::CurveShape(p.curve)) return Status::InvalidCurve;
  if (!tl::math::Finite(accepted.plastic_strain) || accepted.plastic_strain < 0)
    return Status::InvalidHistory;
  for (double s : accepted.stress) if (!tl::math::Finite(s)) return Status::InvalidHistory;
  if (accepted.plastic_strain > p.curve.plastic_strain[p.curve.count - 1])
    return Status::CurveDomainExceeded;
  if (!tl::math::Finite(input.transverse_shear_modulus) || input.transverse_shear_modulus <= 0)
    return Status::InvalidIncrement;
  for (double x : input.strain_increment) if (!tl::math::Finite(x)) return Status::InvalidIncrement;
  double yield = 0, hardening = 0;
  if (!tabulated_shell_detail::CurveValue(p.curve, accepted.plastic_strain, yield, hardening))
    return Status::InvalidCurve;
  double filtered_rate = 0, rate_factor = 1;
  const auto rate_status = tabulated_shell_detail::FilteredRate(p, accepted, input, filtered_rate, rate_factor);
  if (rate_status != Status::Ok) return rate_status;
  if (p.rate.enabled) { yield = yield * rate_factor; hardening = hardening * rate_factor; }
  // Preserve the donor's special virgin branch, including its finite-step
  // hardening behavior. The curve slope replaces E after accumulated PLA > 0.
  if (accepted.plastic_strain == 0) hardening = p.young_pa;
  yield = ::fmax(yield, 1.e-20);
  if (!tl::math::Finite(yield) || !tl::math::Finite(hardening)) return Status::NonfiniteResult;
  TabulatedShellPlasticityResult trial;
  trial.history = accepted; trial.yield_before_pa = yield;
  trial.history.filtered_rate_per_s = filtered_rate;
  auto& s = trial.history.stress;
  const auto& dx = input.strain_increment;
  s[0] = accepted.stress[0] + p.a11*dx[0] + p.a12*dx[1];
  s[1] = accepted.stress[1] + p.a12*dx[0] + p.a11*dx[1];
  s[2] = accepted.stress[2] + p.shear_modulus*dx[2];
  s[3] = accepted.stress[3] + input.transverse_shear_modulus*dx[3];
  s[4] = accepted.stress[4] + input.transverse_shear_modulus*dx[4];
  const double nu = p.poisson_ratio, nnu11 = nu/(1. - nu);
  trial.elastic_thickness_strain = -(dx[0] + dx[1])*nnu11;
  double s1 = s[0] + s[1], s2 = s[0] - s[1];
  const double aa = .25*s1*s1, bb = .75*s2*s2 + 3.*s[2]*s[2];
  const double svm = ::sqrt(aa + bb);
  if (!tl::math::Finite(svm)) return Status::NonfiniteResult;
  if (svm > yield) {
    hardening = ::fmax(0., hardening);
    double next = (svm - yield)/(p.three_g + hardening), dr = 0, pp = 0, qq = 0;
    trial.tangent_ratio = hardening/(hardening + p.young_pa);
    const double nu11 = 1./(1. - nu), nu21 = 1./(1. + nu), nu31 = 1. - nnu11;
    for (int iteration = 0; iteration < 3; ++iteration) {
      // Native commits the third evaluated iterate, not next (the fourth guess).
      trial.plastic_increment = next;
      const double y = yield + hardening*trial.plastic_increment;
      dr = .5*p.young_pa*trial.plastic_increment/y;
      pp = 1./(1. + dr*nu11); qq = 1./(1. + 3.*dr*nu21);
      const double p2 = pp*pp, q2 = qq*qq;
      const double f = aa*p2 + bb*q2 - y*y;
      double df = -(aa*nu11*p2*pp + 3.*bb*nu21*q2*qq)*
          (p.young_pa - 2.*dr*hardening)/y - 2.*hardening*y;
      if (!tl::math::Finite(y) || y <= 0 || !tl::math::Finite(dr) ||
          !tl::math::Finite(pp) || !tl::math::Finite(qq) || !tl::math::Finite(f) ||
          !tl::math::Finite(df)) return Status::NonfiniteResult;
      df = ::copysign(::fmax(::fabs(df), 1.e-20), df);
      if (trial.plastic_increment > 0) {
        const double correction = f/df;
        if (!tl::math::Finite(correction)) return Status::NonfiniteResult;
        next = ::fmax(0., trial.plastic_increment - correction);
      } else next = 0.;
      if (!tl::math::Finite(next)) return Status::NonfiniteResult;
    }
    trial.history.plastic_strain = accepted.plastic_strain + trial.plastic_increment;
    s1 = (s[0] + s[1])*pp; s2 = (s[0] - s[1])*qq;
    s[0] = .5*(s1 + s2); s[1] = .5*(s1 - s2); s[2] = s[2]*qq;
    trial.plastic_thickness_strain = -nu31*dr*s1/p.young_pa;
  }
  if (!tl::math::Finite(trial.history.plastic_strain)) return Status::NonfiniteResult;
  if (trial.history.plastic_strain > p.curve.plastic_strain[p.curve.count - 1])
    return Status::CurveDomainExceeded;
  trial.equivalent_stress_pa = tabulated_shell_detail::EquivalentStress(s);
  trial.plastic_work_density = .5*(tabulated_shell_detail::EquivalentStress(accepted.stress) +
      trial.equivalent_stress_pa)*trial.plastic_increment;
  for (double x : s) if (!tl::math::Finite(x)) return Status::NonfiniteResult;
  if (!tl::math::Finite(trial.tangent_ratio) || !tl::math::Finite(trial.elastic_thickness_strain) ||
      !tl::math::Finite(trial.plastic_thickness_strain) ||
      !tl::math::Finite(trial.equivalent_stress_pa) ||
      !tl::math::Finite(trial.plastic_work_density)) return Status::NonfiniteResult;
  output = trial;
  return Status::Ok;
}
} // namespace tl::material
