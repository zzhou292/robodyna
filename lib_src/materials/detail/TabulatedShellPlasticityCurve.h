// SPDX-License-Identifier: AGPL-3.0-or-later
// VINTER interpolation adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Included by TabulatedShellPlasticity.h; caller owns immutable curve storage.
#pragma once
namespace tl::material {
namespace tabulated_shell_detail {
TL_TABULATED_SHELL_HD inline bool CurveShape(TabulatedShellPlasticityCurve c) noexcept {
  return c.plastic_strain && c.yield_stress_pa && c.count >= 2 && c.count <= 1024;
}
TL_TABULATED_SHELL_HD inline bool CurveValue(TabulatedShellPlasticityCurve c, double x,
    double& value, double& slope) noexcept {
  // Native VINTER uses the left segment at a knot (strict X > next knot).
  // A fresh search avoids an additional history/cache transaction.
  std::uint32_t low = 0, high = c.count - 1;
  while (high - low > 1) {
    const auto middle = low + (high - low) / 2;
    if (x > c.plastic_strain[middle]) low = middle;
    else high = middle;
  }
  const double width = c.plastic_strain[low + 1] - c.plastic_strain[low];
  slope = (c.yield_stress_pa[low + 1] - c.yield_stress_pa[low]) / width;
  value = c.yield_stress_pa[low] + slope * (x - c.plastic_strain[low]);
  return width > 0 && tl::math::Finite(slope) && slope >= 0 &&
         tl::math::Finite(value) && value > 0;
}
} // namespace tabulated_shell_detail

TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
PrepareTabulatedShellPlasticity(double young, double nu, double rho,
    TabulatedShellPlasticityCurve curve, TabulatedShellPlasticityParameters& output) noexcept {
  return PrepareTabulatedShellPlasticity(young, nu, rho, curve, {}, output);
}
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
PrepareTabulatedShellPlasticity(double young, double nu, double rho,
    TabulatedShellPlasticityCurve curve, TabulatedShellPlasticityRate rate,
    TabulatedShellPlasticityParameters& output) noexcept {
  using Status = TabulatedShellPlasticityStatus;
  if (!tl::math::Finite(young) || young <= 0 || !tl::math::Finite(nu) || nu < 0 || nu >= .5 ||
      !tl::math::Finite(rho) || rho <= 0) return Status::InvalidParameters;
  if (!tabulated_shell_detail::CurveShape(curve) || curve.plastic_strain[0] != 0)
    return Status::InvalidCurve;
  for (std::uint32_t i = 0; i < curve.count; ++i) {
    if (!tl::math::Finite(curve.plastic_strain[i]) ||
        !tl::math::Finite(curve.yield_stress_pa[i]) || curve.yield_stress_pa[i] <= 0 ||
        (i && (curve.plastic_strain[i] <= curve.plastic_strain[i - 1] ||
               curve.yield_stress_pa[i] < curve.yield_stress_pa[i - 1])))
      return Status::InvalidCurve;
    if (i) {
      double value = 0, slope = 0;
      if (!tabulated_shell_detail::CurveValue(curve, curve.plastic_strain[i], value, slope))
        return Status::InvalidCurve;
    }
  }
  TabulatedShellPlasticityParameters p;
  if (!tabulated_shell_detail::PrepareRate(p, rate)) return Status::InvalidParameters;
  p.curve = curve; p.young_pa = young; p.poisson_ratio = nu; p.density_kg_m3 = rho;
  // Pinned HM_READ_MAT44 coefficient order; native sound speed is sqrt(A11/rho).
  p.shear_modulus = young / 2. / (1. + nu);
  p.a11 = young / (1. - nu * nu); p.a12 = p.a11 * nu;
  p.three_g = 3. * p.shear_modulus; p.sound_speed = ::sqrt(p.a11 / rho);
  if (!tl::math::Finite(p.shear_modulus) || p.shear_modulus <= 0 ||
      !tl::math::Finite(p.a11) || p.a11 <= 0 || !tl::math::Finite(p.a12) ||
      !tl::math::Finite(p.three_g) || p.three_g <= 0 ||
      !tl::math::Finite(p.sound_speed) || p.sound_speed <= 0) return Status::InvalidParameters;
  output = p;
  return Status::Ok;
}
} // namespace tl::material
