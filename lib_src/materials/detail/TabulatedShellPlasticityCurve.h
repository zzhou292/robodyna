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
  // The same final segment applies beyond the table; admission is a separate
  // explicit parameter policy. No extra point or interpolation branch is needed.
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
  return PrepareTabulatedShellPlasticity(young, nu, rho, curve, rate,
      ShellPlasticityCurveContinuation::StrictDomain, output);
}
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
PrepareTabulatedShellPlasticity(double young, double nu, double rho,
    TabulatedShellPlasticityCurve curve, TabulatedShellPlasticityRate rate,
    ShellPlasticityCurveContinuation continuation,
    TabulatedShellPlasticityParameters& output) noexcept {
  using Status = TabulatedShellPlasticityStatus;
  if (continuation != ShellPlasticityCurveContinuation::StrictDomain &&
      continuation != ShellPlasticityCurveContinuation::NativeLastSegment)
    return Status::InvalidParameters;
  if (!tabulated_shell_detail::ValidElasticInput(young,nu,rho)) return Status::InvalidParameters;
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
  p.curve = curve;
  p.continuation = continuation;
  if (!tabulated_shell_detail::PrepareElastic(young,nu,rho,p)) return Status::InvalidParameters;
  output = p;
  return Status::Ok;
}
} // namespace tl::material
