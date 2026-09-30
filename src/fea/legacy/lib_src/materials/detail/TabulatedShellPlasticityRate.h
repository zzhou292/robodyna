// SPDX-License-Identifier: AGPL-3.0-or-later
// SIGEPS44C VP2 / MULAWC filter adapted from OpenRadioss (C) 2026 Siemens.
#pragma once
namespace tl::material::tabulated_shell_detail {
TL_TABULATED_SHELL_HD inline bool ZeroRateStrengthening(
    TabulatedShellPlasticityRate rate) noexcept {
  return rate.policy == ShellPlasticityRatePolicy::FilteredZeroC;
}
TL_TABULATED_SHELL_HD inline bool ValidRate(TabulatedShellPlasticityRate rate) noexcept {
  if (ZeroRateStrengthening(rate)) {
    return rate.enabled && rate.cowper_symonds_c_per_s == 0 &&
        rate.cowper_symonds_p == 1 && tl::math::Finite(rate.cutoff_hz) && rate.cutoff_hz > 0;
  }
  if (rate.policy != ShellPlasticityRatePolicy::Legacy) return false;
  if (!rate.enabled) {
    return rate.cowper_symonds_c_per_s == 0 && rate.cowper_symonds_p == 0 && rate.cutoff_hz == 0;
  }
  return tl::math::Finite(rate.cowper_symonds_c_per_s) && rate.cowper_symonds_c_per_s > 0 &&
      tl::math::Finite(rate.cowper_symonds_p) && rate.cowper_symonds_p > 0 &&
      tl::math::Finite(rate.cutoff_hz) && rate.cutoff_hz > 0;
}
TL_TABULATED_SHELL_HD inline bool PreparedRate(const TabulatedShellPlasticityParameters& p) noexcept {
  const bool valid_c = ZeroRateStrengthening(p.rate) ? p.inverse_rate_c == 0 :
      tl::math::Finite(p.inverse_rate_c) && p.inverse_rate_c > 0;
  return valid_c && tl::math::Finite(p.inverse_rate_p) && p.inverse_rate_p > 0 &&
      (!ZeroRateStrengthening(p.rate) || p.inverse_rate_p == 1) &&
      tl::math::Finite(p.angular_cutoff_per_s) && p.angular_cutoff_per_s > 0;
}
TL_TABULATED_SHELL_HD inline bool PrepareRate(TabulatedShellPlasticityParameters& p,
    TabulatedShellPlasticityRate rate) noexcept {
  if (!ValidRate(rate)) return false;
  p.rate = rate;
  if (!rate.enabled) return true;
  // HM_READ_MAT44 keeps CC=0 for the explicit zero-C branch. It must not be
  // represented by a large positive C or by disabling ISRATE.
  p.inverse_rate_c = ZeroRateStrengthening(rate) ? 0. : 1. / rate.cowper_symonds_c_per_s;
  p.inverse_rate_p = 1. / rate.cowper_symonds_p;
  p.angular_cutoff_per_s = 2. * ::atan2(0., -1.) * rate.cutoff_hz;
  return PreparedRate(p);
}
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus FilteredRate(
    const TabulatedShellPlasticityParameters& p, const TabulatedShellPlasticityHistory& accepted,
    const TabulatedShellPlasticityInput& in, double& filtered, double& factor) noexcept {
  using Status = TabulatedShellPlasticityStatus;
  if (!ValidRate(p.rate)) return Status::InvalidParameters;
  if (!tl::math::Finite(accepted.filtered_rate_per_s) || accepted.filtered_rate_per_s < 0 ||
      (!p.rate.enabled && accepted.filtered_rate_per_s != 0)) return Status::InvalidHistory;
  filtered = accepted.filtered_rate_per_s;
  factor = 1.;
  if (!p.rate.enabled) return Status::Ok;
  if (!PreparedRate(p)) return Status::InvalidParameters;
  if (!tl::math::Finite(in.dt) || in.dt <= 0 ||
      !tl::math::Finite(in.total_strain_rate_per_s) || in.total_strain_rate_per_s < 0)
    return Status::InvalidIncrement;
  const double alpha = ::fmin(1., p.angular_cutoff_per_s * in.dt);
  filtered = alpha * in.total_strain_rate_per_s + (1. - alpha) * accepted.filtered_rate_per_s;
  // Native SIGEPS44C evaluates this power only for CC1 /= ZERO. The filter
  // above still advances UVAR when CC1 is zero, including parent OFF0.
  if (!ZeroRateStrengthening(p.rate))
    factor = 1. + ::pow(p.inverse_rate_c * filtered, p.inverse_rate_p);
  return tl::math::Finite(filtered) && tl::math::Finite(factor) ? Status::Ok : Status::NonfiniteResult;
}
} // namespace tl::material::tabulated_shell_detail
