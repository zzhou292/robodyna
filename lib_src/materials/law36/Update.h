// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law36/Prepare.h"
namespace tl::material::law36 {
namespace detail {
TL_LAW36_HD inline double EquivalentStress(const double (&s)[6]) noexcept {
  const double d12 = s[0] - s[1];
  const double d23 = s[1] - s[2];
  const double d31 = s[2] - s[0];
  return ::sqrt(.5 * (d12*d12 + d23*d23 + d31*d31) +
                3 * (s[3]*s[3] + s[4]*s[4] + s[5]*s[5]));
}
TL_LAW36_HD inline double DeviatoricRate(const double (&rate)[6]) noexcept {
  const double mean = (rate[0] + rate[1] + rate[2]) * (1.0 / 3.0);
  const double e1 = rate[0] - mean;
  const double e2 = rate[1] - mean;
  const double e3 = rate[2] - mean;
  const double e4 = .5 * rate[3];
  const double e5 = .5 * rate[4];
  const double e6 = .5 * rate[5];
  const double square = .5 * (e1*e1 + e2*e2 + e3*e3) +
                        e4*e4 + e5*e5 + e6*e6;
  return ::sqrt(3 * square) / 1.5;  // MSTRAIN_RATE IDEV1/ISRATE0.
}
TL_LAW36_HD inline Status HistoryStatus(const History& h) noexcept {
  if (!tl::math::Finite(h.plastic_strain) || h.plastic_strain < 0 ||
      !tl::math::Finite(h.deviatoric_rate_per_s) || h.deviatoric_rate_per_s < 0)
    return Status::InvalidHistory;
  double strain_bound = 0;
  for (unsigned i = 0; i < 6; ++i) {
    if (!tl::math::Finite(h.stress_pa[i]) || !tl::math::Finite(h.engineering_strain[i]))
      return Status::InvalidHistory;
    strain_bound += ::fabs(h.engineering_strain[i]);
  }
  // The sum bounds every principal engineering-strain tensor eigenvalue.
  // Stay below the native positive finite failure sentinels, without simulating failure.
  if (h.plastic_strain >= NativeSentinel() ||
      strain_bound >= NativeSentinel() * .25) return Status::SentinelDomainExceeded;
  return Status::Ok;
}
TL_LAW36_HD inline bool Virgin(const History& h) noexcept {
  if (h.plastic_strain != 0 || h.deviatoric_rate_per_s != 0) return false;
  for (unsigned i = 0; i < 6; ++i) {
    if (h.stress_pa[i] != 0 || h.engineering_strain[i] != 0) return false;
  }
  return true;
}
}  // namespace detail

TL_LAW36_HD inline Status Update(const Parameters& p, const History& accepted,
                                const Input& input, Result& output) noexcept {
  if (!detail::ParametersValid(p)) return Status::InvalidParameters;
  const Status prior = detail::HistoryStatus(accepted);
  if (prior != Status::Ok) return prior;
  if (!tl::math::Finite(input.kinematics.dt_s) || input.kinematics.dt_s < 0 ||
      !tl::math::Finite(input.relative_density) || input.relative_density < -1)
    return Status::InvalidInput;
  if (input.kinematics.dt_s == 0 &&
      (!detail::Virgin(accepted) || input.relative_density != 0))
    return Status::InvalidInput;
  Result result{};
  result.history = accepted;
  double increment[6]{};
  for (unsigned i = 0; i < 6; ++i) {
    if (!tl::math::Finite(input.kinematics.engineering_rate_per_s[i])) return Status::InvalidInput;
    increment[i] = input.kinematics.engineering_rate_per_s[i] * input.kinematics.dt_s;
    result.history.engineering_strain[i] += increment[i];
  }
  const Status strain_status = detail::HistoryStatus(result.history);
  if (strain_status != Status::Ok) return strain_status;
  result.history.deviatoric_rate_per_s =
      detail::DeviatoricRate(input.kinematics.engineering_rate_per_s);

  const double mean_increment = (increment[0] + increment[1] + increment[2]) * (1.0 / 3.0);
  const double old_pressure =
      -(accepted.stress_pa[0] + accepted.stress_pa[1] + accepted.stress_pa[2]) * (1.0 / 3.0);
  double (&stress)[6] = result.history.stress_pa;
  for (unsigned i = 0; i < 3; ++i) {
    stress[i] = accepted.stress_pa[i] + old_pressure +
                p.twice_shear_pa * (increment[i] - mean_increment);
  }
  for (unsigned i = 3; i < 6; ++i) {
    stress[i] = accepted.stress_pa[i] + p.shear_pa * increment[i];
  }
  double hardening = 0;
  if (!tl::material::detail::VinterValue(p.curve, accepted.plastic_strain,
                                        result.yield_stress_pa, hardening))
    return Status::InvalidCurve;
  const double trial_square = 3 * (.5 * (stress[0]*stress[0] +
      stress[1]*stress[1] + stress[2]*stress[2]) +
      stress[3]*stress[3] + stress[4]*stress[4] + stress[5]*stress[5]);
  const double yield_square = result.yield_stress_pa * result.yield_stress_pa;
  if (!tl::math::Finite(trial_square) || !tl::math::Finite(yield_square))
    return Status::NonfiniteResult;
  // SIGEPS36 explicit IPLA1, FISOKIN0, NRATE1: evaluate the curve at old PLA,
  // then retain the native single linearized hardening update.
  if (trial_square > yield_square) {
    const double equivalent_trial = ::sqrt(trial_square);
    double ratio = result.yield_stress_pa / ::fmax(equivalent_trial, 1e-20);
    result.plastic_increment = (1 - ratio) * equivalent_trial /
                               ::fmax(p.three_shear_pa + hardening, 1e-20);
    result.yield_stress_pa =
        ::fmax(result.yield_stress_pa + (1 - 0.0) * result.plastic_increment * hardening, 0.0);
    ratio = ::fmin(1.0, result.yield_stress_pa / ::fmax(equivalent_trial, 1e-20));
    for (unsigned i = 0; i < 6; ++i) {
      stress[i] = stress[i] * ratio;
    }
    result.history.plastic_strain += result.plastic_increment;
  }
  const double pressure = p.bulk_pa * input.relative_density;
  for (unsigned i = 0; i < 3; ++i) {
    stress[i] = stress[i] - pressure;
  }
  result.sound_speed_m_s = p.sound_speed_m_s;
  result.equivalent_stress_pa = detail::EquivalentStress(stress);
  const double old_equivalent = detail::EquivalentStress(accepted.stress_pa);
  result.plastic_work_density_j_m3 = .5 * (old_equivalent + result.equivalent_stress_pa) *
      (result.history.plastic_strain - accepted.plastic_strain);
  const Status final_status = detail::HistoryStatus(result.history);
  if (final_status == Status::SentinelDomainExceeded) return final_status;
  if (final_status != Status::Ok || !tl::math::Finite(result.plastic_increment) ||
      !tl::math::Finite(result.yield_stress_pa) ||
      !tl::math::Finite(result.equivalent_stress_pa) ||
      !tl::math::Finite(result.plastic_work_density_j_m3))
    return Status::NonfiniteResult;
  output = result;
  return Status::Ok;
}
}  // namespace tl::material::law36
