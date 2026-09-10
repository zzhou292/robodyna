// SPDX-License-Identifier: AGPL-3.0-or-later
// Constant-strain branch of OpenRadioss FAIL_JOHNSON_C (C) 2026 Siemens.
#pragma once
#include "../../math/Quaternion.h"

#if defined(__CUDACC__)
#define TL_SHELL_FAILURE_HD __host__ __device__
#else
#define TL_SHELL_FAILURE_HD
#endif

namespace tl::material::failure {
// D1 > 0, D2=D3=D4=D5=0, EPSF_MIN=0. This is the converter's constant
// plastic-strain failure criterion, separate from the constitutive LAW44 law.
struct ConstantPlasticFailureParameters { double failure_strain = 0; };
struct ConstantPlasticFailureHistory {
  double damage = 0;
  double failure_time_s = 0; // Applicable only after point_active becomes false.
  bool point_active = true;
};
struct ConstantPlasticFailureInput {
  double plastic_strain_increment = 0;
  double native_evaluation_time_s = 0;
  bool element_active = true; // Exactly native OFF==1, not merely OFF!=0.
};
struct ConstantPlasticFailureResult {
  ConstantPlasticFailureHistory history;
  bool failed_now = false;
};

// Point failure only: no stress deletion, element erosion, contact removal,
// work bookkeeping, clock or accepted-state selection belongs to this leaf.
// All failures leave output untouched; base/result history may alias.
TL_SHELL_FAILURE_HD inline bool UpdateConstantPlasticFailure(
    const ConstantPlasticFailureParameters& parameters,
    const ConstantPlasticFailureHistory& base,
    const ConstantPlasticFailureInput& input,
    ConstantPlasticFailureResult& output) noexcept {
  using tl::math::Finite;
  if (!Finite(parameters.failure_strain) || !(parameters.failure_strain > 0) ||
      !Finite(base.damage) || base.damage < 0 || base.damage > 1 ||
      !Finite(base.failure_time_s) || base.failure_time_s < 0 ||
      !Finite(input.plastic_strain_increment) || input.plastic_strain_increment < 0 ||
      !Finite(input.native_evaluation_time_s) || input.native_evaluation_time_s < 0 ||
      (!base.point_active && base.failure_time_s > input.native_evaluation_time_s)) return false;
  ConstantPlasticFailureResult next{base, false};
  if (input.element_active && base.point_active && input.plastic_strain_increment > 0) {
    next.history.damage = base.damage + input.plastic_strain_increment / parameters.failure_strain;
    if (next.history.damage >= 1) {
      next.history.point_active = false;
      next.history.failure_time_s = input.native_evaluation_time_s;
      next.failed_now = true;
    }
  }
  // Native saturation follows failure detection, including positive overflow.
  if (next.history.damage > 1) next.history.damage = 1;
  if (!Finite(next.history.damage)) return false;
  output = next;
  return true;
}
} // namespace tl::material::failure
#undef TL_SHELL_FAILURE_HD
