// SPDX-License-Identifier: AGPL-3.0-or-later
// Engine SIGEPS90:230–322, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "PointTypes.h"
#include "lib_src/math/NativeSymmetricEigen3.h"
#include "lib_src/math/PositiveIdentityPlusSymmetric3.h"

namespace tl::material::law90::point_detail {
struct Kinematics {
  tl::math::NativeSymmetricSpectrum3 spectrum{};
  double stretch[3]{};
  double compression[3]{};
  double strain_norm = 0;
  double scalar_rate = 0;
};
TL_LAW90_HD inline PointStatus ResolveKinematics(const PointKinematics& input,
                                                Kinematics& output) noexcept {
  double tensor[6];
  double rate[6];
  for (unsigned k = 0; k < 6; ++k) {
    if (!tl::math::Finite(input.total_b_minus_i_engineering[k]) ||
        !tl::math::Finite(input.engineering_rate_s_inverse[k]))
      return PointStatus::InvalidInput;
    tensor[k] = (k < 3 ? 1. : .5) * input.total_b_minus_i_engineering[k];
    rate[k] = (k < 3 ? 1. : .5) * input.engineering_rate_s_inverse[k];
  }
  if (!tl::math::PositiveIdentityPlusSymmetric3(tensor)) return PointStatus::InvalidStretch;
  if (!tl::math::NativeSymmetricEigen3(tensor, output.spectrum))
    return PointStatus::NonfiniteResult;
  for (unsigned k = 0; k < 3; ++k) {
    const double squared_stretch = output.spectrum.value[k] + 1;
    if (squared_stretch <= 0) return PointStatus::InvalidStretch;
    output.stretch[k] = ::sqrt(squared_stretch);
    output.compression[k] = 1 - output.stretch[k];
  }
  const double* strain = output.compression;
  output.strain_norm = ::sqrt(strain[0]*strain[0] + strain[1]*strain[1] + strain[2]*strain[2]);
  const double* v = output.spectrum.vectors.v;
  double principal_rate[3];
  for (unsigned k = 0; k < 3; ++k) {
    const double x = v[k]*rate[0] + v[3+k]*rate[3] + v[6+k]*rate[5];
    const double y = v[k]*rate[3] + v[3+k]*rate[1] + v[6+k]*rate[4];
    const double z = v[k]*rate[5] + v[3+k]*rate[4] + v[6+k]*rate[2];
    const double projected = v[k]*x + v[3+k]*y + v[6+k]*z;
    // Native recomputes 1-compression; do not substitute the earlier stretch.
    principal_rate[k] = projected * (1 - strain[k]);
  }
  output.scalar_rate = ::sqrt(principal_rate[0]*principal_rate[0] +
      principal_rate[1]*principal_rate[1] + principal_rate[2]*principal_rate[2]);
  if (!tl::math::Finite(output.strain_norm) || !tl::math::Finite(output.scalar_rate))
    return PointStatus::NonfiniteResult;
  return PointStatus::Ok;
}
} // namespace tl::material::law90::point_detail
