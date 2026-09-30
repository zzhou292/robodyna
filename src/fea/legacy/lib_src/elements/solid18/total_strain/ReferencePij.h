// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S8E_PIJ LLPIJ72 preparation, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "ReferenceJacobian.h"
#include "ReferenceTransform.h"
#include "lib_src/elements/solid18/Solid18SelectiveShear.h"

namespace tl::fea::solid18::total_strain::detail {
TL_SOLID18_HD inline Status ReferencePointCoefficients(const StartupGeometry& local,
    ReferenceCoefficients& output, CurrentGeometry& preparation) noexcept {
  Matrix3 inverse;
  solid18::detail::InverseScaledJacobian(local.center_scaled_jacobian_m,
      local.center_volume_m3, 1./64., inverse);
  solid18::detail::CenterGradient(inverse, preparation.center_gradient_per_m);
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        const unsigned ip = r+2*s+4*t;
        const auto status = ReferencePointGradient(local, ip,
            preparation.point[ip].regular_per_m);
        if (status != Status::Success) return status;
      }
    }
  }
  // Startup S8E_PIJ calls BIJ even at native nu0; signed zero is retained.
  solid18::detail::SelectiveDerivatives(0.0, preparation);
  const auto transform = MakeTransform(local.frame);
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        const unsigned ip = r+2*s+4*t;
        const auto status = TransformPoint(transform, preparation.point[ip],
                                           output.point_pij_per_m[ip]);
        if (status != Status::Success) return status;
      }
    }
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::total_strain::detail
