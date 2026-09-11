// SPDX-License-Identifier: AGPL-3.0-or-later
// S8E_SIGP/S8ZSIGP3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18ForceTypes.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline Status SelectAcceptedPoint(const Material& material,
    const HistoryValues& history, ForceDiagnostics& result) noexcept {
  double minimum = history.global.plastic_strain;
  unsigned selected = 0;
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        const unsigned ip = r+2*s+4*t;
        const double plastic = history.point[ip].material.point.plastic_strain;
        if (plastic <= minimum && plastic > 0) {
          selected = ip;
          minimum = plastic;
        }
      }
    }
  }
  result.selected_point = selected;
  const double nu = ::fmin(.5, material.poisson_ratio);
  // PM32 is the bulk modulus; preserve the caller's reconstruction of E0.
  const double young = 3*(1-2*nu)*material.bulk_pa;
  double factor = 0;
  if (history.global.plastic_strain > 0) {
    const auto& stress = history.point[selected].material.point.stress_pa;
    const double s1 = stress[0]-stress[1];
    const double s2 = stress[1]-stress[2];
    const double s3 = stress[0]-stress[2];
    const double square = (s1*s1+s2*s2+s3*s3)*.5+
        3*(stress[3]*stress[3]+stress[4]*stress[4]+stress[5]*stress[5]);
    const double total = ::sqrt(square)/young+history.global.plastic_strain;
    factor = history.global.plastic_strain/total;
  }
  result.selection_factor = factor;
  result.selective_poisson_ratio = nu;
  if (factor > 0) result.selective_poisson_ratio = nu+(.5-nu)*factor;
  if (!tl::math::Finite(factor) || !tl::math::Finite(result.selective_poisson_ratio))
    return Status::NonfiniteResult;
  return Status::Success;
}
}  // namespace tl::fea::solid18::detail
