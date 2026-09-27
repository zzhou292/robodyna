// SPDX-License-Identifier: AGPL-3.0-or-later
// HM_READ_MAT42 -> HM_READ_MAT -> UPDMAT, OpenRadioss a62b27e6 (C) 2026 Siemens.
#pragma once
#include "Prepare.h"
namespace tl::material::law42 {
// Mechanical PM slots after the no-Prony update. In this source path PM32 is
// GS, whereas PM100 retains reader BULK. Neither is interchangeable with G.
struct MechanicalSlots {
  double pm20_young_pa=0, pm21_poisson_ratio=0, pm22_gs_pa=0;
  double pm32_pa=0, pm100_reader_bulk_pa=0, pm107_control_pa=0;
};
TL_LAW42_HD inline Status PrepareMechanicalSlots(const Parameters& material,
                                                MechanicalSlots& output) noexcept {
  Parameters canonical;
  const auto status=Prepare(material.mu_pa,material.poisson_ratio,material.density_kg_m3,
                            material.tension_cutoff_pa,canonical);
  if(status!=Status::Ok || !tl::math::Finite(material.bulk_pa) ||
     material.bulk_pa!=canonical.bulk_pa)return Status::InvalidParameters;
  double gs=0;
  gs=gs+material.mu_pa*2.0; // HM_READ_MAT42 GS loop, alpha=2, one term.
  MechanicalSlots next;
  next.pm20_young_pa=gs*(1.0+material.poisson_ratio);
  next.pm21_poisson_ratio=material.poisson_ratio;
  next.pm22_gs_pa=gs;
  next.pm32_pa=gs; // HM_READ_MAT: BULK=PARMAT(1), not reader BULK.
  next.pm100_reader_bulk_pa=canonical.bulk_pa;
  next.pm107_control_pa=2.0*::fmax(next.pm32_pa,next.pm100_reader_bulk_pa);
  if(!tl::math::Finite(next.pm20_young_pa)||!tl::math::Finite(next.pm107_control_pa))
    return Status::NonfiniteResult;
  output=next;
  return Status::Ok;
}
} // namespace tl::material::law42
