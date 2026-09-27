// SPDX-License-Identifier: AGPL-3.0-or-later
// HM_READ_MAT90 -> HM_READ_MAT -> LAW90_UPD -> UPDMAT (OpenRadioss a62).
#pragma once
#include "Types.h"
namespace tl::material::law90 {
struct MechanicalSlots {
  double pm21_poisson_ratio=0,pm22_shear_pa=0,pm32_bulk_pa=0;
  double pm100_reader_contact_bulk_pa=0,pm107_control_pa=0;
};
TL_LAW90_HD inline Status PrepareMechanicalSlots(const PreparedMaterial& material,
                                                MechanicalSlots& output) noexcept {
  if(!material.initialized())return Status::InvalidInput;
  MechanicalSlots next;
  next.pm21_poisson_ratio=material.reader().poisson_ratio;
  next.pm22_shear_pa=material.updated().shear_pa;
  next.pm32_bulk_pa=material.updated().bulk_pa;
  // LAW90_UPD changes PM32 after HM_READ_MAT has retained the contact bulk in PM100.
  next.pm100_reader_contact_bulk_pa=material.reader().contact_bulk_pa;
  next.pm107_control_pa=2.0*::fmax(next.pm32_bulk_pa,next.pm100_reader_contact_bulk_pa);
  if(!tl::math::Finite(next.pm107_control_pa))return Status::NonfiniteResult;
  output=next;return Status::Ok;
}
} // namespace tl::material::law90
