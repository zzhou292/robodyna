// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceTypes.h"
#include "lib_src/collision/radioss_type25/coefficients/UnitFactors.h"
namespace tl::fea::solid_common::distortion {
using UnitScale=tlfea::contact::radioss_type25::UnitScale;
struct ForceInput {
  Vec3 position_m[8]{},velocity_m_s[8]{},incoming_force_n[8]{};
  double dt_s=0,raw_stiffness_n_m=0,distortion_energy_j=0;
};
struct ForceResult {
  Vec3 force_n[8]{};
  double raw_stiffness_n_m=0,distortion_energy_j=0,distortion_work_increment_j=0;
  int damping_applied=0,center_contacts=0,corner_contacts=0;
};
struct PreparedForceValues {
  native::Parameters parameters;
  native::ForceInput input;
  UnitScale units{}; // SI units per native unit; SI={1,1,1}, mm/Mg/s={.001,1000,1}.
};
using DampingActivity=native::DampingActivity;
namespace units_detail {
using Factors=tlfea::contact::radioss_type25::coefficient_detail::UnitFactors;
TL_BRICK_HD inline bool Make(UnitScale units,Factors& result) noexcept {
  if(!((units.length_m==1&&units.mass_kg==1&&units.time_s==1)||
       (units.length_m==.001&&units.mass_kg==1000&&units.time_s==1)))return false;
  return tlfea::contact::radioss_type25::coefficient_detail::Make(units,result);
}
} // namespace units_detail
} // namespace tl::fea::solid_common::distortion
