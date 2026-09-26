// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Private per-attempt operands. Native points to the immutable legacy cache;
// Si points directly to the authenticated accepted-owner MS slab. No ownership,
// lifetime extension or inverse-mass reconstruction is granted by this value.
struct MassOperands {
  enum class Units { Native,Si };
  const double* values=nullptr;
  Units units=Units::Native;
};
TL_MATH_HOST_DEVICE inline bool NativeMass(MassOperands input,std::size_t node,
    const units_detail::Factors& units,double& output) noexcept {
  if(!input.values)return false;
  const double value=input.values[node];
  if(!normal_detail::Nonnegative(value))return false;
  double next=value;
  if(input.units==MassOperands::Units::Si) {
    next=value/units.mass;
    if(!normal_detail::Nonnegative(next)||(value!=0&&next==0))return false;
  } else if(input.units!=MassOperands::Units::Native)return false;
  output=next;return true;
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
