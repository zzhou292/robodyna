// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected I25FOR3 endpoint tail and I25ASS0, OpenRadioss (C) 2026 Siemens.
// Pinned donor a62b27e6. See README.md for selected branch and claim boundary.
#pragma once
#include "Types.h"
#include "../UnitConversions.h"
namespace tlfea::contact::radioss_type25::assembly {
namespace detail {
TL_MATH_HOST_DEVICE inline bool Supported(const Controls& c) {
  return c.parallel_assembly == 0 && c.pinch == 0 && c.thermal == 0 &&
      c.thermal_formulation == 0 && c.thermal_nodal_timestep == 0 &&
      c.engine.kdtint == 0 && c.engine.idtmins == 0 && c.engine.idtmins_int == 0;
}
TL_MATH_HOST_DEVICE inline bool Finite(Vector v) {
  return tl::math::Finite(v.x) && tl::math::Finite(v.y) && tl::math::Finite(v.z);
}
template<class Units> TL_MATH_HOST_DEVICE inline bool Finite(const Endpoints<Units>& p) {
  if (!p.active) return true; // No endpoint scratch is read on the HH-zero path.
  if (!Finite(p.secondary_resultant) || !normal_detail::Nonnegative(p.secondary_stiffness))
    return false;
  for (unsigned slot = 0; slot < 4; ++slot)
    if (!Finite(p.main_force[slot]) || !normal_detail::Nonnegative(p.main_stiffness[slot]))
      return false;
  return true;
}
} // namespace detail
// Numerical preparation only. A successful packet does not authorize physical
// publication. Input is the qualified response, including cleared inactive H_i.
// All reads precede successful output publication; failed calls leave it intact.
TL_MATH_HOST_DEVICE inline Status PrepareNativeEndpoints(const Controls& controls,
    const NativeFrictionResult& response, NativeEndpoints* output) {
  if (!output) return Status::InvalidInput;
  if (!detail::Supported(controls)) return Status::UnsupportedProfile;
  const auto& h = response.normal.weights;
  for (unsigned slot = 0; slot < 4; ++slot)
    if (!tl::math::Finite(h[slot])) return Status::InvalidInput;
  const double hh = ((h[0] + h[1]) + h[2]) + h[3];
  if (!tl::math::Finite(hh)) return Status::NonfiniteResult;
  NativeEndpoints next;
  if (hh == 0) { *output = next; return Status::Ok; }
  if (!response.contact_active || !detail::Finite(response.native_resultant) ||
      !normal_detail::Nonnegative(response.normal.stability_stiffness))
    return Status::InvalidInput;
  next.active = true;
  next.secondary_resultant = response.native_resultant;
  next.secondary_stiffness = response.normal.stability_stiffness;
  for (unsigned slot = 0; slot < 4; ++slot) {
    next.main_force[slot] = {response.native_resultant.x * h[slot],
        response.native_resultant.y * h[slot], response.native_resultant.z * h[slot]};
    next.main_stiffness[slot] = response.normal.stability_stiffness * ::fabs(h[slot]);
  }
  if (!detail::Finite(next)) return Status::NonfiniteResult;
  *output = next;
  return Status::Ok;
}
// Convert already prepared native endpoints, preserving native multiplication
// order before scaling. History and interpolation weights never change units.
TL_MATH_HOST_DEVICE inline Status EndpointsToSi(UnitScale units,
    const NativeEndpoints& native, SiEndpoints* output) {
  units_detail::Factors factors;
  if (!output || !units_detail::Make(units, factors) || !detail::Finite(native))
    return Status::InvalidInput;
  SiEndpoints next;
  next.active = native.active;
  if (native.active) {
    next.secondary_resultant = {native.secondary_resultant.x * factors.force,
        native.secondary_resultant.y * factors.force, native.secondary_resultant.z * factors.force};
    next.secondary_stiffness = native.secondary_stiffness * factors.stiffness;
    for (unsigned slot = 0; slot < 4; ++slot) {
      next.main_force[slot] = {native.main_force[slot].x * factors.force,
          native.main_force[slot].y * factors.force, native.main_force[slot].z * factors.force};
      next.main_stiffness[slot] = native.main_stiffness[slot] * factors.stiffness;
    }
  }
  if (!detail::Finite(next)) return Status::NonfiniteResult;
  *output = next;
  return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::assembly
