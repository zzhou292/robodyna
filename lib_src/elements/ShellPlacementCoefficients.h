// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected CINMAS/C3INMAS and CNCOEF3/CNDT3 operations, OpenRadioss 2026 Siemens.
#pragma once
#include "ShellReferencePlacement.h"
#include "lib_src/math/Quaternion.h"
#if defined(__CUDACC__)
#define TL_PLACEMENT_HD __host__ __device__
#else
#define TL_PLACEMENT_HD
#endif
namespace tl::fea {
struct ShellPlacementCoefficients {
  double offset=0;
  double stiffness_factor=1;
};
TL_PLACEMENT_HD inline double NativeShellOffsetStiffnessFactor(
    double offset,double thickness) noexcept {
  return 1.+.5*::fabs(offset)/thickness;
}
TL_PLACEMENT_HD inline bool PrepareShellPlacementCoefficients(
    ShellReferencePlacement placement,double thickness,
    ShellPlacementCoefficients& output) noexcept {
  if(!ValidShellReferencePlacement(placement) ||
      !tl::math::Finite(thickness) || !(thickness>0)) return false;
  ShellPlacementCoefficients candidate;
  candidate.offset=NativeShellShift(placement)*thickness;
  candidate.stiffness_factor=NativeShellOffsetStiffnessFactor(candidate.offset,thickness);
  if(!tl::math::Finite(candidate.offset) ||
      !tl::math::Finite(candidate.stiffness_factor)) return false;
  output=candidate;
  return true;
}
// These are native total expressions, not sums of separately rounded diagnostic
// partitions. Q4 mass is per-node; T3 mass is per-element before angle weighting.
// Caller validates finite positive inputs and the placement enum.
TL_PLACEMENT_HD inline double NativeQephPlacementInertia(double mass,double area,
    double thickness,ShellReferencePlacement placement) noexcept {
  const double shift=NativeShellShift(placement);
  return mass*(area/12+thickness*thickness*(1./12+shift*shift));
}
TL_PLACEMENT_HD inline double NativeT3PlacementInertia(double mass,double area,
    double thickness) noexcept {
  // Selected TYPE1 C3INMAS does not add Q4's offset-squared term.
  return mass*(area/(9./2)+thickness*thickness*(1./12));
}
} // namespace tl::fea
#undef TL_PLACEMENT_HD
