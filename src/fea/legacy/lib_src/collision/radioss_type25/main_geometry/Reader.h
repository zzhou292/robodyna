// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6 NORMA1D and VOLINT reader-phase operands.
#pragma once
#include "Types.h"
#include "../normal_math/DoubleFace.h"
#include "lib_src/elements/solid_common/BrickFrame.h"
#include "lib_src/math/ScalarBits.h"
namespace tlfea::contact::radioss_type25 {
// Raw reader packet, including repeated PENTA slots. Finite signed/zero volume
// is an observation; this leaf does not admit a coefficient or physical source.
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeEightSlotReaderVolume(
    const Vector (&raw)[8], double* output) noexcept {
  namespace b = tl::fea::solid_common;
  if (!output) return CoefficientStatus::InvalidInput;
  for (const auto& x : raw) if (!b::Finite(x)) return CoefficientStatus::InvalidInput;
  Vector r, s, t;
  b::Directions(raw, r, s, t);
  const Vector cofactors = b::Cross(r, s);
  // Original VOLINT determinant association; not SignedCenterVolume.
  const double volume = (1./64.)*b::Dot(t, cofactors);
  if (!b::Finite(r) || !b::Finite(s) || !b::Finite(t) || !b::Finite(cofactors) ||
      !tl::math::Finite(volume)) return CoefficientStatus::NonfiniteResult;
  *output = volume;
  return CoefficientStatus::Ok;
}
struct NativeInternalMainGeometryResult {
  Vector normal_before_orientation;
  double area = 0, signed_volume = 0;
};
// Effective IC>=2 returns from INSOL3D before DDS or primary permutation.
// Membership, effective support count and reader/caller phase are external.
// No projection/reversal field is exposed or fabricated for this branch.
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeInternalMainGeometry(
    const NativeExteriorMainGeometryInput& in, NativeInternalMainGeometryResult* output) noexcept {
  namespace b = tl::fea::solid_common;
  if (!output) return CoefficientStatus::InvalidInput;
  if (in.layout != ShellLayout::Quad4 && in.layout != ShellLayout::Triangle3)
    return CoefficientStatus::UnsupportedProfile;
  for (const auto& x : in.face) if (!b::Finite(x)) return CoefficientStatus::InvalidInput;
  for (const auto& x : in.solid_raw) if (!b::Finite(x)) return CoefficientStatus::InvalidInput;
  if (in.layout == ShellLayout::Triangle3 &&
      (!tl::math::SameScalarBits(in.face[2].x, in.face[3].x) ||
       !tl::math::SameScalarBits(in.face[2].y, in.face[3].y) ||
       !tl::math::SameScalarBits(in.face[2].z, in.face[3].z)))
    return CoefficientStatus::InvalidInput;
  normal_math::DoubleFaceResult face;
  if (!normal_math::DoubleFace(in.face, face)) return CoefficientStatus::NonfiniteResult;
  double volume;
  const auto status = EvaluateNativeEightSlotReaderVolume(in.solid_raw, &volume);
  if (status != CoefficientStatus::Ok) return status;
  // Native center accumulations are unconsumed at the IC>=2 return. Deliberately
  // do not evaluate or validate a hypothetical exterior DDS afterward.
  *output = {face.normal, face.area, volume};
  return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
