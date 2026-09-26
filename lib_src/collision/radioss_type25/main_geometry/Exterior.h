// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6 INSOL3D unique-support geometry and VOLINT.
// Copyright (C) 2026 Siemens; see ../LICENSE.md.
#pragma once
#include "Types.h"
#include "../normal_math/DoubleFace.h"
#include "lib_src/elements/solid_common/BrickFrame.h"
#include "lib_src/math/ScalarBits.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeExteriorMainGeometry(
    const NativeExteriorMainGeometryInput& in, NativeExteriorMainGeometryResult* output) noexcept {
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
  Vector face_center{}, solid_center{};
  for (const auto& x : in.face) {
    face_center.x = face_center.x+.25*x.x;
    face_center.y = face_center.y+.25*x.y;
    face_center.z = face_center.z+.25*x.z;
  }
  for (const auto& x : in.solid_raw) {
    solid_center.x = solid_center.x+x.x;
    solid_center.y = solid_center.y+x.y;
    solid_center.z = solid_center.z+x.z;
    if (!b::Finite(solid_center)) return CoefficientStatus::NonfiniteResult;
  }
  solid_center = {solid_center.x*(1./8.), solid_center.y*(1./8.), solid_center.z*(1./8.)};
  const Vector delta{solid_center.x-face_center.x, solid_center.y-face_center.y,
                     solid_center.z-face_center.z};
  const double projection = b::Dot(face.normal, delta);
  Vector r, s, t;
  b::Directions(in.solid_raw, r, s, t);
  const Vector cofactors = b::Cross(r, s);
  // Same directions as CHECKVOLUME_8N, different native determinant reduction.
  // Do not replace this with SignedCenterVolume or a structural volume cache.
  const double volume = (1./64.)*b::Dot(t, cofactors);
  if (!b::Finite(face_center) || !b::Finite(delta) || !tl::math::Finite(projection) ||
      !b::Finite(r) || !b::Finite(s) || !b::Finite(t) || !b::Finite(cofactors) ||
      !tl::math::Finite(volume)) return CoefficientStatus::NonfiniteResult;

  NativeExteriorMainGeometryResult next;
  next.normal_before_orientation = face.normal;
  next.area = face.area;
  next.signed_volume = volume;
  next.center_projection = projection;
  const bool triangle = in.layout == ShellLayout::Triangle3;
  if (triangle) next.source_corner[3] = 2;
  // Native IF(DDS<ZERO) RETURN: zero, including -0, enters the reversal arm.
  if (!(projection < 0.)) {
    next.reversed = true;
    if (triangle) {
      next.source_corner[0] = 1;
      next.source_corner[1] = 0;
    } else {
      for (unsigned k = 0; k < 4; ++k) next.source_corner[k] = 3-k;
    }
  }
  // Finite raw zero/negative volume is an observation, not coefficient admission.
  // EvaluateNativeSolidMainCoefficient still requires strictly positive volume.
  *output = next;
  return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
