// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SRCOOR3/S8EDERIC3/S8EDERIPR3: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid18GaussGeometry.h"
#include "Solid18NaturalDerivatives.h"

namespace tl::fea::solid18::detail {
// Native cofactors and reciprocal order. The scaled Jacobian is transposed
// into rows of physical derivative directions, matching AJI1..AJI9.
TL_SOLID18_HD inline void InverseScaledJacobian(const Matrix3& matrix,
    double volume, double determinant_scale, Matrix3& inverse) noexcept {
  const auto& j = matrix.v;
  const double c59 = j[4]*j[8]-j[5]*j[7];
  const double c67 = j[5]*j[6]-j[3]*j[8];
  const double c48 = j[3]*j[7]-j[4]*j[6];
  const double c38 = -j[1]*j[8]+j[2]*j[7];
  const double c19 = j[0]*j[8]-j[2]*j[6];
  const double c27 = -j[0]*j[7]+j[1]*j[6];
  const double c26 = j[1]*j[5]-j[2]*j[4];
  const double c34 = -j[0]*j[5]+j[2]*j[3];
  const double c15 = j[0]*j[4]-j[1]*j[3];
  const double factor = determinant_scale/volume;
  inverse.v[0] = factor*c59;
  inverse.v[3] = factor*c67;
  inverse.v[6] = factor*c48;
  inverse.v[1] = factor*c38;
  inverse.v[4] = factor*c19;
  inverse.v[7] = factor*c27;
  inverse.v[2] = factor*c26;
  inverse.v[5] = factor*c34;
  inverse.v[8] = factor*c15;
}

TL_SOLID18_HD inline void CenterGradient(const Matrix3& inverse,
                                        double (&p)[3][4]) noexcept {
  for (unsigned axis = 0; axis < 3; ++axis) {
    const double a = inverse.v[3*axis];
    const double b = inverse.v[3*axis+1];
    const double c = inverse.v[3*axis+2];
    const double difference = a-b;
    p[axis][1] = difference-c;
    p[axis][3] = -difference-c;
    const double sum = a+b;
    p[axis][0] = -sum-c;
    p[axis][2] = sum-c;
  }
}

// No current orientation reversal is permitted: source/native slot identity
// was established once at startup. Negative Jacobian fallback is out of scope.
TL_SOLID18_HD inline Status NativeCurrentGeometryValues(const Vec3 (&native)[8],
    const Vec3 (&native_velocity)[8], CurrentGeometry& result,
    StartupGeometry& shared) noexcept {
  if (!Frame(native, shared.frame)) return Status::InvalidGeometry;
  for (unsigned n = 0; n < 8; ++n) {
    shared.native_position_m[n] = Local(shared.frame, native[n]);
    result.local_position_m[n] = shared.native_position_m[n];
    result.local_velocity_m_s[n] = Local(shared.frame, native_velocity[n]);
    if (!Finite(result.local_position_m[n]) || !Finite(result.local_velocity_m_s[n]))
      return Status::NonfiniteResult;
  }
  // S8ZDERIC3/S8EDERIC3 share these exact positive-Jacobian center and
  // S8EJACIP3 equations. The startup helper's length reduction is unused here.
  Status status = CenterGeometry(shared);
  if (status != Status::Success) return status;
  status = GaussGeometry(shared);
  if (status != Status::Success) return status;
  result.frame = shared.frame;
  result.center_volume_m3 = shared.center_volume_m3;
  result.inverse_center_face_scale_per_m2 = shared.inverse_center_face_scale_per_m2;
  Matrix3 center_inverse;
  InverseScaledJacobian(shared.center_scaled_jacobian_m, shared.center_volume_m3,
      1.0/64.0, center_inverse);
  CenterGradient(center_inverse, result.center_gradient_per_m);
  for (unsigned ip = 0; ip < 8; ++ip) {
    auto& point = result.point[ip];
    point.current_volume_m3 = shared.point[ip].initial_volume_m3;
    InverseScaledJacobian(shared.point[ip].scaled_jacobian_m,
        point.current_volume_m3, 1.0/512.0, point.inverse_scaled_jacobian_per_m);
    RegularDerivatives(ip, point.inverse_scaled_jacobian_per_m, point.regular_per_m);
    for (const auto& row : point.regular_per_m) {
      for (double value : row) {
        if (!tl::math::Finite(value)) return Status::NonfiniteResult;
      }
    }
  }
  return Status::Success;
}
TL_SOLID18_HD inline Status CurrentGeometryValues(const Reference& reference,
    const PrescribedInterval& interval, CurrentGeometry& result,
    StartupGeometry& shared) noexcept {
  Vec3 native[8];
  Vec3 velocity[8];
  for (unsigned n = 0; n < 8; ++n) {
    native[n] = interval.position_endpoint_m[reference.source_slot(n)];
    velocity[n] = interval.velocity_midpoint_m_s[reference.source_slot(n)];
  }
  return NativeCurrentGeometryValues(native, velocity, result, shared);
}
// Value-only callers retain their existing automatic staging.
TL_SOLID18_HD inline Status CurrentGeometryValues(const Reference& reference,
    const PrescribedInterval& interval, CurrentGeometry& result) noexcept {
  StartupGeometry shared;
  return CurrentGeometryValues(reference,interval,result,shared);
}
}  // namespace tl::fea::solid18::detail
