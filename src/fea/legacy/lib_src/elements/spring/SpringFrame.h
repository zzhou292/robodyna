// SPDX-License-Identifier: AGPL-3.0-or-later
// Shared R4EVEC3 arithmetic; donor inventory in qualification/type25/native.
#pragma once
#include "../../math/Fixed3Operations.h"
#if defined(__CUDACC__)
#define TL_SPRING_HD __host__ __device__
#else
#define TL_SPRING_HD
#endif

namespace tl::fea::spring {
using tl::math::Vec3;
using tl::math::Matrix3;
enum class Status { Success, NonfiniteResult, DegenerateGeometry };
// Length fields use the caller's consistent coordinate units.
struct FrameValues {
  Matrix3 axes{};
  Matrix3 midpoint_axes{};
  double length = 0;
  double midpoint_length = 0;
};

TL_SPRING_HD inline bool FinishFrame(Vec3 x, Vec3 y, Vec3 z, double cy,
                                    double sz, Matrix3& out) {
  y = tl::math::fixed3::Add(tl::math::fixed3::Scale(y, cy),
                           tl::math::fixed3::Scale(z, sz));
  y = tl::math::fixed3::Divide(y, ::fmax(1e-15, tl::math::fixed3::Norm(y)));
  z = tl::math::fixed3::Cross(x, y);
  z = tl::math::fixed3::Divide(z, ::fmax(1e-15, tl::math::fixed3::Norm(z)));
  const auto axes = tl::math::fixed3::Columns(x, y, z);
  if (!tl::math::fixed3::Orthonormal(axes)) {
    return false;
  }
  out = axes;
  return true;
}

// Preconditions: finite nodes/duration, unit accepted transverse, positive floor.
// Zero duration is supported for a fresh native force packet. The caller owns
// startup/interval admission and units; no clock or state is stored here.
template <class Nodes>
TL_SPRING_HD inline Status AdvanceFrame(
    Vec3 accepted_transverse, const Nodes& nodes, double dt,
    double length_floor, FrameValues& output) {
  const double half_dt = .5 * dt;
  const auto chord = tl::math::fixed3::Subtract(nodes[1].position,
                                               nodes[0].position);
  const auto middle = tl::math::fixed3::Subtract(chord,
      tl::math::fixed3::Scale(tl::math::fixed3::Subtract(
          nodes[1].velocity, nodes[0].velocity), half_dt));
  FrameValues next;
  next.length = tl::math::fixed3::Norm(chord);
  next.midpoint_length = tl::math::fixed3::Norm(middle);
  if (!tl::math::fixed3::Finite(next.length) ||
      !tl::math::fixed3::Finite(next.midpoint_length)) {
    return Status::NonfiniteResult;
  }
  if (next.length <= length_floor || next.midpoint_length <= length_floor) {
    return Status::DegenerateGeometry;
  }
  const auto x = tl::math::fixed3::Divide(chord, next.length);
  const auto xm = tl::math::fixed3::Divide(middle, next.midpoint_length);
  const auto z = tl::math::fixed3::Cross(x, accepted_transverse);
  const auto zm = tl::math::fixed3::Cross(xm, accepted_transverse);
  const auto y = tl::math::fixed3::Cross(z, x);
  const auto ym = tl::math::fixed3::Cross(zm, xm);
  const double w1 = tl::math::fixed3::Dot(xm, nodes[0].angular_velocity);
  const double w2 = tl::math::fixed3::Dot(xm, nodes[1].angular_velocity);
  const double angle = (w1 + w2) / 2 * half_dt;
  if (!tl::math::fixed3::Finite(angle)) {
    return Status::NonfiniteResult;
  }
  const double c = ::cos(angle);
  const double s = ::sin(angle);
  if (!FinishFrame(x, y, z,
          (2 * c * c - 1) / ::fmax(1e-15, tl::math::fixed3::Norm(y)),
          (2 * c * s) / ::fmax(1e-15, tl::math::fixed3::Norm(z)), next.axes) ||
      !FinishFrame(xm, ym, zm,
          c / ::fmax(1e-15, tl::math::fixed3::Norm(ym)),
          s / ::fmax(1e-15, tl::math::fixed3::Norm(zm)), next.midpoint_axes)) {
    return Status::DegenerateGeometry;
  }
  output = next;
  return Status::Success;
}
} // namespace tl::fea::spring
#undef TL_SPRING_HD
