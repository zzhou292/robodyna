// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedPatchGeometry.h"

namespace tl::constraints::tied_shell {
namespace detail {
// Native I2VIROT3 sums all positive products before all negative products.
// An interleaved sum of four Cross() values would change its arithmetic.
TL_TIED_PATCH_HD inline Vec3 MomentSum(const Vec3 (&x)[4], const Vec3 (&v)[4]) noexcept {
  return {
      x[0].y*v[0].z + x[1].y*v[1].z + x[2].y*v[2].z + x[3].y*v[3].z
          - x[0].z*v[0].y - x[1].z*v[1].y - x[2].z*v[2].y - x[3].z*v[3].y,
      x[0].z*v[0].x + x[1].z*v[1].x + x[2].z*v[2].x + x[3].z*v[3].x
          - x[0].x*v[0].z - x[1].x*v[1].z - x[2].x*v[2].z - x[3].x*v[3].z,
      x[0].x*v[0].y + x[1].x*v[1].y + x[2].x*v[2].y + x[3].x*v[3].y
          - x[0].y*v[0].x - x[1].y*v[1].x - x[2].y*v[2].x - x[3].y*v[3].x};
}
TL_TIED_PATCH_HD inline Vec3 OffsetMotion(Vec3 mean, Vec3 spin, Vec3 offset) noexcept {
  return {mean.x + spin.y*offset.z - spin.z*offset.y,
          mean.y + spin.z*offset.x - spin.x*offset.z,
          mean.z + spin.x*offset.y - spin.y*offset.x};
}
} // namespace detail

// I2VIROT3 maps both native velocity and acceleration packets with the same
// current DPARA. It does not create a second integration or update positions.
TL_TIED_PATCH_HD inline Status RecoverMotion(const Patch& patch, const MasterMotion& motion,
    SecondaryMotion& output) noexcept {
  using detail::math::Finite;
  if (!patch.prepared()) return Status::InvalidInput;
  for (unsigned i = 0; i < 4; ++i) {
    if (!Finite(motion.velocity[i]) || !Finite(motion.acceleration[i])) return Status::InvalidInput;
  }
  const auto& p = patch.values();
  SecondaryMotion next;
  next.angular_velocity = detail::ApplyCofactors(p, detail::MomentSum(p.master_offset, motion.velocity));
  next.angular_acceleration = detail::ApplyCofactors(p, detail::MomentSum(p.master_offset, motion.acceleration));
  next.velocity = detail::OffsetMotion(detail::Mean(motion.velocity), next.angular_velocity, p.secondary_offset);
  next.acceleration = detail::OffsetMotion(detail::Mean(motion.acceleration), next.angular_acceleration, p.secondary_offset);
  if (!Finite(next.velocity) || !Finite(next.angular_velocity) ||
      !Finite(next.acceleration) || !Finite(next.angular_acceleration)) return Status::NonfiniteResult;
  output = next;
  return Status::Success;
}
} // namespace tl::constraints::tied_shell
