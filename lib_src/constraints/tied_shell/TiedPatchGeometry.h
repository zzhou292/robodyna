// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedPatchTypes.h"
#include <cfloat>

namespace tl::constraints::tied_shell {
namespace detail {
namespace math = tl::math::fixed3;

TL_TIED_PATCH_HD inline Vec3 Mean(const Vec3 (&v)[4]) noexcept {
  return {.25 * (v[0].x + v[1].x + v[2].x + v[3].x),
          .25 * (v[0].y + v[1].y + v[2].y + v[3].y),
          .25 * (v[0].z + v[1].z + v[2].z + v[3].z)};
}

// Explicit source cofactor multiplication preserves native expression order.
TL_TIED_PATCH_HD inline Vec3 ApplyCofactors(const PatchValues& p, Vec3 v) noexcept {
  const auto& c = p.cofactor;
  return {c[0] * (v.x*c[1] + v.y*c[6] + v.z*c[5]),
          c[0] * (v.y*c[2] + v.z*c[4] + v.x*c[6]),
          c[0] * (v.z*c[3] + v.x*c[5] + v.y*c[4])};
}
} // namespace detail

// Allocation-free current geometry, selected I2FOR28_CIN explicit branch.
// No search, source admission, coefficient redistribution or accepted stamp.
// All rejected preparations preserve the previous complete patch.
TL_TIED_PATCH_HD inline Status PreparePatch(const PatchInput& input, Patch& output) noexcept {
  using detail::math::Finite;
  if (!Finite(input.secondary_position)) return Status::InvalidInput;
  for (const auto& x : input.master_position) {
    if (!Finite(x)) return Status::InvalidInput;
  }
  Patch next;
  auto& p = next.values_;
  p.center = detail::Mean(input.master_position);
  p.secondary_offset = detail::math::Subtract(input.secondary_position, p.center);
  for (unsigned i = 0; i < 4; ++i) {
    p.master_offset[i] = detail::math::Subtract(input.master_position[i], p.center);
    if (!Finite(p.master_offset[i])) return Status::NonfiniteResult;
  }
  if (!Finite(p.center) || !Finite(p.secondary_offset)) return Status::NonfiniteResult;
  const auto& a = p.master_offset[0];
  const auto& b = p.master_offset[1];
  const auto& c = p.master_offset[2];
  const auto& d = p.master_offset[3];
  const double xx = a.x*a.x + b.x*b.x + c.x*c.x + d.x*d.x;
  const double yy = a.y*a.y + b.y*b.y + c.y*c.y + d.y*d.y;
  const double zz = a.z*a.z + b.z*b.z + c.z*c.z + d.z*d.z;
  const double xy = a.x*a.y + b.x*b.y + c.x*c.y + d.x*d.y;
  const double yz = a.y*a.z + b.y*b.z + c.y*c.z + d.y*d.z;
  const double zx = a.z*a.x + b.z*b.x + c.z*c.x + d.z*d.x;
  const double zzz = xx + yy, xxx = yy + zz, yyy = zz + xx;
  const double xy2 = xy*xy, yz2 = yz*yz, zx2 = zx*zx;
  const double determinant = xxx*yyy*zzz - xxx*yz2 - yyy*zx2 - zzz*xy2 - 2*xy*yz*zx;
  const double scale = ::fmax(xxx, ::fmax(yyy, zzz));
  const double scale_cubed = scale*scale*scale;
  if (!Finite(determinant) || !Finite(scale_cubed)) return Status::NonfiniteResult;
  // A declared numerical domain guard, not a native determinant clamp.
  if (!(scale > 0) || !(determinant > 64*DBL_EPSILON*scale_cubed)) {
    return Status::SingularPatch;
  }
  p.cofactor[0] = 1/determinant;
  p.cofactor[1] = zzz*yyy - yz2;
  p.cofactor[2] = xxx*zzz - zx2;
  p.cofactor[3] = yyy*xxx - xy2;
  p.cofactor[6] = zzz*xy + yz*zx;
  p.cofactor[4] = xxx*yz + zx*xy;
  p.cofactor[5] = yyy*zx + xy*yz;
  for (double value : p.cofactor) {
    if (!Finite(value)) return Status::NonfiniteResult;
  }
  next.prepared_ = true;
  output = next;
  return Status::Success;
}
} // namespace tl::constraints::tied_shell
