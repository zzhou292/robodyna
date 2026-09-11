// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected CBACOOR: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "QbatReference.h"
#include "QbatPointDerivatives.h"
#include "lib_src/elements/qeph/QephCurrentFrame.h"

namespace tl::fea::qbat {
// Current coordinates are explicitly supplied by the caller. This does not
// manufacture a clock, material-point history or a staggered velocity sample.
// The input frame is recalculated with the engine CLSKEW3 arithmetic; the
// immutable reference keeps the different starter CLSKEW3 normalization.
// Output is published only after every point is finite and admissible.
namespace detail {
TL_QBAT_HD inline Status CurrentSurfaceGeometry(const Reference& reference,
    const CurrentInput& input, Geometry& output) {
  if (!reference.prepared()) return Status::kInvalidReference;
  for (auto p:input.position_m) {
    if (!detail::Finite(p)) return Status::kInvalidInput;
  }
  Geometry next;
  // This existing pure leaf is the identical CBACOOR R/S + engine CLSKEW3 K0.
  // No QEPH characteristic-length, warpage switch or constitutive code runs.
  auto status=qeph::detail::CurrentFrame(input.position_m,next.frame,next.area_m2);
  if (status!=Status::kSuccess) return status;
  next.reciprocal_area_per_m2=1/next.area_m2;
  const auto& x=input.position_m;
  const Vec3 center{.25*(x[2].x+x[3].x+x[0].x+x[1].x),
      .25*(x[2].y+x[3].y+x[0].y+x[1].y), .25*(x[2].z+x[3].z+x[0].z+x[1].z)};
  const auto& f=next.frame;
  const double dx=x[0].x-center.x;
  const double dy=x[0].y-center.y;
  const double dz=x[0].z-center.z;
  next.actual_warpage_m=f.v[2]*dx+f.v[5]*dy+f.v[8]*dz;
  Vec3 local[4]{};
  for (unsigned i=1;i<4;++i) {
    const double px=x[i].x-x[0].x;
    const double py=x[i].y-x[0].y;
    const double pz=x[i].z-x[0].z;
    local[i].x=f.v[0]*px+f.v[3]*py+f.v[6]*pz;
    local[i].y=f.v[1]*px+f.v[4]*py+f.v[7]*pz;
  }
  const double cx=.25*(local[1].x+local[2].x+local[3].x);
  const double cy=.25*(local[1].y+local[2].y+local[3].y);
  auto& p=next.centered_projected_position_m;
  p[0]={-cx,-cy,0};
  for (unsigned i=1;i<4;++i) {
    p[i]={local[i].x-cx,local[i].y-cy,0};
  }
  // NPTT1/NPINCH0 forces this native branch even when ZL1 is nonzero.
  status=detail::PointGeometry(next);
  if (status!=Status::kSuccess) return status;
  if (!detail::Finite(next)) return Status::kNonfiniteResult;
  output=next;
  return Status::kSuccess;
}
} // namespace detail
TL_QBAT_HD inline Status EvaluateGeometry(const Reference& reference,
    const CurrentInput& input, Geometry& output) {
  if (!reference.prepared()) return Status::kInvalidReference;
  if (input.native_off!=1) return Status::kInvalidInput;
  return detail::CurrentSurfaceGeometry(reference,input,output);
}
} // namespace tl::fea::qbat
