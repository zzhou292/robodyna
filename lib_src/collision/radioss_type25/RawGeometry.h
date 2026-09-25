// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "geometry/Sharp.h"
namespace tlfea::contact::radioss_type25 {
namespace geometry_detail {
template<class U> TL_MATH_HOST_DEVICE inline bool Finite(const RawGeometryResult<U>& r) {
  if (!v::Finite(r.normal) || !normal_detail::Nonnegative(r.geometric_penetration) ||
      !tl::math::Finite(r.gap) || !tl::math::Finite(r.distance) ||
      !normal_detail::Nonnegative(r.incoming_stiffness)) return false;
  for (double value : r.weights) if (!tl::math::Finite(value)) return false;
  return true;
}
TL_MATH_HOST_DEVICE inline void Weights(const NativeGeometryInput& in, const Work& w,
                                       NativeRawGeometryResult& out) {
  if (w.triangle) {
    out.weights[0] = in.lb; out.weights[1] = in.lc;
    out.weights[2] = w.la; out.weights[3] = 0;
  } else {
    const double h0 = .25 * w.la;
    for (auto& weight : out.weights) weight = h0;
    out.weights[w.a] = h0 + in.lb;
    out.weights[w.b] = h0 + in.lc;
  }
}
} // namespace geometry_detail
// Pure selected current geometry; no row-history mutation, force or publication.
TL_MATH_HOST_DEVICE inline GeometryStatus EvaluateNativeRawGeometry(
    const GeometryProfile& profile, const NativeGeometryInput& in,
    NativeRawGeometryResult* output) {
  using namespace geometry_detail;
  if (!output || !Valid(in)) return GeometryStatus::InvalidInput;
  if (!Supported(profile)) return GeometryStatus::UnsupportedProfile;
  Work work;
  work.shell_contact = in.segment_type != 0 || in.secondary_gap > 0;
  Prepare(in, work);
  NativeRawGeometryResult result;
  result.key = in.key; result.selection_code = in.selection_code;
  result.incoming_stiffness = in.incoming_stiffness;
  General(in, work, result);
  Boundary(in, work);
  if (!Sharp(in, work, result)) return GeometryStatus::InvalidInput;
  Weights(in, work, result); result.gap = work.gap;
  if (!Finite(result)) return GeometryStatus::NonfiniteResult;
  *output = result;
  return GeometryStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
