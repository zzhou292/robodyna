// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RawGeometry.h"
#include "UnitConversions.h"
namespace tlfea::contact::radioss_type25 {
// Production bridge: owner coordinates may be SI, but geometry and persistent
// history stay native. There is no SI penetration/history round-trip.
TL_MATH_HOST_DEVICE inline GeometryStatus EvaluateNativeGeometryFromSi(
    const GeometryProfile& p, UnitScale units, const SiGeometryInput& in,
    NativeRawGeometryResult* output) {
  using namespace geometry_detail;
  units_detail::Factors f;
  if (!output || !Valid(in) || !units_detail::Make(units, f)) return GeometryStatus::InvalidInput;
  if (!Supported(p)) return GeometryStatus::UnsupportedProfile;
  NativeGeometryInput native;
  native.key = in.key; native.segment_type = in.segment_type; native.selection_code = in.selection_code;
  native.lb = in.lb; native.lc = in.lc;
  native.incoming_stiffness = in.incoming_stiffness / f.stiffness;
  native.secondary = v::Divide(in.secondary, f.length);
  native.secondary_gap = in.secondary_gap / f.length;
  for (unsigned i = 0; i < 4; ++i) {
    native.main_node_ids[i] = in.main_node_ids[i];
    native.main_vertices[i] = v::Divide(in.main_vertices[i], f.length);
    native.corner_normal[i] = in.corner_normal[i]; native.neighbors[i] = in.neighbors[i];
    native.boundary_ids[i] = in.boundary_ids[i]; native.main_gap[i] = in.main_gap[i] / f.length;
    for (unsigned j = 0; j < 2; ++j) native.vertex_bisector[i][j] = in.vertex_bisector[i][j];
  }
  if (!Valid(native)) return GeometryStatus::NonfiniteResult;
  return EvaluateNativeRawGeometry(p, native, output);
}
// Explicit interoperability/output view. Do not feed rounded SI penetration
// back into a native history update; use EvaluateNativeGeometryFromSi instead.
TL_MATH_HOST_DEVICE inline GeometryStatus EvaluateSiRawGeometry(
    const GeometryProfile& p, UnitScale units, const SiGeometryInput& in,
    SiRawGeometryResult* output) {
  using namespace geometry_detail;
  if (!output) return GeometryStatus::InvalidInput;
  NativeRawGeometryResult result;
  const auto status = EvaluateNativeGeometryFromSi(p, units, in, &result);
  if (status != GeometryStatus::Ok) return status;
  units_detail::Factors f;
  if (!units_detail::Make(units, f)) return GeometryStatus::InvalidInput;
  SiRawGeometryResult si;
  si.key = result.key; si.selection_code = result.selection_code; si.normal = result.normal;
  for (unsigned i = 0; i < 4; ++i) si.weights[i] = result.weights[i];
  si.geometric_penetration = result.geometric_penetration * f.length;
  si.gap = result.gap * f.length; si.distance = result.distance * f.length;
  si.incoming_stiffness = result.incoming_stiffness * f.stiffness;
  if (!Finite(si)) return GeometryStatus::NonfiniteResult;
  *output = si; return GeometryStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
