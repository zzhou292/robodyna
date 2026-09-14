// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../SelfContactCurrentRegularityValues.h"

#include "../SurfaceContactGeometry.h"
#include "../SurfaceMaterialBounds.h"
#include "../fixed_triangle_features/ExactPredicates.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <cfloat>
#include <cmath>

namespace tlfea::contact {

SelfContactCurrentFacetStatus EvaluateCurrentFacetRegularity(
    const Vec3 (&vertices)[3], Vec3 chart_direction,
    SelfContactCurrentFacetWitness* output) noexcept {
  using S = SelfContactCurrentFacetStatus;
  if (!output ||
      !tl::fea::trial_identity::Disjoint(
          output, sizeof(*output), vertices, sizeof(vertices)))
    return S::InvalidInput;
  for (const auto& vertex : vertices)
    if (!IsFinite(vertex)) return S::InvalidInput;
  if (!IsFinite(chart_direction) ||
      !(geometry_detail::MaxAbs(chart_direction) > 0))
    return S::InvalidInput;

  const Vec3 ab = Subtract(vertices[1], vertices[0]);
  const Vec3 ac = Subtract(vertices[2], vertices[0]);
  if (!IsFinite(ab) || !IsFinite(ac)) return S::Unrepresentable;
  const double scale = geometry_detail::Maximum(
      geometry_detail::MaxAbs(ab), geometry_detail::MaxAbs(ac));
  if (!(scale > 0)) return S::Degenerate;
  const Vec3 u = geometry_detail::Divide(ab, scale);
  const Vec3 v = geometry_detail::Divide(ac, scale);
  const Vec3 normal = geometry_detail::Cross(u, v);
  const double normal_length = geometry_detail::Length(normal);
  const Vec3 uv = Subtract(v, u);
  const double largest_edge_squared = geometry_detail::Maximum(
      Dot(u, u), geometry_detail::Maximum(Dot(v, v), Dot(uv, uv)));
  if (!IsFinite(normal) || !IsFinite(normal_length) ||
      !IsFinite(largest_edge_squared) || !(largest_edge_squared > 0))
    return S::Unrepresentable;
  if (normal_length <=
      64 * DBL_EPSILON * largest_edge_squared)
    return S::Degenerate;

  material_detail::CrossBounds first, second, cross;
  Q4IntegralInterval norm, directed;
  const bool bounded =
      material_detail::Edge(vertices[1], vertices[0], 1, &first) &&
      material_detail::Edge(vertices[2], vertices[0], 1, &second) &&
      material_detail::Cross(first, second, &cross) &&
      material_detail::Norm(cross, &norm) &&
      material_detail::Dot(cross, chart_direction, &directed);
  // The outward interval is an exact sign filter.  Only an interval touching
  // zero needs the fixed-capacity dyadic predicate; no classification depends
  // on an unguarded floating-point sign.
  fixed_triangle_features::exact::Sign orientation;
  if (bounded && directed.lower > 0)
    orientation = {1, true};
  else if (bounded && directed.upper < 0)
    orientation = {-1, true};
  else
    orientation = fixed_triangle_features::exact::DirectedTriangle(
        vertices[0], vertices[1], vertices[2], chart_direction);
  if (!orientation.valid) return S::Unrepresentable;
  if (orientation.value <= 0) return S::Reversed;
  if (!bounded)
    return S::Unrepresentable;
  const Vec3 nominal = geometry_detail::Cross(ab, ac);
  const double nominal_norm = geometry_detail::Length(nominal);
  SelfContactCurrentFacetWitness next;
  if (!IsFinite(nominal) || !IsFinite(nominal_norm) ||
      !q4_bounds::Certify(nominal_norm, norm, &next.double_area_m2))
    return S::Unrepresentable;
  next.directed_chart_measure_m2 = directed;
  next.scaled_jacobian_quality =
      normal_length / largest_edge_squared;
  next.exact_orientation_sign = orientation.value;
  if (!IsFinite(next.scaled_jacobian_quality) ||
      !(next.scaled_jacobian_quality > 64 * DBL_EPSILON))
    return S::Degenerate;
  *output = next;
  return S::Ok;
}

}  // namespace tlfea::contact
