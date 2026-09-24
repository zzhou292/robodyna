// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "PrismQualification.h"
#include <cmath>

namespace tlfea::contact::self_contact_filters::detail {
// Keep host classification behavior; CUDA supplies the matching scalar test.
TL_SURFACE_HD inline bool ScalarFinite(double value) noexcept {
#if defined(__CUDA_ARCH__)
  return ::isfinite(value);
#else
  return std::isfinite(value);
#endif
}

// Preserve std::min/max first-operand ties, including signed zeros.
TL_SURFACE_HD inline double OrderedMin(double a, double b) noexcept { return b < a ? b : a; }
TL_SURFACE_HD inline double OrderedMax(double a, double b) noexcept { return a < b ? b : a; }

TL_SURFACE_HD inline double Down(double value) noexcept {
  return ::nextafter(
      value, -HUGE_VAL);
}

TL_SURFACE_HD inline double Up(double value) noexcept {
  return ::nextafter(
      value, HUGE_VAL);
}

struct Interval {
  double lower = 0;
  double upper = 0;
};

template <class Triangle>
TL_SURFACE_HD inline bool Finite(const Triangle& triangle) noexcept {
  for (const auto point : triangle.vertices)
    if (!IsFinite(point))
      return false;
  return true;
}

template <class Triangle>
TL_SURFACE_HD inline bool SameGeometry(
    const Triangle& first,
    const Triangle& second) noexcept {
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    const auto a = first.vertices[vertex];
    const auto b = second.vertices[vertex];
    if (a.x != b.x || a.y != b.y || a.z != b.z)
      return false;
  }
  return true;
}

TL_SURFACE_HD inline bool ProductInterval(double a, double b, Interval* output) noexcept {
  const double value = a * b;
  if (!ScalarFinite(value))
    return false;
  *output = {Down(value), Up(value)};
  return ScalarFinite(output->lower) &&
      ScalarFinite(output->upper);
}

TL_SURFACE_HD inline bool AddInterval(Interval a, Interval b, Interval* output) noexcept {
  const double lower = Down(a.lower + b.lower);
  const double upper = Up(a.upper + b.upper);
  if (!ScalarFinite(lower) || !ScalarFinite(upper))
    return false;
  *output = {lower, upper};
  return true;
}

TL_SURFACE_HD inline bool DotInterval(Vec3 point, Vec3 axis, Interval* output) noexcept {
  Interval x, y, z, sum;
  return ProductInterval(point.x, axis.x, &x) &&
      ProductInterval(point.y, axis.y, &y) &&
      ProductInterval(point.z, axis.z, &z) &&
      AddInterval(x, y, &sum) &&
      AddInterval(sum, z, output);
}

template <class Triangle>
TL_SURFACE_HD inline bool ProjectionBounds(
    const Triangle& base,
    const Triangle& current, Vec3 axis,
    Interval* output) noexcept {
  bool first = true;
  Interval next;
  const Triangle* endpoints[2]{&base, &current};
  const unsigned endpoint_count = SameGeometry(base, current) ? 1 : 2;
  for (unsigned endpoint = 0; endpoint < endpoint_count; ++endpoint) {
    const auto* triangle = endpoints[endpoint];
    for (const auto point : triangle->vertices) {
      Interval projection;
      if (!DotInterval(point, axis, &projection))
        return false;
      if (first) {
        next = projection;
        first = false;
      } else {
        next.lower = OrderedMin(next.lower, projection.lower);
        next.upper = OrderedMax(next.upper, projection.upper);
      }
    }
  }
  *output = next;
  return true;
}

template <class Triangle>
TL_SURFACE_HD inline Vec3 EdgeAxis(
    const Triangle& triangle, unsigned edge) noexcept {
  const auto& first = triangle.vertices[edge];
  const auto& second = triangle.vertices[(edge + 1) % 3];
  return {
      second.x - first.x,
      second.y - first.y,
      second.z - first.z};
}

TL_SURFACE_HD inline Vec3 CrossAxis(Vec3 first, Vec3 second) noexcept {
  return {
      first.y * second.z - first.z * second.y,
      first.z * second.x - first.x * second.z,
      first.x * second.y - first.y * second.x};
}

TL_SURFACE_HD inline Vec3 Difference(Vec3 first, Vec3 second) noexcept {
  return {
      first.x - second.x,
      first.y - second.y,
      first.z - second.z};
}

template <class Triangle>
TL_SURFACE_HD inline Vec3 FaceAxis(const Triangle& triangle) noexcept {
  const Vec3 second = Difference(
      triangle.vertices[2], triangle.vertices[0]);
  return CrossAxis(EdgeAxis(triangle, 0), second);
}

template <class Triangle>
TL_SURFACE_HD inline Vec3 VertexEdgeAxis(
    Vec3 vertex, const Triangle& triangle,
    unsigned edge) noexcept {
  const auto edge_axis = EdgeAxis(triangle, edge);
  const auto vertex_from_start = Difference(
      vertex, triangle.vertices[edge]);
  // e x ((v - e0) x e) is division-free, perpendicular to the line, and
  // points along the represented line-to-vertex displacement.
  return CrossAxis(
      edge_axis, CrossAxis(vertex_from_start, edge_axis));
}

TL_SURFACE_HD inline bool InflateProjection(
    double thickness, double norm_l1, Interval* interval) noexcept {
  const double margin = Up(thickness * norm_l1);
  if (!ScalarFinite(margin))
    return false;
  const double lower = Down(interval->lower - margin);
  const double upper = Up(interval->upper + margin);
  if (!ScalarFinite(lower) || !ScalarFinite(upper))
    return false;
  *interval = {lower, upper};
  return true;
}

template <bool observe, class Triangle>
TL_SURFACE_HD inline bool AxisSeparates(
    const Triangle& first_base,
    const Triangle& first_current,
    const Triangle& second_base,
    const Triangle& second_current,
    double first_thickness, double second_thickness,
    Vec3 axis, self_contact_filters::PrismCounts* counts) noexcept {
  if constexpr (observe) ++counts->axis_tests;
  if (!IsFinite(axis))
    return false;
  const double norm_l1 = Up(Up(
      ::fabs(axis.x) + ::fabs(axis.y)) +
      ::fabs(axis.z));
  if (!ScalarFinite(norm_l1) || !(norm_l1 > 0))
    return false;
  Interval first, second;
  if (!ProjectionBounds(first_base, first_current, axis, &first) ||
      !ProjectionBounds(second_base, second_current, axis, &second))
    return false;
  if (!InflateProjection(first_thickness, norm_l1, &first) ||
      !InflateProjection(second_thickness, norm_l1, &second))
    return false;
  return first.upper < second.lower ||
      second.upper < first.lower;
}

TL_SURFACE_HD inline double Component(Vec3 value, unsigned axis) noexcept {
  return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

template <class Triangle>
TL_SURFACE_HD inline bool InflatedFacetBoundsSeparated(
    const Triangle& first, double first_thickness,
    const Triangle& second, double second_thickness) noexcept {
  const double infinity = HUGE_VAL;
  for (unsigned axis = 0; axis < 3; ++axis) {
    double first_lower = Component(first.vertices[0], axis);
    double first_upper = first_lower;
    double second_lower = Component(second.vertices[0], axis);
    double second_upper = second_lower;
    for (unsigned vertex = 1; vertex < 3; ++vertex) {
      const auto a = Component(first.vertices[vertex], axis);
      const auto b = Component(second.vertices[vertex], axis);
      first_lower = OrderedMin(first_lower, a);
      first_upper = OrderedMax(first_upper, a);
      second_lower = OrderedMin(second_lower, b);
      second_upper = OrderedMax(second_upper, b);
    }
    first_lower = ::nextafter(
        first_lower - first_thickness, -infinity);
    first_upper = ::nextafter(
        first_upper + first_thickness, infinity);
    second_lower = ::nextafter(
        second_lower - second_thickness, -infinity);
    second_upper = ::nextafter(
        second_upper + second_thickness, infinity);
    if (first_upper < second_lower ||
        second_upper < first_lower)
      return true;
  }
  return false;
}


}  // namespace tlfea::contact::self_contact_filters::detail
