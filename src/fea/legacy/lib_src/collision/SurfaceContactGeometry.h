// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// Authors of the source geometry utilities: Alessandro Tasora, Radu Serban,
// Dario Fusai.
// =============================================================================
// Adapted for TL-FEA from src/chrono/utils/ChUtilsGeometry.cpp at Chrono revision
// 5698ef5183fbcfc7acc3740fffa5535565e27195: PointTrianglePlaneDistance and
// ClosestLinesPoints. Changes: bounded feature candidates, barycentric and
// stable feature outputs, scale-aware arithmetic, degenerate/parallel endpoint
// handling, checked finite results, and allocation-free host/device functions.
// This file contains discrete geometry only: no CCD, force law, mesh feature
// ownership, side-selection policy, friction, or FE integration.
//
// The upstream distribution license is reproduced here for this adapted file:
// Copyright (c) 2016, Project Chrono Development Team
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//  - Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  - Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//  - Neither the name of the nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "SurfaceContactTypes.h"

namespace tlfea {
namespace contact {

enum class FeatureKind : std::uint8_t { kVertex, kEdge, kFace };

// Vertex IDs must be globally unique across the participating meshes. An edge
// key uses its sorted endpoint IDs, so either incident triangle produces the
// same key. A face key uses its stable face ID. This identity does NOT decide
// which incident face owns a contact or prevent double counting by itself.
struct FeatureKey {
  FeatureKind kind = FeatureKind::kVertex;
  std::uint64_t first = 0;
  std::uint64_t second = 0;
};

struct SegmentGeometry {
  Vec3 vertices[2];
  std::uint64_t vertex_ids[2] = {0, 1};
};

struct TriangleGeometry {
  Vec3 vertices[3];
  std::uint64_t vertex_ids[3] = {0, 1, 2};
  std::uint64_t face_id = 0;
};

struct SegmentPointGeometry {
  Vec3 point;
  double parameter = 0;  // (1-t)*vertex[0] + t*vertex[1].
  double distance = 0;   // Unsigned distance; no contact-side convention.
  FeatureKey feature;
  bool degenerate = false;
};

struct TrianglePointGeometry {
  Vec3 point;
  double weights[3] = {1, 0, 0};
  double distance = 0;
  FeatureKey feature;
  Vec3 face_normal;                  // Winding-oriented; zero if degenerate.
  double signed_plane_distance = 0;  // Defined only when !degenerate.
  bool degenerate = false;           // Includes unresolved nearly collinear faces.
};

struct SegmentPairGeometry {
  Vec3 point_a;
  Vec3 point_b;
  double parameter_a = 0;
  double parameter_b = 0;
  double distance = 0;
  FeatureKey feature_a;
  FeatureKey feature_b;
  bool degenerate_a = false;
  bool degenerate_b = false;
  bool parallel = false;
};

namespace geometry_detail {

TL_SURFACE_HD inline double Maximum(double a, double b) { return a > b ? a : b; }
TL_SURFACE_HD inline double MaxAbs(Vec3 a) {
  return Maximum(::fabs(a.x), Maximum(::fabs(a.y), ::fabs(a.z)));
}
TL_SURFACE_HD inline Vec3 Divide(Vec3 a, double denominator) {
  return {a.x / denominator, a.y / denominator, a.z / denominator};
}
TL_SURFACE_HD inline Vec3 Cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}
TL_SURFACE_HD inline double Length(Vec3 a) {
  return ::hypot(::hypot(a.x, a.y), a.z);
}
TL_SURFACE_HD inline Vec3 Blend(Vec3 a, Vec3 b, double t) {
  if (t == 0)
    return a;
  if (t == 1)
    return b;
  return Add(Scale(a, 1 - t), Scale(b, t));
}
TL_SURFACE_HD inline FeatureKey EdgeKey(std::uint64_t a, std::uint64_t b) {
  return {FeatureKind::kEdge, a < b ? a : b, a < b ? b : a};
}
TL_SURFACE_HD inline FeatureKey SegmentKey(const SegmentGeometry& segment, double t) {
  if (t == 0)
    return {FeatureKind::kVertex, segment.vertex_ids[0], 0};
  if (t == 1)
    return {FeatureKind::kVertex, segment.vertex_ids[1], 0};
  return EdgeKey(segment.vertex_ids[0], segment.vertex_ids[1]);
}
TL_SURFACE_HD inline bool KeyLess(FeatureKey a, FeatureKey b) {
  if (a.kind != b.kind)
    return static_cast<unsigned>(a.kind) < static_cast<unsigned>(b.kind);
  return a.first != b.first ? a.first < b.first : a.second < b.second;
}
TL_SURFACE_HD inline FeatureKey TriangleKey(const TriangleGeometry& triangle,
                                            const double* weights) {
  for (int i = 0; i < 3; ++i)
    if (weights[i] == 1)
      return {FeatureKind::kVertex, triangle.vertex_ids[i], 0};
  for (int i = 0; i < 3; ++i)
    if (weights[i] == 0)
      return EdgeKey(triangle.vertex_ids[(i + 1) % 3],
                     triangle.vertex_ids[(i + 2) % 3]);
  return {FeatureKind::kFace, triangle.face_id, 0};
}
TL_SURFACE_HD inline bool PairKeyLess(FeatureKey a, FeatureKey b,
                                     FeatureKey old_a, FeatureKey old_b) {
  if (KeyLess(b, a)) {
    const auto temporary = a;
    a = b;
    b = temporary;
  }
  if (KeyLess(old_b, old_a)) {
    const auto temporary = old_a;
    old_a = old_b;
    old_b = temporary;
  }
  if (KeyLess(a, old_a))
    return true;
  if (KeyLess(old_a, a))
    return false;
  return KeyLess(b, old_b);
}

}  // namespace geometry_detail

// The bounded-segment projection closes the endpoint and zero-length cases
// left to callers of Chrono's infinite-line helpers. Finite but unrepresentable
// arithmetic returns an explicit failure, never a plausible zero distance.
TL_SURFACE_HD inline Status ClosestPointOnSegment(
    Vec3 query, const SegmentGeometry& segment, SegmentPointGeometry* out) {
  if (!out)
    return Status::kInvalidArgument;
  *out = {};
  if (!IsFinite(query) || !IsFinite(segment.vertices[0]) || !IsFinite(segment.vertices[1]))
    return Status::kInvalidArgument;
  const Vec3 edge = Subtract(segment.vertices[1], segment.vertices[0]);
  const Vec3 delta = Subtract(query, segment.vertices[0]);
  if (!IsFinite(edge) || !IsFinite(delta))
    return Status::kNonFiniteResult;
  const double edge_scale = geometry_detail::MaxAbs(edge);
  SegmentPointGeometry result;
  if (edge_scale == 0) {
    result.degenerate = true;
    result.parameter = segment.vertex_ids[0] <= segment.vertex_ids[1] ? 0 : 1;
  } else {
    const Vec3 direction = geometry_detail::Divide(edge, edge_scale);
    const double query_scale = geometry_detail::Maximum(edge_scale, geometry_detail::MaxAbs(delta));
    const double length_scaled = Dot(direction, direction) * (edge_scale / query_scale);
    const double projection = Dot(direction, geometry_detail::Divide(delta, query_scale));
    if (length_scaled <= 0 || !IsFinite(length_scaled) || !IsFinite(projection))
      return Status::kNonFiniteResult;
    result.parameter = projection <= 0 ? 0 : (projection >= length_scaled ? 1 : projection / length_scaled);
  }
  result.point = geometry_detail::Blend(segment.vertices[0], segment.vertices[1], result.parameter);
  result.distance = geometry_detail::Length(Subtract(query, result.point));
  result.feature = geometry_detail::SegmentKey(segment, result.parameter);
  if (!IsFinite(result.point) || !IsFinite(result.distance))
    return Status::kNonFiniteResult;
  *out = result;
  return Status::kOk;
}

// Complete closest point on the CLOSED triangle: the projected face point if
// its barycentric coordinates are inside, plus the three bounded edge minima.
// The barycentric plane construction is adapted from Chrono's utility above;
// cross products avoid subtraction of nearly equal Gram determinants.
// Nearly collinear triangles reduce to their edges and report degeneracy.
// This is floating-point geometry, not an exact-predicate intersection test.
TL_SURFACE_HD inline Status ClosestPointOnTriangle(
    Vec3 query, const TriangleGeometry& triangle, TrianglePointGeometry* out) {
  if (!out)
    return Status::kInvalidArgument;
  *out = {};
  if (!IsFinite(query))
    return Status::kInvalidArgument;
  for (int i = 0; i < 3; ++i)
    if (!IsFinite(triangle.vertices[i]))
      return Status::kInvalidArgument;

  TrianglePointGeometry result;
  result.distance = DBL_MAX;
  bool have_result = false;
  for (int i = 0; i < 3; ++i) {
    const int j = (i + 1) % 3;
    const SegmentGeometry edge{{triangle.vertices[i], triangle.vertices[j]},
                               {triangle.vertex_ids[i], triangle.vertex_ids[j]}};
    SegmentPointGeometry point;
    const auto status = ClosestPointOnSegment(query, edge, &point);
    if (status != Status::kOk)
      return status;
    if (!have_result || point.distance < result.distance ||
        (point.distance == result.distance && geometry_detail::KeyLess(point.feature, result.feature))) {
      result.point = point.point;
      result.distance = point.distance;
      result.feature = point.feature;
      for (int k = 0; k < 3; ++k)
        result.weights[k] = 0;
      result.weights[i] = 1 - point.parameter;
      result.weights[j] = point.parameter;
      have_result = true;
    }
  }

  const Vec3 ab = Subtract(triangle.vertices[1], triangle.vertices[0]);
  const Vec3 ac = Subtract(triangle.vertices[2], triangle.vertices[0]);
  const double scale = geometry_detail::Maximum(geometry_detail::MaxAbs(ab), geometry_detail::MaxAbs(ac));
  result.degenerate = scale == 0;
  if (!result.degenerate) {
    const Vec3 u = geometry_detail::Divide(ab, scale);
    const Vec3 v = geometry_detail::Divide(ac, scale);
    const Vec3 n = geometry_detail::Cross(u, v);
    const double n_length = geometry_detail::Length(n);
    const double largest_edge_squared = geometry_detail::Maximum(
        Dot(u, u), geometry_detail::Maximum(Dot(v, v), Dot(Subtract(v, u), Subtract(v, u))));
    result.degenerate = n_length <= 64 * DBL_EPSILON * largest_edge_squared;
    if (!result.degenerate) {
      result.face_normal = geometry_detail::Divide(n, n_length);
      const Vec3 delta = Subtract(query, triangle.vertices[0]);
      const Vec3 w = geometry_detail::Divide(delta, scale);
      if (!IsFinite(w))
        return Status::kNonFiniteResult;
      result.signed_plane_distance = Dot(delta, result.face_normal);
      const double weight_b = Dot(geometry_detail::Cross(w, v), result.face_normal) / n_length;
      const double weight_c = Dot(geometry_detail::Cross(u, w), result.face_normal) / n_length;
      if (!IsFinite(result.signed_plane_distance) || !IsFinite(weight_b) || !IsFinite(weight_c))
        return Status::kNonFiniteResult;
      if (weight_b >= 0 && weight_c >= 0 && weight_b + weight_c <= 1) {
        const double weights[3] = {1 - (weight_b + weight_c), weight_b, weight_c};
        const Vec3 point = Add(Add(Scale(triangle.vertices[0], weights[0]),
                                   Scale(triangle.vertices[1], weights[1])),
                              Scale(triangle.vertices[2], weights[2]));
        const double distance = geometry_detail::Length(Subtract(query, point));
        const auto feature = geometry_detail::TriangleKey(triangle, weights);
        if (!IsFinite(point) || !IsFinite(distance))
          return Status::kNonFiniteResult;
        // A valid plane projection is the analytic global minimum. Do not
        // lose it to an endpoint when a large normal distance rounds both
        // unsigned distances to the same floating-point number.
        result.point = point;
        result.distance = distance;
        result.feature = feature;
        for (int i = 0; i < 3; ++i)
          result.weights[i] = weights[i];
      }
    }
  }
  *out = result;
  return Status::kOk;
}

namespace geometry_detail {
TL_SURFACE_HD inline Status ConsiderSegmentPair(
    const SegmentGeometry& a, const SegmentGeometry& b, double parameter_a,
    double parameter_b, bool* have_result, SegmentPairGeometry* result) {
  const Vec3 point_a = Blend(a.vertices[0], a.vertices[1], parameter_a);
  const Vec3 point_b = Blend(b.vertices[0], b.vertices[1], parameter_b);
  const double distance = Length(Subtract(point_a, point_b));
  if (!IsFinite(point_a) || !IsFinite(point_b) || !IsFinite(distance))
    return Status::kNonFiniteResult;
  const auto feature_a = SegmentKey(a, parameter_a);
  const auto feature_b = SegmentKey(b, parameter_b);
  if (!*have_result || distance < result->distance ||
      (distance == result->distance &&
       PairKeyLess(feature_a, feature_b, result->feature_a, result->feature_b))) {
    result->point_a = point_a;
    result->point_b = point_b;
    result->parameter_a = parameter_a;
    result->parameter_b = parameter_b;
    result->distance = distance;
    result->feature_a = feature_a;
    result->feature_b = feature_b;
    *have_result = true;
  }
  return Status::kOk;
}
}  // namespace geometry_detail

// Minimize the convex segment-distance quadratic over [0,1]^2. Four boundary
// minima cover endpoints and parallel/zero-length segments; a valid interior
// line minimum completes the domain. Unlike ClosestLinesPoints, parallel lines
// are not rejected. A zero distance deliberately does not invent a normal.
TL_SURFACE_HD inline Status ClosestPointsBetweenSegments(
    const SegmentGeometry& a, const SegmentGeometry& b, SegmentPairGeometry* out) {
  if (!out)
    return Status::kInvalidArgument;
  *out = {};
  for (int i = 0; i < 2; ++i)
    if (!IsFinite(a.vertices[i]) || !IsFinite(b.vertices[i]))
      return Status::kInvalidArgument;
  SegmentPairGeometry result;
  bool have_result = false;
  for (int i = 0; i < 2; ++i) {
    SegmentPointGeometry on_b, on_a;
    auto status = ClosestPointOnSegment(a.vertices[i], b, &on_b);
    if (status != Status::kOk)
      return status;
    status = ClosestPointOnSegment(b.vertices[i], a, &on_a);
    if (status != Status::kOk)
      return status;
    result.degenerate_a = on_a.degenerate;
    result.degenerate_b = on_b.degenerate;
    status = geometry_detail::ConsiderSegmentPair(a, b, i, on_b.parameter, &have_result, &result);
    if (status != Status::kOk)
      return status;
    status = geometry_detail::ConsiderSegmentPair(a, b, on_a.parameter, i, &have_result, &result);
    if (status != Status::kOk)
      return status;
  }
  if (!result.degenerate_a && !result.degenerate_b) {
    const Vec3 edge_a = Subtract(a.vertices[1], a.vertices[0]);
    const Vec3 edge_b = Subtract(b.vertices[1], b.vertices[0]);
    const double scale_a = geometry_detail::MaxAbs(edge_a);
    const double scale_b = geometry_detail::MaxAbs(edge_b);
    const Vec3 u = geometry_detail::Divide(edge_a, scale_a);
    const Vec3 v = geometry_detail::Divide(edge_b, scale_b);
    const Vec3 n = geometry_detail::Cross(u, v);
    const double n_length = geometry_detail::Length(n);
    result.parallel = n_length == 0;
    if (!result.parallel) {
      const Vec3 normal = geometry_detail::Divide(n, n_length);
      const Vec3 delta = Subtract(a.vertices[0], b.vertices[0]);
      const double delta_scale = geometry_detail::Maximum(
          geometry_detail::MaxAbs(delta), geometry_detail::Maximum(scale_a, scale_b));
      const Vec3 w = geometry_detail::Divide(delta, delta_scale);
      // Equivalent to Chrono's closest-line system, evaluated using cross
      // products rather than a*c-b*b (which cancels for near-parallel edges).
      const double s = (Dot(geometry_detail::Cross(v, w), normal) / n_length) * (delta_scale / scale_a);
      const double t = (Dot(geometry_detail::Cross(u, w), normal) / n_length) * (delta_scale / scale_b);
      if (!IsFinite(s) || !IsFinite(t))
        return Status::kNonFiniteResult;
      if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        // A valid interior line minimum is globally closest on the bounded
        // segments, even if large separation hides its improvement in distance.
        have_result = false;
        const auto status = geometry_detail::ConsiderSegmentPair(a, b, s, t, &have_result, &result);
        if (status != Status::kOk)
          return status;
      }
    }
  }
  *out = result;
  return Status::kOk;
}

}  // namespace contact
}  // namespace tlfea
