// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Geometry.h"

#include "../FixedTriangleFeatureDiscovery.h"
#include "../SurfaceContactGeometry.h"
#include "../weighted_surface/Mapping.h"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace tlfea::contact {
namespace fixed_triangle_features {
namespace {

template <class T>
int ScalarCompare(const T& a, const T& b) noexcept {
  return a < b ? -1 : (b < a ? 1 : 0);
}

bool Same(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  return Compare(a, b) == 0;
}

bool Same(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept {
  return Compare(a, b) == 0;
}

bool Same(const FixedTriangleKey& a, const FixedTriangleKey& b) noexcept {
  return Compare(a, b) == 0;
}

bool Same(Vec3 a, Vec3 b) noexcept {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool SameTriangleValue(const CurrentFixedTriangle& a,
                       const CurrentFixedTriangle& b) noexcept {
  if (!Same(a.key, b.key))
    return false;
  for (unsigned i = 0; i < 3; ++i) {
    if (!Same(a.vertices[i], b.vertices[i]) ||
        !Same(a.vertex_keys[i], b.vertex_keys[i]) ||
        !Same(a.edge_keys[i], b.edge_keys[i]))
      return false;
  }
  return true;
}

FixedTriangleDiscoveryStatus ValidateTriangle(
    const CurrentFixedTriangle& value) noexcept {
  TriangleGeometry triangle;
  triangle.face_id = 1;
  for (unsigned i = 0; i < 3; ++i) {
    if (!IsFinite(value.vertices[i]))
      return FixedTriangleDiscoveryStatus::InvalidInput;
    if (value.vertex_keys[i].source_instance_id !=
        value.key.source_instance_id)
      return FixedTriangleDiscoveryStatus::InvalidInput;
    triangle.vertices[i] = value.vertices[i];
    triangle.vertex_ids[i] = i;
    for (unsigned j = 0; j < i; ++j)
      if (Same(value.vertex_keys[i], value.vertex_keys[j]))
        return FixedTriangleDiscoveryStatus::InvalidInput;

    const unsigned next = (i + 1) % 3;
    const bool edge_matches =
        (Same(value.edge_keys[i].endpoints[0], value.vertex_keys[i]) &&
         Same(value.edge_keys[i].endpoints[1], value.vertex_keys[next])) ||
        (Same(value.edge_keys[i].endpoints[1], value.vertex_keys[i]) &&
         Same(value.edge_keys[i].endpoints[0], value.vertex_keys[next]));
    if (!edge_matches)
      return FixedTriangleDiscoveryStatus::InvalidInput;
    if (Compare(value.edge_keys[i].endpoints[1],
                value.edge_keys[i].endpoints[0]) <= 0)
      return FixedTriangleDiscoveryStatus::InvalidInput;
  }
  TrianglePointGeometry point;
  const auto status =
      ClosestPointOnTriangle(value.vertices[0], triangle, &point);
  if (status != Status::kOk)
    return status == Status::kNonFiniteResult
               ? FixedTriangleDiscoveryStatus::NonFiniteResult
               : FixedTriangleDiscoveryStatus::InvalidInput;
  return point.degenerate ? FixedTriangleDiscoveryStatus::DegenerateTriangle
                          : FixedTriangleDiscoveryStatus::Ok;
}

FixedTriangleDiscoveryStatus ValidateSharedFeatureValues(
    const CurrentFixedTriangle& a,
    const CurrentFixedTriangle& b) noexcept {
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      if (Same(a.vertex_keys[i], b.vertex_keys[j]) &&
          !Same(a.vertices[i], b.vertices[j]))
        return FixedTriangleDiscoveryStatus::IdentityMismatch;
  return FixedTriangleDiscoveryStatus::Ok;
}

TriangleGeometry Geometry(const CurrentFixedTriangle& value) noexcept {
  TriangleGeometry result;
  result.face_id = 1;
  for (unsigned i = 0; i < 3; ++i) {
    result.vertices[i] = value.vertices[i];
    result.vertex_ids[i] = i;
  }
  return result;
}

SegmentGeometry EdgeGeometry(const CurrentFixedTriangle& value,
                             unsigned edge) noexcept {
  const unsigned next = (edge + 1) % 3;
  return {{value.vertices[edge], value.vertices[next]}, {edge, next}};
}

bool VertexInTriangleTopology(const FacetVertexKey& vertex,
                              const CurrentFixedTriangle& triangle) noexcept {
  for (const auto& other : triangle.vertex_keys)
    if (Same(vertex, other))
      return true;
  return false;
}

bool EdgesIncident(const FacetEdgeKey& a,
                   const FacetEdgeKey& b) noexcept {
  if (Same(a, b))
    return true;
  for (const auto& av : a.endpoints)
    for (const auto& bv : b.endpoints)
      if (Same(av, bv))
        return true;
  return false;
}

double CanonicalEdgeParameter(const CurrentFixedTriangle& triangle,
                              unsigned edge, double parameter) noexcept {
  return Same(triangle.vertex_keys[edge],
              triangle.edge_keys[edge].endpoints[0])
             ? parameter
             : 1 - parameter;
}

struct L2 {
  long double x = 0;
  long double y = 0;
};

struct L3 {
  long double x = 0;
  long double y = 0;
  long double z = 0;
};

L3 ToLong(Vec3 value) noexcept {
  return {value.x, value.y, value.z};
}

L3 Subtract(L3 a, L3 b) noexcept {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

L3 Add(L3 a, L3 b) noexcept {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

L3 Scale(L3 a, long double scale) noexcept {
  return {a.x * scale, a.y * scale, a.z * scale};
}

long double Dot(L3 a, L3 b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

L3 Cross(L3 a, L3 b) noexcept {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

long double Cross(L2 a, L2 b) noexcept {
  return a.x * b.y - a.y * b.x;
}

L2 Subtract(L2 a, L2 b) noexcept {
  return {a.x - b.x, a.y - b.y};
}

long double Orient(L2 a, L2 b, L2 p) noexcept {
  return Cross(Subtract(b, a), Subtract(p, a));
}

int DominantAxis(L3 normal) noexcept {
  const long double x = std::fabs(normal.x);
  const long double y = std::fabs(normal.y);
  const long double z = std::fabs(normal.z);
  return x >= y && x >= z ? 0 : (y >= z ? 1 : 2);
}

L2 Project(L3 value, int drop) noexcept {
  if (drop == 0)
    return {value.y, value.z};
  if (drop == 1)
    return {value.x, value.z};
  return {value.x, value.y};
}

bool InClosedTriangle(L2 point, const L2* triangle) noexcept {
  const long double a = Orient(triangle[0], triangle[1], point);
  const long double b = Orient(triangle[1], triangle[2], point);
  const long double c = Orient(triangle[2], triangle[0], point);
  return (a >= 0 && b >= 0 && c >= 0) ||
         (a <= 0 && b <= 0 && c <= 0);
}

bool PointOnSegment(L2 point, L2 a, L2 b) noexcept {
  if (Orient(a, b, point) != 0)
    return false;
  return point.x >= std::min(a.x, b.x) &&
         point.x <= std::max(a.x, b.x) &&
         point.y >= std::min(a.y, b.y) &&
         point.y <= std::max(a.y, b.y);
}

struct Witnesses {
  L3 values[48];
  std::size_t count = 0;

  void Add(L3 value) noexcept {
    if (count < 48)
      values[count++] = value;
  }
};

void SegmentSegmentWitnesses(L3 a0, L3 a1, L3 b0, L3 b1, int drop,
                             Witnesses* output) noexcept {
  const L2 p = Project(a0, drop);
  const L2 p1 = Project(a1, drop);
  const L2 q = Project(b0, drop);
  const L2 q1 = Project(b1, drop);
  const L2 r = Subtract(p1, p);
  const L2 s = Subtract(q1, q);
  const long double denominator = Cross(r, s);
  const L2 qp = Subtract(q, p);
  if (denominator != 0) {
    const long double t = Cross(qp, s) / denominator;
    const long double u = Cross(qp, r) / denominator;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
      output->Add(Add(a0, Scale(Subtract(a1, a0), t)));
    return;
  }
  if (Cross(qp, r) != 0)
    return;
  if (PointOnSegment(p, q, q1))
    output->Add(a0);
  if (PointOnSegment(p1, q, q1))
    output->Add(a1);
  if (PointOnSegment(q, p, p1))
    output->Add(b0);
  if (PointOnSegment(q1, p, p1))
    output->Add(b1);
}

bool CoplanarPositiveArea(const L3* a, const L3* b, int drop,
                          bool* separated) noexcept {
  L2 p[3], q[3];
  for (unsigned i = 0; i < 3; ++i) {
    p[i] = Project(a[i], drop);
    q[i] = Project(b[i], drop);
  }
  bool strict_on_every_axis = true;
  for (unsigned owner = 0; owner < 2; ++owner) {
    const L2* axes = owner == 0 ? p : q;
    for (unsigned edge = 0; edge < 3; ++edge) {
      const L2 delta = Subtract(axes[(edge + 1) % 3], axes[edge]);
      const L2 axis{-delta.y, delta.x};
      long double min_p = axis.x * p[0].x + axis.y * p[0].y;
      long double max_p = min_p;
      long double min_q = axis.x * q[0].x + axis.y * q[0].y;
      long double max_q = min_q;
      for (unsigned i = 1; i < 3; ++i) {
        const long double pp = axis.x * p[i].x + axis.y * p[i].y;
        const long double qq = axis.x * q[i].x + axis.y * q[i].y;
        min_p = std::min(min_p, pp);
        max_p = std::max(max_p, pp);
        min_q = std::min(min_q, qq);
        max_q = std::max(max_q, qq);
      }
      const long double overlap =
          std::min(max_p, max_q) - std::max(min_p, min_q);
      if (overlap < 0) {
        *separated = true;
        return false;
      }
      if (overlap == 0)
        strict_on_every_axis = false;
    }
  }
  *separated = false;
  return strict_on_every_axis;
}

void CoplanarWitnesses(const L3* a, const L3* b, int drop,
                       Witnesses* output) noexcept {
  L2 p[3], q[3];
  for (unsigned i = 0; i < 3; ++i) {
    p[i] = Project(a[i], drop);
    q[i] = Project(b[i], drop);
  }
  for (unsigned i = 0; i < 3; ++i) {
    if (InClosedTriangle(p[i], q))
      output->Add(a[i]);
    if (InClosedTriangle(q[i], p))
      output->Add(b[i]);
    for (unsigned j = 0; j < 3; ++j)
      SegmentSegmentWitnesses(a[i], a[(i + 1) % 3], b[j],
                              b[(j + 1) % 3], drop, output);
  }
}

long double PlaneSide(L3 point, L3 origin, L3 normal) noexcept {
  return Dot(Subtract(point, origin), normal);
}

void SegmentTriangleWitnesses(L3 p0, L3 p1, const L3* triangle,
                              L3 normal, int drop,
                              Witnesses* output) noexcept {
  const long double s0 = PlaneSide(p0, triangle[0], normal);
  const long double s1 = PlaneSide(p1, triangle[0], normal);
  L2 projected[3];
  for (unsigned i = 0; i < 3; ++i)
    projected[i] = Project(triangle[i], drop);
  if (s0 == 0 && s1 == 0) {
    if (InClosedTriangle(Project(p0, drop), projected))
      output->Add(p0);
    if (InClosedTriangle(Project(p1, drop), projected))
      output->Add(p1);
    for (unsigned i = 0; i < 3; ++i)
      SegmentSegmentWitnesses(p0, p1, triangle[i],
                              triangle[(i + 1) % 3], drop, output);
    return;
  }
  if ((s0 > 0 && s1 > 0) || (s0 < 0 && s1 < 0))
    return;
  const long double denominator = s0 - s1;
  if (denominator == 0)
    return;
  const long double parameter = s0 / denominator;
  if (parameter < 0 || parameter > 1)
    return;
  const L3 point = Add(p0, Scale(Subtract(p1, p0), parameter));
  if (InClosedTriangle(Project(point, drop), projected))
    output->Add(point);
}

bool SamePoint(L3 point, Vec3 value) noexcept {
  return point.x == static_cast<long double>(value.x) &&
         point.y == static_cast<long double>(value.y) &&
         point.z == static_cast<long double>(value.z);
}

bool PointOnSegment(L3 point, Vec3 first, Vec3 second) noexcept {
  const L3 a = ToLong(first);
  const L3 b = ToLong(second);
  const L3 edge = Subtract(b, a);
  const L3 delta = Subtract(point, a);
  const L3 cross = Cross(edge, delta);
  if (cross.x != 0 || cross.y != 0 || cross.z != 0)
    return false;
  const long double projection = Dot(delta, edge);
  return projection >= 0 && projection <= Dot(edge, edge);
}

struct SharedTopology {
  int vertex_a = -1;
  int vertex_b = -1;
  int edge_a = -1;
  int edge_b = -1;
};

SharedTopology FindSharedTopology(const CurrentFixedTriangle& a,
                                  const CurrentFixedTriangle& b) noexcept {
  SharedTopology result;
  for (unsigned i = 0; i < 3; ++i) {
    for (unsigned j = 0; j < 3; ++j) {
      if (Same(a.vertex_keys[i], b.vertex_keys[j]) &&
          result.vertex_a < 0) {
        result.vertex_a = static_cast<int>(i);
        result.vertex_b = static_cast<int>(j);
      }
      if (Same(a.edge_keys[i], b.edge_keys[j]) && result.edge_a < 0) {
        result.edge_a = static_cast<int>(i);
        result.edge_b = static_cast<int>(j);
      }
    }
  }
  return result;
}

FixedTriangleLocalExclusion LocalIntersectionExclusion(
    const CurrentFixedTriangle& a, const CurrentFixedTriangle& b,
    const Witnesses& witnesses, bool positive_area) noexcept {
  if (Same(a.key, b.key))
    return FixedTriangleLocalExclusion::IdenticalFace;
  if (positive_area || witnesses.count == 0)
    return FixedTriangleLocalExclusion::None;
  const auto shared = FindSharedTopology(a, b);
  if (shared.edge_a >= 0) {
    const unsigned next_a = (shared.edge_a + 1) % 3;
    const unsigned next_b = (shared.edge_b + 1) % 3;
    bool local = true;
    for (std::size_t i = 0; i < witnesses.count; ++i) {
      local = local &&
              PointOnSegment(witnesses.values[i],
                             a.vertices[shared.edge_a],
                             a.vertices[next_a]) &&
              PointOnSegment(witnesses.values[i],
                             b.vertices[shared.edge_b],
                             b.vertices[next_b]);
    }
    if (local)
      return FixedTriangleLocalExclusion::SharedEdgeOnly;
  }
  if (shared.vertex_a >= 0) {
    bool local = true;
    for (std::size_t i = 0; i < witnesses.count; ++i) {
      local = local &&
              SamePoint(witnesses.values[i],
                        a.vertices[shared.vertex_a]) &&
              SamePoint(witnesses.values[i],
                        b.vertices[shared.vertex_b]);
    }
    if (local)
      return FixedTriangleLocalExclusion::SharedVertexOnly;
  }
  return FixedTriangleLocalExclusion::None;
}

bool ClassifyIntersection(const CurrentFixedTriangle& a,
                          const CurrentFixedTriangle& b,
                          FixedTriangleIntersection* output) noexcept {
  L3 av[3], bv[3];
  for (unsigned i = 0; i < 3; ++i) {
    av[i] = ToLong(a.vertices[i]);
    bv[i] = ToLong(b.vertices[i]);
  }
  const L3 normal_a = Cross(Subtract(av[1], av[0]),
                            Subtract(av[2], av[0]));
  const L3 normal_b = Cross(Subtract(bv[1], bv[0]),
                            Subtract(bv[2], bv[0]));
  bool coplanar = true;
  for (unsigned i = 0; i < 3; ++i)
    coplanar = coplanar && PlaneSide(bv[i], av[0], normal_a) == 0;

  Witnesses witnesses;
  bool positive_area = false;
  if (coplanar) {
    bool separated = false;
    const int drop = DominantAxis(normal_a);
    positive_area =
        CoplanarPositiveArea(av, bv, drop, &separated);
    if (separated)
      return false;
    CoplanarWitnesses(av, bv, drop, &witnesses);
    output->kind = positive_area
                       ? FixedTriangleIntersectionKind::CoplanarOverlap
                       : FixedTriangleIntersectionKind::CoplanarTouch;
  } else {
    const int drop_a = DominantAxis(normal_a);
    const int drop_b = DominantAxis(normal_b);
    for (unsigned i = 0; i < 3; ++i) {
      SegmentTriangleWitnesses(av[i], av[(i + 1) % 3], bv,
                               normal_b, drop_b, &witnesses);
      SegmentTriangleWitnesses(bv[i], bv[(i + 1) % 3], av,
                               normal_a, drop_a, &witnesses);
    }
    if (witnesses.count == 0)
      return false;
    output->kind = FixedTriangleIntersectionKind::Transverse;
  }
  output->local_exclusion =
      LocalIntersectionExclusion(a, b, witnesses, positive_area);
  return true;
}

FixedTriangleDiscoveryStatus AddVertexFace(
    const CurrentFixedTriangle& vertex_triangle, unsigned vertex,
    const CurrentFixedTriangle& face_triangle, bool vertex_is_first,
    const CurrentFixedTriangle& canonical_first,
    const CurrentFixedTriangle& canonical_second,
    PairResult* output) noexcept {
  ++output->feature_task_count;
  if (VertexInTriangleTopology(vertex_triangle.vertex_keys[vertex],
                               face_triangle))
    return FixedTriangleDiscoveryStatus::Ok;
  TrianglePointGeometry closest;
  const auto status = ClosestPointOnTriangle(
      vertex_triangle.vertices[vertex], Geometry(face_triangle), &closest);
  if (status != Status::kOk)
    return status == Status::kNonFiniteResult
               ? FixedTriangleDiscoveryStatus::NonFiniteResult
               : FixedTriangleDiscoveryStatus::InvalidInput;
  auto& result = output->features[output->feature_count++];
  result.kind = FixedTriangleCandidateKind::VertexFace;
  result.triangles[0] = canonical_first.key;
  result.triangles[1] = canonical_second.key;
  result.local_features[vertex_is_first ? 0 : 1] = vertex;
  result.local_features[vertex_is_first ? 1 : 0] = 3;
  result.vertex = vertex_triangle.vertex_keys[vertex];
  result.face = face_triangle.key;
  result.points[0] = vertex_triangle.vertices[vertex];
  result.points[1] = closest.point;
  result.distance_m = closest.distance;
  for (unsigned i = 0; i < 3; ++i)
    result.face_weights[i] = closest.weights[i];
  return FixedTriangleDiscoveryStatus::Ok;
}

FixedTriangleDiscoveryStatus AddEdgeEdge(
    const CurrentFixedTriangle& a, unsigned edge_a,
    const CurrentFixedTriangle& b, unsigned edge_b,
    PairResult* output) noexcept {
  ++output->feature_task_count;
  if (EdgesIncident(a.edge_keys[edge_a], b.edge_keys[edge_b]))
    return FixedTriangleDiscoveryStatus::Ok;
  SegmentPairGeometry closest;
  const auto status = ClosestPointsBetweenSegments(
      EdgeGeometry(a, edge_a), EdgeGeometry(b, edge_b), &closest);
  if (status != Status::kOk)
    return status == Status::kNonFiniteResult
               ? FixedTriangleDiscoveryStatus::NonFiniteResult
               : FixedTriangleDiscoveryStatus::InvalidInput;
  auto& result = output->features[output->feature_count++];
  result.kind = FixedTriangleCandidateKind::EdgeEdge;
  result.triangles[0] = a.key;
  result.triangles[1] = b.key;
  result.local_features[0] = edge_a;
  result.local_features[1] = edge_b;
  const double parameter_a =
      CanonicalEdgeParameter(a, edge_a, closest.parameter_a);
  const double parameter_b =
      CanonicalEdgeParameter(b, edge_b, closest.parameter_b);
  if (Compare(a.edge_keys[edge_a], b.edge_keys[edge_b]) <= 0) {
    result.edges[0] = a.edge_keys[edge_a];
    result.edges[1] = b.edge_keys[edge_b];
    result.points[0] = closest.point_a;
    result.points[1] = closest.point_b;
    result.edge_parameters[0] = parameter_a;
    result.edge_parameters[1] = parameter_b;
  } else {
    result.edges[0] = b.edge_keys[edge_b];
    result.edges[1] = a.edge_keys[edge_a];
    result.points[0] = closest.point_b;
    result.points[1] = closest.point_a;
    result.edge_parameters[0] = parameter_b;
    result.edge_parameters[1] = parameter_a;
  }
  result.distance_m = closest.distance;
  return FixedTriangleDiscoveryStatus::Ok;
}

}  // namespace

int Compare(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  const auto aa = std::tie(a.source_instance_id, a.kind, a.first, a.second,
                           a.numerator, a.denominator, a.level, a.grid_i,
                           a.grid_j);
  const auto bb = std::tie(b.source_instance_id, b.kind, b.first, b.second,
                           b.numerator, b.denominator, b.level, b.grid_i,
                           b.grid_j);
  return aa < bb ? -1 : (bb < aa ? 1 : 0);
}

int Compare(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept {
  int value = ScalarCompare(a.parent_boundary, b.parent_boundary);
  if (!value)
    value = ScalarCompare(a.parent_eid, b.parent_eid);
  if (!value)
    value = Compare(a.endpoints[0], b.endpoints[0]);
  if (!value)
    value = Compare(a.endpoints[1], b.endpoints[1]);
  return value;
}

int Compare(const FixedTriangleKey& a, const FixedTriangleKey& b) noexcept {
  const auto aa =
      std::tie(a.source_instance_id, a.parent_eid, a.level, a.local_facet);
  const auto bb =
      std::tie(b.source_instance_id, b.parent_eid, b.level, b.local_facet);
  return aa < bb ? -1 : (bb < aa ? 1 : 0);
}

bool FeatureLess(const FixedTriangleFeatureCandidate& a,
                 const FixedTriangleFeatureCandidate& b) noexcept {
  int value = Compare(a.triangles[0], b.triangles[0]);
  if (!value)
    value = Compare(a.triangles[1], b.triangles[1]);
  if (!value)
    value = ScalarCompare(a.kind, b.kind);
  if (!value)
    value = ScalarCompare(a.local_features[0], b.local_features[0]);
  if (!value)
    value = ScalarCompare(a.local_features[1], b.local_features[1]);
  if (!value && a.kind == FixedTriangleCandidateKind::VertexFace)
    value = Compare(a.vertex, b.vertex);
  if (!value && a.kind == FixedTriangleCandidateKind::VertexFace)
    value = Compare(a.face, b.face);
  if (!value && a.kind == FixedTriangleCandidateKind::EdgeEdge)
    value = Compare(a.edges[0], b.edges[0]);
  if (!value && a.kind == FixedTriangleCandidateKind::EdgeEdge)
    value = Compare(a.edges[1], b.edges[1]);
  return value < 0;
}

bool SameFeatureTask(const FixedTriangleFeatureCandidate& a,
                     const FixedTriangleFeatureCandidate& b) noexcept {
  return !FeatureLess(a, b) && !FeatureLess(b, a);
}

bool IntersectionLess(const FixedTriangleIntersection& a,
                      const FixedTriangleIntersection& b) noexcept {
  const int first = Compare(a.triangles[0], b.triangles[0]);
  return first ? first < 0
               : Compare(a.triangles[1], b.triangles[1]) < 0;
}

bool SameIntersectionPair(const FixedTriangleIntersection& a,
                          const FixedTriangleIntersection& b) noexcept {
  return !IntersectionLess(a, b) && !IntersectionLess(b, a);
}

FixedTriangleDiscoveryStatus EvaluatePair(const CurrentFixedTriangle& input_a,
                                          const CurrentFixedTriangle& input_b,
                                          PairResult* output) noexcept {
  if (!output)
    return FixedTriangleDiscoveryStatus::InvalidInput;
  *output = {};
  const CurrentFixedTriangle* a = &input_a;
  const CurrentFixedTriangle* b = &input_b;
  if (Compare(b->key, a->key) < 0)
    std::swap(a, b);
  if (Same(a->key, b->key) && !SameTriangleValue(*a, *b))
    return FixedTriangleDiscoveryStatus::IdentityMismatch;
  auto status = ValidateTriangle(*a);
  if (status != FixedTriangleDiscoveryStatus::Ok)
    return status;
  status = ValidateTriangle(*b);
  if (status != FixedTriangleDiscoveryStatus::Ok)
    return status;
  status = ValidateSharedFeatureValues(*a, *b);
  if (status != FixedTriangleDiscoveryStatus::Ok)
    return status;

  output->intersection.triangles[0] = a->key;
  output->intersection.triangles[1] = b->key;
  output->intersects =
      ClassifyIntersection(*a, *b, &output->intersection);

  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    status = AddVertexFace(*a, vertex, *b, true, *a, *b, output);
    if (status != FixedTriangleDiscoveryStatus::Ok)
      return status;
    status = AddVertexFace(*b, vertex, *a, false, *a, *b, output);
    if (status != FixedTriangleDiscoveryStatus::Ok)
      return status;
  }
  for (unsigned edge_a = 0; edge_a < 3; ++edge_a) {
    for (unsigned edge_b = 0; edge_b < 3; ++edge_b) {
      status = AddEdgeEdge(*a, edge_a, *b, edge_b, output);
      if (status != FixedTriangleDiscoveryStatus::Ok)
        return status;
    }
  }
  return FixedTriangleDiscoveryStatus::Ok;
}

}  // namespace fixed_triangle_features

Status EvaluateCurrentFixedTriangle(const FixedContactFacet& facet,
                                    VectorView positions,
                                    CurrentFixedTriangle* output) noexcept {
  if (!output || !positions.valid())
    return Status::kInvalidArgument;
  CurrentFixedTriangle result;
  result.key = {facet.source_instance_id, facet.source.source_parent_id,
                facet.level, facet.local_facet};
  for (unsigned i = 0; i < 3; ++i) {
    const auto status = EvaluateWeightedSurfacePosition(
        positions, facet.vertices[i], &result.vertices[i]);
    if (status != Status::kOk)
      return status;
    result.vertex_keys[i] = facet.vertex_keys[i];
    result.edge_keys[i] = facet.edge_keys[i];
  }
  *output = result;
  return Status::kOk;
}

}  // namespace tlfea::contact
