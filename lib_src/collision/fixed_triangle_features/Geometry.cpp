// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Geometry.h"

#include "ExactPredicates.h"
#include "../FixedTriangleFeatureDiscovery.h"
#include "../SurfaceContactGeometry.h"
#include "../weighted_surface/Mapping.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
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

bool SameEndpoints(const FacetEdgeKey& a,
                   const FacetEdgeKey& b) noexcept {
  return Same(a.endpoints[0], b.endpoints[0]) &&
      Same(a.endpoints[1], b.endpoints[1]);
}

bool Same(const FixedTriangleKey& a, const FixedTriangleKey& b) noexcept {
  return Compare(a, b) == 0;
}

bool Same(Vec3 a, Vec3 b) noexcept {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool SameTriangleValueImpl(const CurrentFixedTriangle& a,
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

FixedTriangleDiscoveryStatus ValidateTriangleImpl(
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
    if (value.edge_keys[i].parent_boundary !=
        (value.edge_keys[i].parent_eid == 0))
      return FixedTriangleDiscoveryStatus::InvalidInput;
    if (!value.edge_keys[i].parent_boundary &&
        value.edge_keys[i].parent_eid != value.key.parent_eid)
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

struct CanonicalTriangleGeometry {
  TriangleGeometry geometry;
  Vec3 vertices[3];
  unsigned original_local[3] = {0, 1, 2};
};

CanonicalTriangleGeometry CanonicalGeometry(
    const CurrentFixedTriangle& value) noexcept {
  CanonicalTriangleGeometry result;
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = i + 1; j < 3; ++j)
      if (Compare(value.vertex_keys[result.original_local[j]],
                  value.vertex_keys[result.original_local[i]]) < 0)
        std::swap(result.original_local[i], result.original_local[j]);
  result.geometry.face_id = 1;
  for (unsigned i = 0; i < 3; ++i) {
    result.vertices[i] = value.vertices[result.original_local[i]];
    result.geometry.vertices[i] = result.vertices[i];
    result.geometry.vertex_ids[i] = i;
  }
  return result;
}

unsigned EdgeWithEndpoints(const CurrentFixedTriangle& triangle,
                           const FacetVertexKey& a,
                           const FacetVertexKey& b) noexcept {
  for (unsigned edge = 0; edge < 3; ++edge) {
    const auto& candidate = triangle.edge_keys[edge];
    if ((Same(candidate.endpoints[0], a) &&
         Same(candidate.endpoints[1], b)) ||
        (Same(candidate.endpoints[0], b) &&
         Same(candidate.endpoints[1], a)))
      return edge;
  }
  return 3;
}

Vec3 VertexValue(const CurrentFixedTriangle& triangle,
                 const FacetVertexKey& key) noexcept {
  for (unsigned i = 0; i < 3; ++i)
    if (Same(triangle.vertex_keys[i], key))
      return triangle.vertices[i];
  return {};
}

SegmentGeometry CanonicalEdgeGeometry(
    const CurrentFixedTriangle& triangle, unsigned edge) noexcept {
  return {{VertexValue(triangle, triangle.edge_keys[edge].endpoints[0]),
           VertexValue(triangle, triangle.edge_keys[edge].endpoints[1])},
          {0, 1}};
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

bool Valid(const exact::Sign& value) noexcept {
  return value.valid;
}

double InteriorUnit(double value) noexcept {
  const double lower = std::nextafter(0.0, 1.0);
  const double upper = std::nextafter(1.0, 0.0);
  return value <= 0 ? lower : (value >= 1 ? upper : value);
}

bool RepresentedFaceWeights(const double input[3],
                            double output[3]) noexcept {
  double sum = 0;
  unsigned anchor = 0;
  for (unsigned i = 0; i < 3; ++i) {
    output[i] = InteriorUnit(input[i]);
    sum += output[i];
    if (output[i] > output[anchor]) anchor = i;
  }
  if (!std::isfinite(sum) || sum <= 0) return false;
  for (unsigned i = 0; i < 3; ++i) output[i] /= sum;
  const unsigned first = (anchor + 1) % 3;
  const unsigned second = (anchor + 2) % 3;
  output[anchor] = 1 - (output[first] + output[second]);
  return output[0] > 0 && output[0] < 1 &&
      output[1] > 0 && output[1] < 1 &&
      output[2] > 0 && output[2] < 1;
}

double RepresentationError(Vec3 rounded, Vec3 represented) noexcept {
  const double value =
      geometry_detail::Length(Subtract(rounded, represented));
  return value == 0 ? 0
                    : std::nextafter(value,
                                     std::numeric_limits<double>::infinity());
}

int ProjectionAxis(const CurrentFixedTriangle& triangle,
                   bool* valid) noexcept {
  for (int drop = 0; drop < 3; ++drop) {
    const auto sign = exact::Orient2D(
        triangle.vertices[0], triangle.vertices[1],
        triangle.vertices[2], drop);
    if (!Valid(sign)) {
      *valid = false;
      return 0;
    }
    if (sign.value) {
      *valid = true;
      return drop;
    }
  }
  *valid = false;
  return 0;
}

bool OnSegment(Vec3 point, Vec3 a, Vec3 b, int drop,
               bool* valid) noexcept {
  const auto orientation = exact::Orient2D(a, b, point, drop);
  if (!Valid(orientation)) {
    *valid = false;
    return false;
  }
  *valid = true;
  if (orientation.value)
    return false;
  const auto first = [drop](Vec3 value) {
    return drop == 0 ? value.y : value.x;
  };
  const auto second = [drop](Vec3 value) {
    return drop == 2 ? value.y : value.z;
  };
  return first(point) >= std::min(first(a), first(b)) &&
         first(point) <= std::max(first(a), first(b)) &&
         second(point) >= std::min(second(a), second(b)) &&
         second(point) <= std::max(second(a), second(b));
}

bool PointInTriangle(Vec3 point, const CurrentFixedTriangle& triangle,
                     int drop, bool* valid) noexcept {
  int signs[3];
  for (unsigned i = 0; i < 3; ++i) {
    const auto sign = exact::Orient2D(
        triangle.vertices[i], triangle.vertices[(i + 1) % 3],
        point, drop);
    if (!Valid(sign)) {
      *valid = false;
      return false;
    }
    signs[i] = sign.value;
  }
  *valid = true;
  return (signs[0] >= 0 && signs[1] >= 0 && signs[2] >= 0) ||
         (signs[0] <= 0 && signs[1] <= 0 && signs[2] <= 0);
}

enum class SegmentIntersection { None, Point, Overlap };

SegmentIntersection IntersectSegments(
    Vec3 a0, Vec3 a1, Vec3 b0, Vec3 b1, int drop,
    bool* valid) noexcept {
  const auto ab0 = exact::Orient2D(a0, a1, b0, drop);
  const auto ab1 = exact::Orient2D(a0, a1, b1, drop);
  const auto ba0 = exact::Orient2D(b0, b1, a0, drop);
  const auto ba1 = exact::Orient2D(b0, b1, a1, drop);
  if (!Valid(ab0) || !Valid(ab1) || !Valid(ba0) || !Valid(ba1)) {
    *valid = false;
    return SegmentIntersection::None;
  }
  *valid = true;
  if (!ab0.value && !ab1.value && !ba0.value && !ba1.value) {
    const auto coordinate = [drop, a0, a1, b0, b1](Vec3 value) {
      const double ax = drop == 0 ? a0.y : a0.x;
      const double ay = drop == 0 ? a1.y : a1.x;
      const double bx = drop == 0 ? b0.y : b0.x;
      const double by = drop == 0 ? b1.y : b1.x;
      const bool use_first =
          std::max(ax, ay) != std::min(ax, ay) ||
          std::max(bx, by) != std::min(bx, by);
      if (use_first)
        return drop == 0 ? value.y : value.x;
      return drop == 2 ? value.y : value.z;
    };
    const double lo = std::max(
        std::min(coordinate(a0), coordinate(a1)),
        std::min(coordinate(b0), coordinate(b1)));
    const double hi = std::min(
        std::max(coordinate(a0), coordinate(a1)),
        std::max(coordinate(b0), coordinate(b1)));
    return lo > hi ? SegmentIntersection::None
                   : (lo == hi ? SegmentIntersection::Point
                               : SegmentIntersection::Overlap);
  }
  bool on = false;
  if (!ab0.value && OnSegment(b0, a0, a1, drop, &on) && on)
    return SegmentIntersection::Point;
  if (!ab1.value && OnSegment(b1, a0, a1, drop, &on) && on)
    return SegmentIntersection::Point;
  if (!ba0.value && OnSegment(a0, b0, b1, drop, &on) && on)
    return SegmentIntersection::Point;
  if (!ba1.value && OnSegment(a1, b0, b1, drop, &on) && on)
    return SegmentIntersection::Point;
  return ab0.value * ab1.value < 0 && ba0.value * ba1.value < 0
             ? SegmentIntersection::Point
             : SegmentIntersection::None;
}

struct CoplanarClassification {
  bool valid = false;
  bool intersects = false;
  bool positive_area = false;
};

CoplanarClassification ClassifyCoplanar(
    const CurrentFixedTriangle& a,
    const CurrentFixedTriangle& b, int drop) noexcept {
  CoplanarClassification result;
  result.valid = true;
  result.intersects = true;
  result.positive_area = true;
  const CurrentFixedTriangle* triangles[2]{&a, &b};
  for (unsigned owner = 0; owner < 2; ++owner) {
    const auto& source = *triangles[owner];
    const auto& other = *triangles[1 - owner];
    for (unsigned edge = 0; edge < 3; ++edge) {
      const unsigned next = (edge + 1) % 3;
      const unsigned opposite = (edge + 2) % 3;
      const auto interior = exact::Orient2D(
          source.vertices[edge], source.vertices[next],
          source.vertices[opposite], drop);
      if (!Valid(interior) || !interior.value) {
        result.valid = false;
        return result;
      }
      bool any_inside = false;
      bool any_on = false;
      for (unsigned vertex = 0; vertex < 3; ++vertex) {
        const auto side = exact::Orient2D(
            source.vertices[edge], source.vertices[next],
            other.vertices[vertex], drop);
        if (!Valid(side)) {
          result.valid = false;
          return result;
        }
        const int normalized = side.value * interior.value;
        any_inside = any_inside || normalized > 0;
        any_on = any_on || normalized == 0;
      }
      if (!any_inside && !any_on) {
        result.intersects = false;
        result.positive_area = false;
        return result;
      }
      if (!any_inside)
        result.positive_area = false;
    }
  }
  return result;
}

bool CoplanarSegmentTriangleIntersects(
    Vec3 p0, Vec3 p1, const CurrentFixedTriangle& triangle,
    int drop, bool* valid) noexcept {
  if (PointInTriangle(p0, triangle, drop, valid) ||
      PointInTriangle(p1, triangle, drop, valid))
    return *valid;
  if (!*valid)
    return false;
  for (unsigned edge = 0; edge < 3; ++edge) {
    const auto relation = IntersectSegments(
        p0, p1, triangle.vertices[edge],
        triangle.vertices[(edge + 1) % 3], drop, valid);
    if (!*valid || relation != SegmentIntersection::None)
      return *valid;
  }
  return false;
}

bool SegmentTriangleIntersects(
    Vec3 p0, Vec3 p1, const CurrentFixedTriangle& triangle,
    int drop, bool* valid) noexcept {
  const auto side0 = exact::Orient3D(
      triangle.vertices[0], triangle.vertices[1],
      triangle.vertices[2], p0);
  const auto side1 = exact::Orient3D(
      triangle.vertices[0], triangle.vertices[1],
      triangle.vertices[2], p1);
  if (!Valid(side0) || !Valid(side1)) {
    *valid = false;
    return false;
  }
  if (side0.value && side0.value == side1.value) {
    *valid = true;
    return false;
  }
  if (!side0.value && !side1.value)
    return CoplanarSegmentTriangleIntersects(
        p0, p1, triangle, drop, valid);
  if (!side0.value || !side1.value) {
    const Vec3 point = !side0.value ? p0 : p1;
    return PointInTriangle(point, triangle, drop, valid);
  }
  int around[3];
  for (unsigned edge = 0; edge < 3; ++edge) {
    const auto sign = exact::Orient3D(
        p0, p1, triangle.vertices[edge],
        triangle.vertices[(edge + 1) % 3]);
    if (!Valid(sign)) {
      *valid = false;
      return false;
    }
    around[edge] = sign.value;
  }
  *valid = true;
  return (around[0] >= 0 && around[1] >= 0 && around[2] >= 0) ||
         (around[0] <= 0 && around[1] <= 0 && around[2] <= 0);
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
      // A physical parent boundary and another parent's fixed-facet
      // tessellation edge intentionally have different FacetEdgeKeys.  Two
      // shared canonical endpoints nevertheless identify the same exact
      // fixed-triangle segment for local intersection classification.
      if (SameEndpoints(a.edge_keys[i], b.edge_keys[j]) &&
          result.edge_a < 0) {
        result.edge_a = static_cast<int>(i);
        result.edge_b = static_cast<int>(j);
      }
    }
  }
  return result;
}

bool EdgeContains(const CurrentFixedTriangle& triangle, unsigned edge,
                  const FacetVertexKey& vertex) noexcept {
  return Same(triangle.vertex_keys[edge], vertex) ||
         Same(triangle.vertex_keys[(edge + 1) % 3], vertex);
}

bool CoplanarIncidentEdgeHasPositiveOverlap(
    Vec3 other, const CurrentFixedTriangle& target,
    const FacetVertexKey& shared_vertex, int drop,
    bool* valid) noexcept {
  // The target is the intersection of three projected exact half-planes.
  // At its shared vertex, the nonincident half-plane is strict.  A ray from
  // that vertex therefore overlaps the target for positive length exactly
  // when it is on the interior side of both incident edge lines.  Orient2D
  // is exact dyadic arithmetic, so boundary rays and one-ULP departures have
  // deterministic classes without constructing or rounding an intersection.
  for (unsigned edge = 0; edge < 3; ++edge) {
    if (!EdgeContains(target, edge, shared_vertex))
      continue;
    const unsigned next = (edge + 1) % 3;
    const unsigned opposite = (edge + 2) % 3;
    const auto interior = exact::Orient2D(
        target.vertices[edge], target.vertices[next],
        target.vertices[opposite], drop);
    const auto side = exact::Orient2D(
        target.vertices[edge], target.vertices[next], other, drop);
    if (!Valid(interior) || !Valid(side) || !interior.value) {
      *valid = false;
      return false;
    }
    if (side.value * interior.value < 0) {
      *valid = true;
      return false;
    }
  }
  *valid = true;
  return true;
}

bool OnlySharedCoplanarVertex(
    const CurrentFixedTriangle& a, const CurrentFixedTriangle& b,
    const SharedTopology& shared, int drop, bool* valid) noexcept {
  const FacetVertexKey& vertex = a.vertex_keys[shared.vertex_a];
  for (unsigned i = 0; i < 3; ++i) {
    if (i != static_cast<unsigned>(shared.vertex_a) &&
        PointInTriangle(a.vertices[i], b, drop, valid))
      return false;
    if (!*valid)
      return false;
    if (i != static_cast<unsigned>(shared.vertex_b) &&
        PointInTriangle(b.vertices[i], a, drop, valid))
      return false;
    if (!*valid)
      return false;
  }
  for (unsigned i = 0; i < 3; ++i) {
    for (unsigned j = 0; j < 3; ++j) {
      const auto relation = IntersectSegments(
          a.vertices[i], a.vertices[(i + 1) % 3],
          b.vertices[j], b.vertices[(j + 1) % 3], drop, valid);
      if (!*valid)
        return false;
      if (relation == SegmentIntersection::Overlap)
        return false;
      if (relation == SegmentIntersection::Point &&
          (!EdgeContains(a, i, vertex) ||
           !EdgeContains(b, j, vertex)))
        return false;
    }
  }
  return true;
}

bool OnlySharedTransverseVertex(
    const CurrentFixedTriangle& a, const CurrentFixedTriangle& b,
    const SharedTopology& shared, int drop_a, int drop_b,
    bool* valid) noexcept {
  const FacetVertexKey& vertex = a.vertex_keys[shared.vertex_a];
  const CurrentFixedTriangle* source[2]{&a, &b};
  const CurrentFixedTriangle* target[2]{&b, &a};
  const int drops[2]{drop_b, drop_a};
  for (unsigned owner = 0; owner < 2; ++owner) {
    for (unsigned edge = 0; edge < 3; ++edge) {
      if (!SegmentTriangleIntersects(
              source[owner]->vertices[edge],
              source[owner]->vertices[(edge + 1) % 3],
              *target[owner], drops[owner], valid)) {
        if (!*valid)
          return false;
        continue;
      }
      if (!EdgeContains(*source[owner], edge, vertex))
        return false;
      const unsigned other =
          Same(source[owner]->vertex_keys[edge], vertex)
              ? (edge + 1) % 3
              : edge;
      const auto side = exact::Orient3D(
          target[owner]->vertices[0], target[owner]->vertices[1],
          target[owner]->vertices[2],
          source[owner]->vertices[other]);
      if (!Valid(side)) {
        *valid = false;
        return false;
      }
      if (!side.value &&
          CoplanarIncidentEdgeHasPositiveOverlap(
              source[owner]->vertices[other], *target[owner],
              vertex, drops[owner], valid))
        return false;
      if (!*valid)
        return false;
    }
  }
  return true;
}

FixedTriangleDiscoveryStatus ClassifyIntersection(
    const CurrentFixedTriangle& a, const CurrentFixedTriangle& b,
    FixedTriangleIntersection* output, bool* intersects) noexcept {
  int sides[3];
  bool coplanar = true;
  for (unsigned i = 0; i < 3; ++i) {
    const auto side = exact::Orient3D(
        a.vertices[0], a.vertices[1], a.vertices[2], b.vertices[i]);
    if (!Valid(side))
      return FixedTriangleDiscoveryStatus::NonFiniteResult;
    sides[i] = side.value;
    coplanar = coplanar && !side.value;
  }
  bool valid = false;
  const int drop_a = ProjectionAxis(a, &valid);
  if (!valid)
    return FixedTriangleDiscoveryStatus::DegenerateTriangle;
  bool positive_area = false;
  if (coplanar) {
    const auto classification = ClassifyCoplanar(a, b, drop_a);
    if (!classification.valid)
      return FixedTriangleDiscoveryStatus::NonFiniteResult;
    if (!classification.intersects) {
      *intersects = false;
      return FixedTriangleDiscoveryStatus::Ok;
    }
    positive_area = classification.positive_area;
    output->kind = positive_area
                       ? FixedTriangleIntersectionKind::CoplanarOverlap
                       : FixedTriangleIntersectionKind::CoplanarTouch;
  } else {
    bool all_positive = true;
    bool all_negative = true;
    for (int side : sides) {
      all_positive = all_positive && side > 0;
      all_negative = all_negative && side < 0;
    }
    if (all_positive || all_negative) {
      *intersects = false;
      return FixedTriangleDiscoveryStatus::Ok;
    }
    const int drop_b = ProjectionAxis(b, &valid);
    if (!valid)
      return FixedTriangleDiscoveryStatus::DegenerateTriangle;
    bool found = false;
    for (unsigned i = 0; i < 3 && !found; ++i)
      found = SegmentTriangleIntersects(
          a.vertices[i], a.vertices[(i + 1) % 3], b, drop_b, &valid);
    for (unsigned i = 0; i < 3 && !found; ++i)
      found = SegmentTriangleIntersects(
          b.vertices[i], b.vertices[(i + 1) % 3], a, drop_a, &valid);
    if (!valid)
      return FixedTriangleDiscoveryStatus::NonFiniteResult;
    if (!found) {
      *intersects = false;
      return FixedTriangleDiscoveryStatus::Ok;
    }
    output->kind = FixedTriangleIntersectionKind::Transverse;
  }

  output->local_exclusion = FixedTriangleLocalExclusion::None;
  if (Same(a.key, b.key)) {
    output->local_exclusion =
        FixedTriangleLocalExclusion::IdenticalFace;
  } else if (!positive_area) {
    const auto shared = FindSharedTopology(a, b);
    if (shared.edge_a >= 0) {
      output->local_exclusion =
          FixedTriangleLocalExclusion::SharedEdgeOnly;
    } else if (shared.vertex_a >= 0) {
      bool only = false;
      if (coplanar) {
        only = OnlySharedCoplanarVertex(
            a, b, shared, drop_a, &valid);
      } else {
        const int drop_b = ProjectionAxis(b, &valid);
        if (valid)
          only = OnlySharedTransverseVertex(
              a, b, shared, drop_a, drop_b, &valid);
      }
      if (!valid)
        return FixedTriangleDiscoveryStatus::NonFiniteResult;
      if (only)
        output->local_exclusion =
            FixedTriangleLocalExclusion::SharedVertexOnly;
    }
  }
  *intersects = true;
  return FixedTriangleDiscoveryStatus::Ok;
}

FixedTriangleDiscoveryStatus AddVertexFace(
    const CurrentFixedTriangle& vertex_triangle, unsigned vertex,
    const CurrentFixedTriangle& face_triangle, bool vertex_is_first,
    const CurrentFixedTriangle& canonical_first,
    const CurrentFixedTriangle& canonical_second,
    FixedTriangleFeatureCandidate* output, std::size_t capacity,
    PairFeatureResult* result) noexcept {
  const auto canonical_face = CanonicalGeometry(face_triangle);
  TrianglePointGeometry closest;
  const auto status = ClosestPointOnTriangle(
      vertex_triangle.vertices[vertex], canonical_face.geometry, &closest);
  if (status != Status::kOk) {
    result->arithmetic_reason =
        FixedTriangleArithmeticReason::ClosestPoint;
    return status == Status::kNonFiniteResult
               ? FixedTriangleDiscoveryStatus::NonFiniteResult
               : FixedTriangleDiscoveryStatus::InvalidInput;
  }
  exact::ClosestTriangleStratum stratum;
  if (!exact::ClosestStratum(vertex_triangle.vertices[vertex],
                             canonical_face.vertices, &stratum)) {
    result->arithmetic_reason =
        FixedTriangleArithmeticReason::ExactClosestStratum;
    return FixedTriangleDiscoveryStatus::NonFiniteResult;
  }
  if (VertexInTriangleTopology(vertex_triangle.vertex_keys[vertex],
                               face_triangle))
    return FixedTriangleDiscoveryStatus::Ok;
  if (result->feature_count >= capacity)
    return FixedTriangleDiscoveryStatus::ResourceLimit;
  FixedTriangleFeatureCandidate candidate;
  std::memset(&candidate, 0, sizeof(candidate));
  candidate.face_weights[0] = 1;
  candidate.key.kind = FixedTriangleCandidateKind::VertexFace;
  candidate.key.vertex_face.vertex =
      vertex_triangle.vertex_keys[vertex];
  candidate.triangles[0] = canonical_first.key;
  candidate.triangles[1] = canonical_second.key;
  candidate.local_features[vertex_is_first ? 0 : 1] = vertex;
  candidate.local_features[vertex_is_first ? 1 : 0] = 3;
  candidate.points[0] = vertex_triangle.vertices[vertex];
  candidate.points[1] = closest.point;
  candidate.distance_m = closest.distance;
  for (unsigned i = 0; i < 3; ++i)
    candidate.face_weights[canonical_face.original_local[i]] =
        closest.weights[i];

  if (stratum.kind == exact::ClosestStratumKind::Vertex) {
    const unsigned target_vertex =
        canonical_face.original_local[stratum.local];
    candidate.key.vertex_face.target.SetVertex(
        face_triangle.vertex_keys[target_vertex]);
    candidate.points[1] = face_triangle.vertices[target_vertex];
    for (double& weight : candidate.face_weights)
      weight = 0;
    candidate.face_weights[target_vertex] = 1;
  } else if (stratum.kind == exact::ClosestStratumKind::Edge) {
    const unsigned canonical_next = (stratum.local + 1) % 3;
    const unsigned edge = EdgeWithEndpoints(
        face_triangle,
        face_triangle.vertex_keys[
            canonical_face.original_local[stratum.local]],
        face_triangle.vertex_keys[
            canonical_face.original_local[canonical_next]]);
    if (edge >= 3)
      return FixedTriangleDiscoveryStatus::IdentityMismatch;
    const auto edge_geometry =
        CanonicalEdgeGeometry(face_triangle, edge);
    SegmentPointGeometry on_edge;
    const auto edge_status = ClosestPointOnSegment(
        vertex_triangle.vertices[vertex], edge_geometry, &on_edge);
    if (edge_status != Status::kOk) {
      result->arithmetic_reason =
          FixedTriangleArithmeticReason::EdgeClosestPoint;
      return edge_status == Status::kNonFiniteResult
                 ? FixedTriangleDiscoveryStatus::NonFiniteResult
                 : FixedTriangleDiscoveryStatus::InvalidInput;
    }
    const double complement = 1 - on_edge.parameter;
    if (!(on_edge.parameter > 0 && on_edge.parameter < 1 &&
          complement > 0 && complement < 1) ||
        !Same(on_edge.point,
              geometry_detail::Blend(edge_geometry.vertices[0],
                                     edge_geometry.vertices[1],
                                     on_edge.parameter))) {
      result->arithmetic_reason =
          FixedTriangleArithmeticReason::EdgeInteriorRepresentation;
      return FixedTriangleDiscoveryStatus::NonFiniteResult;
    }
    candidate.key.vertex_face.target.SetEdge(
        face_triangle.edge_keys[edge]);
    candidate.points[1] = on_edge.point;
    candidate.distance_m = on_edge.distance;
    candidate.edge_parameters[0] = on_edge.parameter;
    for (double& weight : candidate.face_weights)
      weight = 0;
    for (unsigned i = 0; i < 3; ++i) {
      if (Same(face_triangle.vertex_keys[i],
               face_triangle.edge_keys[edge].endpoints[0]))
        candidate.face_weights[i] = complement;
      if (Same(face_triangle.vertex_keys[i],
               face_triangle.edge_keys[edge].endpoints[1]))
        candidate.face_weights[i] = on_edge.parameter;
    }
  } else {
    double represented_weights[3];
    if (!RepresentedFaceWeights(
            closest.weights, represented_weights)) {
      result->arithmetic_reason =
          FixedTriangleArithmeticReason::FaceInteriorRepresentation;
      return FixedTriangleDiscoveryStatus::NonFiniteResult;
    }
    const Vec3 represented = Add(
        Add(Scale(canonical_face.vertices[0], represented_weights[0]),
            Scale(canonical_face.vertices[1], represented_weights[1])),
        Scale(canonical_face.vertices[2], represented_weights[2]));
    candidate.representation_error_m =
        RepresentationError(closest.point, represented);
    candidate.points[1] = represented;
    candidate.distance_m = geometry_detail::Length(
        Subtract(candidate.points[0], represented));
    for (unsigned i = 0; i < 3; ++i)
      candidate.face_weights[canonical_face.original_local[i]] =
          represented_weights[i];
    if (!IsFinite(represented) || !IsFinite(candidate.distance_m) ||
        !IsFinite(candidate.representation_error_m)) {
      result->arithmetic_reason =
          FixedTriangleArithmeticReason::FaceInteriorRepresentation;
      return FixedTriangleDiscoveryStatus::NonFiniteResult;
    }
    candidate.key.vertex_face.target.SetFace(face_triangle.key);
  }
  output[result->feature_count++] = candidate;
  return FixedTriangleDiscoveryStatus::Ok;
}

FixedTriangleDiscoveryStatus AddEdgeEdge(
    const CurrentFixedTriangle& a, unsigned edge_a,
    const CurrentFixedTriangle& b, unsigned edge_b,
    FixedTriangleFeatureCandidate* output, std::size_t capacity,
    PairFeatureResult* result) noexcept {
  const bool a_first =
      Compare(a.edge_keys[edge_a], b.edge_keys[edge_b]) <= 0;
  const auto first_geometry = a_first
                                  ? CanonicalEdgeGeometry(a, edge_a)
                                  : CanonicalEdgeGeometry(b, edge_b);
  const auto second_geometry = a_first
                                   ? CanonicalEdgeGeometry(b, edge_b)
                                   : CanonicalEdgeGeometry(a, edge_a);
  SegmentPairGeometry closest;
  const auto status = ClosestPointsBetweenSegments(
      first_geometry, second_geometry, &closest);
  if (status != Status::kOk) {
    result->arithmetic_reason =
        FixedTriangleArithmeticReason::SegmentClosestPoints;
    return status == Status::kNonFiniteResult
               ? FixedTriangleDiscoveryStatus::NonFiniteResult
               : FixedTriangleDiscoveryStatus::InvalidInput;
  }
  if (EdgesIncident(a.edge_keys[edge_a], b.edge_keys[edge_b]))
    return FixedTriangleDiscoveryStatus::Ok;
  if (result->feature_count >= capacity)
    return FixedTriangleDiscoveryStatus::ResourceLimit;
  FixedTriangleFeatureCandidate candidate;
  std::memset(&candidate, 0, sizeof(candidate));
  candidate.face_weights[0] = 1;
  candidate.key.SetEdgeEdge();
  candidate.key.edge_edge.edges[0] =
      a_first ? a.edge_keys[edge_a] : b.edge_keys[edge_b];
  candidate.key.edge_edge.edges[1] =
      a_first ? b.edge_keys[edge_b] : a.edge_keys[edge_a];
  candidate.triangles[0] = a.key;
  candidate.triangles[1] = b.key;
  candidate.local_features[0] = edge_a;
  candidate.local_features[1] = edge_b;
  candidate.points[0] = closest.point_a;
  candidate.points[1] = closest.point_b;
  candidate.edge_parameters[0] = closest.parameter_a;
  candidate.edge_parameters[1] = closest.parameter_b;
  candidate.distance_m = closest.distance;
  output[result->feature_count++] = candidate;
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

int Compare(const FixedTriangleStratumKey& a,
            const FixedTriangleStratumKey& b) noexcept {
  int value = ScalarCompare(a.kind, b.kind);
  if (!value && a.kind == FixedTriangleStratumKind::Vertex)
    value = Compare(a.vertex, b.vertex);
  if (!value && a.kind == FixedTriangleStratumKind::Edge)
    value = Compare(a.edge, b.edge);
  if (!value && a.kind == FixedTriangleStratumKind::Face)
    value = Compare(a.face, b.face);
  return value;
}

int Compare(const FixedTriangleFeatureKey& a,
            const FixedTriangleFeatureKey& b) noexcept {
  int value = ScalarCompare(a.kind, b.kind);
  if (!value && a.kind == FixedTriangleCandidateKind::VertexFace)
    value = Compare(a.vertex_face.vertex, b.vertex_face.vertex);
  if (!value && a.kind == FixedTriangleCandidateKind::VertexFace)
    value = Compare(a.vertex_face.target, b.vertex_face.target);
  if (!value && a.kind == FixedTriangleCandidateKind::EdgeEdge)
    value = Compare(a.edge_edge.edges[0], b.edge_edge.edges[0]);
  if (!value && a.kind == FixedTriangleCandidateKind::EdgeEdge)
    value = Compare(a.edge_edge.edges[1], b.edge_edge.edges[1]);
  return value;
}

bool ProducerLess(const FixedTriangleFeatureCandidate& a,
                  const FixedTriangleFeatureCandidate& b) noexcept {
  int value = Compare(a.triangles[0], b.triangles[0]);
  if (!value)
    value = Compare(a.triangles[1], b.triangles[1]);
  if (!value)
    value = ScalarCompare(a.local_features[0], b.local_features[0]);
  if (!value)
    value = ScalarCompare(a.local_features[1], b.local_features[1]);
  return value < 0;
}

bool FeatureLess(const FixedTriangleFeatureCandidate& a,
                 const FixedTriangleFeatureCandidate& b) noexcept {
  const int value = Compare(a.key, b.key);
  return value ? value < 0 : ProducerLess(a, b);
}

bool SameFeatureKey(const FixedTriangleFeatureCandidate& a,
                    const FixedTriangleFeatureCandidate& b) noexcept {
  return Compare(a.key, b.key) == 0;
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

bool SameTriangleValue(const CurrentFixedTriangle& a,
                       const CurrentFixedTriangle& b) noexcept {
  return SameTriangleValueImpl(a, b);
}

FixedTriangleDiscoveryStatus ValidateTriangle(
    const CurrentFixedTriangle& value) noexcept {
  return ValidateTriangleImpl(value);
}

std::size_t CountPairFeatureCandidates(
    const CurrentFixedTriangle& a,
    const CurrentFixedTriangle& b) noexcept {
  std::size_t result = 0;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    result += !VertexInTriangleTopology(a.vertex_keys[vertex], b);
    result += !VertexInTriangleTopology(b.vertex_keys[vertex], a);
  }
  for (unsigned edge_a = 0; edge_a < 3; ++edge_a)
    for (unsigned edge_b = 0; edge_b < 3; ++edge_b)
      result += !EdgesIncident(a.edge_keys[edge_a], b.edge_keys[edge_b]);
  return result;
}

FixedTriangleFeatureTaskMask PairLocalFeatureTaskMask(
    const CurrentFixedTriangle& input_a,
    const CurrentFixedTriangle& input_b) noexcept {
  const CurrentFixedTriangle* a = &input_a;
  const CurrentFixedTriangle* b = &input_b;
  if (Compare(b->key, a->key) < 0)
    std::swap(a, b);
  FixedTriangleFeatureTaskMask result;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    if (VertexInTriangleTopology(a->vertex_keys[vertex], *b))
      result.local_tasks |= FixedTriangleFeatureTaskBit(
          FixedTriangleVertexFaceTaskSlot(0, vertex));
    if (VertexInTriangleTopology(b->vertex_keys[vertex], *a))
      result.local_tasks |= FixedTriangleFeatureTaskBit(
          FixedTriangleVertexFaceTaskSlot(1, vertex));
  }
  for (unsigned edge_a = 0; edge_a < 3; ++edge_a)
    for (unsigned edge_b = 0; edge_b < 3; ++edge_b)
      if (EdgesIncident(a->edge_keys[edge_a], b->edge_keys[edge_b]))
        result.local_tasks |= FixedTriangleFeatureTaskBit(
            FixedTriangleEdgeEdgeTaskSlot(edge_a, edge_b));
  return result;
}

}  // namespace fixed_triangle_features

FixedTriangleDiscoveryStatus BuildFixedTriangleFeatureTaskMask(
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second,
    FixedTriangleFeatureTaskMask* output) noexcept {
  if (!output)
    return FixedTriangleDiscoveryStatus::InvalidInput;
  if (fixed_triangle_features::Compare(first.key, second.key) == 0)
    return FixedTriangleDiscoveryStatus::IdentityMismatch;
  const auto next =
      fixed_triangle_features::PairLocalFeatureTaskMask(first, second);
  *output = next;
  return FixedTriangleDiscoveryStatus::Ok;
}

namespace fixed_triangle_features {

FixedTriangleDiscoveryStatus EvaluatePairFeaturesMaskedOnce(
    const CurrentFixedTriangle& input_a,
    const CurrentFixedTriangle& input_b,
    FixedTriangleFeatureTaskMask mask,
    FixedTriangleFeatureCandidate* output, std::size_t output_capacity,
    PairFeatureResult* result) noexcept {
  if (!result || (output_capacity && !output) ||
      (mask.local_tasks & ~FixedTriangleFeatureTaskBits))
    return FixedTriangleDiscoveryStatus::InvalidInput;
  *result = {};
  const CurrentFixedTriangle* a = &input_a;
  const CurrentFixedTriangle* b = &input_b;
  if (Compare(b->key, a->key) < 0)
    std::swap(a, b);
  FixedTriangleDiscoveryStatus status =
      FixedTriangleDiscoveryStatus::Ok;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    const unsigned first_task =
        FixedTriangleVertexFaceTaskSlot(0, vertex);
    if (!(mask.local_tasks &
          FixedTriangleFeatureTaskBit(first_task))) {
      result->input_task = first_task;
      ++result->feature_tasks;
      status = AddVertexFace(*a, vertex, *b, true, *a, *b, output,
                             output_capacity, result);
      if (status != FixedTriangleDiscoveryStatus::Ok)
        return status;
    }
    const unsigned second_task =
        FixedTriangleVertexFaceTaskSlot(1, vertex);
    if (!(mask.local_tasks &
          FixedTriangleFeatureTaskBit(second_task))) {
      result->input_task = second_task;
      ++result->feature_tasks;
      status = AddVertexFace(*b, vertex, *a, false, *a, *b, output,
                             output_capacity, result);
      if (status != FixedTriangleDiscoveryStatus::Ok)
        return status;
    }
  }
  for (unsigned edge_a = 0; edge_a < 3; ++edge_a) {
    for (unsigned edge_b = 0; edge_b < 3; ++edge_b) {
      const unsigned task =
          FixedTriangleEdgeEdgeTaskSlot(edge_a, edge_b);
      if (mask.local_tasks & FixedTriangleFeatureTaskBit(task))
        continue;
      result->input_task = task;
      ++result->feature_tasks;
      status = AddEdgeEdge(*a, edge_a, *b, edge_b, output,
                           output_capacity, result);
      if (status != FixedTriangleDiscoveryStatus::Ok)
        return status;
    }
  }
  result->input_task = SIZE_MAX;
  return FixedTriangleDiscoveryStatus::Ok;
}

FixedTriangleDiscoveryStatus EvaluatePairFeaturesOnce(
    const CurrentFixedTriangle& input_a,
    const CurrentFixedTriangle& input_b,
    FixedTriangleFeatureCandidate* output, std::size_t output_capacity,
    PairFeatureResult* result) noexcept {
  return EvaluatePairFeaturesMaskedOnce(
      input_a, input_b, {}, output, output_capacity, result);
}

FixedTriangleDiscoveryStatus ClassifyPairIntersection(
    const CurrentFixedTriangle& input_a,
    const CurrentFixedTriangle& input_b,
    FixedTriangleIntersection* output, bool* intersects) noexcept {
  if (!output || !intersects)
    return FixedTriangleDiscoveryStatus::InvalidInput;
  *output = {};
  *intersects = false;
  const CurrentFixedTriangle* a = &input_a;
  const CurrentFixedTriangle* b = &input_b;
  if (Compare(b->key, a->key) < 0)
    std::swap(a, b);
  output->triangles[0] = a->key;
  output->triangles[1] = b->key;
  return ClassifyIntersection(*a, *b, output, intersects);
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
