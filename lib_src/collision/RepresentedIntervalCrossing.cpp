// SPDX-License-Identifier: AGPL-3.0-or-later
#include "RepresentedIntervalCrossing.h"

#include <algorithm>
#include <boost/multiprecision/cpp_int.hpp>
#include <cstring>
#include <limits>
#include <new>
#include <tuple>
#include <utility>
#include <vector>

namespace tlfea::contact {
namespace {

template <class T>
int ScalarCompare(const T& a, const T& b) noexcept {
  return a < b ? -1 : (b < a ? 1 : 0);
}

int Compare(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  const auto aa =
      std::tie(a.source_instance_id, a.kind, a.first, a.second, a.numerator,
               a.denominator, a.level, a.grid_i, a.grid_j);
  const auto bb =
      std::tie(b.source_instance_id, b.kind, b.first, b.second, b.numerator,
               b.denominator, b.level, b.grid_i, b.grid_j);
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

int Compare(const RepresentedTrianglePathKey& a,
            const RepresentedTrianglePathKey& b) noexcept {
  const auto aa =
      std::tie(a.source_instance_id, a.parent_eid, a.level, a.local_facet);
  const auto bb =
      std::tie(b.source_instance_id, b.parent_eid, b.level, b.local_facet);
  return aa < bb ? -1 : (bb < aa ? 1 : 0);
}

int Compare(const RepresentedIntervalPairKey& a,
            const RepresentedIntervalPairKey& b) noexcept {
  const int first = Compare(a.paths[0], b.paths[0]);
  return first ? first : Compare(a.paths[1], b.paths[1]);
}

bool Same(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  return Compare(a, b) == 0;
}

bool Same(const RepresentedTrianglePathKey& a,
          const RepresentedTrianglePathKey& b) noexcept {
  return Compare(a, b) == 0;
}

bool Same(Vec3 a, Vec3 b) noexcept {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool AddSize(std::size_t a, std::size_t b, std::size_t* output) noexcept {
  if (a > SIZE_MAX - b)
    return false;
  *output = a + b;
  return true;
}

bool MultiplySize(std::size_t a, std::size_t b,
                  std::size_t* output) noexcept {
  if (a && b > SIZE_MAX / a)
    return false;
  *output = a * b;
  return true;
}

bool RangeDisjoint(const void* a, std::size_t a_bytes, const void* b,
                   std::size_t b_bytes) noexcept {
  if (!a_bytes || !b_bytes)
    return true;
  if (!a || !b)
    return false;
  const auto aa = reinterpret_cast<std::uintptr_t>(a);
  const auto bb = reinterpret_cast<std::uintptr_t>(b);
  if (aa > UINTPTR_MAX - a_bytes || bb > UINTPTR_MAX - b_bytes)
    return false;
  return aa + a_bytes <= bb || bb + b_bytes <= aa;
}

struct CanonicalPair {
  std::uint32_t first = 0;
  std::uint32_t second = 0;
  std::size_t input_pair = SIZE_MAX;
  RepresentedIntervalPairKey key;
};

bool PairLess(const CanonicalPair& a, const CanonicalPair& b) noexcept {
  return Compare(a.key, b.key) < 0;
}

bool SamePair(const CanonicalPair& a, const CanonicalPair& b) noexcept {
  return Compare(a.key, b.key) == 0;
}

using ExactBackend = boost::multiprecision::cpp_int_backend<
    16384, 16384, boost::multiprecision::signed_magnitude,
    boost::multiprecision::checked, void>;
using ExactInteger =
    boost::multiprecision::number<ExactBackend,
                                  boost::multiprecision::et_off>;

struct Dyadic {
  ExactInteger numerator = 0;
  int exponent = 0;
};

Dyadic Exact(double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "binary64 representation");
  std::memcpy(&bits, &value, sizeof(bits));
  const bool negative = (bits >> 63) != 0;
  const unsigned encoded_exponent =
      static_cast<unsigned>((bits >> 52) & 0x7ffu);
  const std::uint64_t fraction = bits & ((std::uint64_t{1} << 52) - 1);
  Dyadic result;
  if (encoded_exponent == 0) {
    result.numerator = fraction;
    result.exponent = -1074;
  } else {
    result.numerator = (std::uint64_t{1} << 52) | fraction;
    result.exponent = static_cast<int>(encoded_exponent) - 1023 - 52;
  }
  if (negative)
    result.numerator = -result.numerator;
  return result;
}

Dyadic Add(Dyadic a, Dyadic b) {
  if (a.numerator == 0)
    return b;
  if (b.numerator == 0)
    return a;
  const int exponent = std::min(a.exponent, b.exponent);
  const auto shift = [](ExactInteger* value, unsigned amount) {
    const bool negative = *value < 0;
    if (negative)
      *value = -*value;
    *value <<= amount;
    if (negative)
      *value = -*value;
  };
  shift(&a.numerator, static_cast<unsigned>(a.exponent - exponent));
  shift(&b.numerator, static_cast<unsigned>(b.exponent - exponent));
  return {a.numerator + b.numerator, exponent};
}

Dyadic Negate(Dyadic value) {
  value.numerator = -value.numerator;
  return value;
}

Dyadic Subtract(Dyadic a, Dyadic b) { return Add(a, Negate(b)); }

Dyadic Multiply(const Dyadic& a, const Dyadic& b) {
  return {a.numerator * b.numerator, a.exponent + b.exponent};
}

Dyadic Scale(Dyadic value, std::uint64_t factor) {
  value.numerator *= factor;
  return value;
}

int Sign(const Dyadic& value) noexcept {
  return value.numerator < 0 ? -1 : (value.numerator > 0 ? 1 : 0);
}

int Compare(const Dyadic& a, const Dyadic& b) {
  return Sign(Subtract(a, b));
}

struct ExactVec3 {
  Dyadic x, y, z;
};

ExactVec3 Add(ExactVec3 a, ExactVec3 b) {
  return {Add(a.x, b.x), Add(a.y, b.y), Add(a.z, b.z)};
}

ExactVec3 Subtract(ExactVec3 a, ExactVec3 b) {
  return {Subtract(a.x, b.x), Subtract(a.y, b.y),
          Subtract(a.z, b.z)};
}

ExactVec3 Cross(ExactVec3 a, ExactVec3 b) {
  return {Subtract(Multiply(a.y, b.z), Multiply(a.z, b.y)),
          Subtract(Multiply(a.z, b.x), Multiply(a.x, b.z)),
          Subtract(Multiply(a.x, b.y), Multiply(a.y, b.x))};
}

Dyadic Dot(ExactVec3 a, ExactVec3 b) {
  return Add(Add(Multiply(a.x, b.x), Multiply(a.y, b.y)),
             Multiply(a.z, b.z));
}

bool Zero(ExactVec3 value) noexcept {
  return Sign(value.x) == 0 && Sign(value.y) == 0 && Sign(value.z) == 0;
}

Dyadic Component(ExactVec3 value, unsigned component) {
  return component == 0 ? value.x : (component == 1 ? value.y : value.z);
}

struct DyadicTime {
  std::uint64_t numerator = 0;
  unsigned depth = 0;
};

struct Cell {
  std::uint64_t lower = 0;
  std::uint64_t upper = 1;
  unsigned depth = 0;
};

DyadicTime Lower(const Cell& cell) { return {cell.lower, cell.depth}; }
DyadicTime Upper(const Cell& cell) { return {cell.upper, cell.depth}; }
DyadicTime Middle(const Cell& cell) {
  return {cell.lower + cell.upper, cell.depth + 1};
}

double Component(Vec3 value, unsigned component) noexcept {
  return component == 0 ? value.x : (component == 1 ? value.y : value.z);
}

ExactVec3 At(const RepresentedVertexPath& path, DyadicTime time) {
  const std::uint64_t denominator = std::uint64_t{1} << time.depth;
  ExactVec3 result;
  Dyadic* target[3] = {&result.x, &result.y, &result.z};
  for (unsigned component = 0; component < 3; ++component) {
    const Dyadic a = Scale(Exact(Component(path.endpoint[0], component)),
                           denominator - time.numerator);
    const Dyadic b =
        Scale(Exact(Component(path.endpoint[1], component)),
              time.numerator);
    *target[component] = Add(a, b);
    target[component]->exponent -= static_cast<int>(time.depth);
  }
  return result;
}

struct ExactTriangle {
  ExactVec3 vertex[3];
};

ExactTriangle At(const RepresentedTrianglePath& path, DyadicTime time) {
  ExactTriangle result;
  for (unsigned i = 0; i < 3; ++i)
    result.vertex[i] = At(path.vertices[i], time);
  return result;
}

ExactVec3 Edge(const ExactTriangle& triangle, unsigned edge) {
  return Subtract(triangle.vertex[(edge + 1) % 3],
                  triangle.vertex[edge]);
}

ExactVec3 Normal(const ExactTriangle& triangle) {
  return Cross(Edge(triangle, 0),
               Subtract(triangle.vertex[2], triangle.vertex[0]));
}

bool Degenerate(const ExactTriangle& triangle) {
  return Zero(Normal(triangle));
}

bool SeparatedOnAxis(const ExactTriangle& a, const ExactTriangle& b,
                     ExactVec3 axis) {
  if (Zero(axis))
    return false;
  Dyadic minimum_a = Dot(a.vertex[0], axis);
  Dyadic maximum_a = minimum_a;
  Dyadic minimum_b = Dot(b.vertex[0], axis);
  Dyadic maximum_b = minimum_b;
  for (unsigned i = 1; i < 3; ++i) {
    const Dyadic pa = Dot(a.vertex[i], axis);
    const Dyadic pb = Dot(b.vertex[i], axis);
    if (Compare(pa, minimum_a) < 0)
      minimum_a = pa;
    if (Compare(pa, maximum_a) > 0)
      maximum_a = pa;
    if (Compare(pb, minimum_b) < 0)
      minimum_b = pb;
    if (Compare(pb, maximum_b) > 0)
      maximum_b = pb;
  }
  return Compare(maximum_a, minimum_b) < 0 ||
         Compare(maximum_b, minimum_a) < 0;
}

struct StaticIntersection {
  bool intersects = false;
  bool coplanar = false;
};

StaticIntersection Intersects(const ExactTriangle& a,
                              const ExactTriangle& b) {
  const ExactVec3 normal_a = Normal(a);
  const ExactVec3 normal_b = Normal(b);
  StaticIntersection result;
  result.coplanar = true;
  for (unsigned i = 0; i < 3; ++i) {
    result.coplanar =
        result.coplanar &&
        Sign(Dot(Subtract(b.vertex[i], a.vertex[0]), normal_a)) == 0 &&
        Sign(Dot(Subtract(a.vertex[i], b.vertex[0]), normal_b)) == 0;
  }
  if (SeparatedOnAxis(a, b, normal_a) ||
      SeparatedOnAxis(a, b, normal_b))
    return result;
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      if (SeparatedOnAxis(a, b, Cross(Edge(a, i), Edge(b, j))))
        return result;
  if (result.coplanar) {
    for (unsigned i = 0; i < 3; ++i) {
      if (SeparatedOnAxis(a, b, Cross(normal_a, Edge(a, i))) ||
          SeparatedOnAxis(a, b, Cross(normal_a, Edge(b, i))))
        return result;
    }
  }
  result.intersects = true;
  return result;
}

bool PointInClosedTriangle(ExactVec3 point,
                           const ExactTriangle& triangle) {
  const ExactVec3 normal = Normal(triangle);
  if (Sign(Dot(Subtract(point, triangle.vertex[0]), normal)) != 0)
    return false;
  int orientation = 0;
  for (unsigned i = 0; i < 3; ++i) {
    const int sign =
        Sign(Dot(Cross(Edge(triangle, i),
                       Subtract(point, triangle.vertex[i])),
                 normal));
    if (sign) {
      if (orientation && sign != orientation)
        return false;
      orientation = sign;
    }
  }
  return true;
}

bool SegmentsIntersect(ExactVec3 a0, ExactVec3 a1, ExactVec3 b0,
                       ExactVec3 b1) {
  const ExactVec3 a = Subtract(a1, a0);
  const ExactVec3 b = Subtract(b1, b0);
  const ExactVec3 delta = Subtract(b0, a0);
  const ExactVec3 normal = Cross(a, b);
  if (!Zero(normal)) {
    if (Sign(Dot(delta, normal)) != 0)
      return false;
    const Dyadic denominator = Dot(normal, normal);
    const Dyadic parameter_a = Dot(Cross(delta, b), normal);
    const Dyadic parameter_b = Dot(Cross(delta, a), normal);
    return Sign(parameter_a) >= 0 &&
           Compare(parameter_a, denominator) <= 0 &&
           Sign(parameter_b) >= 0 &&
           Compare(parameter_b, denominator) <= 0;
  }
  if (!Zero(Cross(a, delta)))
    return false;
  unsigned component = 0;
  for (unsigned i = 1; i < 3; ++i)
    if (Sign(Component(a, component)) == 0)
      component = i;
  Dyadic aa0 = Component(a0, component);
  Dyadic aa1 = Component(a1, component);
  Dyadic bb0 = Component(b0, component);
  Dyadic bb1 = Component(b1, component);
  if (Compare(aa1, aa0) < 0)
    std::swap(aa0, aa1);
  if (Compare(bb1, bb0) < 0)
    std::swap(bb0, bb1);
  return Compare(aa1, bb0) >= 0 && Compare(bb1, aa0) >= 0;
}

bool FeatureLess(const RepresentedFeaturePathKey& a,
                 const RepresentedFeaturePathKey& b) noexcept {
  int value = ScalarCompare(a.kind, b.kind);
  if (!value && a.kind == RepresentedFeatureKind::VertexFace)
    value = Compare(a.vertex, b.vertex);
  if (!value && a.kind == RepresentedFeatureKind::VertexFace)
    value = Compare(a.face, b.face);
  if (!value && a.kind == RepresentedFeatureKind::EdgeEdge)
    value = Compare(a.edges[0], b.edges[0]);
  if (!value && a.kind == RepresentedFeatureKind::EdgeEdge)
    value = Compare(a.edges[1], b.edges[1]);
  return value < 0;
}

void ConsiderFeature(const RepresentedFeaturePathKey& candidate,
                     bool* have, RepresentedFeaturePathKey* result) noexcept {
  if (!*have || FeatureLess(candidate, *result)) {
    *result = candidate;
    *have = true;
  }
}

RepresentedFeaturePathKey IntersectionFeature(
    const RepresentedTrianglePath& path_a,
    const RepresentedTrianglePath& path_b, const ExactTriangle& a,
    const ExactTriangle& b) {
  bool have = false;
  RepresentedFeaturePathKey result;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    if (PointInClosedTriangle(a.vertex[vertex], b)) {
      RepresentedFeaturePathKey candidate;
      candidate.kind = RepresentedFeatureKind::VertexFace;
      candidate.vertex = path_a.vertices[vertex].key;
      candidate.face = path_b.key;
      ConsiderFeature(candidate, &have, &result);
    }
    if (PointInClosedTriangle(b.vertex[vertex], a)) {
      RepresentedFeaturePathKey candidate;
      candidate.kind = RepresentedFeatureKind::VertexFace;
      candidate.vertex = path_b.vertices[vertex].key;
      candidate.face = path_a.key;
      ConsiderFeature(candidate, &have, &result);
    }
  }
  for (unsigned edge_a = 0; edge_a < 3; ++edge_a) {
    for (unsigned edge_b = 0; edge_b < 3; ++edge_b) {
      if (!SegmentsIntersect(a.vertex[edge_a],
                             a.vertex[(edge_a + 1) % 3],
                             b.vertex[edge_b],
                             b.vertex[(edge_b + 1) % 3]))
        continue;
      RepresentedFeaturePathKey candidate;
      candidate.kind = RepresentedFeatureKind::EdgeEdge;
      if (Compare(path_a.edge_keys[edge_a],
                  path_b.edge_keys[edge_b]) <= 0) {
        candidate.edges[0] = path_a.edge_keys[edge_a];
        candidate.edges[1] = path_b.edge_keys[edge_b];
      } else {
        candidate.edges[0] = path_b.edge_keys[edge_b];
        candidate.edges[1] = path_a.edge_keys[edge_a];
      }
      ConsiderFeature(candidate, &have, &result);
    }
  }
  if (!have)
    result.kind = RepresentedFeatureKind::TriangleIntersection;
  return result;
}

bool RegularCell(const ExactTriangle samples[3]) {
  ExactVec3 normal[3] = {Normal(samples[0]), Normal(samples[1]),
                         Normal(samples[2])};
  for (unsigned component = 0; component < 3; ++component) {
    const Dyadic first = Component(normal[0], component);
    const Dyadic middle = Component(normal[1], component);
    const Dyadic last = Component(normal[2], component);
    // Twice the middle Bernstein coefficient has the sign of
    // 4*n(mid)-n(lower)-n(upper).
    const Dyadic control =
        Subtract(Subtract(Scale(middle, 4), first), last);
    const int a = Sign(first);
    const int b = Sign(control);
    const int c = Sign(last);
    if (a && a == b && b == c)
      return true;
  }
  return false;
}

bool SweptBoxesSeparated(const ExactTriangle endpoint_a[2],
                         const ExactTriangle endpoint_b[2]) {
  for (unsigned component = 0; component < 3; ++component) {
    Dyadic minimum_a = Component(endpoint_a[0].vertex[0], component);
    Dyadic maximum_a = minimum_a;
    Dyadic minimum_b = Component(endpoint_b[0].vertex[0], component);
    Dyadic maximum_b = minimum_b;
    for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
      for (unsigned vertex = 0; vertex < 3; ++vertex) {
        const Dyadic a =
            Component(endpoint_a[endpoint].vertex[vertex], component);
        const Dyadic b =
            Component(endpoint_b[endpoint].vertex[vertex], component);
        if (Compare(a, minimum_a) < 0)
          minimum_a = a;
        if (Compare(a, maximum_a) > 0)
          maximum_a = a;
        if (Compare(b, minimum_b) < 0)
          minimum_b = b;
        if (Compare(b, maximum_b) > 0)
          maximum_b = b;
      }
    }
    if (Compare(maximum_a, minimum_b) < 0 ||
        Compare(maximum_b, minimum_a) < 0)
      return true;
  }
  return false;
}

int ReasonPriority(RepresentedIntervalReason reason) noexcept {
  switch (reason) {
    case RepresentedIntervalReason::ExactArithmeticRange:
      return 3;
    case RepresentedIntervalReason::DegenerateGeometry:
      return 2;
    case RepresentedIntervalReason::WorkExhausted:
      return 1;
    default:
      return 0;
  }
}

RepresentedIntervalResult Unresolved(const RepresentedIntervalPairKey& key,
                                     RepresentedIntervalReason reason,
                                     std::size_t work) noexcept {
  RepresentedIntervalResult result;
  result.key = key;
  result.classification = RepresentedIntervalClassification::Unresolved;
  result.reason = reason;
  result.work = work;
  return result;
}

struct PairContext {
  const RepresentedTrianglePath& a;
  const RepresentedTrianglePath& b;
  RepresentedIntervalLimits limits;
  RepresentedIntervalPairKey key;
  std::size_t work = 0;
};

RepresentedIntervalResult Visit(PairContext* context, Cell cell) {
  if (context->work >= context->limits.max_work_per_pair)
    return Unresolved(context->key,
                      RepresentedIntervalReason::WorkExhausted,
                      context->work);
  ++context->work;

  const DyadicTime times[3] = {Lower(cell), Middle(cell), Upper(cell)};
  ExactTriangle a[3], b[3];
  for (unsigned sample = 0; sample < 3; ++sample) {
    a[sample] = At(context->a, times[sample]);
    b[sample] = At(context->b, times[sample]);
    if (Degenerate(a[sample]) || Degenerate(b[sample]))
      return Unresolved(context->key,
                        RepresentedIntervalReason::DegenerateGeometry,
                        context->work);
  }
  for (unsigned sample = 0; sample < 3; ++sample) {
    const auto intersection = Intersects(a[sample], b[sample]);
    if (!intersection.intersects)
      continue;
    RepresentedIntervalResult result;
    result.key = context->key;
    result.feature =
        IntersectionFeature(context->a, context->b, a[sample], b[sample]);
    result.classification =
        RepresentedIntervalClassification::CertifiedCrossingContact;
    result.reason = RepresentedIntervalReason::None;
    result.geometry =
        intersection.coplanar ? RepresentedIntersectionGeometry::Coplanar
                              : RepresentedIntersectionGeometry::Transverse;
    result.witness_time_numerator = times[sample].numerator;
    result.witness_time_depth = times[sample].depth;
    result.work = context->work;
    return result;
  }

  const ExactTriangle regular_a[3] = {a[0], a[1], a[2]};
  const ExactTriangle regular_b[3] = {b[0], b[1], b[2]};
  if (RegularCell(regular_a) && RegularCell(regular_b)) {
    const ExactTriangle endpoints_a[2] = {a[0], a[2]};
    const ExactTriangle endpoints_b[2] = {b[0], b[2]};
    if (SweptBoxesSeparated(endpoints_a, endpoints_b)) {
      RepresentedIntervalResult result;
      result.key = context->key;
      result.classification =
          RepresentedIntervalClassification::CertifiedSeparated;
      result.reason = RepresentedIntervalReason::None;
      result.work = context->work;
      return result;
    }
  }

  if (cell.depth >= context->limits.max_depth)
    return Unresolved(context->key,
                      RepresentedIntervalReason::WorkExhausted,
                      context->work);
  const Cell left{cell.lower * 2, cell.lower + cell.upper,
                  cell.depth + 1};
  const Cell right{cell.lower + cell.upper, cell.upper * 2,
                   cell.depth + 1};
  auto first = Visit(context, left);
  if (first.classification ==
      RepresentedIntervalClassification::CertifiedCrossingContact) {
    first.work = context->work;
    return first;
  }
  auto second = Visit(context, right);
  if (second.classification ==
      RepresentedIntervalClassification::CertifiedCrossingContact) {
    second.work = context->work;
    return second;
  }
  if (first.classification ==
          RepresentedIntervalClassification::CertifiedSeparated &&
      second.classification ==
          RepresentedIntervalClassification::CertifiedSeparated) {
    first.work = context->work;
    return first;
  }
  const auto reason =
      ReasonPriority(first.reason) >= ReasonPriority(second.reason)
          ? first.reason
          : second.reason;
  return Unresolved(context->key, reason, context->work);
}

RepresentedIntervalResult CertifyPair(
    const RepresentedTrianglePath& a, const RepresentedTrianglePath& b,
    RepresentedIntervalLimits limits, RepresentedIntervalPairKey key) noexcept {
  if (a.motion != RepresentedMotion::LinearNodalV1 ||
      b.motion != RepresentedMotion::LinearNodalV1)
    return Unresolved(key, RepresentedIntervalReason::UnsupportedMotion, 0);
  PairContext context{a, b, limits, key};
  try {
    return Visit(&context, {});
  } catch (...) {
    return Unresolved(key, RepresentedIntervalReason::ExactArithmeticRange,
                      context.work);
  }
}

bool KnownMotion(RepresentedMotion motion) noexcept {
  return motion == RepresentedMotion::LinearNodalV1 ||
         motion == RepresentedMotion::RigidArc ||
         motion == RepresentedMotion::Nonlinear;
}

RepresentedIntervalStatus ValidatePath(
    const RepresentedTrianglePath& path) noexcept {
  if (!KnownMotion(path.motion))
    return RepresentedIntervalStatus::InvalidInput;
  for (unsigned i = 0; i < 3; ++i) {
    if (!IsFinite(path.vertices[i].endpoint[0]) ||
        !IsFinite(path.vertices[i].endpoint[1]))
      return RepresentedIntervalStatus::InvalidInput;
    for (unsigned j = 0; j < i; ++j)
      if (Same(path.vertices[i].key, path.vertices[j].key))
        return RepresentedIntervalStatus::InvalidInput;
    const unsigned next = (i + 1) % 3;
    const bool edge_matches =
        (Same(path.edge_keys[i].endpoints[0], path.vertices[i].key) &&
         Same(path.edge_keys[i].endpoints[1],
              path.vertices[next].key)) ||
        (Same(path.edge_keys[i].endpoints[1], path.vertices[i].key) &&
         Same(path.edge_keys[i].endpoints[0],
              path.vertices[next].key));
    if (!edge_matches)
      return RepresentedIntervalStatus::InvalidInput;
  }
  return RepresentedIntervalStatus::Ok;
}

const char* Message(RepresentedIntervalStatus status) noexcept {
  switch (status) {
    case RepresentedIntervalStatus::Ok:
      return "OK";
    case RepresentedIntervalStatus::AlreadyInitialized:
      return "already initialized";
    case RepresentedIntervalStatus::NotInitialized:
      return "not initialized";
    case RepresentedIntervalStatus::InvalidInput:
      return "invalid input";
    case RepresentedIntervalStatus::IdentityMismatch:
      return "duplicate path identity";
    case RepresentedIntervalStatus::ResourceLimit:
      return "resource limit";
  }
  return "invalid status";
}

RepresentedIntervalReport Failure(RepresentedIntervalStatus status) noexcept {
  RepresentedIntervalReport result;
  result.status = status;
  result.message = Message(status);
  return result;
}

}  // namespace

struct RepresentedIntervalCrossing::Impl {
  explicit Impl(RepresentedIntervalLimits input) : limits(input) {}
  RepresentedIntervalLimits limits;
  RepresentedIntervalForecast forecast;
  std::vector<std::uint32_t> path_indices;
  std::vector<CanonicalPair> pairs;
  std::vector<RepresentedIntervalResult> published;
  std::vector<RepresentedIntervalResult> staging;
  bool complete = false;
};

RepresentedIntervalCrossing::RepresentedIntervalCrossing() noexcept = default;
RepresentedIntervalCrossing::~RepresentedIntervalCrossing() = default;
RepresentedIntervalCrossing::RepresentedIntervalCrossing(
    RepresentedIntervalCrossing&&) noexcept = default;
RepresentedIntervalCrossing& RepresentedIntervalCrossing::operator=(
    RepresentedIntervalCrossing&&) noexcept = default;

RepresentedIntervalPreflight RepresentedIntervalCrossing::Preflight(
    RepresentedIntervalLimits limits) noexcept {
  RepresentedIntervalPreflight result;
  if (!limits.max_paths || !limits.max_input_pairs || !limits.max_results ||
      !limits.max_work_per_pair || !limits.max_total_work ||
      limits.max_depth > 52) {
    result.report = Failure(RepresentedIntervalStatus::InvalidInput);
    return result;
  }
  result.forecast.path_index_capacity = limits.max_paths;
  result.forecast.pair_capacity = limits.max_input_pairs;
  result.forecast.result_capacity = limits.max_results;
  std::size_t paths = 0, pairs = 0, results = 0, total = sizeof(Impl);
  if (!MultiplySize(limits.max_paths, sizeof(std::uint32_t), &paths) ||
      !MultiplySize(limits.max_input_pairs, sizeof(CanonicalPair), &pairs) ||
      !MultiplySize(limits.max_results,
                    2 * sizeof(RepresentedIntervalResult), &results) ||
      !AddSize(total, paths, &total) || !AddSize(total, pairs, &total) ||
      !AddSize(total, results, &total)) {
    result.report = Failure(RepresentedIntervalStatus::ResourceLimit);
    return result;
  }
  result.forecast.owned_host_bytes = total;
  if (total > limits.max_host_bytes) {
    result.report = Failure(RepresentedIntervalStatus::ResourceLimit);
    return result;
  }
  return result;
}

RepresentedIntervalReport RepresentedIntervalCrossing::Initialize(
    RepresentedIntervalLimits limits) noexcept {
  if (impl_)
    return Failure(RepresentedIntervalStatus::AlreadyInitialized);
  const auto plan = Preflight(limits);
  if (plan.report.status != RepresentedIntervalStatus::Ok)
    return plan.report;
  try {
    auto next = std::make_unique<Impl>(limits);
    next->forecast = plan.forecast;
    next->path_indices.reserve(limits.max_paths);
    next->pairs.reserve(limits.max_input_pairs);
    next->published.reserve(limits.max_results);
    next->staging.reserve(limits.max_results);
    impl_ = std::move(next);
  } catch (...) {
    return Failure(RepresentedIntervalStatus::ResourceLimit);
  }
  return {};
}

RepresentedIntervalReport RepresentedIntervalCrossing::Certify(
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count) noexcept {
  if (!impl_)
    return Failure(RepresentedIntervalStatus::NotInitialized);
  auto& storage = *impl_;
  RepresentedIntervalReport report;
  report.input_paths = path_count;
  report.input_pairs = pair_count;
  if ((path_count && !paths) || (pair_count && !pairs) ||
      path_count > storage.limits.max_paths ||
      pair_count > storage.limits.max_input_pairs) {
    report.status =
        path_count > storage.limits.max_paths ||
                pair_count > storage.limits.max_input_pairs
            ? RepresentedIntervalStatus::ResourceLimit
            : RepresentedIntervalStatus::InvalidInput;
    report.message = Message(report.status);
    return report;
  }
  std::size_t path_bytes = 0, pair_bytes = 0;
  if (!MultiplySize(path_count, sizeof(*paths), &path_bytes) ||
      !MultiplySize(pair_count, sizeof(*pairs), &pair_bytes) ||
      (!storage.published.empty() &&
       (!RangeDisjoint(paths, path_bytes, storage.published.data(),
                       storage.published.size() *
                           sizeof(RepresentedIntervalResult)) ||
        !RangeDisjoint(pairs, pair_bytes, storage.published.data(),
                       storage.published.size() *
                           sizeof(RepresentedIntervalResult))))) {
    report.status = RepresentedIntervalStatus::InvalidInput;
    report.message = Message(report.status);
    return report;
  }

  storage.path_indices.clear();
  for (std::size_t i = 0; i < path_count; ++i) {
    const auto status = ValidatePath(paths[i]);
    if (status != RepresentedIntervalStatus::Ok) {
      report.status = status;
      report.input_path = i;
      report.message = Message(status);
      return report;
    }
    storage.path_indices.push_back(static_cast<std::uint32_t>(i));
  }
  std::sort(storage.path_indices.begin(), storage.path_indices.end(),
            [&](std::uint32_t a, std::uint32_t b) {
              return Compare(paths[a].key, paths[b].key) < 0;
            });
  for (std::size_t i = 1; i < storage.path_indices.size(); ++i) {
    if (Same(paths[storage.path_indices[i - 1]].key,
             paths[storage.path_indices[i]].key)) {
      report.status = RepresentedIntervalStatus::IdentityMismatch;
      report.input_path = storage.path_indices[i];
      report.message = Message(report.status);
      return report;
    }
  }

  storage.pairs.clear();
  for (std::size_t i = 0; i < pair_count; ++i) {
    if (pairs[i].first >= path_count || pairs[i].second >= path_count ||
        pairs[i].first == pairs[i].second) {
      report.status = RepresentedIntervalStatus::InvalidInput;
      report.input_pair = i;
      report.message = Message(report.status);
      return report;
    }
    CanonicalPair pair;
    pair.first = pairs[i].first;
    pair.second = pairs[i].second;
    pair.input_pair = i;
    if (Compare(paths[pair.second].key, paths[pair.first].key) < 0)
      std::swap(pair.first, pair.second);
    pair.key.paths[0] = paths[pair.first].key;
    pair.key.paths[1] = paths[pair.second].key;
    storage.pairs.push_back(pair);
  }
  std::sort(storage.pairs.begin(), storage.pairs.end(), PairLess);
  storage.pairs.erase(
      std::unique(storage.pairs.begin(), storage.pairs.end(), SamePair),
      storage.pairs.end());
  report.unique_pairs = storage.pairs.size();
  if (storage.pairs.size() > storage.limits.max_results) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }

  storage.staging.clear();
  try {
    for (const auto& pair : storage.pairs) {
      auto result =
          CertifyPair(paths[pair.first], paths[pair.second], storage.limits,
                      pair.key);
      if (result.work > storage.limits.max_total_work - report.work) {
        report.status = RepresentedIntervalStatus::ResourceLimit;
        report.input_pair = pair.input_pair;
        report.message = Message(report.status);
        storage.staging.clear();
        return report;
      }
      report.work += result.work;
      if (result.classification ==
          RepresentedIntervalClassification::CertifiedSeparated)
        ++report.certified_separated;
      else if (result.classification ==
               RepresentedIntervalClassification::CertifiedCrossingContact)
        ++report.certified_crossing_contact;
      else
        ++report.unresolved;
      storage.staging.push_back(result);
    }
  } catch (...) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message = Message(report.status);
    storage.staging.clear();
    return report;
  }
  storage.published.swap(storage.staging);
  storage.staging.clear();
  storage.complete = true;
  return report;
}

RepresentedIntervalForecast RepresentedIntervalCrossing::forecast() const
    noexcept {
  return impl_ ? impl_->forecast : RepresentedIntervalForecast{};
}

RepresentedIntervalResultView RepresentedIntervalCrossing::results() const
    noexcept {
  if (!impl_)
    return {};
  return {impl_->published.data(), impl_->published.size(), impl_->complete};
}

}  // namespace tlfea::contact
