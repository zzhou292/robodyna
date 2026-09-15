// SPDX-License-Identifier: AGPL-3.0-or-later
#include "RepresentedIntervalCrossing.h"

#include <algorithm>
#include <atomic>
#include <boost/multiprecision/cpp_int.hpp>
#include <cerrno>
#include <cstring>
#include <limits>
#include <new>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <tuple>
#include <unistd.h>
#include <utility>
#include <vector>

namespace tlfea::contact {
namespace {

constexpr std::size_t kWorkerStackBytes = 2u << 20;

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

bool SameBits(double a, double b) noexcept {
  std::uint64_t aa = 0, bb = 0;
  std::memcpy(&aa, &a, sizeof(aa));
  std::memcpy(&bb, &b, sizeof(bb));
  return aa == bb;
}

bool SameBits(Vec3 a, Vec3 b) noexcept {
  return SameBits(a.x, b.x) && SameBits(a.y, b.y) &&
         SameBits(a.z, b.z);
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

bool Wait(sem_t* semaphore) noexcept {
  if (!semaphore)
    return false;
  while (sem_wait(semaphore) != 0) {
    if (errno != EINTR)
      return false;
  }
  return true;
}

std::size_t PageBytes() noexcept {
  const long value = sysconf(_SC_PAGESIZE);
  return value > 0 ? static_cast<std::size_t>(value) : 0;
}

struct CanonicalPair {
  std::uint32_t first = 0;
  std::uint32_t second = 0;
  std::size_t input_pair = SIZE_MAX;
  RepresentedIntervalPairKey key;
};

struct VertexLedgerRow {
  FacetVertexKey key;
  Vec3 endpoint[2];
  RepresentedMotion motion = RepresentedMotion::LinearNodalV1;
  std::size_t input_path = SIZE_MAX;
};

bool VertexLedgerLess(const VertexLedgerRow& a,
                      const VertexLedgerRow& b) noexcept {
  return Compare(a.key, b.key) < 0;
}

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

bool CommonTranslation(
    const RepresentedTrianglePath& a,
    const RepresentedTrianglePath& b) {
  Dyadic reference[3];
  for (unsigned component = 0; component < 3; ++component) {
    reference[component] = Subtract(
        Exact(Component(a.vertices[0].endpoint[1], component)),
        Exact(Component(a.vertices[0].endpoint[0], component)));
  }
  const RepresentedTrianglePath* paths[2]{&a, &b};
  for (const auto* path : paths)
    for (const auto& vertex : path->vertices)
      for (unsigned component = 0; component < 3; ++component) {
        const auto displacement = Subtract(
            Exact(Component(vertex.endpoint[1], component)),
            Exact(Component(vertex.endpoint[0], component)));
        if (Compare(displacement, reference[component]) != 0)
          return false;
      }
  return true;
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

bool SweptBoxesSeparated(const ExactTriangle samples_a[3],
                         const ExactTriangle samples_b[3]) {
  for (unsigned component = 0; component < 3; ++component) {
    Dyadic minimum_a = Component(samples_a[0].vertex[0], component);
    Dyadic maximum_a = minimum_a;
    Dyadic minimum_b = Component(samples_b[0].vertex[0], component);
    Dyadic maximum_b = minimum_b;
    for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
      const unsigned sample = endpoint * 2;
      for (unsigned vertex = 0; vertex < 3; ++vertex) {
        const Dyadic a =
            Component(samples_a[sample].vertex[vertex], component);
        const Dyadic b =
            Component(samples_b[sample].vertex[vertex], component);
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

void CopyKey(const FacetVertexKey& source,
             FacetVertexKey* target) noexcept {
  target->source_instance_id = source.source_instance_id;
  target->first = source.first;
  target->second = source.second;
  target->kind = source.kind;
  target->numerator = source.numerator;
  target->denominator = source.denominator;
  target->level = source.level;
  target->grid_i = source.grid_i;
  target->grid_j = source.grid_j;
}

void CopyKey(const FacetEdgeKey& source,
             FacetEdgeKey* target) noexcept {
  CopyKey(source.endpoints[0], &target->endpoints[0]);
  CopyKey(source.endpoints[1], &target->endpoints[1]);
  target->parent_eid = source.parent_eid;
  target->parent_boundary = source.parent_boundary;
}

void CopyKey(const RepresentedTrianglePathKey& source,
             RepresentedTrianglePathKey* target) noexcept {
  target->source_instance_id = source.source_instance_id;
  target->parent_eid = source.parent_eid;
  target->level = source.level;
  target->local_facet = source.local_facet;
}

void StoreResult(const RepresentedIntervalResult& source,
                 RepresentedIntervalResult* target) noexcept {
  std::fill_n(reinterpret_cast<unsigned char*>(target), sizeof(*target),
              static_cast<unsigned char>(0));
  CopyKey(source.key.paths[0], &target->key.paths[0]);
  CopyKey(source.key.paths[1], &target->key.paths[1]);
  target->feature.kind = source.feature.kind;
  CopyKey(source.feature.vertex, &target->feature.vertex);
  CopyKey(source.feature.face, &target->feature.face);
  CopyKey(source.feature.edges[0], &target->feature.edges[0]);
  CopyKey(source.feature.edges[1], &target->feature.edges[1]);
  target->classification = source.classification;
  target->reason = source.reason;
  target->geometry = source.geometry;
  target->witness_time_numerator = source.witness_time_numerator;
  target->witness_time_depth = source.witness_time_depth;
  target->work = source.work;
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

struct ExactScratch {
  ExactTriangle a[3];
  ExactTriangle b[3];
};

enum class CellDisposition : std::uint8_t {
  Separated,
  Crossing,
  Split,
  Unresolved,
};

struct CellEvaluation {
  CellDisposition disposition = CellDisposition::Split;
  RepresentedIntervalReason reason = RepresentedIntervalReason::None;
  RepresentedIntervalResult crossing;
};

CellEvaluation EvaluateCell(const RepresentedTrianglePath& path_a,
                            const RepresentedTrianglePath& path_b,
                            const RepresentedIntervalPairKey& key, Cell cell,
                            ExactScratch* scratch) {
  const DyadicTime times[3] = {Lower(cell), Middle(cell), Upper(cell)};
  bool degenerate = false;
  for (unsigned sample = 0; sample < 3; ++sample) {
    scratch->a[sample] = At(path_a, times[sample]);
    scratch->b[sample] = At(path_b, times[sample]);
    degenerate = degenerate || Degenerate(scratch->a[sample]) ||
                 Degenerate(scratch->b[sample]);
  }
  for (unsigned sample = 0; sample < 3; ++sample) {
    if (Degenerate(scratch->a[sample]) ||
        Degenerate(scratch->b[sample]))
      continue;
    const auto intersection =
        Intersects(scratch->a[sample], scratch->b[sample]);
    if (!intersection.intersects)
      continue;
    CellEvaluation evaluation;
    evaluation.disposition = CellDisposition::Crossing;
    auto& result = evaluation.crossing;
    result.key = key;
    result.feature = IntersectionFeature(path_a, path_b,
                                         scratch->a[sample],
                                         scratch->b[sample]);
    result.classification =
        RepresentedIntervalClassification::CertifiedCrossingContact;
    result.reason = RepresentedIntervalReason::None;
    result.geometry =
        intersection.coplanar ? RepresentedIntersectionGeometry::Coplanar
                              : RepresentedIntersectionGeometry::Transverse;
    result.witness_time_numerator = times[sample].numerator;
    result.witness_time_depth = times[sample].depth;
    return evaluation;
  }
  if (degenerate)
    return {CellDisposition::Unresolved,
            RepresentedIntervalReason::DegenerateGeometry, {}};
  if (RegularCell(scratch->a) && RegularCell(scratch->b) &&
      SweptBoxesSeparated(scratch->a, scratch->b)) {
    return {CellDisposition::Separated, RepresentedIntervalReason::None, {}};
  }
  return {};
}

void RaiseReason(RepresentedIntervalReason candidate,
                 RepresentedIntervalReason* current) noexcept {
  if (ReasonPriority(candidate) > ReasonPriority(*current))
    *current = candidate;
}

RepresentedIntervalResult CertifyPair(
    const RepresentedTrianglePath& a, const RepresentedTrianglePath& b,
    RepresentedIntervalLimits limits, RepresentedIntervalPairKey key,
    Cell* dfs, std::size_t dfs_capacity, ExactScratch* scratch) noexcept {
  if (a.motion != RepresentedMotion::LinearNodalV1 ||
      b.motion != RepresentedMotion::LinearNodalV1)
    return Unresolved(key, RepresentedIntervalReason::UnsupportedMotion, 0);
  std::size_t work = 0;
  std::size_t dfs_size = 0;
  bool all_leaves_separated = true;
  RepresentedIntervalReason unresolved = RepresentedIntervalReason::None;
  dfs[dfs_size++] = {};
  try {
    // A bit-exact common translation preserves every relative point,
    // segment and triangle predicate over the complete represented interval.
    // Test the exact binary64-real displacements rather than rounded double
    // differences: a single static exact evaluation is then a whole-interval
    // certificate, even when the absolute swept AABBs overlap.
    if (CommonTranslation(a, b)) {
      auto evaluation = EvaluateCell(a, b, key, {}, scratch);
      if (evaluation.disposition == CellDisposition::Crossing) {
        evaluation.crossing.work = 1;
        return evaluation.crossing;
      }
      if (evaluation.disposition == CellDisposition::Unresolved)
        return Unresolved(key, evaluation.reason, 1);
      RepresentedIntervalResult result;
      result.key = key;
      result.classification =
          RepresentedIntervalClassification::CertifiedSeparated;
      result.reason = RepresentedIntervalReason::None;
      result.work = 1;
      return result;
    }
    while (dfs_size) {
      if (work >= limits.max_work_per_pair) {
        all_leaves_separated = false;
        RaiseReason(RepresentedIntervalReason::WorkExhausted, &unresolved);
        break;
      }
      const Cell cell = dfs[--dfs_size];
      ++work;
      auto evaluation = EvaluateCell(a, b, key, cell, scratch);
      if (evaluation.disposition == CellDisposition::Crossing) {
        evaluation.crossing.work = work;
        return evaluation.crossing;
      }
      if (evaluation.disposition == CellDisposition::Separated)
        continue;
      if (evaluation.disposition == CellDisposition::Unresolved) {
        all_leaves_separated = false;
        RaiseReason(evaluation.reason, &unresolved);
        continue;
      }
      if (cell.depth >= limits.max_depth) {
        all_leaves_separated = false;
        RaiseReason(RepresentedIntervalReason::WorkExhausted, &unresolved);
        continue;
      }
      const Cell right{cell.lower + cell.upper, cell.upper * 2,
                       cell.depth + 1};
      const Cell left{cell.lower * 2, cell.lower + cell.upper,
                      cell.depth + 1};
      if (dfs_size + 2 > dfs_capacity) {
        all_leaves_separated = false;
        RaiseReason(RepresentedIntervalReason::ExactArithmeticRange,
                    &unresolved);
        break;
      }
      dfs[dfs_size++] = right;
      dfs[dfs_size++] = left;
    }
  } catch (...) {
    return Unresolved(key, RepresentedIntervalReason::ExactArithmeticRange,
                      work);
  }
  if (all_leaves_separated) {
    RepresentedIntervalResult result;
    result.key = key;
    result.classification =
        RepresentedIntervalClassification::CertifiedSeparated;
    result.reason = RepresentedIntervalReason::None;
    result.work = work;
    return result;
  }
  if (unresolved == RepresentedIntervalReason::None)
    unresolved = RepresentedIntervalReason::WorkExhausted;
  return Unresolved(key, unresolved, work);
}

bool KnownMotion(RepresentedMotion motion) noexcept {
  return motion == RepresentedMotion::LinearNodalV1 ||
         motion == RepresentedMotion::RigidArc ||
         motion == RepresentedMotion::Nonlinear;
}

bool CanonicalVertexKey(const FacetVertexKey& key,
                        const RepresentedTrianglePathKey& path) noexcept {
  if (key.source_instance_id != path.source_instance_id ||
      key.denominator == 0)
    return false;
  if (key.kind == FacetVertexKind::SourceVertex)
    return key.second == 0 && key.numerator == 0 &&
           key.denominator == 1 && key.level == 0 && key.grid_i == 0 &&
           key.grid_j == 0;
  if (key.kind == FacetVertexKind::SourceEdge)
    return key.first < key.second && key.numerator > 0 &&
           key.numerator < key.denominator &&
           (key.denominator & (key.denominator - 1)) == 0 &&
           key.denominator <= (1u << path.level) &&
           (key.denominator == 1 || (key.numerator & 1u)) &&
           key.level == 0 && key.grid_i == 0 && key.grid_j == 0;
  if (key.kind == FacetVertexKind::ParentInterior) {
    const unsigned n = 1u << path.level;
    return key.first == path.parent_eid && key.second == 0 &&
           key.numerator == 0 && key.denominator == 1 &&
           key.level == path.level && key.grid_i <= n && key.grid_j <= n;
  }
  return false;
}

bool SameTrajectory(const RepresentedVertexPath& a,
                    const RepresentedVertexPath& b) noexcept {
  return Same(a.key, b.key) && SameBits(a.endpoint[0], b.endpoint[0]) &&
         SameBits(a.endpoint[1], b.endpoint[1]);
}

bool CompatiblePath(const RepresentedTrianglePath& a,
                    const RepresentedTrianglePath& b) noexcept {
  if (a.motion != b.motion)
    return false;
  for (const auto& vertex : a.vertices) {
    bool found = false;
    for (const auto& other : b.vertices)
      found = found || SameTrajectory(vertex, other);
    if (!found)
      return false;
  }
  for (const auto& edge : a.edge_keys) {
    bool found = false;
    for (const auto& other : b.edge_keys)
      found = found || Compare(edge, other) == 0;
    if (!found)
      return false;
  }
  return true;
}

RepresentedIntervalStatus ValidatePath(
    const RepresentedTrianglePath& path) noexcept {
  if (!KnownMotion(path.motion) || path.key.level > 2)
    return RepresentedIntervalStatus::InvalidInput;
  for (unsigned i = 0; i < 3; ++i) {
    if (!IsFinite(path.vertices[i].endpoint[0]) ||
        !IsFinite(path.vertices[i].endpoint[1]) ||
        !CanonicalVertexKey(path.vertices[i].key, path.key))
      return RepresentedIntervalStatus::InvalidInput;
    for (unsigned j = 0; j < i; ++j)
      if (Same(path.vertices[i].key, path.vertices[j].key))
        return RepresentedIntervalStatus::InvalidInput;
    const unsigned next = (i + 1) % 3;
    const auto& edge = path.edge_keys[i];
    const bool edge_matches =
        (Same(edge.endpoints[0], path.vertices[i].key) &&
         Same(edge.endpoints[1], path.vertices[next].key)) ||
        (Same(edge.endpoints[1], path.vertices[i].key) &&
         Same(edge.endpoints[0], path.vertices[next].key));
    const bool parent_matches =
        edge.parent_boundary ? edge.parent_eid == 0
                             : edge.parent_eid == path.key.parent_eid;
    if (!edge_matches || Compare(edge.endpoints[0], edge.endpoints[1]) >= 0 ||
        !parent_matches)
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

RepresentedIntervalReport FreshReport() noexcept {
  RepresentedIntervalReport result;
  std::fill_n(reinterpret_cast<unsigned char*>(&result), sizeof(result),
              static_cast<unsigned char>(0));
  result.input_path = SIZE_MAX;
  result.input_pair = SIZE_MAX;
  result.message = Message(RepresentedIntervalStatus::Ok);
  return result;
}

RepresentedIntervalReport Failure(RepresentedIntervalStatus status) noexcept {
  RepresentedIntervalReport result = FreshReport();
  result.status = status;
  result.message = Message(status);
  return result;
}

}  // namespace

struct RepresentedIntervalCrossing::Impl {
  enum class Phase : unsigned {
    Constructing,
    Warm,
    Running,
    Failed,
    Stopping,
  };

  struct PairStatus {
    bool complete = false;
  };

  struct WorkerSlot {
    Impl* owner = nullptr;
    std::size_t index = 0;
    pthread_t thread{};
    void* stack_mapping = nullptr;
    sem_t start{};
    bool start_initialized = false;
    bool started = false;
    bool failed = false;
  };

  explicit Impl(RepresentedIntervalLimits input) : limits(input) {}
  ~Impl() { Shutdown(); }

  RepresentedIntervalLimits limits;
  RepresentedIntervalForecast forecast;
  std::vector<std::uint32_t> path_indices;
  std::vector<CanonicalPair> pairs;
  std::vector<VertexLedgerRow> vertex_ledger;
  std::vector<RepresentedIntervalResult> published;
  std::vector<RepresentedIntervalResult> staging;
  std::unique_ptr<PairStatus[]> pair_status;
  std::unique_ptr<WorkerSlot[]> workers;
  std::unique_ptr<Cell[]> dfs_frames;
  std::unique_ptr<ExactScratch[]> exact_scratch;
  sem_t completed{};
  bool completed_initialized = false;
  std::size_t started_workers = 0;
  std::atomic<std::size_t> next_pair{0};
  std::atomic<bool> stop{false};
  std::atomic<bool> pool_failed{false};
  std::atomic<bool> busy{false};
  std::atomic<Phase> phase{Phase::Constructing};
  const RepresentedTrianglePath* job_paths = nullptr;
  std::size_t job_pair_count = 0;
  bool complete = false;

  bool WorkerStacksDisjoint(const void* input,
                            std::size_t bytes) const noexcept {
    if (!bytes)
      return true;
    if (!input || !workers)
      return false;
    const std::size_t page_bytes = PageBytes();
    if (!page_bytes || kWorkerStackBytes > SIZE_MAX - page_bytes)
      return false;
    const std::size_t mapping_bytes = kWorkerStackBytes + page_bytes;
    for (unsigned i = 0; i < limits.worker_count; ++i)
      if (!workers[i].stack_mapping ||
          !RangeDisjoint(input, bytes, workers[i].stack_mapping,
                         mapping_bytes))
        return false;
    return true;
  }

  static void* WorkerEntry(void* opaque) noexcept {
    auto* worker = static_cast<WorkerSlot*>(opaque);
    if (!worker || !worker->owner)
      return nullptr;
    auto& owner = *worker->owner;
    for (;;) {
      if (!Wait(&worker->start)) {
        worker->failed = true;
        owner.pool_failed.store(true, std::memory_order_release);
        owner.phase.store(Phase::Failed, std::memory_order_release);
        return nullptr;
      }
      if (owner.stop.load(std::memory_order_acquire))
        return nullptr;
      worker->failed = false;
      owner.EvaluateJobs(worker->index);
      if (sem_post(&owner.completed) != 0) {
        worker->failed = true;
        owner.pool_failed.store(true, std::memory_order_release);
        owner.phase.store(Phase::Failed, std::memory_order_release);
        return nullptr;
      }
    }
  }

  void EvaluateJobs(std::size_t worker_index) noexcept {
    Cell* dfs = dfs_frames.get() +
        worker_index * forecast.dfs_frame_capacity;
    ExactScratch* scratch = exact_scratch.get() + worker_index;
    for (;;) {
      const std::size_t pair_index =
          next_pair.fetch_add(1, std::memory_order_relaxed);
      if (pair_index >= job_pair_count)
        return;
      // Scheduling affects only worker ownership. Each canonical pair index
      // has one staging/status writer and private DFS/exact scratch; the host
      // folds staging in increasing pair_index order after all workers join.
      const auto& pair = pairs[pair_index];
      const auto result = CertifyPair(
          job_paths[pair.first], job_paths[pair.second], limits, pair.key,
          dfs, forecast.dfs_frame_capacity, scratch);
      StoreResult(result, &staging[pair_index]);
      pair_status[pair_index].complete = true;
    }
  }

  bool RunWorkers(const RepresentedTrianglePath* paths,
                  std::size_t pair_count) noexcept {
    Phase expected = Phase::Warm;
    if (!phase.compare_exchange_strong(
            expected, Phase::Running, std::memory_order_acq_rel))
      return false;
    job_paths = paths;
    job_pair_count = pair_count;
    next_pair.store(0, std::memory_order_relaxed);
    pool_failed.store(false, std::memory_order_release);
    std::size_t posted = 0;
    for (; posted < started_workers; ++posted)
      if (sem_post(&workers[posted].start) != 0)
        break;
    bool failed = posted != started_workers;
    for (std::size_t i = 0; i < posted; ++i)
      if (!Wait(&completed))
        failed = true;
    failed = failed || pool_failed.load(std::memory_order_acquire);
    phase.store(failed ? Phase::Failed : Phase::Warm,
                std::memory_order_release);
    return !failed;
  }

  bool StartWorkers() noexcept {
    const std::size_t page_bytes = PageBytes();
    if (!page_bytes ||
        kWorkerStackBytes < static_cast<std::size_t>(PTHREAD_STACK_MIN) ||
        kWorkerStackBytes % page_bytes ||
        kWorkerStackBytes > SIZE_MAX - page_bytes ||
        sem_init(&completed, 0, 0) != 0)
      return false;
    completed_initialized = true;
    const std::size_t mapping_bytes = kWorkerStackBytes + page_bytes;
    for (unsigned i = 0; i < limits.worker_count; ++i) {
      auto& worker = workers[i];
      worker.owner = this;
      worker.index = i;
      if (sem_init(&worker.start, 0, 0) != 0)
        return false;
      worker.start_initialized = true;
#ifdef MAP_STACK
      constexpr int stack_flag = MAP_STACK;
#else
      constexpr int stack_flag = 0;
#endif
      worker.stack_mapping = mmap(
          nullptr, mapping_bytes, PROT_READ | PROT_WRITE,
          MAP_PRIVATE | MAP_ANONYMOUS | stack_flag, -1, 0);
      if (worker.stack_mapping == MAP_FAILED) {
        worker.stack_mapping = nullptr;
        return false;
      }
      auto* stack = static_cast<unsigned char*>(worker.stack_mapping) +
          page_bytes;
      std::memset(stack, 0, kWorkerStackBytes);
      if (mprotect(worker.stack_mapping, page_bytes, PROT_NONE) != 0)
        return false;
      pthread_attr_t attributes;
      if (pthread_attr_init(&attributes) != 0)
        return false;
      const int guard_status =
          pthread_attr_setguardsize(&attributes, 0);
      const int stack_status = guard_status
          ? guard_status
          : pthread_attr_setstack(
                &attributes, stack, kWorkerStackBytes);
      const int create_status = stack_status
          ? stack_status
          : pthread_create(
                &worker.thread, &attributes, &Impl::WorkerEntry, &worker);
      pthread_attr_destroy(&attributes);
      if (create_status != 0)
        return false;
      worker.started = true;
      ++started_workers;
    }
    phase.store(Phase::Warm, std::memory_order_release);
    // Exercise every thread, stack and semaphore before publication.
    return RunWorkers(nullptr, 0);
  }

  void Shutdown() noexcept {
    phase.store(Phase::Stopping, std::memory_order_release);
    stop.store(true, std::memory_order_release);
    if (workers) {
      for (std::size_t i = 0; i < started_workers; ++i)
        if (workers[i].started)
          sem_post(&workers[i].start);
      for (std::size_t i = 0; i < started_workers; ++i) {
        if (workers[i].started)
          pthread_join(workers[i].thread, nullptr);
        workers[i].started = false;
      }
      const std::size_t page_bytes = PageBytes();
      const std::size_t mapping_bytes =
          page_bytes && kWorkerStackBytes <= SIZE_MAX - page_bytes
              ? kWorkerStackBytes + page_bytes
              : 0;
      for (unsigned i = 0; i < limits.worker_count; ++i) {
        if (workers[i].start_initialized)
          sem_destroy(&workers[i].start);
        workers[i].start_initialized = false;
        if (workers[i].stack_mapping && mapping_bytes)
          munmap(workers[i].stack_mapping, mapping_bytes);
        workers[i].stack_mapping = nullptr;
      }
    }
    started_workers = 0;
    if (completed_initialized)
      sem_destroy(&completed);
    completed_initialized = false;
  }
};

RepresentedIntervalCrossing::RepresentedIntervalCrossing() noexcept = default;
RepresentedIntervalCrossing::~RepresentedIntervalCrossing() = default;
RepresentedIntervalCrossing::RepresentedIntervalCrossing(
    RepresentedIntervalCrossing&&) noexcept = default;
RepresentedIntervalCrossing& RepresentedIntervalCrossing::operator=(
    RepresentedIntervalCrossing&&) noexcept = default;

RepresentedIntervalPreflight RepresentedIntervalCrossing::Preflight(
    RepresentedIntervalLimits limits) noexcept {
  RepresentedIntervalPreflight result{};
  result.report = FreshReport();
  if (!limits.max_paths || !limits.max_input_pairs || !limits.max_results ||
      !limits.max_work_per_pair || !limits.max_total_work ||
      limits.max_depth > 52 || limits.max_paths > UINT32_MAX ||
      !limits.worker_count ||
      limits.worker_count > RepresentedIntervalMaximumWorkerCount) {
    result.report = Failure(RepresentedIntervalStatus::InvalidInput);
    return result;
  }
  result.forecast.path_index_capacity = limits.max_paths;
  result.forecast.pair_capacity = limits.max_input_pairs;
  result.forecast.result_capacity = limits.max_results;
  result.forecast.pair_status_capacity = limits.max_input_pairs;
  result.forecast.worker_count = limits.worker_count;
  result.forecast.dfs_frame_capacity =
      static_cast<std::size_t>(limits.max_depth) + 1;
  std::size_t all_dfs_frames = 0;
  const std::size_t page_bytes = PageBytes();
  if (!MultiplySize(limits.max_paths, 3,
                    &result.forecast.vertex_ledger_capacity) ||
      !MultiplySize(limits.max_paths, sizeof(std::uint32_t),
                    &result.forecast.path_index_bytes) ||
      !MultiplySize(limits.max_input_pairs, sizeof(CanonicalPair),
                    &result.forecast.pair_bytes) ||
      !MultiplySize(limits.max_results,
                    2 * sizeof(RepresentedIntervalResult),
                    &result.forecast.result_bytes) ||
      !MultiplySize(result.forecast.vertex_ledger_capacity,
                    sizeof(VertexLedgerRow),
                    &result.forecast.vertex_ledger_bytes) ||
      !MultiplySize(result.forecast.dfs_frame_capacity,
                    limits.worker_count, &all_dfs_frames) ||
      !MultiplySize(all_dfs_frames, sizeof(Cell),
                    &result.forecast.dfs_frame_bytes) ||
      !MultiplySize(limits.worker_count, sizeof(ExactScratch),
                    &result.forecast.exact_scratch_bytes) ||
      !MultiplySize(limits.max_input_pairs, sizeof(Impl::PairStatus),
                    &result.forecast.pair_status_bytes) ||
      !MultiplySize(limits.worker_count, sizeof(Impl::WorkerSlot),
                    &result.forecast.worker_metadata_bytes) ||
      !page_bytes ||
      kWorkerStackBytes <
          static_cast<std::size_t>(PTHREAD_STACK_MIN) ||
      kWorkerStackBytes % page_bytes ||
      kWorkerStackBytes > SIZE_MAX - page_bytes ||
      !MultiplySize(limits.worker_count,
                    kWorkerStackBytes + page_bytes,
                    &result.forecast.worker_stack_bytes)) {
    result.report = Failure(RepresentedIntervalStatus::ResourceLimit);
    return result;
  }
  std::size_t total = sizeof(Impl);
  const std::size_t regions[]{
      result.forecast.path_index_bytes, result.forecast.pair_bytes,
      result.forecast.result_bytes, result.forecast.vertex_ledger_bytes,
      result.forecast.dfs_frame_bytes, result.forecast.exact_scratch_bytes,
      result.forecast.pair_status_bytes,
      result.forecast.worker_metadata_bytes,
      result.forecast.worker_stack_bytes};
  for (const auto bytes : regions) {
    if (!AddSize(total, bytes, &total)) {
      result.report = Failure(RepresentedIntervalStatus::ResourceLimit);
      return result;
    }
  }
  result.forecast.owned_host_bytes = total;
  if (total > limits.max_host_bytes) {
    result.report = Failure(RepresentedIntervalStatus::ResourceLimit);
    return result;
  }
  result.forecast.startup_host_bytes = total;
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
    next->vertex_ledger.reserve(plan.forecast.vertex_ledger_capacity);
    next->published.reserve(limits.max_results);
    next->staging.reserve(limits.max_results);
    const std::size_t all_dfs_frames =
        plan.forecast.dfs_frame_bytes / sizeof(Cell);
    next->pair_status.reset(new (std::nothrow)
        Impl::PairStatus[limits.max_input_pairs]);
    next->workers.reset(new (std::nothrow)
        Impl::WorkerSlot[limits.worker_count]);
    next->dfs_frames.reset(new (std::nothrow) Cell[all_dfs_frames]);
    next->exact_scratch.reset(new (std::nothrow)
        ExactScratch[limits.worker_count]);
    if (next->path_indices.capacity() != limits.max_paths ||
        next->pairs.capacity() != limits.max_input_pairs ||
        next->vertex_ledger.capacity() !=
            plan.forecast.vertex_ledger_capacity ||
        next->published.capacity() != limits.max_results ||
        next->staging.capacity() != limits.max_results ||
        !next->pair_status || !next->workers || !next->dfs_frames ||
        !next->exact_scratch)
      return Failure(RepresentedIntervalStatus::ResourceLimit);
    if (!next->StartWorkers())
      return Failure(RepresentedIntervalStatus::ResourceLimit);
    impl_ = std::move(next);
  } catch (...) {
    return Failure(RepresentedIntervalStatus::ResourceLimit);
  }
  return FreshReport();
}

RepresentedIntervalReport RepresentedIntervalCrossing::Certify(
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count) noexcept {
  if (!impl_)
    return Failure(RepresentedIntervalStatus::NotInitialized);
  auto& storage = *impl_;
  RepresentedIntervalReport report = FreshReport();
  report.input_paths = path_count;
  report.input_pairs = pair_count;
  bool expected_idle = false;
  if (!storage.busy.compare_exchange_strong(
          expected_idle, true, std::memory_order_acq_rel)) {
    report.status = RepresentedIntervalStatus::InvalidInput;
    report.message =
        "represented interval crossing does not accept concurrent calls";
    return report;
  }
  struct BusyRelease {
    std::atomic<bool>* value;
    ~BusyRelease() { value->store(false, std::memory_order_release); }
  } busy_release{&storage.busy};
  if (storage.phase.load(std::memory_order_acquire) !=
      Impl::Phase::Warm) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message =
        "represented interval crossing worker pool is unavailable";
    return report;
  }
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
  const auto disjoint_from_owned = [&](const void* data,
                                       std::size_t bytes) noexcept {
    if (!RangeDisjoint(data, bytes, &storage, sizeof(storage)) ||
        !RangeDisjoint(data, bytes, storage.exact_scratch.get(),
                       storage.forecast.exact_scratch_bytes) ||
        !RangeDisjoint(data, bytes, storage.dfs_frames.get(),
                       storage.forecast.dfs_frame_bytes) ||
        !RangeDisjoint(data, bytes, storage.pair_status.get(),
                       storage.forecast.pair_status_bytes) ||
        !RangeDisjoint(data, bytes, storage.workers.get(),
                       storage.forecast.worker_metadata_bytes) ||
        !storage.WorkerStacksDisjoint(data, bytes))
      return false;
    struct Range {
      const void* data;
      std::size_t count;
      std::size_t element;
    };
    const Range ranges[]{
        {storage.path_indices.data(), storage.path_indices.capacity(),
         sizeof(std::uint32_t)},
        {storage.pairs.data(), storage.pairs.capacity(),
         sizeof(CanonicalPair)},
        {storage.vertex_ledger.data(), storage.vertex_ledger.capacity(),
         sizeof(VertexLedgerRow)},
        {storage.published.data(), storage.published.capacity(),
         sizeof(RepresentedIntervalResult)},
        {storage.staging.data(), storage.staging.capacity(),
         sizeof(RepresentedIntervalResult)}};
    for (const auto& range : ranges) {
      std::size_t owned_bytes = 0;
      if (!MultiplySize(range.count, range.element, &owned_bytes) ||
          !RangeDisjoint(data, bytes, range.data, owned_bytes))
        return false;
    }
    return true;
  };
  if (!MultiplySize(path_count, sizeof(*paths), &path_bytes) ||
      !MultiplySize(pair_count, sizeof(*pairs), &pair_bytes) ||
      !RangeDisjoint(paths, path_bytes, pairs, pair_bytes) ||
      !disjoint_from_owned(paths, path_bytes) ||
      !disjoint_from_owned(pairs, pair_bytes)) {
    report.status = RepresentedIntervalStatus::InvalidInput;
    report.message = Message(report.status);
    return report;
  }

  storage.path_indices.clear();
  storage.vertex_ledger.clear();
  for (std::size_t i = 0; i < path_count; ++i) {
    const auto status = ValidatePath(paths[i]);
    if (status != RepresentedIntervalStatus::Ok) {
      report.status = status;
      report.input_path = i;
      report.message = Message(status);
      return report;
    }
    storage.path_indices.push_back(static_cast<std::uint32_t>(i));
    for (const auto& vertex : paths[i].vertices)
      storage.vertex_ledger.push_back(
          {vertex.key, {vertex.endpoint[0], vertex.endpoint[1]},
           paths[i].motion, i});
  }
  std::sort(storage.path_indices.begin(), storage.path_indices.end(),
            [&](std::uint32_t a, std::uint32_t b) {
              return Compare(paths[a].key, paths[b].key) < 0;
            });
  for (std::size_t i = 1; i < storage.path_indices.size(); ++i) {
    const auto previous = storage.path_indices[i - 1];
    const auto current = storage.path_indices[i];
    if (Same(paths[previous].key, paths[current].key) &&
        !CompatiblePath(paths[previous], paths[current])) {
      report.status = RepresentedIntervalStatus::IdentityMismatch;
      report.input_path = current;
      report.message = Message(report.status);
      return report;
    }
  }
  std::sort(storage.vertex_ledger.begin(), storage.vertex_ledger.end(),
            VertexLedgerLess);
  for (std::size_t i = 1; i < storage.vertex_ledger.size(); ++i) {
    const auto& previous = storage.vertex_ledger[i - 1];
    const auto& current = storage.vertex_ledger[i];
    if (Compare(previous.key, current.key) == 0 &&
        (previous.motion != current.motion ||
         !SameBits(previous.endpoint[0], current.endpoint[0]) ||
         !SameBits(previous.endpoint[1], current.endpoint[1]))) {
      report.status = RepresentedIntervalStatus::IdentityMismatch;
      report.input_path = current.input_path;
      report.message = "inconsistent vertex trajectory identity";
      return report;
    }
  }

  storage.pairs.clear();
  for (std::size_t i = 0; i < pair_count; ++i) {
    if (pairs[i].first >= path_count || pairs[i].second >= path_count ||
        pairs[i].first == pairs[i].second ||
        Same(paths[pairs[i].first].key, paths[pairs[i].second].key)) {
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
    storage.staging.resize(storage.pairs.size());
  } catch (...) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message = Message(report.status);
    storage.staging.clear();
    return report;
  }
  for (std::size_t pair = 0; pair < storage.pairs.size(); ++pair)
    storage.pair_status[pair].complete = false;
  if (!storage.RunWorkers(paths, storage.pairs.size())) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message =
        "represented interval crossing persistent worker execution failed";
    storage.staging.clear();
    return report;
  }
  for (std::size_t pair_index = 0;
       pair_index < storage.pairs.size(); ++pair_index) {
    const auto& pair = storage.pairs[pair_index];
    if (!storage.pair_status[pair_index].complete) {
      report.status = RepresentedIntervalStatus::ResourceLimit;
      report.input_pair = pair.input_pair;
      report.message =
          "represented interval crossing worker result is incomplete";
      storage.staging.clear();
      return report;
    }
    const auto& result = storage.staging[pair_index];
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
  if (!impl_ || impl_->busy.load(std::memory_order_acquire))
    return {};
  return {impl_->published.data(), impl_->published.size(), impl_->complete};
}

}  // namespace tlfea::contact
