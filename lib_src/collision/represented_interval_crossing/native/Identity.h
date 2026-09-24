// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Modes.h"
#include <algorithm>
#include <cstring>
#include <tuple>
namespace tlfea::contact::represented_interval_crossing::native {
template <class T>
inline int ScalarCompare(const T& a, const T& b) noexcept {
  return a < b ? -1 : (b < a ? 1 : 0);
}

inline int Compare(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  const auto aa =
      std::tie(a.source_instance_id, a.kind, a.first, a.second, a.numerator,
               a.denominator, a.level, a.grid_i, a.grid_j);
  const auto bb =
      std::tie(b.source_instance_id, b.kind, b.first, b.second, b.numerator,
               b.denominator, b.level, b.grid_i, b.grid_j);
  return aa < bb ? -1 : (bb < aa ? 1 : 0);
}

inline int Compare(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept {
  int value = ScalarCompare(a.parent_boundary, b.parent_boundary);
  if (!value)
    value = ScalarCompare(a.parent_eid, b.parent_eid);
  if (!value)
    value = Compare(a.endpoints[0], b.endpoints[0]);
  if (!value)
    value = Compare(a.endpoints[1], b.endpoints[1]);
  return value;
}

inline int Compare(const RepresentedTrianglePathKey& a,
            const RepresentedTrianglePathKey& b) noexcept {
  const auto aa =
      std::tie(a.source_instance_id, a.parent_eid, a.level, a.local_facet);
  const auto bb =
      std::tie(b.source_instance_id, b.parent_eid, b.level, b.local_facet);
  return aa < bb ? -1 : (bb < aa ? 1 : 0);
}

inline int Compare(const RepresentedIntervalPairKey& a,
            const RepresentedIntervalPairKey& b) noexcept {
  const int first = Compare(a.paths[0], b.paths[0]);
  return first ? first : Compare(a.paths[1], b.paths[1]);
}

inline bool Same(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  return Compare(a, b) == 0;
}

inline bool Same(const RepresentedTrianglePathKey& a,
          const RepresentedTrianglePathKey& b) noexcept {
  return Compare(a, b) == 0;
}

inline std::uint64_t CoordinateBits(double value) noexcept {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

inline bool SameBits(double a, double b) noexcept {
  return CoordinateBits(a) == CoordinateBits(b);
}

// Unlike floating ==, this preserves a nonzero subnormal under ambient DAZ.
// Inputs were authenticated finite; only the two real-zero encodings merge.
inline bool SameFiniteCoordinate(double a, double b) noexcept {
  const auto first = CoordinateBits(a), second = CoordinateBits(b);
  constexpr std::uint64_t magnitude = UINT64_MAX >> 1;
  return first == second || ((first & magnitude) == 0 && (second & magnitude) == 0);
}

inline bool SameBits(Vec3 a, Vec3 b) noexcept {
  return SameBits(a.x, b.x) && SameBits(a.y, b.y) &&
         SameBits(a.z, b.z);
}

inline bool FeatureLess(const RepresentedFeaturePathKey& a,
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

inline void ConsiderFeature(const RepresentedFeaturePathKey& candidate,
                     bool* have, RepresentedFeaturePathKey* result) noexcept {
  if (!*have || FeatureLess(candidate, *result)) {
    *result = candidate;
    *have = true;
  }
}

inline int ReasonPriority(RepresentedIntervalReason reason) noexcept {
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

inline void CopyKey(const FacetVertexKey& source,
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

inline void CopyKey(const FacetEdgeKey& source,
             FacetEdgeKey* target) noexcept {
  CopyKey(source.endpoints[0], &target->endpoints[0]);
  CopyKey(source.endpoints[1], &target->endpoints[1]);
  target->parent_eid = source.parent_eid;
  target->parent_boundary = source.parent_boundary;
}

inline void CopyKey(const RepresentedTrianglePathKey& source,
             RepresentedTrianglePathKey* target) noexcept {
  target->source_instance_id = source.source_instance_id;
  target->parent_eid = source.parent_eid;
  target->level = source.level;
  target->local_facet = source.local_facet;
}

inline void StoreResult(const RepresentedIntervalResult& source,
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

inline RepresentedIntervalResult Unresolved(const RepresentedIntervalPairKey& key,
                                     RepresentedIntervalReason reason,
                                     std::size_t work) noexcept {
  RepresentedIntervalResult result;
  result.key = key;
  result.classification = RepresentedIntervalClassification::Unresolved;
  result.reason = reason;
  result.work = work;
  return result;
}

inline unsigned CanonicalAnchor(const RepresentedTrianglePath& path) noexcept {
  unsigned result = 0;
  for (unsigned vertex = 1; vertex < 3; ++vertex)
    if (Compare(path.vertices[vertex].key, path.vertices[result].key) < 0) result = vertex;
  return result;
}

// Bit equality of finite binary64 coordinates (with signed zeros merged) is
// exact real equality independent of ambient FTZ/DAZ or rounding modes.
// Only the original path endpoints are queried;
// interior dyadic samples retain the original exact predicate traversal.
// Source identities are deliberately irrelevant to this geometric fact.
inline bool CommonEndpointPoint(const RepresentedTrianglePath& a,
                         const RepresentedTrianglePath& b, DyadicTime time,
                         CommonPointCounters* counters) noexcept {
  unsigned endpoint = 0;
  if (time.numerator != 0) {
    if (time.numerator != (std::uint64_t{1} << time.depth)) return false;
    endpoint = 1;
  }
  CountCommonPointOperation(counters, &CommonPointCounters::endpoint_queries);
  for (const auto& first : a.vertices) for (const auto& second : b.vertices) {
    CountCommonPointOperation(counters, &CommonPointCounters::point_comparisons);
    const auto p = first.endpoint[endpoint], q = second.endpoint[endpoint];
    if (SameFiniteCoordinate(p.x, q.x) && SameFiniteCoordinate(p.y, q.y) &&
        SameFiniteCoordinate(p.z, q.z)) return true;
  }
  return false;
}


}  // namespace tlfea::contact::represented_interval_crossing::native
