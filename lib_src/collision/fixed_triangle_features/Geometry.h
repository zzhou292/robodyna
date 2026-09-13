// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../FixedTriangleFeatureTypes.h"

namespace tlfea::contact::fixed_triangle_features {

struct PairResult {
  FixedTriangleFeatureCandidate features[15];
  std::size_t feature_task_count = 0;
  std::size_t feature_count = 0;
  FixedTriangleIntersection intersection;
  bool intersects = false;
};

int Compare(const FacetVertexKey& a, const FacetVertexKey& b) noexcept;
int Compare(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept;
int Compare(const FixedTriangleKey& a, const FixedTriangleKey& b) noexcept;
bool FeatureLess(const FixedTriangleFeatureCandidate& a,
                 const FixedTriangleFeatureCandidate& b) noexcept;
bool SameFeatureTask(const FixedTriangleFeatureCandidate& a,
                     const FixedTriangleFeatureCandidate& b) noexcept;
bool IntersectionLess(const FixedTriangleIntersection& a,
                      const FixedTriangleIntersection& b) noexcept;
bool SameIntersectionPair(const FixedTriangleIntersection& a,
                          const FixedTriangleIntersection& b) noexcept;

FixedTriangleDiscoveryStatus EvaluatePair(const CurrentFixedTriangle& first,
                                          const CurrentFixedTriangle& second,
                                          PairResult* output) noexcept;

}  // namespace tlfea::contact::fixed_triangle_features
