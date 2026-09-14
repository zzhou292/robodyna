// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../FixedTriangleFeatureTypes.h"

namespace tlfea::contact::fixed_triangle_features {

struct PairFeatureResult {
  std::size_t feature_tasks = 0;
  std::size_t feature_count = 0;
  std::size_t input_task = SIZE_MAX;
  FixedTriangleArithmeticReason arithmetic_reason =
      FixedTriangleArithmeticReason::None;
};

int Compare(const FacetVertexKey& a, const FacetVertexKey& b) noexcept;
int Compare(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept;
int Compare(const FixedTriangleKey& a, const FixedTriangleKey& b) noexcept;
int Compare(const FixedTriangleStratumKey& a,
            const FixedTriangleStratumKey& b) noexcept;
int Compare(const FixedTriangleFeatureKey& a,
            const FixedTriangleFeatureKey& b) noexcept;
bool FeatureLess(const FixedTriangleFeatureCandidate& a,
                 const FixedTriangleFeatureCandidate& b) noexcept;
bool SameFeatureKey(const FixedTriangleFeatureCandidate& a,
                    const FixedTriangleFeatureCandidate& b) noexcept;
bool IntersectionLess(const FixedTriangleIntersection& a,
                      const FixedTriangleIntersection& b) noexcept;
bool SameIntersectionPair(const FixedTriangleIntersection& a,
                          const FixedTriangleIntersection& b) noexcept;

bool SameTriangleValue(const CurrentFixedTriangle& a,
                       const CurrentFixedTriangle& b) noexcept;
FixedTriangleDiscoveryStatus ValidateTriangle(
    const CurrentFixedTriangle& value) noexcept;
std::size_t CountPairFeatureCandidates(
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second) noexcept;
FixedTriangleFeatureTaskMask PairLocalFeatureTaskMask(
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second) noexcept;
FixedTriangleDiscoveryStatus EvaluatePairFeaturesOnce(
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second,
    FixedTriangleFeatureCandidate* output,
    std::size_t output_capacity,
    PairFeatureResult* result) noexcept;
FixedTriangleDiscoveryStatus EvaluatePairFeaturesMaskedOnce(
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second,
    FixedTriangleFeatureTaskMask mask,
    FixedTriangleFeatureCandidate* output,
    std::size_t output_capacity,
    PairFeatureResult* result) noexcept;
FixedTriangleDiscoveryStatus ClassifyPairIntersection(
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second,
    FixedTriangleIntersection* output,
    bool* intersects) noexcept;

}  // namespace tlfea::contact::fixed_triangle_features
