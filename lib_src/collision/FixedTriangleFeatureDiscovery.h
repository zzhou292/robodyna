// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "FixedTriangleFeatureTypes.h"
#include "SurfaceContactTypes.h"

#include <memory>

namespace tlfea::contact {

// Evaluate one immutable fixed facet at current nodal positions.  The complete
// output is preserved on failure.  This does not establish current regularity;
// Discover rejects every consumed degenerate triangle before publication.
Status EvaluateCurrentFixedTriangle(const FixedContactFacet& facet,
                                    VectorView positions,
                                    CurrentFixedTriangle* output) noexcept;

// Bounded host discovery for fixed physical triangles.  A query evaluates all
// six VF orientations and all nine EE pairs for every supplied triangle pair.
// It also performs a separate triangle/triangle intersection classification,
// because positive VF/EE boundary distances do not rule out a piercing.
//
// Published records are sorted and deduplicated by immutable source-feature
// keys, independent of pair/catalog order.  Every capacity is checked after a
// complete count; failure revokes publication and never returns a prefix.
// Geometry uses closed represented-coordinate boundaries: exact zero is a
// touch/coplanarity predicate and an adjacent nonzero representable value is
// not widened by a tolerance.  Current triangles at or below the
// SurfaceContactGeometry scale-aware degeneracy threshold reject the query.
// Input arrays and coordinates are borrowed for the duration of Discover and
// must not alias this object's storage or be concurrently mutated.
class FixedTriangleFeatureDiscovery {
 public:
  FixedTriangleFeatureDiscovery() noexcept;
  ~FixedTriangleFeatureDiscovery();
  FixedTriangleFeatureDiscovery(const FixedTriangleFeatureDiscovery&) = delete;
  FixedTriangleFeatureDiscovery& operator=(
      const FixedTriangleFeatureDiscovery&) = delete;

  static FixedTriangleFeaturePreflight Preflight(
      FixedTriangleFeatureLimits limits = {}) noexcept;
  FixedTriangleDiscoveryReport Initialize(
      FixedTriangleFeatureLimits limits = {}) noexcept;
  FixedTriangleDiscoveryReport Discover(
      const CurrentFixedTriangle* triangles, std::size_t triangle_count,
      const FixedTrianglePair* pairs, std::size_t pair_count) noexcept;

  bool initialized() const noexcept { return bool(impl_); }
  FixedTriangleFeatureForecast forecast() const noexcept;
  FixedTriangleFeatureView features() const noexcept;
  FixedTriangleIntersectionView intersections() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tlfea::contact
