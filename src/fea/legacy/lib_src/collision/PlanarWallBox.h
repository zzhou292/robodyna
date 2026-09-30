#pragma once

#include "PlanarWallGeometry.h"

namespace tlfea::contact {

enum class PlanarWallBoxMode { Exact, ConservativeExpansion };
struct PlanarWallBox {
  Vec3 minimum,maximum;  // Projected WORLD Y/Z bounds; X is exactly wall X.
};
struct PlanarWallBoxCoverage {
  PlanarWallBox physical,query;
  // Outward bounds on query extension beyond each physical endpoint. X is 0.
  Vec3 lower_expansion_upper,upper_expansion_upper;
  PlanarWallBoxMode mode=PlanarWallBoxMode::Exact;
  bool covered=false;
};

// Host-only finite -X wall query. Exact retains the C5a box triangles and query
// order. ConservativeExpansion enlarges only zero/ill-conditioned Y/Z spans to
// a declared roundoff-scale query, allowing regular surfaces exactly edge-on
// to the wall. It never moves a physical point or changes a material measure.
// The actual query must still pass the existing exposed-edge/hole clearance
// checks. Neither mode admits finite-edge crossing or creates contact forces.
// Two triangle classifications, fixed cost bounded by existing wall capacities.
// A failed, outside or unresolved query preserves every output field.
PlanarContactReport CheckPlanarWallBox(
    const PlanarWallGeometry&,PlanarWallBox,double exposed_clearance,
    std::uint64_t feature_id,PlanarWallBoxMode,PlanarWallBoxCoverage*);

} // namespace tlfea::contact
