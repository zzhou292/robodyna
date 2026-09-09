#pragma once

#include "PlanarWallGeometry.h"
#include "Q4PlanarGeometry.h"

#include <array>

namespace tlfea::contact {

struct Q4PlanarSweepLimits {
  double exposed_clearance = 0;
  // Declared geometric admissibility floor, not a contact-law or accuracy
  // tolerance. Positive determinant is measured relative to reference A0/4.
  double minimum_projected_jacobian_ratio = 1e-6;
};
struct Q4PlanarParentSweep {
  SurfaceQ4 parent;
  Vec3 projected_minimum, projected_maximum;  // Both X coordinates are wall X.
  Q4IntegralInterval projected_jacobian;      // d(Y,Z)/d(u,v), throughout u,v,t.
  Q4IntegralInterval projected_jacobian_ratio;
  double required_jacobian_lower = 0;
  // Copied unchanged from the immutable, rectangular C3 reference. The swept
  // box area and current projected area NEVER replace the material measure.
  double reference_material_area = 0;
  Q4IntegralInterval reference_material_area_enclosure;
};
struct Q4PlanarSweepGeometry {
  std::array<Q4PlanarParentSweep,MaxQ4PlanarParents> parents{};
  std::uint32_t parent_count = 0;
  bool valid = false;
};

// Host-only prescribed geometry query, no state/owner/time/stream/force/mass.
// Reference must originate from immutable Q4PlanarGeometry preparation against
// this same finite -X wall. Natural order is (+,+),(-,+),(-,-),(+,-) in Y/Z.
// Source identity/connectivity and zero offset are checked at both endpoints;
// finite nonzero Y/Z velocities and changing Y/Z coordinates are permitted.
//
// Each node follows the straight segment base->candidate, t in [0,1]. Positive
// Q4 shape functions place the complete projected sweep inside the eight-corner
// YZ AABB. Both triangles of that box must remain covered and clear of exposed
// boundaries/holes according to the existing finite-wall classifier.
//
// Separately, the projected Jacobian is affine in (u,v) and quadratic in t.
// Outward Bernstein coefficients at all four parameter corners bound the whole
// sweep. If their lower hull cannot prove the requested positive ratio, return
// UnsupportedMotion: this may be conservative, and does NOT establish folding.
// No subdivision or endpoint-only fallback changes that declared decision.
//
// This admits no finite-edge crossing, warped reference measure, thickness skin,
// self-contact, arbitrary curved node trajectory or CCD claim. Current normal
// warping and moderate projected rotation/shear can be admitted. The old C3
// fixed-YZ API is unchanged. Every output field is preserved on any failure;
// pointee lengths, lifetimes, producer provenance and nonaliasing are caller
// contracts. Failure reports identify the parent through report.sample.
PlanarContactReport CheckQ4PlanarSweep(
    const PlanarWallGeometry& wall,Q4PlanarReferenceView reference,
    const Q4SurfaceView& base,const Q4SurfaceView& candidate,
    const Q4PlanarSweepLimits& limits,Q4PlanarSweepGeometry* output);

} // namespace tlfea::contact
