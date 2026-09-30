#pragma once

#include "Q4PlanarSweepGeometry.h"
#include "Q4RectangularIntegrationTypes.h"

namespace tlfea::contact {
struct Q4PrescribedPlanarConfig {
  double stiffness_per_area = 0, maximum_penetration = 0;
  Q4IntegrationLimits integration;
  Q4PlanarSweepLimits sweep;
};
enum class Q4PrescribedPlanarStatus {
  Ok, InvalidInput, GeometryFailure, MassFailure, FixedMotion,
  IntegrationFailure, NonFiniteArithmetic
};
struct Q4PrescribedPlanarReport {
  Q4PrescribedPlanarStatus status = Q4PrescribedPlanarStatus::InvalidInput;
  const char* message = "Invalid request";
  std::uint32_t parent = UINT32_MAX;
  PlanarContactReport geometry;
  Status mass_status = Status::kInvalidArgument;
  Q4IntegrationReport integration;
};
struct Q4PrescribedPlanarResult {
  Q4PlanarSweepGeometry geometry;
  std::array<Q4RectangularResult,MaxQ4PlanarParents> parents{};
  bool valid = false;
};

// Host-only, bounded prescribed geometry/force operation. Reference originates
// from Q4PlanarGeometry::InitializeReference against this same finite -X wall.
// Every parent must pass the C5a swept box and projected Jacobian proofs before
// integration. Positive wall projection is this contact chart's restriction,
// not a shell-quality or physical shell-inversion criterion. The physical mass
// is the actual free-XYZ/fully-fixed nodal view;
// partial component constraints are unsupported, never recast as fixed Y/Z.
// Fully fixed node positions must agree between the supplied endpoints and both
// velocities must vanish. This checks the supplied interval, not state history.
// Both endpoint penetration bounds are checked; linear nodal motion and positive
// Q4 shapes then bound penetration throughout the prescribed interval.
//
// Integrate endpoint forces/potential using immutable rectangular material A0,
// zero offset and the existing undamped normal law. Changing Y/Z does not change
// this measure or add tangential traction. All direct nodal couples are zero;
// a global moment uses CURRENT nodal positions crossed with returned forces.
// Per-parent outputs retain natural source nodes; shared nodes are not merged
// or written here. Exact-coordinate area bounds expand the existing certificates
// once, and the original absolute N/J budgets are checked again afterward.
//
// One caller-owned rectangular scratch is reused serially for <=2 parents.
// Scratch may change on failure; every caller result field remains unchanged.
// No hidden allocations, owner, assembly, time advance, stream or CUDA state
// admission. Pointee lifetime/length/nonaliasing and source provenance remain
// caller contracts. Successful prescribed evaluation is not a dynamics receipt.
Q4PrescribedPlanarReport IntegrateQ4PrescribedPlanarContact(
    const PlanarWallGeometry& wall,Q4PlanarReferenceView reference,
    const Q4SurfaceView& base,const Q4SurfaceView& endpoint,
    const LumpedTranslationMassView& mass,const Q4PrescribedPlanarConfig& config,
    std::uint64_t attempt,Q4RectangularScratch scratch,Q4PrescribedPlanarResult* output);
}  // namespace tlfea::contact
