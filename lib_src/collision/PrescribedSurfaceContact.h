#pragma once

#include "Q4ParametricContact.h"
#include "T3ContactIntegrationTypes.h"

namespace tlfea::contact {
enum class PrescribedSurfaceFamily { Unspecified, Q4CenterAreaUniformNatural, T3NativeLinear };
struct PrescribedQ4Request {
  const Q4ParametricReference* reference=nullptr;
  std::uint32_t reference_parent=0;
  const SurfaceQ4* base=nullptr;
  const SurfaceQ4* endpoint=nullptr;
};
struct PrescribedT3Request {
  const T3MaterialMeasure* reference=nullptr;
  const SurfaceTriangle* base=nullptr;
  const SurfaceTriangle* endpoint=nullptr;
};
struct PrescribedSurfaceParent {
  PrescribedSurfaceFamily family=PrescribedSurfaceFamily::Unspecified;
  PrescribedQ4Request q4;
  PrescribedT3Request t3;
};
struct PrescribedSurfaceInput {
  VectorView base_positions,base_velocities,endpoint_positions,endpoint_velocities;
  LumpedTranslationMassView mass;
  const PrescribedSurfaceParent* parents=nullptr;
  std::uint32_t parent_count=0;
};
struct PrescribedSurfaceConfig {
  double stiffness_per_area=0,maximum_penetration=0,exposed_clearance=0;
  Q4IntegrationLimits q4;
  T3IntegrationLimits t3;
};
enum class PrescribedSurfaceStatus {
  Ok,InvalidInput,ReferenceFailure,GeometryFailure,MassFailure,FixedMotion,
  IntegrationFailure,NonFiniteArithmetic
};
struct PrescribedSurfaceReport {
  PrescribedSurfaceStatus status=PrescribedSurfaceStatus::InvalidInput;
  const char* message="Invalid prescribed surface request";
  std::uint32_t parent=UINT32_MAX;
  PlanarContactReport geometry;
  PrescribedNodeReport node;
  Status mass_status=Status::kInvalidArgument;
  Q4IntegrationReport q4;
  T3IntegrationReport t3;
};
struct PrescribedSurfaceParentResult {
  PrescribedSurfaceFamily family=PrescribedSurfaceFamily::Unspecified;
  PlanarWallBoxCoverage coverage;
  Q4RectangularResult q4;
  T3IntegrationResult t3;
};
struct PrescribedSurfaceResult {
  std::array<PrescribedSurfaceParentResult,MaxQ4PlanarParents> parents{};
  std::uint32_t parent_count=0;
  bool valid=false;
};

// Bounded host composition, one common physical node space, at most two parents.
// Exactly one native request arm is populated per parent. Q4 borrows the whole
// immutable reference plus index, retaining its prepared global node count and
// named center-area measure; T3 borrows its native immutable reference. Both
// supplied endpoint bindings must match the selected reference exactly. Source
// parent/feature identities are unique even across element families.
//
// Every selected family, view/identity, mass stencil, fixed-node motion, nodal
// endpoint cap and finite-wall swept box is checked before any integration.
// Then dispatch the existing Q4 rectangular integral/one area expansion or the
// native T3 integral without changing arithmetic or declared family budgets.
// Coverage proves no current intrinsic regularity or shell-quality property.
//
// One caller-owned Q4 scratch is shared serially (unused for T3-only requests).
// Outputs, including the other family's inactive fields, are staged together;
// a later failure never publishes an earlier parent's result. Scratch may change
// after preflight. No reference/state owner, hidden allocation, physical scatter,
// wall dynamics, stream, clock or dynamics admission is introduced. Lifetimes,
// disjoint outputs/pointees and immutable borrowed reference data are caller
// contracts. Shared physical node forces remain separate parent contributions.
PrescribedSurfaceReport IntegratePrescribedSurfaceContact(
    const PlanarWallGeometry&,const PrescribedSurfaceInput&,const PrescribedSurfaceConfig&,
    std::uint64_t attempt,Q4RectangularScratch,PrescribedSurfaceResult*);
} // namespace tlfea::contact
