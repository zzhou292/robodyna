#pragma once

#include "SurfaceMaterialMeasure.h"

namespace tlfea::contact {

// Native linear simplex; the immutable density comes from its actual three
// reference nodes. No fourth node, wall mass, current-area update or clock.
struct T3NormalIntegrationInput {
  const T3MaterialMeasure* reference=nullptr;
  LinearTriangleSurfaceView surface;
  LumpedTranslationMassView mass;
  std::uint32_t triangle_index=0;
  std::uint64_t attempt=0;
  double wall_x=0,stiffness_per_area=0,max_penetration=0;
};
struct T3IntegrationLimits {
  double force_error=0,energy_error=0; // Absolute N/J, including area uncertainty.
};
enum class T3IntegrationStatus {
  Ok,InvalidInput,InvalidReference,UnsupportedInput,NonFiniteArithmetic,
  NoDynamicDofs,PenetrationLimit,UnattainableAccuracy
};
struct T3IntegrationReport {
  T3IntegrationStatus status=T3IntegrationStatus::InvalidInput;
  Status cause=Status::kInvalidArgument;
  std::uint32_t node=UINT32_MAX,sample=UINT32_MAX;
};
struct T3IntegrationResult {
  TriangleNodalForces nodal; // Zero direct couples for this zero-offset mapping.
  Q4CertifiedIntegral force[3],resultant,potential; // Shared scalar certificate PODs.
  Q4IntegralInterval active_area;
  std::uint64_t feature_id=0,parent_element_id=0,base_epoch=0,attempt=0;
  std::uint32_t parent_face_id=0;
  // Nominal-gap partition diagnostics. Lower/upper truth fields can have
  // different positive-region topologies within the reported certificates.
  std::uint32_t subtriangle_count=0,sample_count=0;
  bool valid=false;
};

// Pure prescribed-state operation. At most two positive subtriangles and six
// positive degree-two quadrature samples, with fixed local storage and no heap.
// Both-endpoint fixed-node/cap checks and finite-wall swept coverage belong to
// the host adapter (PrescribedSurfaceInterval / PlanarWallBox), before this call.
// Input pointees and output must be disjoint and valid in the executing memory
// space. The surface inverse-mass pointer must equal mass.inverse_mass. Every
// failure preserves the complete caller result; no shared node scatter occurs.
// Bounds use Q4ContactBounds' IEEE binary64 RN/no-fast-math/no-FTZ contract.
TL_SURFACE_HD inline T3IntegrationReport IntegrateT3NormalContact(
    const T3NormalIntegrationInput&,const T3IntegrationLimits&,T3IntegrationResult*);

} // namespace tlfea::contact
