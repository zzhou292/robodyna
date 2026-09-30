#pragma once

#include "SurfaceMaterialMeasure.h"
#include "PlanarWallBox.h"
#include "Q4PlanarGeometry.h" // Existing bounded parent capacity.
#include "Q4RectangularIntegrationTypes.h"
#include "PrescribedSurfaceInterval.h"

namespace tlfea::contact {
enum class Q4ReferenceContactMeasure { CenterAreaUniformNatural };
inline constexpr const char* Q4CenterAreaContactModel="center-area-uniform-natural-v1";
enum class Q4ParametricStatus {
  Ok, InvalidInput, ReferenceFailure, GeometryFailure, MassFailure, FixedMotion,
  IntegrationFailure, NonFiniteArithmetic
};
struct Q4ParametricReport {
  Q4ParametricStatus status=Q4ParametricStatus::InvalidInput;
  const char* message="Invalid request";
  std::uint32_t parent=UINT32_MAX;
  SurfaceMeasureStatus reference_status=SurfaceMeasureStatus::NotPrepared;
  PlanarContactReport geometry;
  PrescribedNodeReport node;
  Status mass_status=Status::kInvalidArgument;
  Q4IntegrationReport integration;
};
struct Q4ParametricParent {
  Q4MaterialMeasure intrinsic;
  Q4CertifiedIntegral area; // A0=4|Xu(0,0) x Xv(0,0)|, immutable m^2.
};
class Q4ParametricReference {
 public:
  Q4ParametricReport Initialize(VectorView,const SurfaceQ4*,std::uint32_t parent_count);
  bool prepared() const noexcept { return prepared_; }
  std::uint32_t parent_count() const noexcept { return parent_count_; }
  std::uint32_t node_count() const noexcept { return node_count_; }
  const Q4ParametricParent& parent(std::uint32_t index) const { return parents_[index]; }
  Q4ReferenceContactMeasure measure() const noexcept { return Q4ReferenceContactMeasure::CenterAreaUniformNatural; }
 private:
  std::array<Q4ParametricParent,MaxQ4PlanarParents> parents_{};
  std::uint32_t parent_count_=0,node_count_=0;
  bool prepared_=false;
};
struct Q4ParametricConfig {
  Q4ReferenceContactMeasure measure=Q4ReferenceContactMeasure::CenterAreaUniformNatural;
  double stiffness_per_area=0,maximum_penetration=0,exposed_clearance=0;
  Q4IntegrationLimits integration;
};
struct Q4ParametricResult {
  std::array<PlanarWallBoxCoverage,MaxQ4PlanarParents> coverage{};
  std::array<Q4RectangularResult,MaxQ4PlanarParents> parents{};
  Q4ReferenceContactMeasure measure=Q4ReferenceContactMeasure::CenterAreaUniformNatural;
  std::uint32_t parent_count=0;
  bool valid=false;
};

// Explicit discrete model dA0=(A0/4) du dv with immutable center-area A0.
// This is NOT the varying |Xu x Xv| surface integral for a warped reference,
// current projected area, native shell-card contact equivalence or a change to
// structural mass. Source IDs, cyclic ordering and intrinsic reference geometry
// are preserved. Reference preparation is independent of the wall orientation.
//
// All parents' exact source bindings, genuine mass, finite/fixed-node motion,
// both endpoint penetration caps and whole finite-wall swept coverage pass
// before any integration touches scratch. The box admits no finite-edge crossing
// and proves no current intrinsic regularity: shell mechanics owns that gate.
// There is no positive wall-projected Jacobian restriction; exactly edge-on and
// mixed projections use the existing reported conservative coverage query.
//
// Reuse the unchanged C2 rectangular parametric integral with actual nodal mass,
// bilinear X gaps, unchanged N/J budgets and J^T scatter. Expand the area bounds
// exactly once with the immutable exact-coordinate A0 certificate. Current Y/Z
// and velocities do not enter this frictionless potential; direct couples are
// zero, but global moments and power use current nodal positions/velocities.
// One caller-owned rectangular scratch is reused serially for <=2 parents.
// Output is unchanged on every rejection (scratch may change after preflight).
// No allocations, mechanics owner, assembly, state clock or dynamics receipt.
Q4ParametricReport IntegrateQ4ParametricContact(
    const PlanarWallGeometry&,const Q4ParametricReference&,
    const Q4SurfaceView& base,const Q4SurfaceView& endpoint,const LumpedTranslationMassView&,
    const Q4ParametricConfig&,std::uint64_t attempt,Q4RectangularScratch,Q4ParametricResult*);
} // namespace tlfea::contact
