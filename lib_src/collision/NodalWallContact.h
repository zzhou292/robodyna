#pragma once

#include "Q4ContactBounds.h"
#include "lib_src/solvers/ExplicitStepStability.h"
#include <array>

namespace tlfea::contact {
class Q4ParametricReference;
class T3MaterialMeasure;

inline constexpr const char* NodalWallContactModel="reference-area-lumped-nodal-wall-v1";
// Bounded prescribed operations only. These capacities do not change the
// physical nodal owner's capacity or admit an owner, stream, clock or history.
inline constexpr std::uint32_t MaxNodalWallParents=128,MaxNodalWallNodes=128;
enum class NodalWallStatus {
  Ok,InvalidInput,InvalidReference,Capacity,DuplicateParent,MassFailure,
  FixedMotion,FixedPenetration,PenetrationLimit,NonFiniteArithmetic,Accuracy
};
struct NodalWallReport {
  NodalWallStatus status=NodalWallStatus::InvalidInput;
  Status cause=Status::kInvalidArgument;
  std::uint32_t parent=UINT32_MAX,node=UINT32_MAX;
};
enum class NodalWallParentFamily { Q4CenterArea,T3Native };
// Exactly one immutable prepared reference. Neither arbitrary area scalars
// nor display triangles replace the owning reference-measure operations.
struct NodalWallParentInput {
  const Q4ParametricReference* q4=nullptr;
  std::uint32_t q4_parent=0;
  const T3MaterialMeasure* t3=nullptr;
};
struct NodalWallParentWeight {
  std::uint64_t parent_element_id=0,feature_id=0;
  std::uint32_t parent_face_id=0,arity=0,nodes[4]{};
  NodalWallParentFamily family=NodalWallParentFamily::Q4CenterArea;
  Q4CertifiedIntegral area,share; // Each native node receives A/arity, m^2.
};
struct NodalWallNodeWeight {
  std::uint32_t node=0;
  Q4CertifiedIntegral area; // Sum of the explicitly retained parent shares.
};
class NodalWallWeights {
 public:
  // Sorted parent identity order makes the arithmetic independent of input
  // order. Parent records retain native connectivity; nodes are sorted by ID.
  // Unused global nodes are legal and absent from node(). Empty selection is
  // invalid. Replacement is staged; no allocation and no writes on failure.
  NodalWallReport Initialize(std::uint32_t global_node_count,const NodalWallParentInput*,std::uint32_t count);
  bool prepared() const { return prepared_; }
  std::uint32_t node_count() const { return node_count_; }
  std::uint32_t parent_count() const { return parent_count_; }
  std::uint32_t global_node_count() const { return global_node_count_; }
  const NodalWallParentWeight& parent(unsigned i) const { return parents_[i]; }
  const NodalWallNodeWeight& node(unsigned i) const { return nodes_[i]; }
  Q4CertifiedIntegral total_area() const { return total_area_; }
 private:
  std::array<NodalWallParentWeight,MaxNodalWallParents> parents_{};
  std::array<NodalWallNodeWeight,MaxNodalWallNodes> nodes_{};
  Q4CertifiedIntegral total_area_;
  std::uint32_t node_count_=0,parent_count_=0,global_node_count_=0;
  bool prepared_=false;
};

struct NodalWallConfig {
  double wall_x=0,stiffness_per_area=0,maximum_penetration=0;
  // Applied separately to each PARENT's every force/resultant and potential.
  // Global and unique-node uncertainties are outward sums, not these limits.
  double parent_force_error=0,parent_energy_error=0;
};
struct NodalWallPointResult {
  Q4CertifiedIntegral force,potential,stiffness;
  Vec3 force_world,wall_point,wall_reaction,wall_moment;
  // The local timestep is a RAW SHARE's velocity-first diagnostic only. Host
  // composition sets it to zero; its all-active rows, plus structural rows,
  // are the inputs to a future scheme-specific coupled admission.
  double surface_power=0,local_velocity_first_timestep=0;
  tl::fea::stability::RowContribution row;
  std::uint64_t base_epoch=0,attempt=0;
  std::uint32_t node=0;
  bool fixed=false,touching_or_penetrating=false,valid=false;
};
struct NodalWallParentResult {
  Q4CertifiedIntegral force[4],resultant,potential;
  std::uint64_t parent_element_id=0,feature_id=0;
  std::uint32_t parent_face_id=0,arity=0;
  NodalWallParentFamily family=NodalWallParentFamily::Q4CenterArea;
  bool valid=false;
};
struct NodalWallResult {
  std::array<NodalWallParentResult,MaxNodalWallParents> parents{};
  std::array<NodalWallPointResult,MaxNodalWallNodes> nodes{};
  Q4CertifiedIntegral resultant,potential;
  Vec3 wall_reaction,wall_moment;
  double surface_power=0;
  std::uint64_t base_epoch=0,attempt=0;
  std::uint32_t parent_count=0,node_count=0;
  bool valid=false;
};

// Raw prescribed covered-plane operation: caller establishes actual finite
// mesh coverage and admissible swept/fixed positions separately. At this layer
// free XYZ or fully fixed isotropic mass only; fixed velocity must be zero and
// its exact-coordinate depth upper bound nonpositive. No friction, damping,
// thickness/director offset, direct couple, history or timestep admission.
// Every parent's energy/force is checked before one staged unique-node/global
// reduction. Shared contributions are added ONCE; summed area is not a second
// force. All fields of caller output remain unchanged on any failure.
NodalWallReport EvaluateNodalWallContact(const NodalWallWeights&,VectorView position,VectorView velocity,
    const LumpedTranslationMassView&,const NodalWallConfig&,std::uint64_t attempt,NodalWallResult*);

namespace nodal_wall_detail {
TL_SURFACE_HD inline bool Certificate(Q4CertifiedIntegral a,bool positive=false) {
  Q4CertifiedIntegral checked;
  return IsFinite(a.error) && a.error>=0 && (!positive || (a.value>0 && a.lower>0)) &&
      q4_bounds::Certify(a.value,{a.lower,a.upper},&checked) && checked.error<=a.error;
}
TL_SURFACE_HD inline NodalWallReport Report(NodalWallStatus status,Status cause=Status::kInvalidArgument,
                                          std::uint32_t node=UINT32_MAX) {
  return {status,cause,UINT32_MAX,node};
}
} // namespace nodal_wall_detail
} // namespace tlfea::contact
