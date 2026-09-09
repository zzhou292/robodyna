#pragma once
#include "NodalWallContact.h"
#include "SurfaceContactLaw.h"
#include "SurfaceContactGeometry.h"

namespace tlfea::contact {
// One immutable area contribution at one real physical node. The successful
// preparation owns its provenance; this raw POD entry cannot authenticate a
// coordinated fabricated area. It also accepts a prepared assembled node area
// for independent model comparisons, never in addition to its parent shares.
// No per-parent budget is imposed on a single share: composition owns that
// accounting. All failures preserve caller output, including legacy helpers
// which clear THEIR local temporaries. IEEE RN/no fast-math/no FTZ required.
TL_SURFACE_HD inline NodalWallReport EvaluateNodalWallPoint(
    const NodalWallNodeWeight& weight,Vec3 position,Vec3 velocity,
    const LumpedTranslationMassView& mass,const NodalWallConfig& config,
    std::uint64_t attempt,NodalWallPointResult* output) {
  using Code=NodalWallStatus;
  using nodal_wall_detail::Report;
  const auto node=weight.node;
  if (!output || !attempt || !IsFinite(position) || !IsFinite(velocity) ||
      !IsFinite(config.wall_x) || !IsFinite(config.stiffness_per_area) || config.stiffness_per_area<=0 ||
      !IsFinite(config.maximum_penetration) || config.maximum_penetration<=0)
    return Report(Code::InvalidInput,Status::kInvalidArgument,node);
  if (!nodal_wall_detail::Certificate(weight.area,true))
    return Report(Code::InvalidReference,Status::kInvalidArgument,node);
  if (mass.model!=TranslationMassModel::kIsotropicLumped || !mass.inverse_mass || !mass.fixed || !mass.node_count)
    return Report(Code::MassFailure,Status::kInvalidArgument,node);
  auto status=mass_detail::CheckNode(mass,node);
  if (status!=Status::kOk) return Report(Code::MassFailure,status,node);
  Q4IntegralInterval depth;
  if (!q4_bounds::Difference(position.x,config.wall_x,&depth))
    return Report(Code::NonFiniteArithmetic,Status::kNonFiniteResult,node);
  if (depth.upper>config.maximum_penetration) return Report(Code::PenetrationLimit,Status::kInvalidArgument,node);
  NodalWallPointResult next;
  next.node=node; next.base_epoch=mass.base_epoch; next.attempt=attempt;
  next.wall_point={config.wall_x,position.y,position.z};
  next.touching_or_penetrating=position.x>=config.wall_x;
  next.fixed=mass.fixed[node]!=0;
  if (next.fixed) {
    if (velocity.x!=0 || velocity.y!=0 || velocity.z!=0) return Report(Code::FixedMotion,Status::kInvalidArgument,node);
    if (depth.upper>0) return Report(Code::FixedPenetration,Status::kNoDynamicDofs,node);
    // Exactly zero response, no invented dynamic mass/timestep/row.
    next.valid=true; *output=next; return Report(Code::Ok,Status::kOk,node);
  }
  Q4IntegralInterval stiffness;
  const double nominal_stiffness=config.stiffness_per_area*weight.area.value;
  if (!q4_bounds::Scale({weight.area.lower,weight.area.upper},config.stiffness_per_area,&stiffness) ||
      nominal_stiffness<=0 || !q4_bounds::Certify(nominal_stiffness,stiffness,&next.stiffness))
    return Report(Code::NonFiniteArithmetic,Status::kNonFiniteResult,node);
  const SignedNodeWeight stencil{node,1}; NormalJacobian jacobian;
  status=BuildNormalJacobian(mass,&stencil,1,{-1,0,0},attempt,&jacobian);
  if (status!=Status::kOk) return Report(Code::MassFailure,status,node);
  NormalContactResponse response;
  status=EvaluateNormalContact({nominal_stiffness,0,.8},
      {config.wall_x-position.x,-velocity.x,jacobian.inverse_effective_mass},&response);
  if (status!=Status::kOk) return Report(Code::NonFiniteArithmetic,status,node);
  const Q4IntegralInterval positive{depth.lower>0?depth.lower:0,depth.upper>0?depth.upper:0};
  Q4IntegralInterval force,energy;
  if (!q4_bounds::MultiplyPositive(stiffness,positive,&force) ||
      !q4_bounds::MultiplyPositive(force,positive,&energy) || !q4_bounds::Scale(energy,.5,&energy) ||
      !q4_bounds::Certify(response.force,force,&next.force) ||
      !q4_bounds::Certify(response.elastic_energy,energy,&next.potential))
    return Report(Code::NonFiniteArithmetic,Status::kNonFiniteResult,node);
  // Include the exact reference coefficient AND the used rounded coefficient;
  // a valid nominal estimate need not lie inside its truth interval.
  const double upper=stiffness.upper>nominal_stiffness?stiffness.upper:nominal_stiffness;
  status=tl::fea::stability::MakeRankOneContribution(jacobian,upper,0,&next.row);
  if (status!=Status::kOk) return Report(Code::NonFiniteArithmetic,status,node);
  next.force_world=Scale(jacobian.values[0],response.force); // Existing one-node J^T.
  next.wall_reaction=Scale(next.force_world,-1);
  next.wall_moment=geometry_detail::Cross(next.wall_point,next.wall_reaction);
  next.surface_power=Dot(next.force_world,velocity);
  next.local_velocity_first_timestep=response.stable_timestep;
  if (!IsFinite(next.force_world) || !IsFinite(next.wall_moment) || !IsFinite(next.surface_power))
    return Report(Code::NonFiniteArithmetic,Status::kNonFiniteResult,node);
  next.valid=true; *output=next; return Report(Code::Ok,Status::kOk,node);
}
} // namespace tlfea::contact
