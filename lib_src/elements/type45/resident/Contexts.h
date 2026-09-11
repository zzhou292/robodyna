// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
#include "../../../solvers/NodalTrialIdentity.h"

namespace tl::fea::type45::resident_detail {
// Private supplied-value check after the actual token-authenticated owner query.
// This helper neither makes a receipt nor grants permission to advance/commit.
inline BatchReport Contexts(const Model& model,const NodalStamp& accepted,const NodalPreparedView& view,
    std::uint64_t cin_id,const NodalCinPhysicalMainStamp& receipt,
    const NodalCinPhysicalMain* mains,std::size_t count,AutomaticStiffnessContext* output) noexcept {
  if(!model.prepared() || !mains || !output || !cin_id || accepted.epoch || accepted.time!=0 ||
      receipt.policy!=NodalCinPhysicalMainPolicy::PhysicalAggregateV1 || receipt.owner_id!=accepted.owner_id ||
      receipt.base_epoch!=accepted.epoch || receipt.attempt!=view.attempt ||
      receipt.cin_qualification_id!=cin_id || !detail::Same(receipt.base_time,accepted.time) ||
      !detail::Same(receipt.owner_fixed_dt,accepted.fixed_dt) ||
      !SameRigidGroupInfo(receipt.groups,accepted.rigid_groups) ||
      count!=model.rigid_binding()->groups().size())
    return {BatchStatus::InvalidInput,"Joint initial main receipt differs from the actual owner attempt"};
  const auto groups=model.rigid_binding()->groups();
  for(std::size_t i=0;i<count;++i) {
    const auto& a=mains[i];const auto& b=groups[i];
    const auto inertia=b.principal.inertia;
    const double minimum=::fmin(inertia.x,::fmin(inertia.y,inertia.z));
    if(a.source_kind!=b.source_kind || a.source_group_id!=b.source_id ||
        a.source_node_set_id!=b.source_node_set_id || !detail::Same(a.center_m,b.center) ||
        !detail::Same(a.mass_kg,b.mass_kg) || !detail::Same(a.minimum_principal_inertia_kg_m2,minimum) ||
        !detail::Positive(a.mass_kg) || !detail::Positive(a.minimum_principal_inertia_kg_m2) ||
        !detail::Nonnegative(a.translational_stiffness_n_per_m) || !detail::Nonnegative(a.rotational_stiffness_nm))
      return {BatchStatus::InvalidInput,"Joint physical main source association or values differ",
          SIZE_MAX,SIZE_MAX,Status::Success,NodalStatus::Ok,i};
  }
  const auto make=[&](const Joint& joint) {
    AutomaticStiffnessContext context;context.target_dt_s=receipt.owner_fixed_dt;
    for(unsigned e=0;e<2;++e) {
      const auto& main=mains[joint.body_groups[e]];
      context.main[e]={EndpointRole::RigidMember,main.source_group_id,main.center_m,main.mass_kg,
          main.minimum_principal_inertia_kg_m2,main.translational_stiffness_n_per_m,main.rotational_stiffness_nm};
    }
    return context;
  };
  // Check every reference before writing even this private context staging.
  for(std::size_t j=0;j<model.joints().size();++j) {
    const auto& row=model.joints()[j];
    if(row.body_groups[0]>=count || row.body_groups[1]>=count)
      return {BatchStatus::InvalidInput,"Joint body map is outside the complete main packet",j};
    Reference reference;
    const auto status=Reference::Prepare(row.property,row.geometry,row.damping,make(row),reference);
    if(status!=Status::Success)
      return {BatchStatus::JointFailure,"Joint initial automatic reference is invalid",j,SIZE_MAX,status};
  }
  for(std::size_t j=0;j<model.joints().size();++j) output[j]=make(model.joints()[j]);
  return {};
}
} // namespace tl::fea::type45::resident_detail
