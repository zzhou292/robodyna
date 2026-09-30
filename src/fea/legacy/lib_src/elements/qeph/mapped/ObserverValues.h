// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CandidateDiagnostics.h"
#include "ObserverSums.h"

namespace tl::fea::qeph::mapped {
TL_QEPH_HD inline void ObserveParent(const batch_detail::Storage& storage,
    const batch_detail::Slab& accepted,const batch_detail::Slab& trial,
    const NodalPreparedView& view,const ShellSectionLaw* roles,bool assembled,
    std::size_t parent,ObserverSummary& out) noexcept {
  if(storage.candidate_status[parent]!=Status::kSuccess || !roles ||
      storage.assembly.parent[parent].status!=BatchStatus::Success) {out.serial=true;return;}
  if(roles[parent]==ShellSectionLaw::RigidSkin) return;
  const auto& element=storage.model.element[parent];
  const auto& result=trial.element[parent];
  const auto& history=result.proposed_history.data();
  const double area=result.kinematics.area/element.reference.area;
  const double thickness=history.thickness/element.reference.input.thickness;
  const double dt=result.diagnostics.unscaled_element_dt;
  // Nonpositive native dt remains in the old admitted domain. Its signed-zero
  // minimum/tie behavior is intentionally left to the serial fallback.
  if(!detail::Positive(area)||!detail::Positive(thickness)||!detail::Positive(dt)) {out.serial=true;return;}
  out.minimum_area=out.material_count?::fmin(out.minimum_area,area):area;
  out.minimum_thickness=out.material_count?::fmin(out.minimum_thickness,thickness):thickness;
  out.minimum_dt=out.material_count?::fmin(out.minimum_dt,dt):dt;
  ++out.material_count;
  for(unsigned c=0;c<2;++c) {
    AddObservation(out,c,history.internal_work[c]);
    AddObservation(out,2+c,result.diagnostics.internal_work_increment[c]);
  }
  AddObservation(out,4,history.hourglass_viscous_work);
  AddObservation(out,5,result.diagnostics.hourglass_viscous_work_increment);
  for(unsigned c=0;c<5;++c)
    out.maximum_strain=::fmax(out.maximum_strain,::fabs(history.strain_curvature[c]));
  for(unsigned c=5;c<8;++c)
    out.maximum_curvature=::fmax(out.maximum_curvature,::fabs(element.reference.input.thickness*history.strain_curvature[c]));
  if(!tl::math::Finite(out.maximum_curvature)) out.serial=true;
  if(assembled) {
    WorkObservation kick{out,6},drift{out,7};
    const auto& old=accepted.element[parent];
    shell_batch_fields::AccumulateInternalWork(element.nodes,old.internal_force,old.internal_couple,
        view,storage.model.config.owner.fixed_dt,kick,drift);
  }
}
TL_QEPH_HD inline void ObserveNode(const batch_detail::Storage& storage,
    std::size_t node,ObserverSummary& out) noexcept {
  const auto& value=storage.assembly.node[node];
  if(!tl::math::Finite(value.value[0])||!value.touched) {out.serial=true;return;}
  out.maximum_displacement=::fmax(out.maximum_displacement,value.value[0]);
}
TL_QEPH_HD inline bool ApplyObservations(const batch_detail::Model& model,
    const ObserverSummary& summary,BatchDiagnostics& diagnostics) noexcept {
  if(summary.serial || !model.joined || diagnostics.kinetic_available) return false;
  double* const fields[]={&diagnostics.internal_work[0],&diagnostics.internal_work[1],
    &diagnostics.internal_work_increment[0],&diagnostics.internal_work_increment[1],
    &diagnostics.hourglass_viscous_work,&diagnostics.hourglass_viscous_work_increment,
    &diagnostics.internal_kick_work,&diagnostics.internal_drift_work};
  double largest=summary.maximum_term;
  for(auto* field:fields) {
    if(!tl::math::Finite(*field)) return false;
    largest=::fmax(largest,::fabs(*field));
  }
  if(!FiniteObserverPrefixes(model.config.element_count,largest)) return false;
  if(summary.material_count) {
    for(unsigned c=0;c<ObserverChannels;++c)
      if(c<6 || diagnostics.accepted_force_assembled) *fields[c]+=summary.sum[c];
    diagnostics.minimum_area_ratio=summary.minimum_area;
    diagnostics.minimum_thickness_ratio=summary.minimum_thickness;
    diagnostics.minimum_native_dt=summary.minimum_dt;
    diagnostics.maximum_absolute_strain=::fmax(diagnostics.maximum_absolute_strain,summary.maximum_strain);
    diagnostics.maximum_thickness_curvature=::fmax(diagnostics.maximum_thickness_curvature,summary.maximum_curvature);
  }
  diagnostics.maximum_displacement=::fmax(diagnostics.maximum_displacement,summary.maximum_displacement);
  const double finite[]={diagnostics.kinetic_translation,diagnostics.kinetic_rotation,
    diagnostics.kinetic_physical_isotropic,diagnostics.kinetic_added_isotropic,
    diagnostics.minimum_area_ratio,diagnostics.minimum_thickness_ratio,diagnostics.minimum_native_dt,
    diagnostics.maximum_displacement,diagnostics.maximum_absolute_strain,diagnostics.maximum_thickness_curvature};
  for(double value:finite) if(!tl::math::Finite(value)) return false;
  for(auto* field:fields) if(!tl::math::Finite(*field)) return false;
  diagnostics.valid=true;
  return true;
}
TL_QEPH_HD inline void FinalizeObservations(const batch_detail::Storage& storage,
    const batch_detail::Slab& accepted,const batch_detail::Slab& trial,
    const NodalPreparedView& view,BatchDiagnostics identity,const ShellSectionLaw* roles,
    const ObserverSummary& summary,batch_detail::Control& output) noexcept {
  batch_detail::Control next;
  next.diagnostics=identity;
  if(!roles || !ApplyObservations(storage.model,summary,next.diagnostics)) {
    FinalizeDiagnostics(storage,accepted,trial,view,identity,roles,output);
    return;
  }
  output=next;
}
} // namespace tl::fea::qeph::mapped
