// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../QephBatchDiagnostics.h"
#include "Result.h"

namespace tl::fea::qeph::mapped {
// Accepted assembly has finished before candidate evaluation. Reuse its typed
// transient records until the next assembly overwrites them; no pointer casts,
// lifetime changes, retained mechanics or additional arena are involved.
TL_QEPH_HD inline void PrepareDiagnosticParent(const batch_detail::Model& model,
    const ForceTrial& result,Status status,const ShellSectionLaw* roles,std::size_t parent,
    const NodalPreparedView& view,AssemblyParent& scratch) noexcept {
  scratch.status=BatchStatus::Success;
  // A failed element may leave an unwritten trial packet. Its original status
  // always outranks result validation, so never inspect that packet here.
  if(status!=Status::kSuccess || !roles) return;
  if(!ValidResult(model.element[parent].reference,result,view.proposed_time,
      view.kinematics.base_epoch+1,roles[parent]==ShellSectionLaw::RigidSkin))
    scratch.status=BatchStatus::NonfiniteResult;
}
TL_QEPH_HD inline void PrepareDiagnosticNode(const batch_detail::Model& model,
    const NodalPreparedView& view,std::size_t node,AssemblyNode& scratch) noexcept {
  using namespace shell_batch_fields;
  const auto dx=Difference(ReadVector(view.kinematics.position_xyz,node),model.initial_position[node]);
  scratch.value[0]=::hypot(::hypot(dx.x,dx.y),dx.z);
  scratch.touched=false;
  if(!tl::math::Finite(scratch.value[0])) return;
  const auto* q=view.kinematics.orientation_wxyz+4*node;
  scratch.touched=tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]});
}
TL_QEPH_HD inline void FinalizeDiagnostics(const batch_detail::Storage& storage,
    const batch_detail::Slab& accepted,const batch_detail::Slab& trial,
    const NodalPreparedView& view,BatchDiagnostics identity,const ShellSectionLaw* roles,
    batch_detail::Control& out) noexcept {
  out={};out.diagnostics=identity;
  const auto& model=storage.model;
  for(unsigned parent=0;parent<model.config.element_count;++parent) {
    const auto status=storage.candidate_status[parent];
    if(status!=Status::kSuccess) {
      out.status=BatchStatus::ElementFailure;out.element=parent;out.element_status=status;return;
    }
  }
  if(!roles) {out.status=BatchStatus::InvalidInput;return;}
  for(unsigned parent=0;parent<model.config.element_count;++parent) {
    if(storage.assembly.parent[parent].status!=BatchStatus::Success) {
      out.status=BatchStatus::NonfiniteResult;out.element=parent;return;
    }
  }
  auto& d=out.diagnostics;
  if(d.kinetic_available==model.joined) {out.status=BatchStatus::NonfiniteResult;return;}
  // Qualified mapped participants are joined and report no kinetic sums. Keep
  // the existing whole-owner arithmetic for any nonjoined internal caller.
  if(d.kinetic_available) {
    if(!batch_detail::Measure(model,accepted,trial,view,out,roles)) out.status=BatchStatus::NonfiniteResult;
    else d.valid=true;
    return;
  }
  for(unsigned node=0;node<model.config.owner.node_count;++node) {
    const auto& scratch=storage.assembly.node[node];
    if(!tl::math::Finite(scratch.value[0])) {out.status=BatchStatus::NonfiniteResult;return;}
    d.maximum_displacement=::fmax(d.maximum_displacement,scratch.value[0]);
    if(!scratch.touched) {out.status=BatchStatus::NonfiniteResult;return;}
  }
  if(!batch_detail::MeasureParents(model,accepted,trial,view,out,roles)) {
    out.status=BatchStatus::NonfiniteResult;return;
  }
  d.valid=true;
}
} // namespace tl::fea::qeph::mapped
