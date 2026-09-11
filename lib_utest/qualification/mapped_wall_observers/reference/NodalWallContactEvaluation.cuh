#pragma once
#include "NodalWallContactStorage.h"

namespace tlfea::contact::nodal_wall_device_detail {
// Each point owns its compact result, incident share slots and status. The
// immutable incidence retains original parent/local order on every launch.
template<bool Physical=false>
__device__ inline void EvaluatePoint(Storage& s,const tl::fea::DeviceNodalKinematicsView& k,
    const NodalWallDiagnostics& identity,unsigned compact,const std::uint8_t* activity=nullptr) {
  const auto count=static_cast<std::uint32_t>(k.node_count);
  const VectorView x{k.position_xyz,count,3,1},v{k.velocity_xyz,count,3,1};
  const LumpedTranslationMassView mass{s.model.inverse_mass,s.model.fixed,count,k.base_epoch,
      TranslationMassModel::kIsotropicLumped};
  const unsigned n=s.model.nodes[compact].node; const auto position=x.at(n),velocity=v.at(n);
  auto& status=s.node_status[compact]; const auto initial=s.model.initial_position[n];
  if constexpr (Physical) {
    bool active=false;
    for(unsigned i=s.model.incident_offsets[compact];i<s.model.incident_offsets[compact+1];++i)
      active=active || activity[s.model.incident_slots[i]/4]!=0;
    if(!active) {
      NodalWallPointResult zero;
      zero.node=n;
      zero.base_epoch=k.base_epoch;
      zero.attempt=identity.attempt;
      zero.wall_point={s.model.config.law.wall_x,position.y,position.z};
      zero.valid=true;
      s.result.nodes[compact]=zero;
      for(unsigned i=s.model.incident_offsets[compact];i<s.model.incident_offsets[compact+1];++i)
        s.shares[s.model.incident_slots[i]]=zero;
      return;
    }
  }
  if (!Inside(position,s.model.coverage.physical) || !IsFinite(velocity)) Fail(status,Code::GeometryFailure,n);
  else if (s.model.fixed[n] && (position.x!=initial.x || position.y!=initial.y || position.z!=initial.z))
    Fail(status,Code::GeometryFailure,n);
  else {
    unsigned face=UINT32_MAX; TrianglePointGeometry point;
    if (s.model.query.FindOwner({s.model.config.law.wall_x,position.y,position.z},&face,&point)!=Status::kOk ||
        face==UINT32_MAX) Fail(status,Code::GeometryFailure,n);
    else {
      s.result.wall_face[compact]=s.model.face_ids[face]; NodalWallPointResult node;
      // Immutable source incidence avoids scanning all parents per node.
      // Stored slots retain the original p/l order and omit T3 padding.
      for (unsigned i=s.model.incident_offsets[compact];i<s.model.incident_offsets[compact+1];++i) {
        if (status.status!=Code::Ok) break;
        const unsigned slot=s.model.incident_slots[i],p=slot/4,l=slot%4;
        NodalWallPointResult share;
        NodalWallReport code{NodalWallStatus::Ok,Status::kOk};
        if constexpr (Physical) {
          if(activity[p]) code=EvaluatePhysicalWallPoint({n,s.model.parents[p].share},position,
              velocity,k.base_epoch,s.model.config.law,identity.attempt,&share);
          else {
            share.node=n;
            share.base_epoch=k.base_epoch;
            share.attempt=identity.attempt;
            share.wall_point={s.model.config.law.wall_x,position.y,position.z};
            share.valid=true;
          }
        } else {
          code=EvaluateNodalWallPoint({n,s.model.parents[p].share},position,velocity,
              mass,s.model.config.law,identity.attempt,&share);
        }
        if (code.status!=NodalWallStatus::Ok) { status.point=code; Fail(status,Code::PointFailure,n,p); }
        else if (!(Physical?nodal_wall_reduction::AddPhysicalShare(node,share):
            nodal_wall_reduction::AddShare(node,share))) Fail(status,Code::NonFiniteArithmetic,n,p);
        else s.shares[4*p+l]=share;
      }
      if (status.status==Code::Ok) {
        node.force_world={-node.force.value,0,0}; node.wall_reaction={node.force.value,0,0};
        node.wall_moment=geometry_detail::Cross(node.wall_point,node.wall_reaction);
        node.surface_power=Dot(node.force_world,velocity); node.local_velocity_first_timestep=0;
        if (!node.valid || !IsFinite(node.wall_moment) || !IsFinite(node.surface_power)) Fail(status,Code::NonFiniteArithmetic,n);
        else s.result.nodes[compact]=node;
      }
    }
  }
}
// Parent diagnostics read disjoint original share slots. Floating additions
// keep their original local-node order, including each certificate operation.
__device__ inline bool ReduceParent(Storage& s,unsigned p,Control& status) {
  auto& out=s.result.parents[p]; const auto& parent=s.model.parents[p];
  out.parent_element_id=parent.parent_element_id; out.parent_face_id=parent.parent_face_id;
  out.feature_id=parent.feature_id; out.family=parent.family; out.arity=parent.arity;
  // Stride four preserves the existing storage layout. A T3's fourth
  // share is never read; its unused force output remains zero from reset.
  for (unsigned l=0;l<parent.arity;++l) {
    const auto& share=s.shares[4*p+l]; out.force[l]=share.force;
    if (!nodal_wall_reduction::Sum(out.resultant,share.force) ||
        !nodal_wall_reduction::Sum(out.potential,share.potential)) { return Fail(status,Code::NonFiniteArithmetic,UINT32_MAX,p); }
  }
  bool accurate=out.resultant.error<=s.model.config.law.parent_force_error &&
                out.potential.error<=s.model.config.law.parent_energy_error;
  for (unsigned l=0;l<parent.arity;++l) accurate=accurate && out.force[l].error<=s.model.config.law.parent_force_error;
  if (!accurate) { return Fail(status,Code::Accuracy,UINT32_MAX,p); }
  out.valid=true;
  return true;
}
// Global signed sums deliberately remain in their original compact-node order.
template<bool Physical=false>
__device__ inline void ReduceNodes(Storage& s,const tl::fea::DeviceNodalKinematicsView& k) {
  const VectorView x{k.position_xyz,static_cast<std::uint32_t>(k.node_count),3,1};
  auto& d=s.result.diagnostics;
  if (s.control.status==Code::Ok) for (unsigned i=0;i<s.model.node_count;++i) {
    const auto& node=s.result.nodes[i];
    if (!nodal_wall_reduction::Sum(d.resultant,node.force) || !nodal_wall_reduction::Sum(d.potential,node.potential)) {
      Fail(s.control,Code::NonFiniteArithmetic,node.node); break;
    }
    d.wall_reaction=Add(d.wall_reaction,node.wall_reaction); d.wall_moment=Add(d.wall_moment,node.wall_moment);
    d.surface_power+=node.surface_power;
    const double penetration=x.at(node.node).x-s.model.config.law.wall_x;
    if ((!Physical || node.stiffness.value>0) && penetration>d.maximum_penetration) d.maximum_penetration=penetration;
    if (!IsFinite(d.wall_reaction) || !IsFinite(d.wall_moment) || !IsFinite(d.surface_power)) {
      Fail(s.control,Code::NonFiniteArithmetic,node.node); break;
    }
  }
  d.node_count=s.model.node_count; d.parent_count=s.model.parent_count; d.stiffness_rate_bound=s.model.rate;
}
} // namespace tlfea::contact::nodal_wall_device_detail
